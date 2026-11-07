"""Dozer (Komatsu D37PXi-24, the PWRI OPERA model) spreading the carrier's loads at the spoil area.

The dozer is scripted (kinematic) like the carrier: its tracks follow the ground surface and do not touch
the soil, and it does not slip. Its blade is a kinematic body in the simulator, so the soil it pushes is
MPM particles like any other, and the sand's force on the blade is measured. Two things close the loop
between that force and the machine, as on the real D37PXi:

    grade control   the blade's cutting edge is held at the design surface (the "i" in D37PXi is Komatsu's
                    integrated machine control, which does this from GNSS), within the lift cylinder's range
    load control    when the blade load nears what the tracks can pull, the blade is lifted until the load
                    drops, then lowered back to grade (the real machine does this to limit track slip);
                    the travel speed falls with load as the drawbar power allows, and above the
                    tracks' traction the dozer makes no headway (the tracks slip) until the blade lifts

Spreading (zone frame, the spoil area east of the carrier's dump point), as end-dumped fill is spread: the
carrier always dumps at the same place, so the lanes fan out from there. Every push starts at the dump
heap and heads for a different part of the far edge of the spread area, blade at the design surface, a
lift LIFT above the ground; soil above grade is carried ahead and dropped where the surface is low, so each
load becomes a layer. Each lane is scored by min(soil above grade near the heap, room below grade along
the lane), and the dozer works the best one, backs up with the blade raised, and repeats until no lane's
heap part holds more than DONE_M3; then it parks north of the dump point. A pass that leaves its lane's
soil as it was blocks that lane until the next load arrives (the soil it scores is out of the blade's
reach). When the area holds more soil above grade than room below it, the layer is full and the design
surface rises by one lift (up to MAX_FILL). The carrier
must not dump while the dozer works there: the server holds the carrier at its turn point until the dozer
has parked, and recalls the dozer when the carrier leaves the loading spot.

Geometry from the model (d37pxi_ros2, urdf/d37pxi24.xacro and the meshes): lift arm pivot 0.586 m ahead of
base_link and 0.386 m above it, lift range -0.41..0.19 rad (positive lowers the blade); blade frame 1.588 m
along the arm and 0.104 m below it; blade 3.21 m wide and 1.09 m high, cutting edge 0.41 m ahead of and
0.30 m below the blade frame. base_link is 0.049 m above the track bottom; tracks about 2.6 m on the ground.
"""

import math

import numpy as np

import tracks

ARM_PIVOT = np.array([0.5858, 0.0, 0.38606])  # lift_joint in base_link
BLADE_ON_ARM = np.array([1.5881, 0.0, -0.10373])  # blade frame (angle_joint) in the arm frame
EDGE = np.array([0.41, 0.0, -0.30])  # cutting edge in the blade frame
LIFT_LO, LIFT_HI = -0.410, 0.187  # rad (URDF limits; LIFT_LO raises the blade fully)
BASE_ABOVE_GROUND = 0.049
BLADE_W = 3.214
# blade face (blade frame, x forward, z up) from the mesh: lower plate, middle, top curl; the collider plates
# lie just behind this line
FACE = [(0.41, -0.30), (0.20, 0.0), (0.18, 0.45), (0.30, 0.79)]
PLATE = 0.08  # collider plate thickness
TRACK_X = (-1.2, 1.35)  # track contact length in base_link (rear, front)
TRACK_Y = 0.8  # half gauge
SHOE_B, TRACK_L = 0.61, 2.235  # shoe width and track on the ground (m), D37PX-24 (tracks.py)

# machine data: Komatsu D37PXi-24 operating weight 9.38 t (LECTURA Specs; 9.5 t used here), net power 66.1 kW
# (ISO 9249, as the D37PX-24 with the same SAA4D95LE-7 engine); blade 3.2 m wide (PAT); hydrostatic drive
MASS_KG = 9500.0
P_DRAWBAR = 0.7 * 66e3  # W at the drawbar
# drawbar pull and slip come from the soil (tracks.Track): about 57 kN at full slip on loam, 69 on dry sand, 41 on clay
SPEED_PUSH, SPEED_REVERSE, SPEED_TRAVEL, SPEED_MIN = 1.0, 1.6, 1.4, 0.2  # m/s
ACCEL = 0.5  # m/s2
SPIN_RATE = math.radians(30.0)
LIFT_RATE = 0.3  # rad/s at the lift joint (about 0.5 m/s at the edge)
LOAD_HIGH, LOAD_LOW = 0.85, 0.6  # load control: lift above LOAD_HIGH * the pull at full slip, return to grade below LOAD_LOW
FORCE_TAU = 0.15  # s: the blade force is low-passed (one MPM step's impulses are noisy) for control and reports
EMPTY_N, EMPTY_S = 3000.0, 0.7  # a push ends when the blade carries less than EMPTY_N for EMPTY_S
LIFT = 0.3  # m per layer
TOL = 0.08  # m above grade still counted as spread
DONE_M3 = 0.35  # a lane whose heap part holds less soil than this above grade + TOL is done
HEAP_PART = 0.6  # the first part of each lane, from the dump heap: the soil there is what is left to spread
MAX_FILL = 1.5  # m: the design surface rises by layers up to this above the ground
FUTILE_M3 = 0.15  # a pass that moves less than this out of its lane's heap part blocks the lane
NEW_LOAD_M3 = 1.0  # the area's soil above grade growing by this much means a new load: blocked lanes reopen


def rot_z(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def rot_y(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])


def rot_x(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]])


def quat_from_matrix(R):
    t = R[0, 0] + R[1, 1] + R[2, 2]
    if t > 0:
        s = 0.5 / math.sqrt(t + 1.0)
        q = [(R[2, 1] - R[1, 2]) * s, (R[0, 2] - R[2, 0]) * s, (R[1, 0] - R[0, 1]) * s, 0.25 / s]
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = 2.0 * math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
        q = [0.25 * s, (R[0, 1] + R[1, 0]) / s, (R[0, 2] + R[2, 0]) / s, (R[2, 1] - R[1, 2]) / s]
    elif R[1, 1] > R[2, 2]:
        s = 2.0 * math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
        q = [(R[0, 1] + R[1, 0]) / s, 0.25 * s, (R[1, 2] + R[2, 1]) / s, (R[0, 2] - R[2, 0]) / s]
    else:
        s = 2.0 * math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
        q = [(R[0, 2] + R[2, 0]) / s, (R[1, 2] + R[2, 1]) / s, 0.25 * s, (R[1, 0] - R[0, 1]) / s]
    return np.array(q)


def blade_plates():
    """Collider boxes in the blade frame: (centre, half extents, pitch about y)."""
    out = []
    for (x0, z0), (x1, z1) in zip(FACE[:-1], FACE[1:]):
        L = math.hypot(x1 - x0, z1 - z0)
        nx, nz = -(z1 - z0) / L, (x1 - x0) / L  # normal pointing back (-x side of the face)
        nx, nz = (nx, nz) if nx < 0 else (-nx, -nz)
        c = ((x0 + x1) / 2 + nx * PLATE / 2, 0.0, (z0 + z1) / 2 + nz * PLATE / 2)
        out.append((c, (L / 2 + 0.02, BLADE_W / 2, PLATE / 2), -math.atan2(z1 - z0, x1 - x0)))
    return out


STAGES = ["parked", "to lane", "lower", "push", "raise", "back", "to park"]


class Dozer:
    def __init__(self, park, area, ground_z, soil="dry-sand"):
        """park: (x, y, yaw) of base_link where the dozer waits; area: (x_start, y_start, x_end, end ys) of
        the cutting edge: every push starts at (x_start, y_start), at the dump heap, and runs to x_end, the
        lanes fanning out to the end ys; soil: the preset it spreads (dug, so at its remoulded strength)."""
        self.park, self.ground_z = park, ground_z
        self.track = tracks.Track(SHOE_B, TRACK_L, soil, loose=True)
        self.f_traction = self.track.max_pull(MASS_KG)  # N: drawbar pull at full slip
        self.sinkage = self.track.sinkage(MASS_KG)
        self.x_start, self.y_start, self.x_end, ends = area
        # lanes: heading and the cutting edge's travel to x_end
        self.lanes = [(math.atan2(y - self.y_start, self.x_end - self.x_start), math.hypot(y - self.y_start, self.x_end - self.x_start)) for y in ends]
        self.start = np.array([self.x_start - self._edge_ahead(), self.y_start])  # base_link at the start of every push
        self.reset()

    def reset(self):
        self.x, self.y, self.yaw = self.park
        self.z, self.pitch, self.roll = self.ground_z + BASE_ABOVE_GROUND, 0.0, 0.0
        self.lift, self.speed = LIFT_LO, 0.0
        self.grade = self.ground_z + LIFT
        self.stage, self.stage_time, self.time = "parked", 0.0, 0.0
        self.plan = []  # moves still to make: ("spin", yaw) | ("drive", x, y, dir, speed, stage) | ("lift", "up"/"raise"/"grade") | ("push", x, y)
        self.lane = None
        self.blocked = set()  # lanes whose last pass moved nothing
        self.pass_soil = None  # the lane's heap-part soil when its pass began
        self.area_soil = 0.0  # the area's soil above grade at the last planning
        self.recall = False
        self.force = 0.0  # soil resisting the blade along the heading (N), low-passed
        self.load_lift = False
        self.empty_t = 0.0
        self.prev = None
        self.v, self.w = np.zeros(3), np.zeros(3)
        self.counters = {"passes": 0, "push_m": 0.0, "work_MJ": 0.0, "load_lifts": 0, "lifts_raised": 0, "parked_s": 0.0, "working_s": 0.0, "peak_kN": 0.0, "slip_s": 0.0, "slip_m": 0.0, "futile_passes": 0}
        self.push_force = []  # mean blade force per push (kN)
        self._push_sum = [0.0, 0.0]

    # ------------------------------------------------------------------ geometry
    def base_rot(self):
        return rot_z(self.yaw) @ rot_y(-self.pitch) @ rot_x(self.roll)

    def base_pos(self):
        return np.array([self.x, self.y, self.z])

    def blade_pose(self, lift=None):
        """Blade frame origin and rotation (zone frame)."""
        lift = self.lift if lift is None else lift
        R = self.base_rot()
        Ra = R @ rot_y(lift)
        return self.base_pos() + R @ ARM_PIVOT + Ra @ BLADE_ON_ARM, Ra

    def edge(self, lift=None):
        p, R = self.blade_pose(lift)
        return p + R @ EDGE

    def blade_box(self):
        """World AABB of the blade (for the soil zone's waking)."""
        p, R = self.blade_pose()
        xs = [FACE[0][0], FACE[-1][0], -PLATE]
        zs = [FACE[0][1], FACE[-1][1]]
        pts = np.array([[x, y, z] for x in (min(xs), max(xs)) for y in (-BLADE_W / 2, BLADE_W / 2) for z in zs])
        w = p + pts @ R.T
        return w.min(axis=0) - 0.1, w.max(axis=0) + 0.1

    def lift_for_edge(self, z_target):
        """Lift angle putting the cutting edge at z_target (clamped to the cylinder's range)."""
        lo, hi = LIFT_LO, LIFT_HI  # edge height falls as the lift angle grows
        if self.edge(lo)[2] <= z_target:
            return lo
        if self.edge(hi)[2] >= z_target:
            return hi
        for _ in range(24):
            m = 0.5 * (lo + hi)
            if self.edge(m)[2] > z_target:
                lo = m
            else:
                hi = m
        return 0.5 * (lo + hi)

    @property
    def parked(self):
        return self.stage == "parked"

    # ------------------------------------------------------------------ planning
    def _swath(self, k, s0=0.0, s1=1.0):
        """Points (0.1 m apart) of lane k's swath, along the lane from fraction s0 to s1 of its length."""
        th, L = self.lanes[k]
        d, n = np.array([math.cos(th), math.sin(th)]), np.array([-math.sin(th), math.cos(th)])
        S, T = np.meshgrid(np.arange(s0 * L, s1 * L, 0.1) + 0.05, np.arange(-BLADE_W / 2, BLADE_W / 2, 0.1) + 0.05)
        return np.array([self.x_start, self.y_start]) + S.ravel()[:, None] * d + T.ravel()[:, None] * n

    def _lane_scores(self, surface):
        """Per lane: soil above grade + TOL in its heap part (m3), and room below grade - TOL along it (m3)."""
        soil, room = [], []
        for k in range(len(self.lanes)):
            soil.append(float(np.clip(surface(self._swath(k, 0.0, HEAP_PART)) - self.grade - TOL, 0.0, None).sum() * 0.01))
            room.append(float(np.clip(self.grade - TOL - surface(self._swath(k)), 0.0, None).sum() * 0.01))
        return np.array(soil), np.array(room)

    def _area_points(self):
        """The spread area: every point of any lane's swath, on a 0.1 m grid."""
        pts = np.round(np.concatenate([self._swath(k) for k in range(len(self.lanes))]) / 0.1).astype(int)
        return (np.unique(pts, axis=0) + 0.5) * 0.1

    def spread_report(self, surface):
        """Spread quality over the spread area (the lanes' swaths): soil-covered area, how far the covered
        surface is from the design surface (RMS), the highest point above it, the fraction at grade, and the
        soil not yet spread (m3 above grade + TOL)."""
        h = surface(self._area_points())
        covered = h > self.ground_z + 0.05
        dev = h[covered] - self.grade
        return {
            "design_z_above_ground": round(self.grade - self.ground_z, 2),
            "area_m2": round(len(h) * 0.01, 1),
            "covered_m2": round(float(covered.sum()) * 0.01, 1),
            "rms_from_grade_m": round(float(np.sqrt(np.mean(dev**2))), 3) if covered.any() else None,
            "max_above_grade_m": round(float(dev.max()), 2) if covered.any() else None,
            "at_grade_fraction": round(float(np.mean(h > self.grade - TOL)), 2),
            "unspread_m3": round(float(np.clip(h - self.grade - TOL, 0.0, None).sum() * 0.01), 2),
        }

    def _to_start(self):
        """Moves from the park to the start of the pushes (blade raised): reverse south to it."""
        return [("lift", "up"), ("spin", math.pi / 2), ("drive", *self.start, -1, SPEED_REVERSE, "to lane")]

    def _to_park(self):
        px, py, pyaw = self.park
        return [("lift", "up"), ("spin", pyaw), ("drive", px, py, +1, SPEED_TRAVEL, "to park")]

    def _at_park(self):
        px, py, pyaw = self.park
        return math.hypot(self.x - px, self.y - py) < 0.1 and abs(math.sin(self.yaw - pyaw)) < 0.01 and math.cos(self.yaw - pyaw) > 0

    @staticmethod
    def _edge_ahead():
        """Cutting edge ahead of base_link with the blade down (m)."""
        return float((ARM_PIVOT + rot_y(LIFT_HI) @ (BLADE_ON_ARM + EDGE))[0])

    def _next_job(self, surface, may_work):
        """Pick the next moves once the current ones are done."""
        at_lane = self.lane is not None
        if not may_work:
            self.lane = None
            return [] if self._at_park() else self._to_park()
        soil, room = self._lane_scores(surface)
        if self.lane is not None and self.pass_soil is not None and self.pass_soil - soil[self.lane] < FUTILE_M3:
            self.blocked.add(self.lane)  # the last pass moved nothing: the blade does not reach that soil
            self.counters["futile_passes"] = self.counters.get("futile_passes", 0) + 1
        self.pass_soil = None
        h = surface(self._area_points())
        area_soil = float(np.clip(h - self.grade - TOL, 0.0, None).sum() * 0.01)
        area_room = float(np.clip(self.grade - TOL - h, 0.0, None).sum() * 0.01)
        if area_soil > self.area_soil + NEW_LOAD_M3:
            self.blocked.clear()  # a new load: every lane may have soil to move again
        self.area_soil = area_soil
        if area_soil >= 2 * DONE_M3 and area_soil > area_room and self.grade + LIFT <= self.ground_z + MAX_FILL + 1e-6:
            # more soil above grade than room below it: this layer is full, the next goes on top
            self.grade += LIFT
            self.counters["lifts_raised"] += 1
            self.blocked.clear()
            self.area_soil = 0.0
            return []
        score = np.minimum(soil, room)
        score[list(self.blocked)] = 0.0
        if score.max() < DONE_M3:
            self.lane = None
            return [] if self._at_park() else self._to_park()
        k = int(np.argmax(score))
        th, L = self.lanes[k]
        d = np.array([math.cos(th), math.sin(th)])
        moves = [] if at_lane else self._to_start()
        moves += [("lift", "up"), ("spin", th), ("lift", "grade"), ("push", *(self.start + d * L)), ("lift", "raise"), ("drive", *self.start, -1, SPEED_REVERSE, "back")]
        self.lane, self.pass_soil = k, soil[k]
        return moves

    # ------------------------------------------------------------------ motion
    def _drive(self, tx, ty, direction, v_max, dt):
        heading = np.array([math.cos(self.yaw), math.sin(self.yaw)]) * direction
        dist = float(np.dot(np.array([tx - self.x, ty - self.y]), heading))
        v_stop = math.sqrt(max(2 * ACCEL * max(dist, 0.0), 0.0))
        s = abs(self.speed)
        target = min(v_max, v_stop)
        s = min(target, s + ACCEL * dt) if s < target else max(target, s - ACCEL * dt)
        self.speed = s * direction
        self.x += self.speed * math.cos(self.yaw) * dt
        self.y += self.speed * math.sin(self.yaw) * dt
        if dist <= 0.02 or (s < 0.02 and dist < 0.05):
            self.speed = 0.0
            return True
        return False

    def _spin(self, yaw, dt):
        err = math.atan2(math.sin(yaw - self.yaw), math.cos(yaw - self.yaw))
        step = math.copysign(min(abs(err), SPIN_RATE * dt), err)
        self.yaw += step
        if abs(err) < 1e-4:
            self.yaw = yaw
            return True
        return False

    def _lift_to(self, target, dt):
        step = float(np.clip(target - self.lift, -LIFT_RATE * dt, LIFT_RATE * dt))
        self.lift += step
        return abs(target - self.lift) < 1e-3

    def _follow_ground(self, surface, dt):
        """Tracks on the surface: height, pitch and roll from the ground under the track ends."""
        R = rot_z(self.yaw)
        pts = [(x, s * TRACK_Y) for x in TRACK_X for s in (-1, 1)]
        xy = np.array([self.base_pos()[:2] + (R @ np.array([px, py, 0.0]))[:2] for px, py in pts])
        h = np.maximum(surface(xy), self.ground_z)
        rear, front = h[:2].mean(), h[2:].mean()
        right, left = h[0::2].mean(), h[1::2].mean()
        z = 0.5 * (rear + front) + BASE_ABOVE_GROUND
        pitch = math.atan2(front - rear, TRACK_X[1] - TRACK_X[0])
        roll = math.atan2(left - right, 2 * TRACK_Y)
        self.z += float(np.clip(z - self.z, -0.6 * dt, 0.6 * dt))
        self.pitch += float(np.clip(pitch - self.pitch, -math.radians(20) * dt, math.radians(20) * dt))
        self.roll += float(np.clip(roll - self.roll, -math.radians(20) * dt, math.radians(20) * dt))

    def update(self, dt, surface, blade_force, may_work):
        """Advance one frame. surface(xy (n, 2)) -> heights of the ground and soil; blade_force: the soil's
        force on the blade (zone frame, N); may_work: the dump area is free of the carrier."""
        self.time += dt
        self.stage_time += dt
        heading = np.array([math.cos(self.yaw), math.sin(self.yaw), 0.0])
        f = max(-float(np.dot(blade_force, heading)), 0.0)
        self.force += (f - self.force) * min(dt / FORCE_TAU, 1.0)
        self.counters["parked_s" if self.parked else "working_s"] += dt
        if not may_work and self.plan and self.stage in ("lower", "push"):
            # the carrier is coming: stop pushing, back out with the blade up, and park
            self.plan = [("lift", "raise"), ("drive", *self.start, -1, SPEED_REVERSE, "back")]
            self._end_push()
        if not self.plan:
            self.plan = self._next_job(surface, may_work)
            if not self.plan and self._at_park():
                self._set("parked")
        if self.plan:
            self._do(self.plan[0], dt)
        self._follow_ground(surface, dt)
        self._velocities(dt)

    def _set(self, stage):
        if stage != self.stage:
            self.stage, self.stage_time = stage, 0.0

    def _do(self, move, dt):
        kind = move[0]
        done = False
        if kind == "spin":
            self._set("to park" if self.lane is None else "to lane")
            done = self._spin(move[1], dt)
        elif kind == "drive":
            _, tx, ty, d, v, label = move
            self._set(label)
            done = self._drive(tx, ty, d, v, dt)
        elif kind == "lift":
            if move[1] == "grade":
                self._set("lower")
                done = self._lift_to(self.lift_for_edge(self.grade), dt)
            else:
                if move[1] == "raise":
                    self._set("raise")
                done = self._lift_to(LIFT_LO, dt)
        elif kind == "push":
            self._set("push")
            done = self._push(move[1], move[2], dt)
            if done:
                self._end_push()
        if done:
            self.plan.pop(0)

    def _push(self, tx, ty, dt):
        """Grade and load control while pushing toward tx; True when the pass is over."""
        F = self.force
        if F > LOAD_HIGH * self.f_traction and not self.load_lift:
            self.load_lift = True
            self.counters["load_lifts"] += 1
        elif F < LOAD_LOW * self.f_traction:
            self.load_lift = False
        target = LIFT_LO if self.load_lift else self.lift_for_edge(self.grade)
        if self.load_lift:  # lift only as far as needed: a small step per frame while overloaded
            target = max(self.lift - LIFT_RATE * dt, LIFT_LO)
        self._lift_to(target, dt)
        v_track = float(np.clip(P_DRAWBAR / max(F, 1.0), SPEED_MIN, SPEED_PUSH))  # the tracks' speed
        slip = self.track.slip(MASS_KG, F)
        v_max = v_track * (1.0 - slip)  # the machine's: the soil under the tracks shears by the rest
        if slip >= 1.0:  # more than the tracks can pull: they spin, the dozer stands while the blade lifts
            v_max = 0.0
            self.speed = 0.0
            self.counters["slip_s"] += dt
        x0, y0 = self.x, self.y
        done = self._drive(tx, ty, +1, v_max, dt) if v_max > 0.0 else False
        moved = math.hypot(self.x - x0, self.y - y0)
        self.counters["slip_m"] += moved * slip / max(1.0 - slip, 1e-6) if slip < 1.0 else v_track * dt
        self.counters["push_m"] += moved
        self.counters["work_MJ"] += F * moved / 1e6
        self.counters["peak_kN"] = max(self.counters["peak_kN"], F / 1000)
        self._push_sum[0] += F * dt
        self._push_sum[1] += dt
        # the blade is empty once past the soil: end the pass early
        past_start = self.edge()[0] > self.x_start + 1.5
        self.empty_t = self.empty_t + dt if (F < EMPTY_N and past_start and not self.load_lift) else 0.0
        if self.empty_t > EMPTY_S:
            self.speed = 0.0
            return True
        return done

    def _end_push(self):
        if self._push_sum[1] > 0:
            self.push_force.append(round(self._push_sum[0] / self._push_sum[1] / 1000, 1))
            self.counters["passes"] += 1
        self._push_sum = [0.0, 0.0]
        self.empty_t, self.load_lift = 0.0, False

    def _velocities(self, dt):
        p, R = self.blade_pose()
        if self.prev is not None:
            p0, R0 = self.prev
            self.v = (p - p0) / dt
            dR = R @ R0.T
            angle = math.acos(float(np.clip((np.trace(dR) - 1) / 2, -1.0, 1.0)))
            axis = np.array([dR[2, 1] - dR[1, 2], dR[0, 2] - dR[2, 0], dR[1, 0] - dR[0, 1]])
            n = np.linalg.norm(axis)
            self.w = axis / n * angle / dt if n > 1e-9 else np.zeros(3)
        self.prev = (p, R)

    def blade_quat(self):
        return quat_from_matrix(self.blade_pose()[1])

    def base_quat(self):
        return quat_from_matrix(self.base_rot())
