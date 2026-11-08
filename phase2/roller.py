"""Roller (a 10.6 t single-drum vibratory roller, BOMAG BW 211 D-5 class) compacting the spread fill.

The roller is scripted (kinematic) like the dozer and the carriers: it follows the ground surface and does
not touch the MPM soil. What it does to the soil is in the soil zone (SoilZones.roll): each pass of its
drum over a loose column packs the top LIFT_M of soil by PASS_SHARE of what it still lacks of the soil's
compacted density (soils.py "compacted", from US Army FM 5-434). PASS_SHARE is set so that six passes give
95 % of the density gain (1 - 0.61^6 = 0.95): field guides put the passes a vibratory roller needs on a
0.2-0.3 m lift at 4 to 8 (e.g. West Virginia DOT, Soil & Aggregate Compaction Inspector manual, 2024). Clay
is rolled with the same drum here; on site it would get the padfoot version (BW 211 PD-5), which kneads it.

Rolling, as fill is rolled on site: the roller works the spread area in straight lanes along +x, one drum
width apart less an overlap, forward and back over each lane (a pass each way) until ROLL_PASSES passes have
gone over nearly all the soil in the lane since soil was last added there. The fill area is two cells; the
roller rolls the one the dozer is not spreading, and parks east of the area when it is done. A lane whose
soil still stands well above the design surface (a heap the dozer has not spread) is left for later.

Machine (BOMAG data sheet via dealer listings): operating weight 10.6 t, drum 2.13 m wide and 1.50 m in
diameter, top speed 10.9 km/h. Rolling speed ROLL_SPEED (4 km/h): 3-6 km/h is the usual range for soil.
The roller turns on the spot here, which an articulated roller cannot do; it reaches each lane by a short
diagonal drive instead of the three-point turns a real one would make.
"""

import math

import numpy as np

from dozer import quat_from_matrix, rot_x, rot_y, rot_z

MASS_KG = 10600.0
DRUM_W, DRUM_R = 2.13, 0.75  # m
DRUM_X, AXLE_X = 1.45, -1.45  # drum and rear axle ahead of base (m)
WHEEL_R = 0.75  # rear tyres (m)
OVERLAP = 0.15  # m between lanes
LIFT_M = 0.3  # m: the lift a pass compacts
PASS_SHARE = 0.39  # share of the remaining gain to compacted density that one pass makes
ROLL_PASSES = 6
DONE_SHARE = 0.9  # a lane is done once this share of its soil has had ROLL_PASSES passes
ROUGH_M = 0.4  # a lane whose surface stands this far above the design surface (90th percentile) waits
ROLL_SPEED, TRAVEL_SPEED = 4.0 / 3.6, 2.0  # m/s
ACCEL = 0.5  # m/s2
SPIN_RATE = math.radians(20.0)
CONTACT_L = 0.2  # m of drum on the ground along the heading (for the compaction footprint)
STAGES = ["parked", "to lane", "roll", "to park"]


class Roller:
    def __init__(self, park, x_range, cells, ground_z):
        """park: (x, y, yaw) where it waits; x_range: (x0, x1) of the drum along every lane; cells: (y0, y1)
        covered by each cell's lanes (it rolls one cell at a time)."""
        self.park, self.ground_z = park, ground_z
        self.x0, self.x1 = x_range
        step = DRUM_W - OVERLAP
        self.cell_lanes = []
        for y0, y1 in cells:
            n = max(1, math.ceil((y1 - y0 - DRUM_W) / step) + 1)  # lanes at most a step apart, spread evenly
            self.cell_lanes.append(list(np.linspace(y0 + DRUM_W / 2, y1 - DRUM_W / 2, n)))
        self.reset()

    def reset(self):
        self.x, self.y, self.yaw = self.park
        self.z, self.pitch, self.roll = self.ground_z + DRUM_R, 0.0, 0.0
        self.speed = 0.0
        self.cell, self.lanes = 0, self.cell_lanes[0]
        self.stage = "parked"
        self.plan = []  # ("spin", yaw) | ("drive", x, y, dir, speed, stage)
        self.lane = None
        self.counters = {"lanes_rolled": 0, "roll_m": 0.0, "parked_s": 0.0, "working_s": 0.0, "passes": 0}

    @property
    def parked(self):
        return self.stage == "parked"

    def set_cell(self, cell):
        """Roll this cell from now on (leaving the lane in hand)."""
        if cell != self.cell:
            self.cell, self.lanes, self.lane = cell, self.cell_lanes[cell], None
            self.plan = [] if self._at_park() else self._to_park()

    def drum_pos(self):
        return np.array([self.x, self.y]) + DRUM_X * np.array([math.cos(self.yaw), math.sin(self.yaw)])

    def drum_footprint(self, step=0.04):
        """Points (n, 2) under the drum's contact strip."""
        u = np.arange(-CONTACT_L / 2 + step / 2, CONTACT_L / 2, step)
        v = np.arange(-DRUM_W / 2 + step / 2, DRUM_W / 2, step)
        gu, gv = np.meshgrid(u, v)
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        x, y = self.drum_pos()
        return np.stack([x + c * gu.ravel() - s * gv.ravel(), y + s * gu.ravel() + c * gv.ravel()], 1)

    def cell_points(self):
        """Points (n, 2) under all the lanes of the cell it rolls."""
        return np.concatenate([self._lane_points(k) for k in range(len(self.lanes))])

    def _lane_points(self, k):
        x = np.arange(self.x0, self.x1, 0.1) + 0.05
        y = np.arange(-DRUM_W / 2, DRUM_W / 2, 0.1) + 0.05 + self.lanes[k]
        gx, gy = np.meshgrid(x, y)
        return np.stack([gx.ravel(), gy.ravel()], 1)

    # ------------------------------------------------------------------ planning
    def _start(self, k):
        """Base position with the drum at the lane's west end, heading +x."""
        return self.x0 - DRUM_X, self.lanes[k]

    def _to_park(self):
        px, py, pyaw = self.park
        return [("spin", math.atan2(py - self.y, px - self.x)), ("drive", px, py, +1, TRAVEL_SPEED, "to park"), ("spin", pyaw)]

    def _at_park(self):
        px, py, pyaw = self.park
        return math.hypot(self.x - px, self.y - py) < 0.1 and abs(math.sin(self.yaw - pyaw)) < 0.01

    def _next_job(self, may_work, rolled_share, surface, grade):
        if not may_work:
            self.lane = None
            return [] if self._at_park() else self._to_park()
        short = []
        for k in range(len(self.lanes)):
            pts = self._lane_points(k)
            rough = np.percentile(surface(pts), 90) - grade > ROUGH_M
            short.append(0.0 if rough else 1.0 - rolled_share(pts, ROLL_PASSES))
        if self.lane is not None and short[self.lane] > 1.0 - DONE_SHARE:
            best = self.lane  # finish the lane in hand
        else:
            best = int(np.argmax(short)) if max(short) > 1.0 - DONE_SHARE else None
        if best is None:
            if self.lane is not None:
                self.counters["lanes_rolled"] += 1
            self.lane = None
            return [] if self._at_park() else self._to_park()
        if best != self.lane and self.lane is not None:
            self.counters["lanes_rolled"] += 1
        sx, sy = self._start(best)
        moves = []
        if best != self.lane:
            moves += [("spin", math.atan2(sy - self.y, sx - self.x)), ("drive", sx, sy, +1, TRAVEL_SPEED, "to lane"), ("spin", 0.0)]
        moves += [("drive", self.x1 - DRUM_X, sy, +1, ROLL_SPEED, "roll"), ("drive", sx, sy, -1, ROLL_SPEED, "roll")]
        self.lane = best
        return moves

    # ------------------------------------------------------------------ motion
    def _drive(self, tx, ty, direction, v_max, dt):
        heading = np.array([math.cos(self.yaw), math.sin(self.yaw)]) * direction
        to = np.array([tx - self.x, ty - self.y])
        dist = float(np.dot(to, heading))
        v_stop = math.sqrt(max(2 * ACCEL * max(dist, 0.0), 0.0))
        self.speed = min(self.speed + ACCEL * dt, v_max, v_stop)
        step = min(self.speed * dt, max(dist, 0.0))
        self.x += heading[0] * step
        self.y += heading[1] * step
        if self.stage == "roll":
            self.counters["roll_m"] += step
        if dist <= 0.01:
            self.x, self.y, self.speed = tx, ty, 0.0
            return True
        return False

    def _spin(self, yaw, dt):
        d = math.atan2(math.sin(yaw - self.yaw), math.cos(yaw - self.yaw))
        if abs(d) < 1e-3:
            self.yaw = yaw
            return True
        self.yaw += float(np.clip(d, -SPIN_RATE * dt, SPIN_RATE * dt))
        return False

    def _follow_ground(self, surface):
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        p = np.array([self.x, self.y])
        side = np.array([-s, c]) * (DRUM_W / 2 - 0.2)
        pts = np.array([p + DRUM_X * np.array([c, s]) + side, p + DRUM_X * np.array([c, s]) - side,
                        p + AXLE_X * np.array([c, s]) + side, p + AXLE_X * np.array([c, s]) - side])
        h = np.maximum(surface(pts), self.ground_z)
        front, rear = h[:2].mean(), h[2:].mean()
        self.z = 0.5 * (front + rear) + DRUM_R
        self.pitch = math.atan2(front - rear, DRUM_X - AXLE_X)
        self.roll = math.atan2(h[0] + h[2] - h[1] - h[3], 2 * (DRUM_W - 0.4))

    def update(self, dt, surface, may_work, rolled_share, grade):
        """Advance one frame. surface(xy) -> ground and soil heights; may_work: the dozer is parked with
        nothing to spread; rolled_share(xy, passes) -> share of the soil there rolled that often; grade: the
        design surface."""
        self.counters["parked_s" if self.parked else "working_s"] += dt
        if not may_work and self.lane is not None:
            self.plan, self.lane = self._to_park(), None  # the dozer has work: clear out
        if not self.plan:
            self.plan = self._next_job(may_work, rolled_share, surface, grade)
            if not self.plan and self._at_park():
                self.stage = "parked"
        if self.plan:
            move = self.plan[0]
            if move[0] == "spin":
                self.stage = "to park" if self.lane is None else "to lane"
                done = self._spin(move[1], dt)
            else:
                _, tx, ty, d, v, label = move
                self.stage = label
                done = self._drive(tx, ty, d, v, dt)
            if done:
                self.plan.pop(0)
                if move[0] == "drive" and move[5] == "roll":
                    self.counters["passes"] += 1
        self._follow_ground(surface)

    def base_pos(self):
        return np.array([self.x, self.y, self.z])

    def base_quat(self):
        return quat_from_matrix(rot_z(self.yaw) @ rot_y(-self.pitch) @ rot_x(self.roll))
