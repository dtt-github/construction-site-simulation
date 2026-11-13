"""Crawler dump carriers (IHI/Kato IC120, the PWRI OPERA model) running a haul cycle: loaded beside the
excavator, each drives to a spoil area, tips its vessel to dump, and comes back.

A carrier is scripted (kinematic): it follows its route at the speeds below and does not feel the ground
or the load. The vessel is a kinematic body in the simulator, so the soil in it and the soil leaving it
are MPM particles like any other (excavator_server.py hands the load over: taken from the vessel's soil
zone when the carrier leaves, released as particles from the tipping vessel at the spoil).

Route (zone frame, cab toward +x at the loading spot; the vessel tips about a pin at its rear):
    loading -> drive out forward (+x) -> spin turn to face -x -> (wait while the dozer or another carrier
    is at the dump point) -> reverse to the dump point -> tip, dump, lower -> drive forward (-x) to the
    return point, then
      spot free:  spin turn to face +x -> reverse into the loading spot
      spot busy:  drive to the waiting place north of the road (QUEUE) -> wait -> drive down to the road
                  -> spin turn to face +x -> reverse into the loading spot
Spin turns are made away from the excavator (its tracks are 2.5 m from the parked carrier's corner).

Longer hauls (haul_m > 0, or more than two carriers): the middle of the road is off-screen, two lanes, haul_m
long each way. A loaded carrier drives out, leaves the view ("haul out", timed at its speed with accelerating
and braking), and comes back into view on the dump side facing the dump point, once that is clear ("wait at
dump", still off-screen); it backs in, dumps, drives clear and leaves again ("haul back"). It comes back into
view at the waiting place (QUEUE) once that is free ("queue on road": carriers queue on the road, off-screen),
and goes on to the spot as before. Off-screen, a carrier is parked out of sight below the ground.

The haul road is one lane, so the server gates three moves with the other carriers' positions (Fleet):
a loaded carrier leaves the spot only while every other carrier waits at QUEUE (two never meet head-on
on the lane); a waiting carrier comes down to the spot once it is empty and the carrier that left it is
past its turn point; a carrier turns and backs into the dump point only while no other carrier or the dozer
is there (it waits short of the turn, at WAIT_X). The places were checked against each spin turn's swept
circle (4.15 m radius about base_link), and traffic.py keeps every machine's outline clear of the others.

Geometry from the model (ic120_ros2, urdf/ic120.xacro and the vessel mesh): vessel pin 3.015 m behind
base_link and 0.946 m above the ground, tip range 0..65 deg; the vessel's floor centre 1.575 m ahead of the
pin and 0.354 m above it (inside floor 1.3 m above the ground).
"""

import math

import numpy as np

PIN = np.array([-3.0153, 0.0, 0.94605])  # vessel pin in base_link (m)
FLOOR_FROM_PIN = np.array([1.575, 0.0, 0.354])  # vessel body origin (middle of the floor) in the pin frame
VESSEL_X = PIN[0] + FLOOR_FROM_PIN[0]  # vessel origin x in base_link when lowered (-1.44)
TIP_MAX = math.radians(60.0)  # dumping angle used (the vessel allows 65 deg)

# performance: IC120 spec sheets give 10.9 km/h top speed; on a site haul road loaded carriers run slower
SPEED_LOADED, SPEED_EMPTY, SPEED_REVERSE = 2.2, 2.8, 1.2  # m/s
ACCEL = 0.6  # m/s2
# tracks (tracks.py): IC120 operating weight 14.1 t; 600 mm shoes (assumed), 3.55 m on the ground each,
# centred 0.2 m ahead of base_link, 1.0 m either side
MASS_KG = 14100.0
SHOE_B, TRACK_L, TRACK_X0, HALF_GAUGE = 0.6, 3.55, 0.2, 1.0
SPIN_RATE = math.radians(25.0)  # rad/s, counter-rotating tracks
TIP_RATE, LOWER_RATE = math.radians(7.5), math.radians(12.0)  # rad/s (about 8 s up to 60 deg, 5 s down)
DUMP_HOLD = 3.0  # s at full tip, for the soil to leave
# sticky soil stays in a tipped vessel: the operator shakes it (tip oscillating SHAKE_DEG every SHAKE_PERIOD)
# until less than CARRY_BACK of the load is left, for up to DUMP_MAX s; what is left rides back
SHAKE_DEG, SHAKE_PERIOD, CARRY_BACK, DUMP_MAX = 4.0, 0.8, 0.03, 15.0
# the dumped pile builds up under the tipped vessel's tail and holds the rest of the load in: as drivers do,
# the carrier creeps forward while soil is still in the vessel, so the load spreads out behind it
CREEP_SPEED, CREEP_MAX = 0.4, 2.5  # m/s, m
# base_link x where a loaded carrier waits for the dump point to clear: its cab 0.3 m short of the circle the dozer
# sweeps spinning at the start of its pushes (traffic.py outlines), and so of the dozer's way north to its park
WAIT_X = 17.0
RETURN_X = 18.5  # base_link x where an empty carrier turns back toward the spot, or leaves the road to wait
QUEUE = (14.5, 11.0, -math.pi / 2)  # waiting place north of the road: base_link x, y, heading (cab to the road)
# longer hauls: base_link x where a loaded carrier leaves the view, and where it comes back into it facing the dump
# point (its cab clears a carrier spinning onto the spot from QUEUE, and a loaded one driving out to HIDE_X)
HIDE_X, EMERGE_X = 14.5, 21.5
DOZER_CLEAR_S = 20.0  # s before a loaded carrier is due at the dump side that the dozer stops work there
HIDE_DEPTH = 20.0  # m below the ground where an off-screen carrier is parked (as the dozer's blade when unused)

STAGES = ("loading", "drive out", "turn to dump", "reverse to dump", "tip", "dump", "lower", "return", "turn to load", "reverse in",
          "wait at dump", "to queue", "wait to load", "from queue", "haul out", "haul back", "queue on road")
AT_SPOT = ("loading", "reverse in", "from queue")  # using the loading spot or on the way into it
LEAVING = ("drive out", "turn to dump", "wait at dump")  # loaded, on the lane between the spot and the dump point
AT_DUMP = ("reverse to dump", "tip", "dump", "lower", "return")


def yaw_quat(yaw):
    return np.array([0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2)])


def quat_mul(a, b):
    x1, y1, z1, w1 = a
    x2, y2, z2, w2 = b
    return np.array([w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2, w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
                     w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2, w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2])


def rot(yaw, tip):
    """Rotation of the vessel: yaw about z, then tip about the pin's y axis (front up for tip > 0)."""
    cy, sy, ct, st = math.cos(yaw), math.sin(yaw), math.cos(tip), math.sin(tip)
    Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    Ry = np.array([[ct, 0, -st], [0, 1, 0], [st, 0, ct]])  # about +y by -tip: +x rises
    return Rz @ Ry


class Dumper:
    def __init__(self, load_center, ground_z, dump_x, turn_x, capacity_kg, start="spot", index=0, haul_m=None):
        """load_center: vessel centre (x, y) at the loading spot; dump_x: base_link x when dumping (the
        rear, and the soil, further +x); turn_x: where the loaded carrier spins round; start: "spot" (at the
        loading spot), "queue" (waiting to load) or "road" (off-screen, queueing for the waiting place);
        haul_m: the off-screen road's length each way (None: the short road, no off-screen part)."""
        self.ground_z, self.capacity_kg = ground_z, capacity_kg
        self.haul_m = haul_m
        self.load_base = np.array([load_center[0] - VESSEL_X, load_center[1]])
        self.dump_x, self.turn_x = dump_x, turn_x
        self.start, self.index = start, index
        self.reset()

    def reset(self):
        self.yaw, self.tip, self.speed, self.tip_rate, self.yaw_rate = 0.0, 0.0, 0.0, 0.0, 0.0
        self.stage_time, self.cycle_start = 0.0, 0.0
        self.depart_requested = False
        # gates, set by the server each frame (Fleet)
        self.may_depart = True  # no other carrier on the lane
        self.dump_clear = True  # no dozer or other carrier at the dump point
        self.may_leave_queue = True  # the spot is free and the lane to it clear
        self.spot_busy = False  # another carrier is using or entering the loading spot
        self.queue_free = True  # no other carrier at the waiting place (longer hauls)
        self.hidden, self.hide_left = False, 0.0  # off-screen, and the time left on the off-screen road
        self.arrived = 0.0  # when it began to wait off-screen
        self.dozer_wait, self.depart_wait, self.queue_wait = 0.0, 0.0, 0.0  # s held at each gate
        self.payload_kg = 0.0
        self.residual_kg, self.load_kg = 0.0, 0.0  # soil still in the vessel while dumping; the load taken
        self.delivered_kg, self.loads = 0.0, 0
        self.time = 0.0
        self.trips = []  # per trip: payload and the time spent in each stage away from the loading spot
        self.trip = None
        self.counts = None  # the carried load (server): particle counts per vessel cell, while away
        self.creep_from = None  # where the carrier stood when it began to dump
        self.carry_counted = 0
        if self.start == "road":
            self.base = np.array(QUEUE[:2], dtype=float)
            self.yaw = QUEUE[2]
            self.hidden = True
            self.stage = "queue on road"
            self.plan = [("emerge", *QUEUE, "queue_free", "queue on road"), *self._from_queue()]
        elif self.start == "queue":
            self.base = np.array(QUEUE[:2], dtype=float)
            self.yaw = QUEUE[2]
            self.stage = "wait to load"
            self.plan = self._from_queue()
        else:
            self.base = self.load_base.copy()
            self.stage = "loading"
            self.plan = []

    # ------------------------------------------------------------------ queries
    @property
    def parked(self):
        return self.stage == "loading"

    def vessel_pose(self):
        """Vessel body origin (middle of the floor), rotation, linear and angular velocity (zone frame)."""
        R = rot(self.yaw, self.tip)
        Rz = rot(self.yaw, 0.0)
        pin = np.array([*self.base, self.z]) + Rz @ PIN
        p = pin + R @ FLOOR_FROM_PIN
        w = np.array([0.0, 0.0, self.yaw_rate]) + Rz @ np.array([0.0, -self.tip_rate, 0.0])
        heading = np.array([math.cos(self.yaw), math.sin(self.yaw), 0.0])
        v_pin = self.speed * heading + np.cross([0.0, 0.0, self.yaw_rate], pin - np.array([*self.base, self.z]))
        v = v_pin + np.cross(w, p - pin)
        return p, R, v, w

    @property
    def z(self):
        """base_link height: on the ground, or out of sight below it while off-screen."""
        return self.ground_z - (HIDE_DEPTH if self.hidden else 0.0)

    def vessel_quat(self):
        return quat_mul(yaw_quat(self.yaw), np.array([0.0, math.sin(-self.tip / 2), 0.0, math.cos(-self.tip / 2)]))

    # ------------------------------------------------------------------ routes
    # moves: ("drive", x, y, direction, speed, stage) along the heading to the point nearest (x, y);
    # ("face", x, y, stage) spin to head for (x, y); ("spin", yaw, stage); ("gate", flag, stage) hold until
    # the flag is set; ("tip",), ("dump",), ("lower",); ("choose",) pick the way back once at RETURN_X
    def _trip(self):
        if self.haul_m is not None:
            y = self.base[1]
            return [("drive", HIDE_X, y, +1, SPEED_LOADED, "drive out"), ("hide", self.haul_m, SPEED_LOADED, "haul out"),
                    ("emerge", EMERGE_X, y, math.pi, "dump_clear", "wait at dump"),
                    ("drive", self.dump_x, y, -1, SPEED_REVERSE, "reverse to dump"), ("tip",), ("dump",), ("lower",),
                    ("drive", EMERGE_X, y, +1, SPEED_EMPTY, "return"), ("hide", self.haul_m, SPEED_EMPTY, "haul back"),
                    ("emerge", *QUEUE, "queue_free", "queue on road"), *self._from_queue()]
        # it waits short of its turn: spinning round there sweeps the dozer's way between the heap and its park
        return [("drive", WAIT_X, self.base[1], +1, SPEED_LOADED, "drive out"), ("gate", "dump_clear", "wait at dump"),
                ("drive", self.turn_x, self.base[1], +1, SPEED_LOADED, "drive out"), ("spin", math.pi, "turn to dump"),
                ("drive", self.dump_x, self.base[1], -1, SPEED_REVERSE, "reverse to dump"),
                ("tip",), ("dump",), ("lower",), ("drive", RETURN_X, self.load_base[1], +1, SPEED_EMPTY, "return"), ("choose",)]

    def _into_spot(self, stage):
        return [("spin", 0.0, stage), ("drive", *self.load_base, -1, SPEED_REVERSE, "reverse in")]

    def _from_queue(self):
        return [("gate", "may_leave_queue", "wait to load"), ("drive", QUEUE[0], self.load_base[1], +1, SPEED_EMPTY, "from queue"),
                *self._into_spot("from queue")]

    def _to_queue(self):
        return [("face", QUEUE[0], QUEUE[1], "to queue"), ("drive", QUEUE[0], QUEUE[1], +1, SPEED_EMPTY, "to queue"),
                ("spin", QUEUE[2], "to queue"), *self._from_queue()]

    # ------------------------------------------------------------------ motion
    def _drive(self, tx, ty, direction, v_max, dt):
        """Along the heading, forward or in reverse, to the point nearest (tx, ty); True when there."""
        heading = np.array([math.cos(self.yaw), math.sin(self.yaw)])
        dist = float(np.dot(np.array([tx, ty]) - self.base, heading)) * direction
        v_stop = math.sqrt(max(2 * ACCEL * max(dist, 0.0), 0.0))
        target = min(v_max, v_stop)
        s = abs(self.speed)
        s = min(target, s + ACCEL * dt) if s < target else max(target, s - ACCEL * dt)
        self.speed = s * direction
        self.base += self.speed * heading * dt
        if dist <= 0.01 or (s < 0.02 and dist < 0.05):
            self.speed = 0.0
            return True
        return False

    def _spin_to(self, yaw_target, dt):
        err = math.atan2(math.sin(yaw_target - self.yaw), math.cos(yaw_target - self.yaw))
        step = math.copysign(min(abs(err), SPIN_RATE * dt), err)
        self.yaw_rate = step / dt
        self.yaw += step
        if abs(err) < 1e-4:
            self.yaw, self.yaw_rate = yaw_target, 0.0
            return True
        return False

    def update(self, dt):
        """Advance one frame. Returns an event when the soil hand-off is due: "depart" (take the load out
        of the loading spot's zone before moving), or None."""
        self.time += dt
        self.stage_time += dt
        self.yaw_rate, self.tip_rate = 0.0, 0.0
        event = None
        if self.stage == "loading":
            if self.depart_requested and self.may_depart:
                self.depart_requested = False
                self.trip = {"depart_t": round(self.time, 2), "stages": {}}
                self.plan = self._trip()
                event = "depart"
            elif self.depart_requested:
                self.depart_wait += dt  # full, but another carrier is on the lane
        if self.plan:
            self._do(self.plan[0], dt)
        return event

    def _do(self, move, dt):
        kind, done = move[0], False
        if kind == "drive":
            _, tx, ty, d, v, stage = move
            self._next(stage)
            done = self._drive(tx, ty, d, v, dt)
        elif kind == "spin":
            self._next(move[2])
            done = self._spin_to(move[1], dt)
        elif kind == "face":
            self._next(move[3])
            done = self._spin_to(math.atan2(move[2] - self.base[1], move[1] - self.base[0]), dt)
        elif kind == "gate":
            _, flag, stage = move
            if getattr(self, flag):
                done = True
            else:
                self._next(stage)
                if flag == "dump_clear":
                    self.dozer_wait += dt
                else:
                    self.queue_wait += dt
        elif kind == "hide":
            _, dist, v, stage = move
            if self.stage != stage:
                # the road at speed v, accelerating from rest and braking to it at ACCEL
                self.hide_left = dist / v + v / ACCEL if dist > 0.0 else 0.0
                self.hidden, self.speed = True, 0.0
            self._next(stage)
            self.hide_left -= dt
            done = self.hide_left <= 0.0
        elif kind == "emerge":
            _, x, y, yaw, flag, stage = move
            if getattr(self, flag):
                self.base[:] = (x, y)
                self.yaw, self.hidden = yaw, False
                done = True
            else:
                if self.stage != stage:
                    self.arrived = self.time  # in line from here (Fleet.gates: first come, first served)
                self._next(stage)
                if flag == "dump_clear":
                    self.dozer_wait += dt
                else:
                    self.queue_wait += dt
        elif kind == "tip":
            self._next("tip")
            self.tip_rate = TIP_RATE
            self.tip = min(TIP_MAX, self.tip + TIP_RATE * dt)
            if self.tip >= TIP_MAX:
                self.tip_rate = 0.0
                done = True
        elif kind == "dump":
            if self.stage != "dump":
                self.creep_from = self.base.copy()
            self._next("dump")
            finished = self.payload_kg <= 0.0 and self.residual_kg <= CARRY_BACK * self.load_kg
            crept = float(np.linalg.norm(self.base - self.creep_from))
            if self.stage_time > DUMP_HOLD and not finished and crept < CREEP_MAX:
                heading = np.array([math.cos(self.yaw), math.sin(self.yaw)])
                self.speed = CREEP_SPEED
                self.base += CREEP_SPEED * heading * dt
            else:
                self.speed = 0.0
            if self.stage_time > DUMP_HOLD and self.payload_kg <= 0.0 and (finished or self.stage_time > DUMP_MAX):
                self.tip, self.tip_rate = TIP_MAX, 0.0
                done = True
            elif self.stage_time > DUMP_HOLD and self.payload_kg <= 0.0:
                ph = 2 * math.pi * (self.stage_time - DUMP_HOLD) / SHAKE_PERIOD
                a = math.radians(SHAKE_DEG) / 2
                self.tip = TIP_MAX - a * (1 - math.cos(ph))
                self.tip_rate = -a * math.sin(ph) * 2 * math.pi / SHAKE_PERIOD
        elif kind == "lower":
            self._next("lower")
            self.tip_rate = -LOWER_RATE
            self.tip = max(0.0, self.tip - LOWER_RATE * dt)
            if self.tip <= 0.0:
                self.tip_rate = 0.0
                done = True
        elif kind == "choose":
            self.plan.pop(0)
            self.plan[:0] = self._to_queue() if self.spot_busy else self._into_spot("turn to load")
            return
        if done:
            self.plan.pop(0)
            if not self.plan and self.stage == "reverse in":
                self.base[:] = self.load_base
                self._next("loading")

    def _next(self, stage):
        if stage == self.stage:
            return
        if self.trip is not None:
            st = self.trip["stages"]
            st[self.stage] = round(st.get(self.stage, 0.0) + self.stage_time, 2)
            if stage == "loading":
                self.trip["away_s"] = round(self.time - self.trip["depart_t"], 2)
                self.trips.append(self.trip)
                self.trip = None
        self.stage, self.stage_time = stage, 0.0


class Fleet:
    """The carriers and the traffic rules between them (see the module docstring)."""

    def __init__(self, n, load_center, ground_z, dump_x, turn_x, capacity_kg, haul_m=0.0):
        """haul_m: the off-screen road's length each way; more than two carriers need it (they queue on it)."""
        self.long = haul_m > 0.0 or n > 2
        h = haul_m if self.long else None
        self.carriers = [Dumper(load_center, ground_z, dump_x, turn_x, capacity_kg, start=("spot", "queue")[i] if i < 2 else "road", index=i, haul_m=h)
                         for i in range(n)]

    def __iter__(self):
        return iter(self.carriers)

    def __len__(self):
        return len(self.carriers)

    def reset(self):
        for c in self.carriers:
            c.reset()

    @property
    def at_spot(self):
        """The carrier parked at the loading spot, or None."""
        return next((c for c in self.carriers if c.parked), None)

    def gates(self, dozer_parked):
        if self.long:
            # one carrier at a time on each side of the view: the spot side (driving out, or from the waiting
            # place into the spot), the dump side, and the waiting place
            for c in self.carriers:
                others = [o for o in self.carriers if o is not c]
                c.may_depart = not any(o.stage in ("from queue", "reverse in") for o in others)
                c.dump_clear = dozer_parked and not any(o.stage in AT_DUMP or o.stage == "wait at dump" and (o.arrived, o.index) < (c.arrived, c.index) for o in others)
                c.may_leave_queue = not any(o.stage in AT_SPOT or o.stage == "drive out" for o in others)
                c.queue_free = not any(o.stage in ("wait to load", "from queue") or o.stage == "queue on road" and (o.arrived, o.index) < (c.arrived, c.index)
                                       for o in others)  # first come, first served
            return
        for c in self.carriers:
            others = [o for o in self.carriers if o is not c]
            c.may_depart = all(o.stage == "wait to load" for o in others)
            c.dump_clear = dozer_parked and not any(o.stage in AT_DUMP for o in others)
            c.spot_busy = any(o.stage in AT_SPOT or o.stage == "wait to load" for o in others)
            # the spot is empty and the carrier that left it has turned (it is backing into the dump point,
            # clear of this one's turn into the spot by the time it gets there)
            c.may_leave_queue = not any(o.stage in AT_SPOT or o.stage in LEAVING for o in others)

    def dump_area_free(self):
        """The dozer may work at the dump point: every carrier is at the loading spot or waiting to load,
        or still close to the spot on its way out."""
        if self.long:
            # nor one waiting off-screen to come to it, or due there within DOZER_CLEAR_S: the dozer backs
            # out and parks in time (spreading as soon as a carrier had left held the next one up to 60 s)
            return not any(c.stage in AT_DUMP or c.stage == "wait at dump" or c.stage == "haul out" and c.hide_left < DOZER_CLEAR_S
                           for c in self.carriers)
        return all(c.stage in ("loading", "reverse in", "wait to load", "from queue") or (c.stage == "drive out" and c.base[0] < c.turn_x - 6.0)
                   or c.stage in ("to queue",) and c.base[1] > 8.0 for c in self.carriers)

    @property
    def delivered_kg(self):
        return sum(c.delivered_kg for c in self.carriers)

    @property
    def trips(self):
        return sorted((t for c in self.carriers for t in c.trips), key=lambda t: t["depart_t"])
