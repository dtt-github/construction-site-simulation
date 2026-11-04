"""Crawler dump carrier (IHI/Kato IC120, the PWRI OPERA model) running a haul cycle: loaded beside the
excavator, it drives to a spoil area, tips its vessel to dump, and comes back.

The carrier is scripted (kinematic): it follows its route at the speeds below and does not feel the
ground or the load. The vessel is a kinematic body in the simulator, so the soil in it and the soil
leaving it are MPM particles like any other (excavator_server.py hands the load over: taken from the
vessel's soil zone when the carrier leaves, released as particles from the tipping vessel at the spoil).

Route (zone frame, cab toward +x at the loading spot; the vessel tips about a pin at its rear):
    loading -> drive out forward (+x) -> spin turn to face -x -> reverse to the dump point -> tip, dump,
    lower -> drive forward (-x) -> spin turn to face +x -> reverse into the loading spot
Spin turns are made away from the excavator (its tracks are 2.5 m from the parked carrier's corner).

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
SPIN_RATE = math.radians(25.0)  # rad/s, counter-rotating tracks
TIP_RATE, LOWER_RATE = math.radians(7.5), math.radians(12.0)  # rad/s (about 8 s up to 60 deg, 5 s down)
DUMP_HOLD = 3.0  # s at full tip, for the soil to leave
# sticky soil stays in a tipped vessel: the operator shakes it (tip oscillating SHAKE_DEG every SHAKE_PERIOD)
# until less than CARRY_BACK of the load is left, for up to DUMP_MAX s; what is left rides back
SHAKE_DEG, SHAKE_PERIOD, CARRY_BACK, DUMP_MAX = 4.0, 0.8, 0.03, 15.0


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
    def __init__(self, load_center, ground_z, dump_x, turn_x, capacity_kg):
        """load_center: vessel centre (x, y) at the loading spot; dump_x: base_link x when dumping (the
        rear, and the soil, further +x); turn_x: where the spin turns are made."""
        self.ground_z, self.capacity_kg = ground_z, capacity_kg
        self.load_base = np.array([load_center[0] - VESSEL_X, load_center[1]])
        self.dump_x, self.turn_x = dump_x, turn_x
        self.reset()

    def reset(self):
        self.base = self.load_base.copy()
        self.yaw, self.tip, self.speed, self.tip_rate, self.yaw_rate = 0.0, 0.0, 0.0, 0.0, 0.0
        self.stage, self.stage_time, self.cycle_start = "loading", 0.0, 0.0
        self.depart_requested = False
        self.payload_kg = 0.0
        self.residual_kg, self.load_kg = 0.0, 0.0  # soil still in the vessel while dumping; the load taken
        self.delivered_kg, self.loads = 0.0, 0
        self.time = 0.0
        self.trips = []  # per trip: payload and the time spent in each stage away from the loading spot
        self.trip = None

    # ------------------------------------------------------------------ queries
    @property
    def parked(self):
        return self.stage == "loading"

    def vessel_pose(self):
        """Vessel body origin (middle of the floor), rotation, linear and angular velocity (zone frame)."""
        R = rot(self.yaw, self.tip)
        Rz = rot(self.yaw, 0.0)
        pin = np.array([*self.base, self.ground_z]) + Rz @ PIN
        p = pin + R @ FLOOR_FROM_PIN
        w = np.array([0.0, 0.0, self.yaw_rate]) + Rz @ np.array([0.0, -self.tip_rate, 0.0])
        heading = np.array([math.cos(self.yaw), math.sin(self.yaw), 0.0])
        v_pin = self.speed * heading + np.cross([0.0, 0.0, self.yaw_rate], pin - np.array([*self.base, self.ground_z]))
        v = v_pin + np.cross(w, p - pin)
        return p, R, v, w

    def vessel_quat(self):
        return quat_mul(yaw_quat(self.yaw), np.array([0.0, math.sin(-self.tip / 2), 0.0, math.cos(-self.tip / 2)]))

    # ------------------------------------------------------------------ motion
    def _drive_to(self, x_target, v_max, direction, dt):
        """Straight along x (the route's axis) to x_target, forward or in reverse; True when there."""
        heading = math.cos(self.yaw) * direction  # +1: base moves +x
        dist = (x_target - self.base[0]) * heading
        v_stop = math.sqrt(max(2 * ACCEL * max(dist, 0.0), 0.0))
        target = min(v_max, v_stop)
        s = abs(self.speed)
        s = min(target, s + ACCEL * dt) if s < target else max(target, s - ACCEL * dt)
        self.speed = s * direction
        self.base[0] += self.speed * math.cos(self.yaw) * dt
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
        st, event = self.stage, None
        if st == "loading":
            if self.depart_requested:
                self.depart_requested = False
                self.trip = {"depart_t": round(self.time, 2), "stages": {}}
                self._next("drive out")
                event = "depart"
        elif st == "drive out":
            if self._drive_to(self.turn_x, SPEED_LOADED, +1, dt):
                self._next("turn to dump")
        elif st == "turn to dump":
            if self._spin_to(math.pi, dt):
                self._next("reverse to dump")
        elif st == "reverse to dump":
            if self._drive_to(self.dump_x, SPEED_REVERSE, -1, dt):
                self._next("tip")
        elif st == "tip":
            self.tip_rate = TIP_RATE
            self.tip = min(TIP_MAX, self.tip + TIP_RATE * dt)
            if self.tip >= TIP_MAX:
                self.tip_rate = 0.0
                self._next("dump")
        elif st == "dump":
            done = self.payload_kg <= 0.0 and self.residual_kg <= CARRY_BACK * self.load_kg
            if self.stage_time > DUMP_HOLD and self.payload_kg <= 0.0 and (done or self.stage_time > DUMP_MAX):
                self.tip, self.tip_rate = TIP_MAX, 0.0
                self._next("lower")
            elif self.stage_time > DUMP_HOLD and self.payload_kg <= 0.0:
                ph = 2 * math.pi * (self.stage_time - DUMP_HOLD) / SHAKE_PERIOD
                a = math.radians(SHAKE_DEG) / 2
                self.tip = TIP_MAX - a * (1 - math.cos(ph))
                self.tip_rate = -a * math.sin(ph) * 2 * math.pi / SHAKE_PERIOD
        elif st == "lower":
            self.tip_rate = -LOWER_RATE
            self.tip = max(0.0, self.tip - LOWER_RATE * dt)
            if self.tip <= 0.0:
                self.tip_rate = 0.0
                self._next("return")
        elif st == "return":
            if self._drive_to(self.turn_x, SPEED_EMPTY, +1, dt):
                self._next("turn to load")
        elif st == "turn to load":
            if self._spin_to(0.0, dt):
                self._next("reverse in")
        elif st == "reverse in":
            if self._drive_to(self.load_base[0], SPEED_REVERSE, -1, dt):
                self.base[:] = self.load_base
                self._next("loading")
        return event

    def _next(self, stage):
        if self.trip is not None:
            self.trip["stages"][self.stage] = round(self.stage_time, 2)
            if stage == "loading":
                self.trip["away_s"] = round(self.time - self.trip["depart_t"], 2)
                self.trips.append(self.trip)
                self.trip = None
        self.stage, self.stage_time = stage, 0.0
