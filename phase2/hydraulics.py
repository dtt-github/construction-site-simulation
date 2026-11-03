"""Hitachi ZX200 hydraulic cylinders and bucket linkage.

Data
  * cylinders (Hitachi ZAXIS 200 brochure): boom 2 x bore 120 / rod 85 mm, arm 135 / 95 mm,
    bucket 115 / 80 mm; implement relief 34.3 MPa (36.3 MPa power boost), swing relief 30.4 MPa;
    two main pumps of 194 L/min each
  * swing: 13.3 min^-1 and 68.9 kN m (ZX200-3 and ZX210-6 spec listings). At the 30.4 MPa swing relief that
    torque means 2.27 L/rad at the house, and 13.3 min^-1 then takes 189 L/min, about one pump
  * port (overload) reliefs: 39.2 MPa, typical for this class; not confirmed for the ZX200
  * pin positions: found in the PWRI OPERA ZX200 meshes (zx200_ros2, Apache-2.0) by voting for the
    bore axes of the pin bosses; each mesh is in its URDF link frame (x forward, z up, pins along y).
    Over the URDF joint limits they give cylinder travels of 1.28 m (boom), 1.465 m (arm) and 1.04 m
    (bucket, through the linkage), against published strokes of 1.26, 1.475 and 1.06 m. Where several
    bosses were candidates, the one matching the stroke was taken; for the bucket link pin that choice
    also gives a digging force at the teeth of 149 kN (158 kN with power boost) against the published
    151-158 kN. Arm crowd at the teeth with the bucket in line: 96 kN (102 kN), published 109-114 kN.

Model
  * a cylinder's length L(q) depends only on its own joint; its force F (positive = pushing, head-side
    pressure) gives the joint torque tau = F * dL/dq (virtual work)
  * relief: the head side is limited to p_relief * A_head (pushing), the rod side to p_relief * A_rod
    (pulling), so the torque limits depend on the pose and the direction
  * port reliefs: the pump feeds the side the spool opens, capped by the main relief; the other side
    (or both, with the spool closed) holds an outside load up to the port relief, then gives way
  * flow: a lever opens its spool in proportion; the cylinder speed is flow / area of the side being
    filled, and when the open spools ask for more than the two pumps deliver, all are slowed alike
  * swing: a fixed-displacement motor through the swing reduction; speed = flow / displacement, torque
    capped at the swing relief both ways (cross-over reliefs: they also brake the house in neutral)
  * the bucket cylinder drives the bucket through a four-bar: guide link (arm pin G) and side link
    (bucket pin K) meet the cylinder rod at pin C
"""

import math

import numpy as np

P_RELIEF = 34.3e6  # Pa, implement circuit
P_POWER_BOOST = 36.3e6
P_PORT = 39.2e6  # Pa, port (overload) reliefs on the cylinder lines
P_SWING = 30.4e6  # Pa, swing relief
SWING_TORQUE = 68.9e3  # N m at the house
SWING_SPEED = 13.3 * 2 * math.pi / 60  # rad/s
SWING_DISP = SWING_TORQUE / P_SWING  # m3/rad at the house (motor displacement x reduction)
SWING_FLOW = SWING_SPEED * SWING_DISP  # m3/s at full lever
PUMP_FLOW = 2 * 194.0 / 60000.0  # m3/s, two main pumps
LINKS = ("house", "boom", "stick", "bucket")

# name: joint index, cylinders, bore, rod, max spool flow (m3/s), head pin (link, x, z), rod pin (link, x, z)
CYLINDERS = {
    "boom": (1, 2, 0.120, 0.085, PUMP_FLOW, ("house", 0.686, 0.247), ("boom", 1.776, 2.249)),
    "arm": (2, 1, 0.135, 0.095, PUMP_FLOW, ("boom", 2.276, 2.789), ("stick", -0.778, -0.367)),
    "bucket": (3, 1, 0.115, 0.080, PUMP_FLOW / 2, ("stick", -0.248, 0.733), None),  # rod end: linkage pin C
}
GUIDE_PIN = ("stick", 1.577, 1.878)  # guide link on the arm
SIDE_PIN = ("bucket", -0.328, 0.548)  # side link on the bucket
GUIDE_LEN, SIDE_LEN = 0.561, 0.583

# the meshes (zx200-*-cyl1/2, part1/2): head eye at the mesh origin, axis in the mesh x-z plane,
# pin-to-pin length as modelled; links: the two pin positions in the mesh frame
CYLINDER_MESH = {"boom": ((0.3425, 0.9395), 2.73), "arm": ((0.9486, 0.3165), 1.70), "bucket": ((0.6761, 0.7368), 2.16)}
LINK_MESH = {"side": ((0.0, 0.0), (-0.328, 0.482)), "guide": ((0.0, 0.0), (-0.338, -0.448))}


def area(name):
    _, n, bore, rod, *_ = CYLINDERS[name]
    return n * math.pi * bore**2 / 4, n * math.pi * (bore**2 - rod**2) / 4  # head side, rod side


class Linkage:
    """Cylinder pins and lengths for a pose. frames(q) -> {link: (origin xyz, pitch)} in the house frame."""

    def __init__(self, frames, pitch):
        self.frames, self.pitch = frames, pitch

    def point(self, f, pin):
        link, x, z = pin
        origin, b = f[link]
        return origin + self.pitch(np.array([x, 0.0, z]), b)

    def pins(self, q):
        f = self.frames(q)
        pts = {}
        for name, (_, _, _, _, _, head, rod) in CYLINDERS.items():
            pts[name + "_head"] = self.point(f, head)
            if rod is not None:
                pts[name + "_rod"] = self.point(f, rod)
        g, k = self.point(f, GUIDE_PIN), self.point(f, SIDE_PIN)
        d = np.hypot(k[0] - g[0], k[2] - g[2])
        a = (GUIDE_LEN**2 - SIDE_LEN**2 + d**2) / (2 * d)
        h = math.sqrt(max(GUIDE_LEN**2 - a**2, 0.0))
        ux, uz = (k[0] - g[0]) / d, (k[2] - g[2]) / d
        # the branch on the cylinder's side of the G-K line (perpendicular (-uz, ux) in x-z)
        pts["bucket_rod"] = np.array([g[0] + a * ux - h * uz, g[1], g[2] + a * uz + h * ux])
        pts["guide"], pts["side"] = g, k
        return pts

    def lengths(self, q, pts=None):
        pts = self.pins(q) if pts is None else pts
        return np.array([np.linalg.norm(pts[n + "_rod"] - pts[n + "_head"]) for n in CYLINDERS])

    def state(self, q, eps=1e-4):
        """Pins, lengths L and dL/dq (one per cylinder: each depends only on its own joint)."""
        pts = self.pins(q)
        L = self.lengths(q, pts)
        J = np.zeros(3)
        for k, (name, c) in enumerate(CYLINDERS.items()):
            j = c[0]
            dq = np.zeros(len(q))
            dq[j] = eps
            J[k] = (self.lengths(q + dq)[k] - self.lengths(q - dq)[k]) / (2 * eps)
        return pts, L, J


def torque_limits(J, p_relief, spool=(0.0, 0.0, 0.0), p_port=P_PORT):
    """Joint torque range (lo, hi) the relief valves allow for each cylinder joint: the side the spool
    feeds is capped by the main relief, the other side (both, with the spool closed) by the port relief."""
    lo, hi = np.zeros(3), np.zeros(3)
    p_port = max(p_port, p_relief)
    for k, name in enumerate(CYLINDERS):
        a_head, a_rod = area(name)
        extend, retract = spool[k] * J[k] > 0, spool[k] * J[k] < 0
        push = (p_relief if extend else p_port) * a_head * J[k]
        pull = -(p_relief if retract else p_port) * a_rod * J[k]
        lo[k], hi[k] = min(push, pull), max(push, pull)
    return lo, hi


J_MIN = 0.05  # m/rad: moment arm floor for speed commands near a stroke end (the boom nears toggle there)


def speeds(spool, J):
    """Spool openings [swing, boom, arm, bucket] (-1..1, sign = desired joint direction) -> swing speed
    (rad/s) and cylinder speeds (m/s), with the pumps shared between the open spools."""
    v, flow = np.zeros(3), np.zeros(3)
    for k, (name, c) in enumerate(CYLINDERS.items()):
        a_head, a_rod = area(name)
        extend = spool[k + 1] * J[k] > 0
        a = a_head if extend else a_rod
        flow[k] = abs(spool[k + 1]) * c[4]
        v[k] = math.copysign(flow[k] / a, spool[k + 1] * J[k])
    omega = spool[0] * SWING_SPEED
    total = flow.sum() + abs(spool[0]) * SWING_FLOW
    if total > PUMP_FLOW:
        v *= PUMP_FLOW / total
        omega *= PUMP_FLOW / total
    return omega, v


def cylinder_velocity(spool, J):
    """Cylinder speeds (m/s) for boom, arm and bucket spool openings, the swing closed."""
    return speeds([0.0, *spool], J)[1]


def pressures(tau, J):
    """Joint torques -> cylinder pressure (Pa) on the loaded side, and whether it is the head side."""
    p, head = np.zeros(3), np.zeros(3, dtype=bool)
    for k, name in enumerate(CYLINDERS):
        a_head, a_rod = area(name)
        F = tau[k] / J[k] if abs(J[k]) > 1e-6 else 0.0
        head[k] = F >= 0
        p[k] = F / a_head if F >= 0 else -F / a_rod
    return p, head


def part_frames(pts, axis_y):
    """Pose of each cylinder barrel, rod and linkage plate as (position, 3x3 rotation) in the frame the
    pins are given in, mapping the part's mesh frame onto its pins. axis_y: direction of the pin axes."""
    out = {}

    def frame(p0, p1, local_dir):
        u = p1 - p0
        u /= np.linalg.norm(u)
        w = axis_y - np.dot(axis_y, u) * u
        w /= np.linalg.norm(w)
        d = np.array([local_dir[0], 0.0, local_dir[1]])
        d /= np.linalg.norm(d)
        y = np.array([0.0, 1.0, 0.0])
        world = np.stack([u, w, np.cross(u, w)], axis=1)
        local = np.stack([d, y, np.cross(d, y)], axis=1)
        return world @ local.T

    for name, (axis, l_mesh) in CYLINDER_MESH.items():
        head, rod = pts[name + "_head"], pts[name + "_rod"]
        R = frame(head, rod, axis)
        L = np.linalg.norm(rod - head)
        out[name + "_barrel"] = (head, R)
        out[name + "_rod"] = (head + (rod - head) / L * (L - l_mesh), R)  # the rod slides out of the barrel
    for name, (p0, p1), (a, b) in (("side", LINK_MESH["side"], ("bucket_rod", "side")), ("guide", LINK_MESH["guide"], ("guide", "bucket_rod"))):
        local = np.array(p1) - np.array(p0)
        R = frame(pts[a], pts[b], local)
        out[name + "_link"] = (pts[a] - R @ np.array([p0[0], 0.0, p0[1]]), R)
    return out
