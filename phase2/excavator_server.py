"""Phase 2b: a joint-driven excavator arm digging Newton MPM sand, streamed to the browser.

Replaces the kinematic bucket of soil_server.py with a Hitachi ZX200 (20 t class) standing at the pit
edge: swing, boom, stick and bucket with the geometry, joint frames and limits of the PWRI OPERA URDF and
a curved bucket with teeth traced from its mesh. Each joint is driven by a hydraulic-style actuator:

  * flow limit     - the operator lever sets a joint speed; a reference angle moves at that speed
  * stiff servo    - PD torque pulls the joint to the reference (cylinders are nearly rigid)
  * relief valve   - torque is capped at the relief limit; the reference can't run more than
                     tau_max / Kp ahead of the joint, so a joint that meets more resistance than its
                     relief limit slows down and stalls instead of pushing through

The arm is a Newton articulation solved with MuJoCo; its bucket is a dynamic MPM collider, so the
sand pushes back on the arm (two-way coupling as in Newton's mpm_twoway_coupling example).

Streams the same heightmap/grain data as soil_server.py plus the three link poses, joint angles,
speeds and relief-pressure ratios, and accepts lever / auto / relief / reset commands.

Usage:
    python excavator_server.py                 # ws://127.0.0.1:8765
    python excavator_server.py --bench 40      # headless auto digging
    python excavator_server.py --stall-test    # push the bucket deep into the sand and check it stalls
"""

import argparse
import asyncio
import json
import math
import struct
import time

import numpy as np
import warp as wp

import newton
from newton.solvers import SolverImplicitMPM, SolverMuJoCo

from soil_server import (
    BED_DEPTH,
    G,
    HM_CELL,
    HM_NX,
    HM_NY,
    MAX_SEND_PARTICLES,
    MOVING_SPEED,
    PIT_CENTER_THREE,
    SURFACE_MAX_SPEED,
    WALL_HEIGHT,
    ZONE_HALF,
    heightmap_kernel,
    mckyes_n_gamma,
)

# ------------------------------------------------------------------ machine (zone frame, Z up)
# Hitachi ZX200 (20 t class) from the PWRI OPERA URDF (zx200_ros2, Apache-2.0): joint origins in the
# parent link frame and the fixed pitch of each child frame. The physics links use the URDF link frames
# (body_link, boom_link, arm_link, bucket_link), so the joint angles are the URDF ones and the browser
# places the real ZX200 meshes (assets/machinery/opera_zx200.glb) directly at the streamed link poses.
# URDF frames have the arm along +x; the upper structure is turned 180 deg so the arm faces the pit (-x).
GROUND_Z = 2.35  # site ground next to the pit, zone frame
SWING_POS = np.array([7.4, 0.0, GROUND_Z + 1.1206])  # swing joint, 1.12 m above the track bottom
CRAWLER_SWING = np.array([0.10787, -0.034301, 1.1206])  # swing joint in the crawler frame
CRAWLER_YAW = math.pi / 2  # tracks parallel to the pit edge (display only)
BOOM_ORIGIN, BOOM_PITCH = np.array([0.05467, -0.040187, 0.72492]), math.radians(32.0357)
STICK_ORIGIN, STICK_PITCH = np.array([4.83998, 0.010957, 3.00227]), math.radians(16.9707)
BUCKET_ORIGIN, BUCKET_PITCH = np.array([1.91546, -0.023546, 2.17374]), math.radians(-9.20083)
L_BOOM, L_STICK = float(np.hypot(STICK_ORIGIN[0], STICK_ORIGIN[2])), float(np.hypot(BUCKET_ORIGIN[0], BUCKET_ORIGIN[2]))  # 5.69, 2.90 m pin to pin

# Bucket (bucket_link frame, x-z plane): inner shell profile traced from the OPERA bucket mesh, from the
# top of the back (near the pin) round the bowl to the cutting lip; the mouth is the chord between the
# ends. Inside width 1.06 m: 0.79 m3 struck, 0.91 m3 heaped (SAE J296). Five teeth reach 0.18 m past the
# lip. Plates are 10 cm thick (outside the profile) so the 8 cm MPM grid can't leak sand through them.
BUCKET_SHELL = np.array([
    (0.02, 0.15), (-0.09, 0.18), (-0.15, 0.26), (-0.18, 0.46), (-0.24, 0.66), (-0.25, 0.77), (-0.23, 0.89),
    (-0.17, 0.99), (-0.08, 1.06), (0.03, 1.11), (0.14, 1.13), (0.25, 1.12), (0.35, 1.08), (0.52, 1.01),
    (0.68, 0.94), (0.88, 0.86), (1.075, 0.78),
])
BUCKET_W, PLATE = 1.06, 0.10
TOOTH_Y, TOOTH_TIP = (-0.45, -0.22, 0.0, 0.22, 0.45), np.array([1.25, 0.83])
BUCKET_EDGE = np.array([TOOTH_TIP[0], 0.0, TOOTH_TIP[1]])  # cutting edge (tooth tips), bucket frame
_MOUTH = BUCKET_SHELL[-1] - BUCKET_SHELL[0]
MOUTH_ANGLE = math.atan2(_MOUTH[1], _MOUTH[0])  # chord direction in the bucket frame
# rating: Hitachi ZX200-6 standard bucket, 0.80 m3 ISO heaped, in sand at 1600 kg/m3
BUCKET_RATED_KG = 0.80 * 1600.0

# masses: ZX200-6 operating weight ~20 t; upper structure incl. 4.6 t counterweight, boom, arm and a
# 0.8 m3 bucket estimated from published class figures. Centres of mass and inertias (link frames) are
# from the area-weighted shells of the OPERA meshes, scaled to these masses (plate structures).
MASS = {"house": 9800.0, "boom": 1450.0, "stick": 680.0, "bucket": 640.0}
COM = {"house": (-0.48, 0.17, 0.79), "boom": (2.00, 0.01, 1.95), "stick": (0.34, -0.01, 0.73), "bucket": (0.29, -0.01, 0.72)}
INERTIA = {"house": (10518.0, 17319.0, 20667.0), "boom": (1028.0, 3635.0, 2661.0), "stick": (304.0, 673.0, 383.0), "bucket": (131.0, 159.0, 183.0)}
JOINTS = ["swing", "boom", "stick", "bucket"]
LINKS = ["house", "boom", "stick", "bucket"]
GLB_NODES = {"house": "body_link", "boom": "boom_link", "stick": "arm_link", "bucket": "bucket_link"}
# limits (rad, from the URDF), max speed (rad/s), relief torque (N m), servo gains. In URDF angles a
# positive boom angle lowers the boom, positive stick folds it in and positive bucket curls, so the
# operator levers (raise / out / dump positive) drive those three joints with the opposite sign.
LIMITS = {"swing": (-3.0, 3.0), "boom": (-1.137, 1.0), "stick": (0.594, 2.687), "bucket": (-0.622, 2.372)}
LEVER_SIGN = np.array([1.0, -1.0, -1.0, -1.0])
VMAX = {"swing": 0.8, "boom": 0.35, "stick": 0.5, "bucket": 0.9}
TAU_MAX = {"swing": 65e3, "boom": 450e3, "stick": 160e3, "bucket": 120e3}
KP = {"swing": 2.0e6, "boom": 3.0e7, "stick": 8.0e6, "bucket": 1.5e6}
KD = {"swing": 8.0e5, "boom": 3.0e6, "stick": 6.0e5, "bucket": 4.0e4}

# dump truck parked on the machine's right (zone +y), bed long axis along x
BED_CENTER = np.array([7.0, 5.5])
BED_L, BED_W, BED_H, BED_T = 4.2, 2.3, 1.0, 0.08  # inside length, width, side height, plate thickness
BED_FLOOR_Z = 4.1
BED_LO = np.array([BED_CENTER[0] - BED_L / 2, BED_CENTER[1] - BED_W / 2, BED_FLOOR_Z])
BED_HI = np.array([BED_CENTER[0] + BED_L / 2, BED_CENTER[1] + BED_W / 2, BED_FLOOR_Z + BED_H + 1.0])
TRUCK_CAPACITY_KG = 15000.0
TRUCK_CELL = 0.1
TRUCK_NX = int(round(BED_L / TRUCK_CELL))
TRUCK_NY = int(round(BED_W / TRUCK_CELL))
ARMATURE = 200.0
RIGID_SUBSTEPS = 4
USE_MUJOCO_CPU = False  # GPU MuJoCo, with the rigid substeps recorded as one CUDA graph


def box_inertia(m, hx, hy, hz):
    return wp.mat33(m * (hy * hy + hz * hz) / 3, 0, 0, 0, m * (hx * hx + hz * hz) / 3, 0, 0, 0, m * (hx * hx + hy * hy) / 3)


def rot_y(a):
    return wp.quat_from_axis_angle(wp.vec3(0.0, 1.0, 0.0), float(a))


def rot_z(a):
    return wp.quat_from_axis_angle(wp.vec3(0.0, 0.0, 1.0), float(a))


def swing_vec(psi, v):
    """Rotate a vector about +z by the swing angle."""
    c, s = math.cos(psi), math.sin(psi)
    return np.array([v[0] * c - v[1] * s, v[0] * s + v[1] * c, v[2]])


def pitch(v, b):
    """Rotate v about +y by b (URDF sense: positive turns +x toward -z)."""
    c, s = math.cos(b), math.sin(b)
    return np.array([v[0] * c + v[2] * s, v[1], -v[0] * s + v[2] * c])


def house_to_zone(psi, v):
    """Point in the upper-structure (URDF body_link) frame -> zone frame. The house is turned 180 deg,
    so at swing 0 the arm points along -x; positive swing turns it toward -y (the operator's left)."""
    return SWING_POS + swing_vec(psi, np.array([-v[0], -v[1], v[2]]))


def chain(q):
    """House-frame stick pin, bucket pin and the bucket frame's total pitch for q = [swing, boom, stick, bucket]."""
    b1 = BOOM_PITCH + q[1]
    p1 = BOOM_ORIGIN + pitch(STICK_ORIGIN, b1)
    b2 = b1 + STICK_PITCH + q[2]
    p2 = p1 + pitch(BUCKET_ORIGIN, b2)
    return p1, p2, b2 + BUCKET_PITCH + q[3]


def bucket_point(q, v):
    """Bucket-frame point (x, y, z) -> zone frame."""
    _, p2, b3 = chain(q)
    return house_to_zone(q[0], p2 + pitch(v, b3))


def fk(q):
    """Stick pin, bucket pin and cutting edge (tooth tips) in the zone frame."""
    p1, p2, b3 = chain(q)
    return house_to_zone(q[0], p1), house_to_zone(q[0], p2), house_to_zone(q[0], p2 + pitch(BUCKET_EDGE, b3))


def link_xforms(q):
    """World transforms of the house, boom, stick and bucket link frames."""
    p1, p2, b3 = chain(q)
    b1 = BOOM_PITCH + q[1]
    b2 = b1 + STICK_PITCH + q[2]
    yaw = rot_z(math.pi + q[0])
    return [
        wp.transform(wp.vec3(*SWING_POS), yaw),
        wp.transform(wp.vec3(*house_to_zone(q[0], BOOM_ORIGIN)), yaw * rot_y(b1)),
        wp.transform(wp.vec3(*house_to_zone(q[0], p1)), yaw * rot_y(b2)),
        wp.transform(wp.vec3(*house_to_zone(q[0], p2)), yaw * rot_y(b3)),
    ]


def ik(psi, r, z, phi):
    """Joint angles [swing, boom, stick, bucket] that put the bucket pin at radius r (from the swing axis,
    along the arm) and height z, with the bucket mouth (back-to-lip chord) pointing at angle phi above
    the outward horizontal; phi = -90 deg holds the mouth vertical with the lip down, as when digging."""
    dr, dz = r - BOOM_ORIGIN[0], z - SWING_POS[2] - BOOM_ORIGIN[2]
    d = min(math.hypot(dr, dz), L_BOOM + L_STICK - 1e-3)
    g_boom = math.atan2(STICK_ORIGIN[2], STICK_ORIGIN[0])  # pin-to-pin direction in each link frame
    g_stick = math.atan2(BUCKET_ORIGIN[2], BUCKET_ORIGIN[0])
    delta = -math.acos(max(-1.0, min(1.0, (d * d - L_BOOM**2 - L_STICK**2) / (2 * L_BOOM * L_STICK))))  # stick below the boom line
    a1 = math.atan2(dz, dr) - math.atan2(L_STICK * math.sin(delta), L_BOOM + L_STICK * math.cos(delta))  # boom elevation
    b1 = g_boom - a1
    b2 = g_stick - (a1 + delta)
    b3 = MOUTH_ANGLE - phi
    return np.array([psi, b1 - BOOM_PITCH, b2 - b1 - STICK_PITCH, b3 - b2 - BUCKET_PITCH])


def bucket_plates():
    """Collider boxes (centre, half extents, pitch) in the bucket frame: shell plates along the profile,
    side plates (strips parallel to the mouth, so none reaches past it) and teeth."""
    boxes = []
    centroid = BUCKET_SHELL.mean(axis=0)
    hw = BUCKET_W / 2
    for a, b in zip(BUCKET_SHELL[:-1], BUCKET_SHELL[1:]):
        d = b - a
        n = np.array([-d[1], d[0]]) / np.linalg.norm(d)
        mid = (a + b) / 2
        if np.dot(mid - centroid, n) < 0:
            n = -n
        c = mid + n * PLATE / 2
        boxes.append(((c[0], 0.0, c[1]), (np.linalg.norm(d) / 2 + PLATE / 2, hw + PLATE, PLATE / 2), math.atan2(-d[1], d[0])))
    # sides: strips across the bowl in the mouth's (u, v) frame, from the mouth plane (v = 0) to the shell
    eu = _MOUTH / np.linalg.norm(_MOUTH)
    ev = np.array([-eu[1], eu[0]])
    if np.dot(centroid - BUCKET_SHELL[0], ev) < 0:
        ev = -ev
    dense = np.concatenate([np.linspace(a, b, 12) for a, b in zip(BUCKET_SHELL[:-1], BUCKET_SHELL[1:])])
    uv = np.stack([(dense - BUCKET_SHELL[0]) @ eu, (dense - BUCKET_SHELL[0]) @ ev], axis=1)
    strip = 0.12
    for u0 in np.arange(uv[:, 0].min(), uv[:, 0].max(), strip):
        sel = (uv[:, 0] >= u0 - 0.02) & (uv[:, 0] <= u0 + strip + 0.02)
        if not sel.any():
            continue
        vmax = uv[sel, 1].max() + PLATE
        c = BUCKET_SHELL[0] + eu * (u0 + strip / 2) + ev * (vmax / 2)
        for side in (-1, 1):
            boxes.append(((c[0], side * (hw + PLATE / 2), c[1]), (strip / 2, PLATE / 2, vmax / 2), math.atan2(-eu[1], eu[0])))
    lip = BUCKET_SHELL[-1]
    t = TOOTH_TIP - lip
    c = (lip + TOOTH_TIP) / 2
    for y in TOOTH_Y:
        boxes.append(((c[0], y, c[1]), (np.linalg.norm(t) / 2 + 0.03, 0.04, 0.04), math.atan2(-t[1], t[0])))
    return boxes


def _bucket_outline():
    """Bucket-frame points that bound the bucket (outer shell and tooth tips), for clearance checks."""
    centroid = BUCKET_SHELL.mean(axis=0)
    pts = [p + (p - centroid) / np.linalg.norm(p - centroid) * PLATE for p in BUCKET_SHELL]
    return [np.array([p[0], 0.0, p[1]]) for p in pts] + [BUCKET_EDGE]


BUCKET_OUTLINE = _bucket_outline()

# ------------------------------------------------------------------ kernels
@wp.kernel
def hydraulics_kernel(
    joint_q: wp.array[float],
    joint_qd: wp.array[float],
    v_cmd: wp.array[float],
    q_ref: wp.array[float],
    kp: wp.array[float],
    kd: wp.array[float],
    tau_max: wp.array[float],
    q_lo: wp.array[float],
    q_hi: wp.array[float],
    relief: float,
    dt: float,
    joint_f: wp.array[float],
    pressure: wp.array[float],
):
    j = wp.tid()
    q = joint_q[j]
    tmax = tau_max[j] * relief
    # flow: the reference advances at lever speed, within the joint limits ...
    r = wp.clamp(q_ref[j] + v_cmd[j] * dt, q_lo[j], q_hi[j])
    # ... but can't get further ahead of the joint than the relief torque can pull (stall, no windup)
    lag = tmax / kp[j]
    r = wp.clamp(r, q - lag, q + lag)
    # the reference never leaves the stroke, even when load back-drives the joint past it
    r = wp.clamp(r, q_lo[j], q_hi[j])
    q_ref[j] = r
    tau = wp.clamp(kp[j] * (r - q) + kd[j] * (v_cmd[j] - joint_qd[j]), -tmax, tmax)
    pressure[j] = wp.abs(tau) / tau_max[j]
    # cylinder end stop: beyond the stroke a stiff mechanical stop pushes back, not limited by the relief valve
    over = wp.max(q - q_hi[j], 0.0) - wp.max(q_lo[j] - q, 0.0)
    if over != 0.0:
        tau = tau - 10.0 * kp[j] * over - kd[j] * joint_qd[j]
    joint_f[j] = tau


@wp.kernel
def compute_body_forces(
    dt: float,
    collider_ids: wp.array[int],
    collider_impulses: wp.array[wp.vec3],
    collider_impulse_pos: wp.array[wp.vec3],
    body_ids: wp.array[int],
    body_q: wp.array[wp.transform],
    body_com: wp.array[wp.vec3],
    body_f: wp.array[wp.spatial_vector],
):
    """Sand impulses on collider nodes -> force and torque at each body's centre of mass."""
    i = wp.tid()
    cid = collider_ids[i]
    if cid >= 0 and cid < body_ids.shape[0]:
        b = body_ids[cid]
        if b == -1:
            return
        f = collider_impulses[i] / dt
        r = collider_impulse_pos[i] - wp.transform_point(body_q[b], body_com[b])
        wp.atomic_add(body_f, b, wp.spatial_vector(f, wp.cross(r, f)))


@wp.kernel
def subtract_body_force(
    dt: float,
    body_q: wp.array[wp.transform],
    body_qd: wp.array[wp.spatial_vector],
    body_f: wp.array[wp.spatial_vector],
    body_inv_inertia: wp.array[wp.mat33],
    body_inv_mass: wp.array[float],
    body_q_res: wp.array[wp.transform],
    body_qd_res: wp.array[wp.spatial_vector],
):
    """Velocity the bodies would have without last frame's sand force (input to the MPM collider solve)."""
    b = wp.tid()
    f = body_f[b]
    dv = dt * body_inv_mass[b] * wp.spatial_top(f)
    rot = wp.transform_get_rotation(body_q[b])
    dw = dt * wp.quat_rotate(rot, body_inv_inertia[b] * wp.quat_rotate_inv(rot, wp.spatial_bottom(f)))
    body_q_res[b] = body_q[b]
    body_qd_res[b] = body_qd[b] - wp.spatial_vector(dv, dw)


@wp.kernel
def stream_kernel(
    q: wp.array[wp.vec3],
    qd: wp.array[wp.vec3],
    min_speed: float,
    zone_hx: float,
    zone_hy: float,
    bed_lo: wp.vec3,
    bed_hi: wp.vec3,
    max_out: int,
    counter: wp.array[int],
    out: wp.array[wp.vec3],
):
    """Grains to stream: moving sand, plus sand resting outside the pit and outside the truck bed
    (spill on the ground). Resting sand in the pit and in the bed is sent as heightmaps instead."""
    i = wp.tid()
    p = q[i]
    in_zone = wp.abs(p[0]) <= zone_hx and wp.abs(p[1]) <= zone_hy
    in_bed = p[0] > bed_lo[0] and p[0] < bed_hi[0] and p[1] > bed_lo[1] and p[1] < bed_hi[1] and p[2] > bed_lo[2] - 0.1
    if wp.length(qd[i]) > min_speed or (not in_zone and not in_bed):
        slot = wp.atomic_add(counter, 0, 1)
        if slot < max_out:
            out[slot] = p


@wp.kernel
def bucket_fill_kernel(
    q: wp.array[wp.vec3],
    xf: wp.transform,
    lo: wp.vec3,
    hi: wp.vec3,
    counter: wp.array[int],
):
    i = wp.tid()
    p = wp.transform_point(wp.transform_inverse(xf), q[i])
    if p[0] > lo[0] and p[0] < hi[0] and p[1] > lo[1] and p[1] < hi[1] and p[2] > lo[2] and p[2] < hi[2]:
        wp.atomic_add(counter, 0, 1)


@wp.kernel
def bucket_profile_fill_kernel(
    q: wp.array[wp.vec3],
    xf: wp.transform,
    poly: wp.array[wp.vec2],
    half_w: float,
    counter: wp.array[int],
):
    """Grains inside the bucket: within the inner width and inside the profile polygon (x-z, closed by the mouth chord)."""
    i = wp.tid()
    p = wp.transform_point(wp.transform_inverse(xf), q[i])
    if wp.abs(p[1]) > half_w:
        return
    n = poly.shape[0]
    crossings = int(0)
    for k in range(n):
        a = poly[k]
        b = poly[(k + 1) % n]
        if (a[1] > p[2]) != (b[1] > p[2]):
            x = a[0] + (p[2] - a[1]) * (b[0] - a[0]) / (b[1] - a[1])
            if p[0] < x:
                crossings += 1
    if crossings % 2 == 1:
        wp.atomic_add(counter, 0, 1)


# ------------------------------------------------------------------ simulation
class ExcavatorSim:
    # auto cycle: (stage, swing deg, bucket-pin radius from the swing axis, pin height, bucket angle deg
    # [, speed: fraction of full lever for moving into this stage])
    CYCLE = [
        ("approach", 0, 6.0, 2.6, -95),
        ("penetrate", 0, 7.6, 1.5, -60),  # near full reach of the ZX200 arm
        ("drag", 0, 5.6, 1.1, -100),
        ("drag-curl", 0, 4.7, 1.2, -145),  # keep dragging while curling: the 1.2 m mouth fills toward the top
        ("curl", 0, 4.6, 1.6, -180),
        ("lift", 0, 5.0, 6.6, -165),
        ("swing to truck", -90, 5.4, 6.6, -165),
        ("dump", -90, 5.4, 6.4, -40),
        ("raise", -90, 5.4, 7.0, -110),  # clear the truck sides before swinging back
        ("swing back", 0, 6.0, 6.6, -95),
        ("lower", 0, 6.0, 2.6, -95),
    ]

    def __init__(self, voxel=0.08, ppc=2, density=1600.0, friction=0.75, iterations=60, fps=60.0):
        wp.init()
        self.frame_dt = 1.0 / fps
        self.sim_dt = self.frame_dt / RIGID_SUBSTEPS
        self.voxel = voxel
        self.density = density
        self.relief = 1.0

        b = newton.ModelBuilder()
        no_contact = newton.ModelBuilder.ShapeConfig(density=0.0, mu=0.4, has_shape_collision=False, has_particle_collision=False)
        bucket_cfg = newton.ModelBuilder.ShapeConfig(density=0.0, mu=0.4, has_shape_collision=True, has_particle_collision=True)
        stick_cfg = newton.ModelBuilder.ShapeConfig(density=0.0, mu=0.4, has_shape_collision=True, has_particle_collision=True)
        wall = newton.ModelBuilder.ShapeConfig(mu=0.6, has_shape_collision=False)  # holds sand only
        floor = newton.ModelBuilder.ShapeConfig(mu=0.6)  # hard ground under the sand: the bucket can hit it

        q0 = ik(0.0, 6.0, 2.6, math.radians(-95))
        self.q0 = q0
        xf = link_xforms(q0)

        def link(name, x):
            I = INERTIA[name]
            return b.add_link(xform=x, mass=MASS[name], com=wp.vec3(*COM[name]), inertia=wp.mat33(I[0], 0, 0, 0, I[1], 0, 0, 0, I[2]))

        def along(v, hy, hz):
            """Box from a link's origin to the next pin v (link frame): (centre, half extents, pitch)."""
            return ((v[0] / 2, 0.0, v[2] / 2), (math.hypot(v[0], v[2]) / 2, hy, hz), -math.atan2(v[2], v[0]))

        def add_boxes(body, boxes, cfg):
            for c, h, a in boxes:
                b.add_shape_box(body, xform=wp.transform(wp.vec3(*c), rot_y(a)), hx=h[0], hy=h[1], hz=h[2], cfg=cfg)

        # upper structure (house, counterweight, cab) in the URDF body frame (arm along +x); no contacts.
        # These boxes (and the boom one) are only drawn when the browser can't load the ZX200 model.
        house = link("house", xf[0])
        self.house_boxes = [((-0.6, 0.0, 0.55), (1.8, 1.3, 0.55), 0.0), ((-2.3, 0.0, 0.6), (0.55, 1.3, 0.5), 0.0), ((0.6, 0.75, 1.35), (0.55, 0.5, 0.75), 0.0)]
        add_boxes(house, self.house_boxes, no_contact)
        boom = link("boom", xf[1])
        self.boom_boxes = [along(STICK_ORIGIN, 0.3, 0.35)]
        add_boxes(boom, self.boom_boxes, no_contact)
        stick = link("stick", xf[2])
        self.stick_boxes = [along(BUCKET_ORIGIN, 0.2, 0.22)]
        add_boxes(stick, self.stick_boxes, stick_cfg)
        bucket = link("bucket", xf[3])
        self.bucket_boxes = bucket_plates()
        add_boxes(bucket, self.bucket_boxes, bucket_cfg)

        joint_kw = dict(axis=(0.0, 1.0, 0.0), armature=ARMATURE)
        j0 = b.add_joint_revolute(-1, house, parent_xform=wp.transform(wp.vec3(*SWING_POS), rot_z(math.pi)), axis=(0.0, 0.0, 1.0), armature=ARMATURE, limit_lower=LIMITS["swing"][0], limit_upper=LIMITS["swing"][1])
        j1 = b.add_joint_revolute(house, boom, parent_xform=wp.transform(wp.vec3(*BOOM_ORIGIN), rot_y(BOOM_PITCH)), limit_lower=LIMITS["boom"][0], limit_upper=LIMITS["boom"][1], **joint_kw)
        j2 = b.add_joint_revolute(boom, stick, parent_xform=wp.transform(wp.vec3(*STICK_ORIGIN), rot_y(STICK_PITCH)), limit_lower=LIMITS["stick"][0], limit_upper=LIMITS["stick"][1], **joint_kw)
        j3 = b.add_joint_revolute(stick, bucket, parent_xform=wp.transform(wp.vec3(*BUCKET_ORIGIN), rot_y(BUCKET_PITCH)), limit_lower=LIMITS["bucket"][0], limit_upper=LIMITS["bucket"][1], **joint_kw)
        b.add_articulation([j0, j1, j2, j3])
        for k in range(4):
            b.joint_q[k] = float(q0[k])
            b.joint_target_mode[k] = int(newton.JointTargetMode.EFFORT)
        self.links = {"house": house, "boom": boom, "stick": stick, "bucket": bucket}

        # dump truck: the bed holds sand (MPM collider) and can be hit by the bucket (rigid contact)
        truck_cfg = newton.ModelBuilder.ShapeConfig(mu=0.5)
        cx, cy = BED_CENTER
        L, Wb, Hb, Tb = BED_L, BED_W, BED_H, BED_T
        self.truck_bed_boxes = [
            ((cx, cy, BED_FLOOR_Z - Tb / 2), (L / 2 + Tb, Wb / 2 + Tb, Tb / 2)),  # floor
            ((cx, cy - Wb / 2 - Tb / 2, BED_FLOOR_Z + Hb / 2), (L / 2 + Tb, Tb / 2, Hb / 2)),  # sides
            ((cx, cy + Wb / 2 + Tb / 2, BED_FLOOR_Z + Hb / 2), (L / 2 + Tb, Tb / 2, Hb / 2)),
            ((cx - L / 2 - Tb / 2, cy, BED_FLOOR_Z + Hb / 2), (Tb / 2, Wb / 2, Hb / 2)),  # tailgate
            ((cx + L / 2 + Tb / 2, cy, BED_FLOOR_Z + (Hb + 0.5) / 2), (Tb / 2, Wb / 2, (Hb + 0.5) / 2)),  # headboard
        ]
        for c, h in self.truck_bed_boxes:
            b.add_shape_box(-1, xform=wp.transform(wp.vec3(*c)), hx=h[0], hy=h[1], hz=h[2], cfg=truck_cfg)
        # site ground and pit slopes where spilled sand can land (the pit floor plane is 2.35 m lower).
        # Pit: 6 x 5 m floor, sides 1V:1.5H up to ground level at |x| = 6.75, |y| = 6.25 (as in the site terrain).
        ground = newton.ModelBuilder.ShapeConfig(mu=0.6)
        b.add_shape_box(-1, xform=wp.transform(wp.vec3(10.0, 1.0, GROUND_Z / 2)), hx=3.25, hy=9.0, hz=GROUND_Z / 2, cfg=ground)  # behind the machine
        b.add_shape_box(-1, xform=wp.transform(wp.vec3(0.0, 8.1, GROUND_Z / 2)), hx=6.75, hy=1.85, hz=GROUND_Z / 2, cfg=ground)  # beside the pit, truck side
        slope = newton.ModelBuilder.ShapeConfig(mu=0.6, has_shape_collision=False)  # catches sand only
        tilt = math.atan(1 / 1.5)
        half_len = math.hypot(3.75, 2.5) / 2
        n = 0.5  # half thickness below the slope surface
        b.add_shape_box(-1, xform=wp.transform(wp.vec3(4.875 + n * math.sin(tilt), 0.0, 1.25 - n * math.cos(tilt)), rot_y(-tilt)), hx=half_len, hy=6.25, hz=n, cfg=slope)  # +x side
        b.add_shape_box(-1, xform=wp.transform(wp.vec3(0.0, 4.375 + n * math.sin(tilt), 1.25 - n * math.cos(tilt)), wp.quat_from_axis_angle(wp.vec3(1.0, 0.0, 0.0), tilt)), hx=6.75, hy=half_len, hz=n, cfg=slope)  # +y side

        # static pit walls and floor (MPM colliders only)
        zx, zy = ZONE_HALF
        t = 0.2
        # vertical walls hold the sand on the -x and -y sides; on the +x (machine) and +y (truck) sides the
        # sloped pit sides added below hold it, as in a real pit
        for px, py, hx, hy in [(-zx - t, 0, t, zy + 2 * t), (0, -zy - t, zx, t)]:
            b.add_shape_box(-1, xform=wp.transform(wp.vec3(px, py, WALL_HEIGHT / 2)), hx=hx, hy=hy, hz=WALL_HEIGHT / 2, cfg=wall)
        b.add_ground_plane(cfg=floor)
        self.model = b.finalize()

        # sand
        sb = newton.ModelBuilder()
        SolverImplicitMPM.register_custom_attributes(sb)
        lo = np.array([-zx, -zy, 0.0])
        hi = np.array([zx, zy, BED_DEPTH])
        res = np.array(np.ceil(ppc * (hi - lo) / voxel), dtype=int)
        cell = (hi - lo) / res
        self.radius = float(np.max(cell) * 0.5)
        self.particle_mass = float(np.prod(cell) * density)
        sb.add_particle_grid(
            pos=wp.vec3(lo), rot=wp.quat_identity(), vel=wp.vec3(0.0),
            dim_x=int(res[0]) + 1, dim_y=int(res[1]) + 1, dim_z=int(res[2]) + 1,
            cell_x=float(cell[0]), cell_y=float(cell[1]), cell_z=float(cell[2]),
            mass=self.particle_mass, jitter=2.0 * self.radius, radius_mean=self.radius,
            custom_attributes={"mpm:friction": friction},
        )
        self.sand_model = sb.finalize()
        self.n_particles = self.sand_model.particle_count

        cfg = SolverImplicitMPM.Config()
        cfg.voxel_size = voxel
        cfg.max_iterations = iterations
        cfg.critical_fraction = 0.0
        cfg.strain_basis = "P0"
        self.mpm = SolverImplicitMPM(self.sand_model, config=cfg)
        self.mpm.setup_collider(model=self.model)
        self.collider_body = self.mpm.collider_body_index

        self.solver = SolverMuJoCo(self.model, use_mujoco_contacts=False, njmax=200, nconmax=200, use_mujoco_cpu=USE_MUJOCO_CPU)
        # rigid contacts: bucket and stick against the hard pit floor (the sand acts through MPM)
        self.collision = newton.CollisionPipeline(self.model)
        self.contacts = self.collision.contacts()
        self.n_gamma = mckyes_n_gamma(math.atan(friction), math.atan(0.4))

        # actuator parameters on the GPU
        arr = lambda d: wp.array([float(d[j]) for j in JOINTS], dtype=float)  # noqa: E731
        self.kp, self.kd, self.tau_max = arr(KP), arr(KD), arr(TAU_MAX)
        self.q_lo = wp.array([LIMITS[j][0] for j in JOINTS], dtype=float)
        self.q_hi = wp.array([LIMITS[j][1] for j in JOINTS], dtype=float)
        self.vmax = np.array([VMAX[j] for j in JOINTS])
        self.v_cmd = wp.zeros(4, dtype=float)
        self.q_ref = wp.zeros(4, dtype=float)
        self.pressure = wp.zeros(4, dtype=float)
        self.truck_hm = wp.zeros(TRUCK_NX * TRUCK_NY, dtype=float)
        self.bucket_poly = wp.array(BUCKET_SHELL.astype(np.float32), dtype=wp.vec2)
        self.truck_counter = wp.zeros(1, dtype=int)

        self.hm = wp.zeros(HM_NX * HM_NY, dtype=float)
        self.moving = wp.zeros(MAX_SEND_PARTICLES, dtype=wp.vec3)
        self.counter = wp.zeros(1, dtype=int)
        self.fill_counter = wp.zeros(1, dtype=int)
        max_nodes = 1 << 20
        self.imp = wp.zeros(max_nodes, dtype=wp.vec3)
        self.imp_pos = wp.zeros(max_nodes, dtype=wp.vec3)
        self.imp_ids = wp.full(max_nodes, value=-1, dtype=int)
        self.reset()

    # -------------------------------------------------------------- state
    def reset(self):
        self.state_0 = self.model.state()
        self.state_1 = self.model.state()
        self.control = self.model.control()
        newton.eval_fk(self.model, self.model.joint_q, self.model.joint_qd, self.state_0)
        self.sand = self.sand_model.state()
        self.sand.body_q = wp.empty_like(self.state_0.body_q)
        self.sand.body_qd = wp.empty_like(self.state_0.body_qd)
        self.sand.body_f = wp.empty_like(self.state_0.body_f)
        self.sand_forces = wp.zeros_like(self.state_0.body_f)
        self.imp_ids.fill_(-1)
        self.rigid_graph = None  # new state buffers: record the rigid graph again
        self.q_ref.assign(self.q0.astype(np.float32))
        self.levers = np.zeros(4)
        self.auto = False
        self.auto_stage = 0
        self.auto_s = 0.0
        self.passes = 0
        self.skipped = []
        self._z_cache = {}
        self.stage_time = 0.0
        self.auto_wait = 0.0
        self.stage_name = "manual"
        self.sim_time = 0.0
        self.step_ms = 0.0
        self.q = self.q0.copy()
        self.qd = np.zeros(4)
        self.press = np.zeros(4)
        self.truck_kg = 0.0
        self.truck_hm_np = np.full(TRUCK_NX * TRUCK_NY, BED_FLOOR_Z, dtype=np.float32)
        self.force = np.zeros(3)
        self.fill_kg = 0.0
        self.cut_depth = 0.0
        self.fee_ref = 0.0
        self.hm_np = np.zeros(HM_NX * HM_NY, dtype=np.float32)
        self._collect_impulses()

    def _collect_impulses(self):
        imp, pos, ids = self.mpm.collect_collider_impulses(self.sand)
        self.imp_ids.fill_(-1)
        n = min(imp.shape[0], self.imp.shape[0])
        self.imp[:n].assign(imp[:n])
        self.imp_pos[:n].assign(pos[:n])
        self.imp_ids[:n].assign(ids[:n])

    # -------------------------------------------------------------- operator
    def set_levers(self, swing=None, boom=None, stick=None, bucket=None):
        self.auto = False
        self.stage_name = "manual"
        for k, v in enumerate((swing, boom, stick, bucket)):
            if v is not None:
                self.levers[k] = float(np.clip(v, -1.0, 1.0))

    DIG_STAGES = ("penetrate", "drag", "drag-curl", "curl")
    STAGE_TIMEOUT = 4.0  # s of poor tracking before the auto cycle skips a stage
    DEEPEN_PER_PASS, MAX_DEEPEN = 0.15, 0.45

    FLOOR_CLEARANCE = 0.08  # m: deepened dig paths keep the bucket this far above the hard pit floor

    @staticmethod
    def _bucket_lowest(w_from, z_from, w_to, z_to, steps=9):
        """Lowest corner of the bucket's outer box along the straight pin path between two waypoints."""
        low = 1e9
        for s in np.linspace(0.0, 1.0, steps):
            psi = w_from[1] + s * (w_to[1] - w_from[1])
            r = w_from[2] + s * (w_to[2] - w_from[2])
            z = z_from + s * (z_to - z_from)
            phi = w_from[4] + s * (w_to[4] - w_from[4])
            q = ik(math.radians(psi), r, z, math.radians(phi))
            low = min(low, min(bucket_point(q, v)[2] for v in BUCKET_OUTLINE))
        return low

    def _stage_z(self, wp_):
        """Waypoint pin height. Dig stages go a little deeper on each pass, like an operator would,
        but never so deep that the bucket would run into the hard floor on the way there."""
        if wp_[0] not in self.DIG_STAGES:
            return wp_[3]
        key = (wp_[0], self.passes)
        if key not in self._z_cache:
            i = next(k for k, w in enumerate(self.CYCLE) if w[0] == wp_[0])
            prev = self.CYCLE[i - 1]
            z_prev = self._stage_z(prev)
            level = self.passes // len(self.LANES)  # deepen only after every lane has been dug
            z = wp_[3] - min(self.DEEPEN_PER_PASS * level, self.MAX_DEEPEN)
            while self._bucket_lowest(prev, z_prev, wp_, z) < self.FLOOR_CLEARANCE:
                z += 0.01
            self._z_cache[key] = z
        return self._z_cache[key]

    # successive passes dig side-by-side lanes (swing offsets, deg) before cutting deeper, like an
    # operator working across the face; at 6-8 m reach 9 deg is about one bucket width
    LANES = (0.0, 9.0, -9.0, 17.0, -17.0)
    LANE_STAGES = ("approach", "penetrate", "drag", "drag-curl", "curl", "lift")

    def _stage_psi(self, wp_):
        """Waypoint swing angle (deg): dig-side stages shift to the current pass's lane; the final
        'lower' stage already heads for the next pass's lane."""
        if wp_[0] in self.LANE_STAGES:
            return wp_[1] + self.LANES[self.passes % len(self.LANES)]
        if wp_[0] == "lower":
            return wp_[1] + self.LANES[(self.passes + 1) % len(self.LANES)]
        return wp_[1]

    def _waypoint_q(self, wp_):
        r, phi = wp_[2], wp_[4]
        return ik(math.radians(self._stage_psi(wp_)), r, self._stage_z(wp_), math.radians(phi))

    def _auto_control(self):
        """Track the cycle: move a target along the current segment only while the arm keeps up."""
        n = len(self.CYCLE)
        a = self.CYCLE[self.auto_stage]
        b = self.CYCLE[(self.auto_stage + 1) % n]
        self.stage_name = b[0]
        q_end = self._waypoint_q(b)
        speed = b[5] if len(b) > 5 else 1.0
        seg_time = max(np.max(np.abs(self._waypoint_q(a) - q_end) / (self.vmax * speed)), 0.5)
        s = self.auto_s
        za, zb = self._stage_z(a), self._stage_z(b)
        pa, pb = self._stage_psi(a), self._stage_psi(b)
        mix = [pa + s * (pb - pa), a[2] + s * (b[2] - a[2]), za + s * (zb - za), a[4] + s * (b[4] - a[4])]
        q_goal = ik(math.radians(mix[0]), mix[1], mix[2], math.radians(mix[3]))  # straight bucket-pin path
        err = q_goal - self.q
        self.levers = np.clip(4.0 * err / self.vmax, -speed, speed) * LEVER_SIGN
        # advance along the segment at the pace the slowest joint allows, but only if tracking is good
        if np.max(np.abs(err)) < 0.12:
            self.auto_s = min(1.0, s + self.frame_dt / seg_time)
            self.auto_wait = 0.0
        else:
            self.auto_wait += self.frame_dt
        # a stalled arm (e.g. bucket on the hard floor) must not hang the cycle: give up on the stage
        # after 4 s of poor tracking, or when the stage takes far longer than planned (slow creep)
        self.stage_time += self.frame_dt
        timed_out = self.auto_wait > self.STAGE_TIMEOUT or self.stage_time > 3.0 * seg_time + self.STAGE_TIMEOUT
        if timed_out:
            self.skipped.append({"t": round(self.sim_time, 2), "stage": b[0], "max_err_rad": round(float(np.max(np.abs(err))), 3)})
            self.auto_wait = 0.0
        if timed_out or (self.auto_s >= 1.0 and np.max(np.abs(q_end - self.q)) < 0.06):
            self.auto_stage = (self.auto_stage + 1) % n
            self.auto_s = 0.0
            self.stage_time = 0.0
            if self.auto_stage == 0:
                self.passes += 1

    # -------------------------------------------------------------- stepping
    def _rigid_substeps(self):
        # even substep count: state_0/state_1 end where they started, so a captured graph stays valid
        for _ in range(RIGID_SUBSTEPS):
            self.state_0.clear_forces()
            wp.launch(compute_body_forces, dim=self.imp_ids.shape[0], inputs=[self.frame_dt, self.imp_ids, self.imp, self.imp_pos, self.collider_body, self.state_0.body_q, self.model.body_com, self.state_0.body_f])
            self.sand_forces.assign(self.state_0.body_f)
            wp.launch(
                hydraulics_kernel,
                dim=4,
                inputs=[self.state_0.joint_q, self.state_0.joint_qd, self.v_cmd, self.q_ref, self.kp, self.kd, self.tau_max, self.q_lo, self.q_hi, self.relief, self.sim_dt],
                outputs=[self.control.joint_f, self.pressure],
            )
            self.collision.collide(self.state_0, self.contacts)
            self.solver.step(self.state_0, self.state_1, self.control, self.contacts, self.sim_dt)
            self.state_0, self.state_1 = self.state_1, self.state_0

    def step(self):
        if self.auto:
            self._auto_control()
        self.v_cmd.assign((self.levers * LEVER_SIGN * self.vmax).astype(np.float32))

        wp.synchronize()
        t0 = time.perf_counter()
        if self.rigid_graph is None:
            self._rigid_substeps()  # first run compiles kernels; then record the substeps as a graph
            if wp.get_device().is_cuda and not USE_MUJOCO_CPU:
                with wp.ScopedCapture() as cap:
                    self._rigid_substeps()
                self.rigid_graph = cap.graph
        else:
            wp.capture_launch(self.rigid_graph)

        wp.launch(subtract_body_force, dim=self.model.body_count, inputs=[self.frame_dt, self.state_0.body_q, self.state_0.body_qd, self.sand_forces, self.model.body_inv_inertia, self.model.body_inv_mass, self.sand.body_q, self.sand.body_qd])
        self.mpm.step(self.sand, self.sand, contacts=None, control=None, dt=self.frame_dt)
        self._collect_impulses()
        wp.synchronize()
        ms = (time.perf_counter() - t0) * 1000
        self.step_ms = ms if self.sim_time == 0 else 0.9 * self.step_ms + 0.1 * ms
        self.sim_time += self.frame_dt

        self.q = self.state_0.joint_q.numpy()[:4].astype(float)
        self.qd = self.state_0.joint_qd.numpy()[:4].astype(float)
        self.press = self.pressure.numpy().astype(float)

    # -------------------------------------------------------------- outputs
    def bucket_xform(self):
        bq = self.state_0.body_q.numpy()[self.links["bucket"]]
        return wp.transform(wp.vec3(*bq[:3]), wp.quat(*bq[3:]))

    def measure(self):
        sf = self.sand_forces.numpy()[self.links["bucket"]]
        self.force = sf[:3].astype(float)  # sand force on the bucket (world, zone frame)
        q = self.sand.particle_q
        self.hm.fill_(0.0)
        wp.launch(heightmap_kernel, dim=self.n_particles, inputs=[q, self.sand.particle_qd, SURFACE_MAX_SPEED, self.radius, -ZONE_HALF[0], -ZONE_HALF[1], HM_CELL, HM_NX, HM_NY, self.hm])
        self.counter.zero_()
        wp.launch(stream_kernel, dim=self.n_particles, inputs=[q, self.sand.particle_qd, MOVING_SPEED, ZONE_HALF[0] + 0.6, ZONE_HALF[1] + 0.6, wp.vec3(*BED_LO), wp.vec3(*BED_HI), MAX_SEND_PARTICLES, self.counter, self.moving])
        # truck: load inside the bed, and the surface of the resting load
        self.truck_counter.zero_()
        wp.launch(bucket_fill_kernel, dim=self.n_particles, inputs=[q, wp.transform_identity(), wp.vec3(*BED_LO), wp.vec3(*BED_HI), self.truck_counter])
        self.truck_hm.fill_(BED_FLOOR_Z)
        wp.launch(heightmap_kernel, dim=self.n_particles, inputs=[q, self.sand.particle_qd, SURFACE_MAX_SPEED, self.radius, float(BED_LO[0]), float(BED_LO[1]), TRUCK_CELL, TRUCK_NX, TRUCK_NY, self.truck_hm])
        self.fill_counter.zero_()
        # inside the shell profile, up to the mouth (struck load)
        wp.launch(bucket_profile_fill_kernel, dim=self.n_particles, inputs=[q, self.bucket_xform(), self.bucket_poly, BUCKET_W / 2, self.fill_counter])
        self.hm_np = self.hm.numpy()
        n_moving = min(int(self.counter.numpy()[0]), MAX_SEND_PARTICLES)
        moving = self.moving.numpy()[:n_moving]
        self.fill_kg = float(self.fill_counter.numpy()[0]) * self.particle_mass
        self.truck_kg = float(self.truck_counter.numpy()[0]) * self.particle_mass
        self.truck_hm_np = self.truck_hm.numpy()
        _, _, edge = fk(self.q)
        ix = int((edge[0] + ZONE_HALF[0]) / HM_CELL)
        iy = int((edge[1] + ZONE_HALF[1]) / HM_CELL)
        ix_ahead = max(ix - 3, 0)  # the arm drags toward +x, so uncut ground is at lower x
        surface = float(self.hm_np[iy * HM_NX + ix_ahead]) if 0 <= ix < HM_NX and 0 <= iy < HM_NY else 0.0
        surface = min(surface, BED_DEPTH)
        self.cut_depth = max(surface - edge[2], 0.0)
        self.fee_ref = self.density * G * self.cut_depth**2 * BUCKET_W * self.n_gamma
        return moving

    @staticmethod
    def to_three(p):
        ox, oy, oz = PIT_CENTER_THREE
        return (float(p[0]) + ox, float(p[2]) + oy, oz - float(p[1]))

    @staticmethod
    def quat_to_three(q):
        x, y, z, w = (float(v) for v in q)
        return (x, z, -y, w)

    def geometry(self):
        """Link box geometry in three.js link-local coordinates, sent once in the hello message."""
        def box(c, h, a=0.0):
            # a: pitch about the link's y axis, which is three.js -z
            return {"center": [float(c[0]), float(c[2]), -float(c[1])], "size": [2 * float(h[0]), 2 * float(h[2]), 2 * float(h[1])], "rot_z": -float(a)}

        def world_box(c, h):
            return {"center": list(self.to_three(c)), "size": [2 * h[0], 2 * h[2], 2 * h[1]]}

        cx, cy = BED_CENTER
        g = GROUND_Z
        return {
            "house": [box(*bx) for bx in self.house_boxes],
            "boom": [box(*bx) for bx in self.boom_boxes],
            "stick": [box(*bx) for bx in self.stick_boxes],
            "bucket": [box(*bx) for bx in self.bucket_boxes],
            # the real machine: the browser puts each link's meshes from this glTF at the streamed link pose
            "model": {
                "file": "assets/machinery/opera_zx200.glb",
                "links": GLB_NODES,
                "crawler": {"node": "track_link", "pos": list(self.to_three(SWING_POS - swing_vec(CRAWLER_YAW, CRAWLER_SWING))), "quat": list(self.quat_to_three(rot_z(CRAWLER_YAW)))},
            },
            # static parts in world (three.js) coordinates
            "tracks": [world_box((SWING_POS[0] + side * 1.2, 0, g + 0.45), (0.35, 2.3, 0.45)) for side in (-1, 1)]  # along y, as CRAWLER_YAW
            + [world_box((SWING_POS[0], 0, g + 0.75), (0.9, 1.4, 0.25))],
            "truck_bed": [world_box(c, h) for c, h in self.truck_bed_boxes],
            "truck_body": [
                world_box((cx + 0.9, cy, 3.6), (3.4, 0.5, 0.3)),  # chassis
                world_box((cx + BED_L / 2 + 1.15, cy, g + 1.7), (0.95, 1.2, 1.1)),  # cab
            ],
            "truck_wheels": [
                {"center": list(self.to_three((cx + ax, cy + side * (BED_W / 2 - 0.25), g + 0.55))), "radius": 0.55, "width": 0.45}
                for ax in (-1.3, 0.0, BED_L / 2 + 1.1) for side in (-1, 1)
            ],
            "truck_capacity_kg": TRUCK_CAPACITY_KG,
        }

    def frame_bytes(self, frame_no, moving):
        bq = self.state_0.body_q.numpy()
        links = b"".join(struct.pack("<3f4f", *self.to_three(bq[i][:3]), *self.quat_to_three(bq[i][3:])) for i in (self.links[name] for name in LINKS))
        f = self.force
        header = struct.pack(
            "<4sIfffI",
            b"EXC2", frame_no, self.sim_time, self.step_ms, min(self.frame_dt * 1000 / max(self.step_ms, 1e-3), 99.0), self.n_particles,
        )
        arm = struct.pack(
            "<4f4f4f3f6fB",
            *self.q, *self.qd, *self.press, f[0], f[2], -f[1],
            self.fill_kg, self.cut_depth, self.fee_ref, self.relief, self.truck_kg, TRUCK_CAPACITY_KG,
            self.auto_stage if self.auto else 255,
        )
        ox, oy, oz = PIT_CENTER_THREE
        hm = np.clip((self.hm_np + 1.0) * 1000.0, 0, 65535).astype(np.uint16)
        hm_header = struct.pack("<HHfff", HM_NX, HM_NY, ox - ZONE_HALF[0] + HM_CELL / 2, oz + ZONE_HALF[1] - HM_CELL / 2, HM_CELL)
        thm = np.clip((self.truck_hm_np + 1.0) * 1000.0, 0, 65535).astype(np.uint16)
        thm_header = struct.pack("<HHfff", TRUCK_NX, TRUCK_NY, ox + BED_LO[0] + TRUCK_CELL / 2, oz - BED_LO[1] - TRUCK_CELL / 2, TRUCK_CELL)
        lo = np.array([ox - 4.0, oy - 0.5, oz - 10.0], dtype=np.float32)  # covers the pit, machine and truck
        span = 16.0
        if len(moving):
            t3 = np.stack([moving[:, 0] + ox, moving[:, 2] + oy, oz - moving[:, 1]], axis=1)
            qp = np.clip((t3 - lo) / span * 65535.0, 0, 65535).astype(np.uint16)
        else:
            qp = np.zeros((0, 3), dtype=np.uint16)
        p_header = struct.pack("<I4f", len(qp), *lo, span)
        body = header + links + arm
        body += b"\0" * ((-len(body)) % 4)  # keep the heightmap 4-byte aligned
        return body + hm_header + hm.tobytes() + thm_header + thm.tobytes() + p_header + qp.tobytes()

    def stats(self):
        return {
            "t": round(self.sim_time, 2),
            "stage": self.stage_name,
            "step_ms": round(self.step_ms, 2),
            "q_deg": [round(math.degrees(v), 1) for v in self.q],
            "qd": [round(v, 3) for v in self.qd],
            "pressure": [round(v, 2) for v in self.press],
            "force_kN": [round(v / 1000, 1) for v in self.force],
            "fill_kg": round(self.fill_kg),
            "cut_m": round(self.cut_depth, 3),
            "mckyes_kN": round(self.fee_ref / 1000, 1),
            "edge": [round(v, 2) for v in fk(self.q)[2]],
            "truck_kg": round(self.truck_kg),
        }


# ------------------------------------------------------------------ server
async def serve(sim, host, port):
    import websockets

    clients = set()

    async def handler(ws):
        clients.add(ws)
        await ws.send(json.dumps({"type": "hello", "kind": "excavator", "particles": sim.n_particles, "voxel": sim.voxel, "joints": JOINTS, "limits": LIMITS, "vmax": VMAX, "tau_max": TAU_MAX, "geometry": sim.geometry(), "stages": [c[0] for c in sim.CYCLE]}))
        try:
            async for msg in ws:
                try:
                    cmd = json.loads(msg)
                except ValueError:
                    continue
                kind = cmd.get("type")
                if kind == "levers":
                    sim.set_levers(cmd.get("swing"), cmd.get("boom"), cmd.get("stick"), cmd.get("bucket"))
                elif kind == "auto":
                    sim.auto = bool(cmd.get("on", True))
                    sim.auto_stage, sim.auto_s, sim.passes = 0, 0.0, 0
                    if not sim.auto:
                        sim.set_levers(0, 0, 0, 0)
                elif kind == "relief":
                    sim.relief = float(np.clip(cmd.get("value", 1.0), 0.1, 1.5))
                elif kind == "reset":
                    sim.reset()
        finally:
            clients.discard(ws)

    async def sim_loop():
        frame = 0
        next_send = 0.0
        wall0 = time.perf_counter()
        while True:
            sim.step()
            if sim.sim_time < next_send - 1.0:
                next_send = 0.0
                wall0 = time.perf_counter() - sim.sim_time
            ahead = sim.sim_time - (time.perf_counter() - wall0)
            if ahead > 0:
                await asyncio.sleep(ahead)
            elif ahead < -0.5:
                wall0 = time.perf_counter() - sim.sim_time
            if sim.sim_time >= next_send:
                next_send = sim.sim_time + 1 / 30
                moving = sim.measure()
                if clients:
                    data = sim.frame_bytes(frame, moving)
                    frame += 1
                    for ws in list(clients):
                        try:
                            await ws.send(data)
                        except Exception as err:  # noqa: BLE001
                            print(f"dropping client: {err!r}", flush=True)
                            clients.discard(ws)
                    if frame % 150 == 0:
                        print(json.dumps(sim.stats()), flush=True)
            await asyncio.sleep(0)

    async with websockets.serve(handler, host, port, max_size=2**20, compression=None):
        print(f"excavator server on ws://{host}:{port}  particles={sim.n_particles}", flush=True)
        await sim_loop()


def run_headless(sim, seconds, levers=None, auto=True, every=0.25):
    sim.auto = auto
    if levers is not None:
        sim.set_levers(*levers)
    log = []
    t0 = time.perf_counter()
    next_log = 0.0
    while sim.sim_time < seconds:
        sim.step()
        if sim.sim_time >= next_log:
            next_log += every
            sim.measure()
            log.append(sim.stats())
    return {"sim_s": round(sim.sim_time, 2), "wall_s": round(time.perf_counter() - t0, 2), "realtime_factor": round(sim.sim_time / (time.perf_counter() - t0), 3), "log": log}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=8765)
    ap.add_argument("--voxel", type=float, default=0.08)
    ap.add_argument("--iterations", type=int, default=60, help="MPM solver iterations; 30 lets carried sand creep out of the bucket")
    ap.add_argument("--bench", type=float, default=None)
    ap.add_argument("--hold-test", action="store_true")
    ap.add_argument("--stall-test", action="store_true")
    ap.add_argument("--out", default=None)
    args = ap.parse_args()
    sim = ExcavatorSim(voxel=args.voxel, iterations=args.iterations)
    res = None
    if args.hold_test:
        res = run_headless(sim, 3.0, levers=(0, 0, 0, 0), auto=False)
    elif args.stall_test:
        res = {}
        for relief in (1.0, 0.35):
            sim.reset()
            sim.relief = relief
            # reach out, put the bucket into the sand, then crowd the stick hard toward the machine
            r1 = run_headless(sim, 6.0, levers=None, auto=True)  # approach + start penetrating
            sim.auto = False
            sim.set_levers(0.0, -0.6, 1.0, 0.0)  # boom down, stick out: a deep, hard cut
            t_start = sim.sim_time
            log = []
            while sim.sim_time < t_start + 6.0:
                sim.step()
                if int(sim.sim_time * 60) % 15 == 0:
                    sim.measure()
                    log.append(sim.stats())
            res[f"relief_{relief}"] = {"approach_last": r1["log"][-1], "hard_cut": log}
    elif args.bench:
        res = run_headless(sim, args.bench)
    if res is not None:
        if args.out:
            with open(args.out, "w") as fh:
                json.dump(res, fh, indent=1)
        print(json.dumps(res if not isinstance(res, dict) or "log" not in res else {**{k: v for k, v in res.items() if k != "log"}, "log_head": res["log"][:4], "log_tail": res["log"][-3:]}, indent=1)[:6000])
        return
    asyncio.run(serve(sim, args.host, args.port))


if __name__ == "__main__":
    main()
