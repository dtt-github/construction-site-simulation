"""Phase 2b: a joint-driven excavator arm digging Newton MPM sand, streamed to the browser.

Replaces the kinematic bucket of soil_server.py with a Hitachi ZX200 (20 t class) standing at the pit
edge: swing, boom, stick and bucket with the geometry, joint frames and limits of the PWRI OPERA URDF and
a curved bucket with teeth traced from its mesh. Boom, stick and bucket are driven by the ZX200's
hydraulic cylinders (hydraulics.py: real bores, pins from the OPERA meshes, bucket four-bar linkage):

  * flow           - a lever opens its spool; the cylinder speed is flow / piston area of the side
                     being filled (the two pumps are shared between open spools), and the joint speed
                     follows from the cylinder's moment arm dL/dq at the current pose
  * stiff servo    - PD torque pulls the joint to a reference that moves at that speed (cylinders are
                     nearly rigid)
  * relief valve   - the torque is capped at relief pressure x piston area x moment arm, separately
                     for pushing (head side) and pulling (rod side); the reference can't run further
                     ahead than that torque can pull, so a joint that meets more resistance slows down
                     and stalls instead of pushing through. Swing is a joint-space motor with a fixed
                     torque limit.

The arm is a Newton articulation solved with MuJoCo; its bucket is a dynamic MPM collider, so the
sand pushes back on the arm (two-way coupling as in Newton's mpm_twoway_coupling example).

Only the soil near the bucket is simulated as MPM particles (soil_zone.py): resting soil elsewhere in the
pit and in the truck is kept as dormant columns that act as a rigid heightfield collider, and wakes into
particles when the bucket comes near. This runs the dig-swing-load cycle in real time.

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
    G,
    HM_CELL,
    MAX_SEND_PARTICLES,
    MOVING_SPEED,
    PIT_CENTER_THREE,
    SURFACE_MAX_SPEED,
    ZONE_HALF,
    heightmap_kernel,
)
import hydraulics as hyd
import soils
import dozer as dozer_mod
import dumper as dumper_mod
from dozer import Dozer
from dumper import Fleet
from soil_zone import CELL, LAYER_PER_PARTICLE, PER_LAYER, SPACING, SoilZones

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
HEAP_DEPTH = 0.5  # m beyond the mouth chord that the auto operator counts as heaped in the bucket


def _heaped_profile():
    """Shell profile closed by the mouth chord pushed HEAP_DEPTH outward: the bucket and the heap on it."""
    a, b = BUCKET_SHELL[0], BUCKET_SHELL[-1]
    n = np.array([b[1] - a[1], a[0] - b[0]]) / np.linalg.norm(b - a)
    if np.dot(n, BUCKET_SHELL.mean(axis=0) - a) > 0:
        n = -n
    return np.vstack([BUCKET_SHELL, b + HEAP_DEPTH * n, a + HEAP_DEPTH * n])
TOOTH_Y, TOOTH_TIP = (-0.45, -0.22, 0.0, 0.22, 0.45), np.array([1.25, 0.83])
BUCKET_EDGE = np.array([TOOTH_TIP[0], 0.0, TOOTH_TIP[1]])  # cutting edge (tooth tips), bucket frame
_MOUTH = BUCKET_SHELL[-1] - BUCKET_SHELL[0]
MOUTH_ANGLE = math.atan2(_MOUTH[1], _MOUTH[0])  # chord direction in the bucket frame
# rating: Hitachi ZX200-6 standard bucket, 0.80 m3 ISO heaped (1280 kg in sand at 1600 kg/m3)
BUCKET_RATED_M3 = 0.80
BUCKET_RATED_KG = BUCKET_RATED_M3 * 1600.0

# masses: ZX200-6 operating weight ~20 t; upper structure incl. 4.6 t counterweight, boom, arm and a
# 0.8 m3 bucket estimated from published class figures. Centres of mass and inertias (link frames) are
# from the area-weighted shells of the OPERA meshes, scaled to these masses (plate structures).
MASS = {"house": 9800.0, "boom": 1450.0, "stick": 680.0, "bucket": 640.0}
COM = {"house": (-0.48, 0.17, 0.79), "boom": (2.00, 0.01, 1.95), "stick": (0.34, -0.01, 0.73), "bucket": (0.29, -0.01, 0.72)}
INERTIA = {"house": (10518.0, 17319.0, 20667.0), "boom": (1028.0, 3635.0, 2661.0), "stick": (304.0, 673.0, 383.0), "bucket": (131.0, 159.0, 183.0)}
JOINTS = ["swing", "boom", "stick", "bucket"]
LINKS = ["house", "boom", "stick", "bucket"]
GLB_NODES = {"house": "body_link", "boom": "boom_link", "stick": "arm_link", "bucket": "bucket_link"}
PARTS = ("boom_barrel", "boom_rod", "arm_barrel", "arm_rod", "bucket_barrel", "bucket_rod", "side_link", "guide_link")  # GLB nodes placed by pose
# limits (rad, from the URDF), max speed (rad/s), relief torque (N m), servo gains. In URDF angles a
# positive boom angle lowers the boom, positive stick folds it in and positive bucket curls, so the
# operator levers (raise / out / dump positive) drive those three joints with the opposite sign.
LIMITS = {"swing": (-3.0, 3.0), "boom": (-1.137, 1.0), "stick": (0.594, 2.687), "bucket": (-0.622, 2.372)}
LEVER_SIGN = np.array([1.0, -1.0, -1.0, -1.0])
# swing: a joint-space actuator (swing motor); boom, stick and bucket are driven by their cylinders
# (hydraulics.py), whose flow and relief set the joint speed and torque limits for each pose
SWING_VMAX, SWING_TAU = hyd.SWING_SPEED, hyd.SWING_TORQUE  # 13.3 min^-1, 68.9 kN m (hydraulics.py)
KP = {"swing": 2.0e6, "boom": 3.0e7, "stick": 8.0e6, "bucket": 1.5e6}
KD = {"swing": 8.0e5, "boom": 3.0e6, "stick": 6.0e5, "bucket": 4.0e4}

# crawler dump carrier (IHI/Kato IC120, the PWRI OPERA model) parked beside the machine (zone +y, east of
# the pit edge on flat ground), vessel long axis along x. Vessel from the model's mesh: 4.3 m long and
# 2.6 m wide outside, floor about 1.3 m above the ground; 11 t maximum payload (spec sheet)
BED_CENTER = np.array([8.6, 5.2])
BED_L, BED_W, BED_H, BED_T = 3.8, 2.35, 0.8, 0.08  # inside length, width, side height (0.73-0.84 m on the mesh), plate thickness
BED_FLOOR_Z = GROUND_Z + 1.3
BED_LO = np.array([BED_CENTER[0] - BED_L / 2, BED_CENTER[1] - BED_W / 2, BED_FLOOR_Z])
BED_HI = np.array([BED_CENTER[0] + BED_L / 2, BED_CENTER[1] + BED_W / 2, BED_FLOOR_Z + BED_H + 1.0])
TRUCK_CAPACITY_KG = 11000.0
TRUCK_CELL = 0.1
TRUCK_NX = int(round(BED_L / TRUCK_CELL))
TRUCK_NY = int(round(BED_W / TRUCK_CELL))
ARMATURE = 200.0
RIGID_SUBSTEPS = 4
USE_MUJOCO_CPU = False  # GPU MuJoCo, with the rigid substeps recorded as one CUDA graph
# moving soil zone (soil_zone.py): only soil near the bucket is MPM particles, the rest is dormant columns
POOL = 200_000  # particle capacity (dry sand flowing into a cut wakes about 110 k); the 2 m fill is about 1.5 million particles
ZONE_EVERY = 4  # frames between wake/sleep updates (15 Hz)
MPM_EVERY = 2  # frames per MPM step: sand at 30 Hz (the stream rate), the machine at 60 Hz
MPM_ITERATIONS = {1: 60, 2: 150}  # the longer step needs more solver iterations to hold the same load

# soil: the pit is filled with sand to BED_TOP (2 m on a 2.35 m deep pit with 1:1.5 sides), held by
# walls on the -x / -y sides and by the sloped sides elsewhere. Resting sand is shown as a heightmap
# over the pit and its filled sides.
BED_TOP = 2.0
WALL_HEIGHT = BED_TOP + 1.0
HM_X0, HM_Y0 = -ZONE_HALF[0], -ZONE_HALF[1]
HM_NX, HM_NY = 96, 86  # 0.1 m cells to x = 6.6, y = 6.1: past where the fill meets the sides
DIG_Z = BED_TOP - 0.5  # the dig waypoints below were set for a 0.5 m bed


WALL_T = 0.4  # thickness of the walls on the pit's -x and -y sides


def ground_height(x, y):
    """Ground under the soil (zone frame): pit floor, 1V:1.5H sides on the machine (+x) and truck (+y)
    sides, site ground beyond, and behind the walls on the -x and -y sides (as the static colliders below)."""
    x, y = np.broadcast_arrays(np.asarray(x, dtype=float), np.asarray(y, dtype=float))
    zx, zy = ZONE_HALF
    side = np.maximum(np.clip((x - zx) / 1.5, 0.0, 2.5), np.clip((y - zy) / 1.5, 0.0, 2.5))
    h = np.where((x > 6.75) | (y > 6.25), GROUND_Z, side)
    # under the walls themselves the pit floor (sand against a wall pokes a little into it)
    behind = ((x < -zx - WALL_T) & (y <= zy + WALL_T)) | ((y < -zy - WALL_T) & (x <= zx))
    return np.where(behind, GROUND_Z, h)


# dormant-soil regions, raised ones first: the truck bed, then the pit with its sides and the ground
# around it (spill), from the pit's walled -x / -y sides to past the machine and the truck
TRUCK_REGION = ("truck", float(BED_LO[0]), float(BED_LO[1]), int(BED_L / CELL), int(BED_W / CELL), lambda x, y: np.full(np.shape(x), BED_FLOOR_Z))
# reaching 2 m past the -x / -y walls and over the loading spot, so that spill there can settle: soil outside
# every soil region never sleeps (it stays awake, using up the particle pool)
GROUND_REGION = ("ground", -ZONE_HALF[0] - 2.0, -ZONE_HALF[1] - 2.0, 229, 181, ground_height)
# haul: the carrier spins round at TURN_X and reverses to DUMP_X (its base), tipping onto a spoil area east
# of the machine, flat site ground from x = 13 to 34 m
HAUL_TURN_X, HAUL_DUMP_X = 20.0, 23.0
CARRIER_STAGES = dumper_mod.STAGES
# dozer (dozer.py): spreads the dumped loads east of the dump point in lanes along +x (cutting edge from
# DOZER_X_START to DOZER_X_END), and parks north of the dump point, out of the carrier's way
DOZER_X_START, DOZER_X_END = 25.2, 33.4  # cutting edge x where the pushes start (the dump heap) and end
DOZER_LANE_ENDS = (1.3, 3.9, 6.5, 9.1)  # y where the fanned lanes reach DOZER_X_END (from the dump line, y 5.2)
DOZER_PARK_Y = 11.4
DOZER_SEES_SPEED = 1.0  # m/s: the dozer plans on all soil slower than this (dumped clay creeps for minutes)
# the haul road between the loading spot and the spoil area: soil falling off a carrier settles there
ROAD_REGION = ("road", 13.3, -2.5, 134, 190, lambda x, y: np.full(np.shape(x), GROUND_Z))
# the spoil area reaches 4.6 m past the dozer's lanes, for the soil carried ahead of the blade at the end,
# and 2 m beside them, for the soil spilled off the blade's ends (soil outside a soil region never settles)
SPOIL_REGION = ("spoil", 24.0, -2.5, 175, 190, lambda x, y: np.full(np.shape(x), GROUND_Z))
SPOIL_HM = (24.0, -2.5, 140, 152)  # display heightmap of the spoil area: x0, y0, nx, ny (0.1 m cells)


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


def quat_from_matrix(R):
    """Rotation matrix -> quaternion (x, y, z, w)."""
    w = math.sqrt(max(1.0 + R[0, 0] + R[1, 1] + R[2, 2], 1e-12)) / 2
    if w > 0.1:
        return np.array([(R[2, 1] - R[1, 2]) / (4 * w), (R[0, 2] - R[2, 0]) / (4 * w), (R[1, 0] - R[0, 1]) / (4 * w), w])
    k = int(np.argmax(np.diag(R)))
    i, j = (k + 1) % 3, (k + 2) % 3
    v = np.zeros(4)
    v[k] = math.sqrt(max(1.0 + R[k, k] - R[i, i] - R[j, j], 1e-12)) / 2
    v[i] = (R[i, k] + R[k, i]) / (4 * v[k])
    v[j] = (R[j, k] + R[k, j]) / (4 * v[k])
    v[3] = (R[j, i] - R[i, j]) / (4 * v[k])
    return v


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
CLEAR_POINTS = [v + np.array([0.0, y, 0.0]) for v in BUCKET_OUTLINE for y in (-BUCKET_W / 2, BUCKET_W / 2)]  # both sides

# ------------------------------------------------------------------ kernels
@wp.kernel
def hydraulics_kernel(
    joint_q: wp.array[float],
    joint_qd: wp.array[float],
    v_cmd: wp.array[float],
    q_ref: wp.array[float],
    kp: wp.array[float],
    kd: wp.array[float],
    tau_lo: wp.array[float],
    tau_hi: wp.array[float],
    q_lo: wp.array[float],
    q_hi: wp.array[float],
    dt: float,
    joint_f: wp.array[float],
    torque: wp.array[float],
):
    j = wp.tid()
    q = joint_q[j]
    # flow: the reference advances at lever speed, within the joint limits ...
    r = wp.clamp(q_ref[j] + v_cmd[j] * dt, q_lo[j], q_hi[j])
    # ... but can't get further ahead of the joint than the relief torque can pull (stall, no windup)
    r = wp.clamp(r, q + tau_lo[j] / kp[j], q + tau_hi[j] / kp[j])
    # the reference never leaves the stroke, even when load back-drives the joint past it
    r = wp.clamp(r, q_lo[j], q_hi[j])
    q_ref[j] = r
    tau = wp.clamp(kp[j] * (r - q) + kd[j] * (v_cmd[j] - joint_qd[j]), tau_lo[j], tau_hi[j])
    torque[j] = tau
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
    flags: wp.array[wp.int32],
    min_speed: float,
    hm_lo: wp.vec3,
    hm_hi: wp.vec3,
    bed_lo: wp.vec3,
    bed_hi: wp.vec3,
    max_out: int,
    counter: wp.array[int],
    out: wp.array[wp.vec3],
):
    """Grains to stream: moving sand, plus sand resting outside the pit and outside the truck bed
    (spill on the ground). Resting sand in the pit and in the bed is sent as heightmaps instead."""
    i = wp.tid()
    if (flags[i] & int(newton.ParticleFlags.ACTIVE)) == 0:
        return
    p = q[i]
    in_zone = p[0] >= hm_lo[0] and p[0] <= hm_hi[0] and p[1] >= hm_lo[1] and p[1] <= hm_hi[1]
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


@wp.kernel
def soil_bond_kernel(jp: wp.array[float], cohesion: float, tensile_ratio: float, crack: float, remoulded: float, bond: wp.array[float], yield_stress: wp.array[float], tensile_yield_ratio: wp.array[float]):
    """Cohesive soil breaks up instead of stretching: its bond falls toward the remoulded fraction as a
    particle opens up plastically (Jp > 1) by `crack`. The damage stays while the particle is awake: dug
    clods pressed together in the bucket do not knit back into one lump (in-situ soil woken from the
    dormant bed starts intact)."""
    i = wp.tid()
    b = wp.min(bond[i], remoulded + (1.0 - remoulded) * wp.clamp(1.0 - (jp[i] - 1.0) / crack, 0.0, 1.0))
    bond[i] = b
    yield_stress[i] = cohesion * b
    tensile_yield_ratio[i] = tensile_ratio * b


@wp.kernel
def reset_bond_kernel(slots: wp.array[int], bond: wp.array[float]):
    bond[slots[wp.tid()]] = 1.0


@wp.kernel
def set_bond_kernel(slots: wp.array[int], value: float, bond: wp.array[float]):
    bond[slots[wp.tid()]] = value


# ------------------------------------------------------------------ simulation
class ExcavatorSim:
    # auto cycle: (stage, swing deg, bucket-pin radius from the swing axis, pin height, bucket angle deg
    # [, speed: fraction of full lever for moving into this stage])
    CYCLE = [
        ("approach", 0, 6.0, 2.6 + DIG_Z, -95),
        ("penetrate", 0, 7.6, 1.5 + DIG_Z, -60),  # near full reach of the ZX200 arm
        ("drag", 0, 5.6, 1.1 + DIG_Z, -100),
        ("drag-curl", 0, 4.7, 1.2 + DIG_Z, -145),  # keep dragging while curling: the 1.2 m mouth fills toward the top
        ("curl", 0, 4.6, 1.6 + DIG_Z, -180),
        ("swing to truck", -103, 5.3, 7.0, -165),  # lifts while swinging, as far as the clearance allows; aimed
        # above what clears the truck, so the boom is still rising at full speed as the bucket gets there
        ("dump", -103, 5.3, 6.2, -40),
        ("swing back", 0, 6.0, 2.6 + DIG_Z, -95),  # lowers while swinging, once clear of the truck
    ]

    def __init__(self, voxel=0.08, ppc=2, soil=soils.DEFAULT_SOIL, iterations=None, fps=60.0, pool=POOL, mpm_every=MPM_EVERY, haul=True, dozer=True, carriers=1):
        wp.init()
        self.kwargs = {"voxel": voxel, "ppc": ppc, "soil": soil, "iterations": iterations, "fps": fps, "pool": pool, "mpm_every": mpm_every, "carriers": carriers}
        self.soil = soil
        material = soils.SOILS[soil]
        density, friction = material["density"], material["friction"]
        iterations = iterations or MPM_ITERATIONS[mpm_every]
        self.frame_dt = 1.0 / fps
        self.mpm_every = mpm_every
        self.mpm_dt = self.frame_dt * mpm_every  # sand impulses act on the machine over one MPM step
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

        q0 = ik(0.0, 6.0, 2.6 + DIG_Z, math.radians(-95))  # the approach pose, above the sand
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

        # dump carrier vessel: a kinematic body (scripted motion: driving and tipping) that holds sand (MPM
        # collider) and can be hit by the bucket (rigid contact). Body frame at the middle of the vessel floor.
        truck_cfg = newton.ModelBuilder.ShapeConfig(mu=0.5)
        L, Wb, Hb, Tb = BED_L, BED_W, BED_H, BED_T
        self.vessel_boxes = [
            ((0.0, 0.0, -Tb / 2), (L / 2 + Tb, Wb / 2 + Tb, Tb / 2)),  # floor
            ((0.0, -Wb / 2 - Tb / 2, Hb / 2), (L / 2 + Tb, Tb / 2, Hb / 2)),  # sides
            ((0.0, Wb / 2 + Tb / 2, Hb / 2), (L / 2 + Tb, Tb / 2, Hb / 2)),
            # tail: a rear-dump vessel is open behind; its floor rises there (TAIL_LEN at TAIL_ANGLE), which holds
            # the load while level and still slopes down for it once tipped
            ((-L / 2 - self.TAIL_LEN * math.cos(self.TAIL_ANGLE) / 2, 0.0, self.TAIL_LEN * math.sin(self.TAIL_ANGLE) / 2 - Tb / 2), (self.TAIL_LEN / 2, Wb / 2 + Tb, Tb / 2), self.TAIL_ANGLE),
            ((L / 2 + Tb / 2, 0.0, (Hb + 0.5) / 2), (Tb / 2, Wb / 2, (Hb + 0.5) / 2)),  # headboard, toward the cab
        ]
        self.vessels = []  # one per carrier; the first starts at the loading spot, the others waiting to load
        for i in range(carriers):
            v = b.add_body(xform=wp.transform(wp.vec3(BED_CENTER[0], BED_CENTER[1] + 6.0 * i, BED_FLOOR_Z), wp.quat_identity()), mass=3000.0,
                           com=wp.vec3(0.0), inertia=box_inertia(3000.0, 2.0, 1.3, 0.5), lock_inertia=True, is_kinematic=True, label=f"vessel{i}")
            for c, h, *a in self.vessel_boxes:
                b.add_shape_box(v, xform=wp.transform(wp.vec3(*c), rot_y(a[0] if a else 0.0)), hx=h[0], hy=h[1], hz=h[2], cfg=truck_cfg)
            self.vessels.append(v)
        # dozer blade: a kinematic body (scripted, dozer.py) that pushes sand (MPM collider); body frame = blade_link
        self.blade_boxes = dozer_mod.blade_plates()
        self.blade = b.add_body(xform=wp.transform(wp.vec3(0.0, 0.0, -20.0), wp.quat_identity()), mass=1200.0, com=wp.vec3(0.0),
                                inertia=box_inertia(1200.0, 0.3, 1.6, 0.6), lock_inertia=True, is_kinematic=True, label="blade")
        blade_cfg = newton.ModelBuilder.ShapeConfig(mu=0.5, has_shape_collision=False)
        for c, h, a in self.blade_boxes:
            b.add_shape_box(self.blade, xform=wp.transform(wp.vec3(*c), rot_y(a)), hx=h[0], hy=h[1], hz=h[2], cfg=blade_cfg)
        cx, cy = BED_CENTER
        self.truck_bed_boxes = [((cx + c[0], cy + c[1], BED_FLOOR_Z + c[2]), h) for c, h, *_ in self.vessel_boxes]  # parked
        # site ground and pit slopes where spilled sand can land (the pit floor plane is 2.35 m lower).
        # Pit: 6 x 5 m floor, sides 1V:1.5H up to ground level at |x| = 6.75, |y| = 6.25 (as in the site terrain).
        ground = newton.ModelBuilder.ShapeConfig(mu=0.6)
        b.add_shape_box(-1, xform=wp.transform(wp.vec3(10.0, 1.0, GROUND_Z / 2)), hx=3.25, hy=9.0, hz=GROUND_Z / 2, cfg=ground)  # behind the machine
        b.add_shape_box(-1, xform=wp.transform(wp.vec3(25.625, 3.0, GROUND_Z / 2)), hx=12.375, hy=11.0, hz=GROUND_Z / 2, cfg=ground)  # haul road and spoil area
        b.add_shape_box(-1, xform=wp.transform(wp.vec3(0.0, 8.1, GROUND_Z / 2)), hx=6.75, hy=1.85, hz=GROUND_Z / 2, cfg=ground)  # beside the pit, truck side
        # behind the walls on the -x and -y sides: site ground, where spill from the bucket lands
        b.add_shape_box(-1, xform=wp.transform(wp.vec3(-6.2, -3.05, GROUND_Z / 2)), hx=2.8, hy=5.95, hz=GROUND_Z / 2, cfg=ground)
        b.add_shape_box(-1, xform=wp.transform(wp.vec3(-0.2, -5.95, GROUND_Z / 2)), hx=3.2, hy=3.05, hz=GROUND_Z / 2, cfg=ground)
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

        # sand: a fixed pool of particles on a 4 cm lattice (2 per voxel edge). The bed starts as dormant
        # columns (soil_zone.py); particles are placed where the bucket digs and parked elsewhere.
        assert abs(voxel - 2 * SPACING) < 1e-9 and ppc == 2, "the soil zone lattice assumes an 8 cm voxel, 2 particles per edge"
        sb = newton.ModelBuilder()
        SolverImplicitMPM.register_custom_attributes(sb)
        self.radius = SPACING / 2
        self.particle_mass = SPACING**3 * density
        side = int(math.ceil(pool ** (1 / 3)))
        sb.add_particle_grid(
            pos=wp.vec3(0.0), rot=wp.quat_identity(), vel=wp.vec3(0.0),
            dim_x=side, dim_y=side, dim_z=int(math.ceil(pool / side**2)),
            cell_x=SPACING, cell_y=SPACING, cell_z=SPACING,
            mass=self.particle_mass, jitter=0.0, radius_mean=self.radius,
            custom_attributes=soils.mpm_attributes(soil, self.DILATANCY),
        )
        self.sand_model = sb.finalize()
        self.n_particles = self.sand_model.particle_count
        self.swell = material["swell"]  # bank to loose volume of dug soil
        self.zones = SoilZones([TRUCK_REGION, GROUND_REGION, ROAD_REGION, SPOIL_REGION], self.n_particles, SPACING**3,
                               swell=self.swell, loose_regions=("truck", "road", "spoil"))
        z = self.zones
        # dormant faces lower than a cohesive soil's critical height stand; only higher ones are woken to fail
        z.AVALANCHE_DROP = max(SoilZones.AVALANCHE_DROP, 0.8 * soils.critical_height(soil)[0])
        ground = z.region_mask("ground")
        in_pit = ground & (z.col_x < 6.75) & (z.col_y < 6.25)
        fill = np.floor((BED_TOP - z.col_floor) / LAYER_PER_PARTICLE / 4) * 4
        self.bed_counts = np.where(in_pit & (fill > 0), fill, 0).astype(np.int64)
        self._pit_cols = in_pit & (z.col_x > -ZONE_HALF[0]) & (z.col_y > -ZONE_HALF[1])  # inside the walls
        in_hm = (z.col_x < HM_X0 + HM_NX * HM_CELL) & (z.col_y < HM_Y0 + HM_NY * HM_CELL)
        z.set_display((ground & ~in_hm) | z.region_mask("road"))  # the pit, truck and spoil heightmaps show the rest
        self.truck_floor = z.col_floor[z.region_mask("truck")].copy()
        # the carriers' haul cycle; each vessel and the dozer blade has a free joint (found by its child body)
        self.haul = haul
        self.fleet = Fleet(carriers, BED_CENTER, GROUND_Z, HAUL_DUMP_X, HAUL_TURN_X, TRUCK_CAPACITY_KG)
        child = self.model.joint_child.numpy()
        jq, jqd = self.model.joint_q_start.numpy(), self.model.joint_qd_start.numpy()
        joint_of = {int(bd): j for j, bd in enumerate(child)}
        for c, v in zip(self.fleet, self.vessels):
            c.body, c.vq, c.vqd = v, int(jq[joint_of[v]]), int(jqd[joint_of[v]])
        # the dozer spreading the loads at the spoil area; its blade's free joint follows the vessel's
        self.use_dozer = haul and dozer
        edge_ahead = Dozer._edge_ahead()
        self.dozer = Dozer((DOZER_X_START - edge_ahead, DOZER_PARK_Y, math.pi / 2), (DOZER_X_START, BED_CENTER[1], DOZER_X_END, DOZER_LANE_ENDS), GROUND_Z)
        self.blade_q, self.blade_qd = int(jq[joint_of[self.blade]]), int(jqd[joint_of[self.blade]])

        cfg = SolverImplicitMPM.Config()
        cfg.voxel_size = voxel
        cfg.max_iterations = iterations
        cfg.critical_fraction = self.CRITICAL_FRACTION
        cfg.strain_basis = "P0"
        if "mpm:yield_stress" in soils.mpm_attributes(soil, self.DILATANCY):
            cfg.warmstart_mode = "particles"  # keeps per-particle stress and plastic volume (Jp) for cracking
        self.mpm = SolverImplicitMPM(self.sand_model, config=cfg)
        # colliders: static shapes, the stick and bucket, and the dormant soil columns
        collide = [bid for bid in range(self.model.body_count) if bid in (stick, bucket, *self.vessels, self.blade)]
        self.mpm.setup_collider(
            model=self.model,
            collider_body_ids=[-1, *collide, None],
            collider_meshes=[None] * (1 + len(collide)) + [self.zones.mesh],
            collider_friction=[None] * (1 + len(collide)) + [soils.resting_friction(soil)],  # dormant soil
        )
        self.collider_body = self.mpm.collider_body_index

        self.solver = SolverMuJoCo(self.model, use_mujoco_contacts=False, njmax=200, nconmax=200, use_mujoco_cpu=USE_MUJOCO_CPU)
        # rigid contacts: bucket and stick against the hard pit floor (the sand acts through MPM)
        self.collision = newton.CollisionPipeline(self.model)
        self.contacts = self.collision.contacts()
        # McKyes reference force for the current cut: factors at a 0.3 m cut (they change little with depth)
        f_ref, fac = soils.mckyes(density, material["cohesion"], 0.3, BUCKET_W, math.atan(friction), math.atan(0.4))
        self.n_gamma, self.n_c = fac["N_gamma"], fac["N_c"]
        self.fee_scale = math.cos(math.atan(0.4))  # horizontal part of the tool force (vertical tool)
        self.cohesion = material["cohesion"]
        attrs = soils.mpm_attributes(soil, self.DILATANCY)
        self.bond = (attrs["mpm:yield_stress"], attrs["mpm:tensile_yield_ratio"], soils.CRACK_STRAIN, material["remoulded"]) if "mpm:yield_stress" in attrs else None
        self.bond_state = wp.ones(self.n_particles, dtype=float)  # 1 intact .. remoulded, per particle
        if self.bond is not None:
            self.zones.on_spawn = lambda slots: wp.launch(reset_bond_kernel, dim=slots.shape[0], inputs=[slots, self.bond_state])
        # curl now: sand heaped to 110 % of rated inside the shell (the rest flows off). A cohesive soil is
        # counted with the heap standing on and ahead of the mouth, which does not flow off and still grows
        # while the bucket curls out, so the operator curls short of the payload the soil typically gives
        self.heap_check = material["cohesion"] > 0.0
        # a soil whose cut faces stand taller than the bucket holds its heap while the bucket lifts out of
        # the cut; a weaker one (moist sand) loses it, and is curled along the cut as sand is
        self.lift_out = self.heap_check and soils.critical_height(soil)[0] > self.HEAP_STANDS_M
        self.payload_kg = soils.payload_kg(soil, BUCKET_RATED_M3)
        self.full_kg = self.payload_kg if self.heap_check else 1.1 * BUCKET_RATED_M3 * density

        # actuator parameters on the GPU
        arr = lambda d: wp.array([float(d[j]) for j in JOINTS], dtype=float)  # noqa: E731
        self.kp, self.kd = arr(KP), arr(KD)
        self.tau_lo, self.tau_hi = wp.zeros(4, dtype=float), wp.zeros(4, dtype=float)
        self.linkage = hyd.Linkage(self.link_frames, pitch)
        self.vmax = self.nominal_vmax()
        self.vmax_dir = (self.nominal_vmax(-1.0), self.vmax)  # joint moving down, up
        self.q_lo = wp.array([LIMITS[j][0] for j in JOINTS], dtype=float)
        self.q_hi = wp.array([LIMITS[j][1] for j in JOINTS], dtype=float)
        self.v_cmd = wp.zeros(4, dtype=float)
        self.q_ref = wp.zeros(4, dtype=float)
        self.torque = wp.zeros(4, dtype=float)
        self.truck_hm = wp.zeros(TRUCK_NX * TRUCK_NY, dtype=float)
        self.spoil_hm = wp.zeros(SPOIL_HM[2] * SPOIL_HM[3], dtype=float)
        self.spoil_hm_np = np.zeros(SPOIL_HM[2] * SPOIL_HM[3], dtype=np.float32)
        self.doze_hm = wp.zeros(SPOIL_HM[2] * SPOIL_HM[3], dtype=float)  # what the dozer sees, creeping soil included
        self.doze_hm_np = np.full(SPOIL_HM[2] * SPOIL_HM[3], -0.95, dtype=np.float32)
        self.bucket_poly = wp.array(BUCKET_SHELL.astype(np.float32), dtype=wp.vec2)
        self.heap_poly = wp.array(_heaped_profile().astype(np.float32), dtype=wp.vec2)
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
        self.zones.attach(self.sand_model, self.sand, self.mpm)
        self.zones.reset(self.bed_counts)
        self.bond_state.fill_(1.0)
        self.zone_frame = 0
        self.sand_forces = wp.zeros_like(self.state_0.body_f)
        self.imp_ids.fill_(-1)
        self.rigid_graph = None  # new state buffers: record the rigid graph again
        self.q_ref.assign(self.q0.astype(np.float32))
        self.levers = np.zeros(4)
        self.auto = False
        self.fleet.reset()
        self.dozer.reset()
        self.blade_force = np.zeros(3)
        self.zones.set_region_floor("truck", self.truck_floor)
        self.dispatch_pending, self.truck_wait = False, 0.0
        self.pass_kg, self.truck_mark = 0.0, 0.0
        self.was_waiting = False
        self.auto_stage = 0
        self.auto_s = 0.0
        self.passes = 0
        self.skipped = []
        self.ease, self.seg_from, self.auto_frame, self.auto_fill, self.early_curls = 0.0, None, 0, 0.0, 0
        self.curl_out = None  # curl waypoint replaced for this pass (lifting out of a full cohesive cut)
        self.curl_gain = self.CURL_GAIN_GUESS * self.payload_kg if self.heap_check else 0.0
        self.curl_from = None  # load when this pass's early curl began
        self.swing_inertia = self.SWING_INERTIA_GUESS
        self._z_cache = {}
        self._faces, self.face_log, self.pit_done = {}, [], False
        self.face_rest, self.pass_fill = {}, 0.0  # strips set aside after a poor pass (until pass n); this pass's best fill
        self.stage_time = 0.0
        self.auto_wait = 0.0
        self.low_prev, self.low_rate, self.swing_held, self.arm_held = None, 0.0, 0.0, 0.0
        self.shaking, self.shakes = None, 0
        self.stage_name = "manual"
        self.sim_time = 0.0
        self.step_ms = 0.0
        self.q = self.q0.copy()
        self.qd = np.zeros(4)
        self.press = np.zeros(4)
        self.tau = np.zeros(4)
        self.cyl_p = np.zeros(3)
        self.cyl_head = np.ones(3, dtype=bool)
        self.cyl_pts, self.cyl_L, self.cyl_J = self.linkage.state(self.q0)
        self.truck_kg, self.spot_kg = 0.0, 0.0
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
    DEEPEN_PER_PASS, MAX_DEEPEN = 0.15, BED_TOP  # per level (all lanes dug); the floor clearance stops it

    FLOOR_CLEARANCE = 0.08  # m: deepened dig paths keep the bucket this far above the hard pit floor
    FIRST_CUT = 0.42  # m below the bed top: the first pass's cut (as on the earlier 0.5 m bed)

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
        """Waypoint pin height. Dig stages cut to a level FIRST_CUT below the bed top and go a little
        deeper on each pass, like an operator would, but never so deep that the bucket would run into
        the hard floor on the way there."""
        if wp_[0] in ("swing to truck", "dump"):
            return wp_[3] + self._load_raise()
        if wp_ is self.curl_out:
            return wp_[3]
        if wp_[0] not in self.DIG_STAGES:
            return wp_[3]
        key = (wp_[0], self.passes)
        if key not in self._z_cache:
            i = next(k for k, w in enumerate(self.CYCLE) if w[0] == wp_[0])
            prev = self.CYCLE[i - 1]
            z_prev = self._stage_z(prev)
            face = self._face(self.passes)
            if face is not None:  # cut FIRST_CUT below the chosen strip's own surface, clear of its hard floor
                deepen, lowest = face["deepen"], face["lowest"]
            else:
                level = self.passes // len(self.LANES)  # deepen only after every lane has been dug
                deepen = min(self.DEEPEN_PER_PASS * level, self.MAX_DEEPEN)
                lowest = max(self.FLOOR_CLEARANCE, BED_TOP - self.FIRST_CUT - deepen)
            z = wp_[3] - deepen
            while self._bucket_lowest(prev, z_prev, wp_, z) < lowest:
                z += 0.01
            self._z_cache[key] = z
        return self._z_cache[key]

    # successive passes dig side-by-side lanes (swing offsets, deg) before cutting deeper, like an
    # operator working across the face; at 6-8 m reach 9 deg is about one bucket width
    LANES = (0.0, 9.0, -9.0, 17.0, -17.0)
    LANE_STAGES = ("approach", "penetrate", "drag", "drag-curl", "curl")

    # where to dig: before each pass the operator looks at the pit and takes the strip (a swing direction and a
    # reach) holding the most resting soil above its hard floor, cutting FIRST_CUT below that strip's own
    # surface; the strip is a bucket width along the dig path (cutting edge EDGE_REACH from the swing axis,
    # shifted in or out). It works across the face and around into the sloped sides as the middle is dug out.
    # MPM: the largest fraction of a grid cell particles may fill. With the limit, a particle's volume is the
    # room it takes: soil in place fills the cells (the 4 cm lattice) and dug soil, at (1 + swell) times
    # the volume, spreads out at lower density (bulking, soil_zone.py)
    CRITICAL_FRACTION = 1.0
    # flow-rule dilation (soils.mpm_attributes). Off: the packing limit alone brings dug soil to its loose
    # density (within 6 % for the three soils); 0.1-0.3 cut the payload per pass by up to a third and change
    # the soil's cutting strength, which the force validations were done without
    DILATANCY = 0.0
    adaptive_face = True
    FACE_PSI = (20.0, 10.0, 0.0, -10.0, -20.0, -30.0, -40.0)  # swing offsets (deg; negative toward the truck side)
    # m: reach in or out of the standard dig path. Only the standard one: reaching 1 m further runs the arm into
    # its limits (stages time out, 0.1-0.3 t per pass) and 1 m closer, onto the slope under the machine, digs little
    FACE_SHIFTS = (0.0,)
    EDGE_REACH = (4.4, 7.4)  # m: the cutting edge's distance from the swing axis, end and start of the dig path
    STRIP_HALF_W = 0.55  # m: half the bucket width
    PIT_DONE_M3 = 0.3  # no strip's cut holds more than this: the excavation is finished (within reach)
    POOR_PASS, POOR_REST = 0.3, 12  # a pass bringing back less than this fraction of a full bucket sets its strip aside for this many passes

    def _face(self, p):
        """The cut for pass p: swing offset, reach shift, deepen and the lowest the bucket may go (None in
        trench mode). Planned once, when first asked for (as the bucket swings back for that pass)."""
        if not self.adaptive_face:
            return None
        if p not in self._faces:
            self._faces = {k: v for k, v in self._faces.items() if k >= p - 1}
            self._faces[p] = self._plan_face(p)
        return self._faces[p]

    def _plan_face(self, p):
        z = self.zones
        has = self._pit_cols & (z.count_np > 0)
        cx, cy = z.col_x[has], z.col_y[has]
        top = z.col_floor[has] + z.count_np[has] * z.col_layer_np[has]
        hard = ground_height(cx, cy)
        dx, dy = cx - SWING_POS[0], cy - SWING_POS[1]
        best, best_score = None, -1.0
        for psi in self.FACE_PSI:
            # psi = 0 digs toward -x, positive toward -y (swing + left)
            ux, uy = -math.cos(math.radians(psi)), -math.sin(math.radians(psi))
            along, across = dx * ux + dy * uy, -dx * uy + dy * ux
            for shift in self.FACE_SHIFTS:
                if shift < 0 and abs(psi) > 10.0:
                    continue  # reaching in is for the slope under the machine, straight ahead
                band = (along > self.EDGE_REACH[0] + shift) & (along < self.EDGE_REACH[1] + shift) & (np.abs(across) < self.STRIP_HALF_W)
                if band.sum() < 10 or self.face_rest.get((psi, shift), -1) > p:
                    continue
                band_top = float(np.percentile(top[band], 90))
                # the cut goes FIRST_CUT into the strip, but stays clear of its highest hard floor (a slope)
                lowest = max(float(hard[band].max()) + self.FLOOR_CLEARANCE, band_top - self.FIRST_CUT)
                avail = float(np.clip(top[band] - lowest, 0.0, None).sum() * CELL * CELL)  # what this pass can cut
                score = avail * (1.1 if shift == 0.0 else 1.0)  # the standard path digs best
                if score > best_score:
                    best_score, best = score, {"psi": psi, "shift": shift, "avail_m3": round(avail, 2), "deepen": max(0.0, BED_TOP - band_top), "lowest": lowest}
        if best is None or best["avail_m3"] < self.PIT_DONE_M3:
            self.pit_done = True
            return self._faces.get(max(self._faces, default=None), None) if self._faces else {"psi": 0.0, "shift": 0.0, "avail_m3": 0.0, "deepen": 0.0, "lowest": BED_TOP}
        self.face_log.append({"pass": p, **{k: (round(v, 2) if isinstance(v, float) else v) for k, v in best.items()}})
        return best

    def _stage_r(self, wp_, start=False):
        """Waypoint reach (m): dig-side stages shift with the current pass's cut, 'swing back' heads for the
        next pass's (as _stage_psi)."""
        if self.adaptive_face and wp_[0] in self.LANE_STAGES:
            return wp_[2] + self._face(self.passes)["shift"]
        if self.adaptive_face and wp_[0] == "swing back":
            return wp_[2] + self._face(self.passes + (0 if start else 1))["shift"]
        return wp_[2]

    def set_trench(self, deepen=0.25):
        """Auto cycle digs a single lane and goes deeper every pass (a trench), down to the floor."""
        self.adaptive_face = False
        self.LANES = (0.0,)
        self.DEEPEN_PER_PASS = deepen
        self._z_cache = {}

    def _stage_psi(self, wp_, start=False):
        """Waypoint swing angle (deg): dig-side stages shift to the current pass's lane; the final
        'swing back' stage already heads for the next pass's lane. As the start of the next segment (after
        the pass count went up) 'swing back' is in the current lane."""
        if self.adaptive_face and wp_[0] in self.LANE_STAGES:
            return wp_[1] + self._face(self.passes)["psi"]
        if self.adaptive_face and wp_[0] == "swing back":
            return wp_[1] + self._face(self.passes + (0 if start else 1))["psi"]
        if wp_[0] in self.LANE_STAGES:
            return wp_[1] + self.LANES[self.passes % len(self.LANES)]
        if wp_[0] == "swing back":
            return wp_[1] + self.LANES[(self.passes + (0 if start else 1)) % len(self.LANES)]
        return wp_[1]

    def _waypoint_q(self, wp_, start=False):
        r, phi = self._stage_r(wp_, start), wp_[4]
        return ik(math.radians(self._stage_psi(wp_, start)), r, self._stage_z(wp_), math.radians(phi))

    # operator judgement in the auto cycle: ease off a cut that stalls the arm or bucket at relief, and
    # curl as soon as the bucket is full instead of finishing the planned drag
    EASE_PRESSURE = 0.9  # fraction of relief on the arm or bucket cylinder that counts as stalling
    EASE_RATE, EASE_MAX = 0.4, 0.8  # m/s the dig path rises while stalled; most it rises in one pass
    HEAP_STANDS_M = 1.2  # m: critical cut height above which a heap stands on the bucket as it lifts out
    CURL_OUT = (0.2, 1.0)  # m the pin moves in and up while curling out of a full cut in cohesive soil

    def _pin_pose(self):
        """Current bucket pin as a cycle waypoint (swing deg, radius, height, mouth angle deg)."""
        _, p2, b3 = chain(self.q)
        return (math.degrees(self.q[0]), float(p2[0]), float(p2[2] + SWING_POS[2]), math.degrees(MOUTH_ANGLE - b3))

    def _bucket_fill(self):
        self.fill_counter.zero_()
        # what an operator sees: in a cohesive soil, the load in the bucket and heaped on it (or pushed ahead)
        poly = self.heap_poly if self.heap_check else self.bucket_poly
        wp.launch(bucket_profile_fill_kernel, dim=self.n_particles, inputs=[self.sand.particle_q, self.bucket_xform(), poly, BUCKET_W / 2, self.fill_counter])
        return float(self.fill_counter.numpy()[0]) * self.particle_mass

    def _jump_to(self, stage):
        """Start the segment into `stage` from where the bucket is now."""
        self.auto_stage = next(k for k, w in enumerate(self.CYCLE) if w[0] == stage) - 1
        self.auto_s, self.stage_time, self.auto_wait = 0.0, 0.0, 0.0
        self.seg_from = self._pin_pose()

    # clearance the auto cycle keeps while swinging (m): above the truck's sides and load, and above the
    # soil and ground; the band along the truck's sides is CLEAR_BAND wide on each side of them
    CLEAR_TRUCK, CLEAR_LOAD, CLEAR_GROUND, CLEAR_BAND = 0.25, 0.2, 0.3, 0.3
    LOAD_PEAK = 1.5  # heap top over the load's mean depth (the truck heightmap also sees the sand in the bucket)
    ARM_MARGIN = 0.15  # m: the arm may lower the bucket toward the clearance only to this much above it

    def _load_top(self):
        """Height of the heaped load in the truck, from its mass."""
        return BED_FLOOR_Z + self.LOAD_PEAK * self.truck_kg / self.density / (BED_L * BED_W)

    def _load_raise(self):
        """How much higher (m) to carry and dump the bucket than planned, for a load heaped above the sides."""
        return max(0.0, self._load_top() + self.CLEAR_LOAD - (BED_FLOOR_Z + BED_H + self.CLEAR_TRUCK))

    def _clear_height(self, x, y):
        """Height (m) the bucket must stay above at zone points (x, y)."""
        m = self.CLEAR_BAND
        x0, y0, x1, y1 = BED_LO[0], BED_LO[1], BED_LO[0] + BED_L, BED_LO[1] + BED_W
        in_bed = (x > x0 - m) & (x < x1 + m) & (y > y0 - m) & (y < y1 + m)
        # over the whole vessel the bucket stays above its sides as well as the load: lowered inside it, the
        # bucket could not swing out past a side again (the arm only backs up along its own path)
        truck = max(self._load_top() + self.CLEAR_LOAD, BED_FLOOR_Z + BED_H + self.CLEAR_TRUCK)
        wall = (x < -ZONE_HALF[0] + m) | (y < -ZONE_HALF[1] + m)
        ground = np.where(wall, WALL_HEIGHT, np.maximum(ground_height(x, y), BED_TOP)) + self.CLEAR_GROUND
        return np.where(in_bed, truck, ground)

    @staticmethod
    def _bucket_local(q):
        """Bucket bounding points relative to the swing joint, zone frame, with the house at swing 0."""
        _, p2, b3 = chain(q)
        h = np.array([p2 + pitch(v, b3) for v in CLEAR_POINTS])
        return h * np.array([-1.0, -1.0, 1.0])

    def _clearance(self, local, psi):
        """Least height (m) of the bucket above what it must clear, with the house at each of the angles
        `psi` (negative: it would hit something)."""
        psi = np.atleast_1d(psi)
        c, s = np.cos(psi)[:, None], np.sin(psi)[:, None]
        x = SWING_POS[0] + c * local[:, 0] - s * local[:, 1]
        y = SWING_POS[1] + s * local[:, 0] + c * local[:, 1]
        return np.min(SWING_POS[2] + local[:, 2] - self._clear_height(x, y), axis=1)

    def _sweep(self, direction, steps=8, ahead=0.07):
        """House angles it passes through if it brakes from now, plus `ahead` rad more."""
        v, brake = abs(self.qd[0]), SWING_TAU * self.relief / self.swing_inertia
        t = np.linspace(0.0, v / brake, steps)
        d = v * t - 0.5 * brake * t * t
        return self.q[0] + direction * np.append(d, d[-1] + ahead)

    def _swing_allowance(self, local, target, direction, rise_to_end):
        """Fastest the house may swing toward `target` (rad/s): slow enough to stop before the first angle the
        bucket does not clear yet, and to get there no sooner than the rising bucket clears it (an operator
        feathering the swing while the boom comes up)."""
        span = abs(target - self.q[0])
        psi = self.q[0] + direction * np.linspace(0.0, span, max(2, int(span / 0.035) + 2))
        m = self._clearance(local, psi)
        bad = np.nonzero(m < 0.0)[0]
        if not bad.size:
            return math.inf
        k = bad[0]
        dist = max(abs(psi[k] - self.q[0]) - 0.07, 0.0)
        v = math.sqrt(2.0 * SWING_TAU * self.relief / self.swing_inertia * dist)
        deficit = -float(m[k:].min())
        if self.low_rate > 0.02 and deficit <= rise_to_end + 0.02:
            v = min(v, dist * self.low_rate / deficit)
        return v

    SWING_INERTIA_GUESS = 55e3  # kg m2 about the swing axis, until measured (boom raised, bucket empty)
    SWING_STAGE_DEG = 20  # stages that turn the house further than this are swung like an operator does

    def _swing_lever(self, target, allow=None):
        """Operator swing: full lever, then neutral once the house would coast to the target on the
        cross-over reliefs; small corrections once it has nearly stopped. `allow(direction)` is the fastest
        the bucket's clearance lets the house swing: the lever is eased to it, or neutral (brake) above it."""
        err, v = target - self.q[0], self.qd[0]
        brake = SWING_TAU * self.relief / self.swing_inertia
        stop = v * v / (2 * brake) * 1.1 + 0.02
        if abs(v) > 0.15:
            lever = math.copysign(1.0, err) if (err * v < 0 or abs(err) > stop) else 0.0
        else:
            lever = float(np.clip(4.0 * err / SWING_VMAX, -0.5, 0.5))
        if allow is not None and lever * err > 0:
            v_ok = allow(math.copysign(1.0, err))
            if v_ok < SWING_VMAX:
                self.swing_held += self.frame_dt
                if abs(v) > v_ok + 0.05:
                    return 0.0
                lever = math.copysign(min(abs(lever) if abs(v) <= 0.15 else 1.0, v_ok / SWING_VMAX), lever)
        return lever

    CURL_GAIN_GUESS = 0.3  # of the payload a cohesive load gains (< 0: loses) curling out, until measured
    SHAKE_IF = 0.25  # fraction of a full bucket still in it after the dump that makes the operator shake it out
    SHAKE_S, SHAKE_PERIOD = 2.5, 0.32  # most time spent shaking; one rap: dump to the stop, then a short curl back

    def _shake(self):
        """Rap the bucket against its dump stop (full dump lever, then a short curl), as an operator does to
        free clay stuck in the bucket, until it is nearly empty or SHAKE_S is up."""
        self.shaking += self.frame_dt
        phase = (self.shaking % self.SHAKE_PERIOD) / self.SHAKE_PERIOD
        self.levers = np.zeros(4)
        self.levers[3] = 1.0 if phase < 0.6 else -0.7  # lever sign: + dumps
        if self.auto_frame % 6 == 0 and (self.shaking > self.SHAKE_S or self._bucket_fill() < 0.1 * self.full_kg):
            self.shaking = None
            self._after_dump()
            self.auto_stage = (self.auto_stage + 1) % len(self.CYCLE)
            self.auto_s, self.stage_time, self.auto_wait, self.seg_from = 0.0, 0.0, 0.0, None
            self.low_prev, self.low_rate = None, 0.0
        self.auto_frame += 1

    DUMP_SPOTS = (0.9, -0.9, 0.0)  # m along the vessel (+ toward the cab): passes spread the load, front first

    def _dump_waypoint(self, w):
        """Swing and pin radius over this pass's spot in the vessel (the CYCLE entry aims at its middle)."""
        dx = self.DUMP_SPOTS[self.passes % len(self.DUMP_SPOTS)]
        x, y = BED_CENTER[0] + dx - SWING_POS[0], BED_CENTER[1] - SWING_POS[1]
        psi = math.degrees(math.atan2(-y, -x))  # the arm points along (-cos psi, -sin psi)
        return (w[0], psi, math.hypot(x, y) + w[2] - math.hypot(BED_CENTER[0] - SWING_POS[0], BED_CENTER[1] - SWING_POS[1]), *w[3:])

    def _after_dump(self):
        """A pass is in the carrier: if the next would not fit, send it off (it leaves once the bucket is clear)."""
        if not self.haul:
            return
        self.measure()
        self.pass_kg = self.truck_kg - self.truck_mark  # what this pass put in
        self.truck_mark = self.truck_kg
        if self.truck_kg + max(self.pass_kg, 0.5 * self.full_kg) > TRUCK_CAPACITY_KG:
            self.dispatch()
            self.truck_mark = 0.0

    def _auto_control(self):
        """Track the cycle: move a target along the current segment only while the arm keeps up."""
        n = len(self.CYCLE)
        a = self.CYCLE[self.auto_stage]
        b = self.CYCLE[(self.auto_stage + 1) % n]
        a, b = (self.curl_out if self.curl_out is not None and w[0] == "curl" else w for w in (a, b))
        a, b = (self._dump_waypoint(w) if w[0] in ("swing to truck", "dump") else w for w in (a, b))
        self.stage_name = b[0]
        waiting = self.haul and b[0] == "swing to truck" and self.fleet.at_spot is None
        if waiting:
            # no carrier at the loading spot: swing part way and hold the full bucket up there (the pit's
            # soil goes back to sleep), until one is back
            b = self.WAIT_POSE
            self.stage_name = b[0]
            self.truck_wait += self.frame_dt
            self.was_waiting = True
        elif self.was_waiting and b[0] == "swing to truck":
            self.was_waiting = False
            self.auto_s, self.stage_time, self.auto_wait = 0.0, 0.0, 0.0
            self.seg_from = self._pin_pose()  # on to the carrier from where the bucket waited
        digging = b[0] in self.DIG_STAGES
        if digging and self.auto_frame % 6 == 0:
            self.auto_fill = self._bucket_fill()
            self.pass_fill = max(self.pass_fill, self.auto_fill)
            if self.auto_fill >= self.full_kg - self.curl_gain and b[0] != "curl":
                self.early_curls += 1
                self._jump_to("curl")
                if self.lift_out:
                    # cohesive soil: curl while lifting out where the bucket is, not on along the cut to
                    # the planned curl point (dragging on through clay would tear up another slab)
                    _, r, z, _ = self.seg_from
                    c = self.CYCLE[self.auto_stage + 1]
                    self.curl_out = (c[0], c[1], r - self.CURL_OUT[0], z + self.CURL_OUT[1], c[4])
                self.curl_from = self.auto_fill if self.heap_check else None
                return self._auto_control()
        if self.shaking is not None:
            return self._shake()
        self.auto_frame += 1
        ease_a = self.ease if a[0] in self.DIG_STAGES else 0.0
        ease_b = self.ease if digging else 0.0
        q_end = self._waypoint_q(b)
        rb = self._stage_r(b)
        q_end = ik(q_end[0], rb, self._stage_z(b) + ease_b, math.radians(b[4])) if ease_b else q_end
        speed = b[5] if len(b) > 5 else 1.0
        s = self.auto_s
        if self.seg_from is not None:
            pa, ra, za, fa = self.seg_from
        else:
            pa, ra, za, fa = self._stage_psi(a, start=True), self._stage_r(a, start=True), self._stage_z(a) + ease_a, a[4]
        q_start = ik(math.radians(pa), ra, za, math.radians(fa))
        dq = q_end - q_start
        v_dir = np.where(dq > 0, self.vmax_dir[1], self.vmax_dir[0])
        zb = self._stage_z(b) + ease_b
        pb = self._stage_psi(b)
        swinging = abs(pb - pa) > self.SWING_STAGE_DEG
        # a swing stage moves the house and the arm at the same time, each at its own pace: the plan's clock
        # is the arm's, and the house swings as fast as the bucket's clearance lets it
        moving = slice(1, None) if swinging else slice(None)
        seg_time = max(np.max(np.abs(dq[moving]) / (v_dir[moving] * speed)), 0.5)
        seg_all = max(np.max(np.abs(dq) / (v_dir * speed)), 0.5)  # with the swing, for the stage timeout

        def pose(s):
            return ik(math.radians(pa + s * (pb - pa)), ra + s * (rb - ra), za + s * (zb - za), math.radians(fa + s * (b[4] - fa)))

        q_goal = pose(s)  # straight bucket-pin path
        held = False
        if swinging:
            # the arm lowers the bucket toward what the house would sweep before it could stop only down to
            # ARM_MARGIN above it, and backs up along its path if the bucket is closer than that (lifting is
            # never held); the house swings as fast as the rising bucket's clearance allows
            local = self._bucket_local(self.q)
            low = float(local[:, 2].min())
            self.low_rate += 0.3 * ((low - self.low_prev) / self.frame_dt - self.low_rate) if self.low_prev is not None else 0.0
            self.low_prev = low
            psi_now = self._sweep(math.copysign(1.0, self.qd[0]) if abs(self.qd[0]) > 0.05 else 0.0)
            if s < 1.0:
                m_now = float(self._clearance(local, psi_now).min())
                m_next = float(self._clearance(self._bucket_local(pose(min(1.0, s + self.frame_dt / seg_time))), psi_now).min())
                held = m_next < self.ARM_MARGIN and m_next < m_now
                if held and m_now < 0.5 * self.ARM_MARGIN:
                    s = max(0.0, s - self.frame_dt / seg_time)
                    self.auto_s = s
                    q_goal = pose(s)
            rise_to_end = float(self._bucket_local(ik(0.0, rb, zb, math.radians(b[4])))[:, 2].min()) - low

            def allow(direction):
                return self._swing_allowance(local, math.radians(pb), direction, rise_to_end)

        err = q_goal - self.q
        # lever: the opening the planned pace needs (feed-forward), corrected by the tracking error; in the
        # cut the lever follows the error only, so the bucket's progress is set by what the sand lets through
        # (only while the plan advances: a joint pushed ahead of a held plan would cancel its own correction)
        ff = dq / seg_time / v_dir if s < 1.0 and not digging and not held and self.auto_wait == 0.0 else 0.0
        self.levers = np.clip(ff + 4.0 * err / self.vmax, -speed, speed) * LEVER_SIGN
        if swinging:
            self.levers[0] = self._swing_lever(math.radians(pb), allow) * speed * LEVER_SIGN[0]
            err[0] = 0.0
        # stalled in the cut (arm or bucket at relief and falling behind): raise the dig path
        if digging and max(self.press[2], self.press[3]) >= self.EASE_PRESSURE * self.relief and np.max(np.abs(err)) > 0.05:
            self.ease = min(self.ease + self.EASE_RATE * self.frame_dt, self.EASE_MAX)
        # advance along the segment at the pace the slowest joint allows, but only if tracking is good
        if held:  # waiting for the house to swing clear is not a stall
            self.arm_held += self.frame_dt
        elif np.max(np.abs(err)) < 0.12:
            self.auto_s = min(1.0, s + self.frame_dt / seg_time)
            self.auto_wait = 0.0
        else:
            self.auto_wait += self.frame_dt
        # a stalled arm (e.g. bucket on the hard floor) must not hang the cycle: give up on the stage
        # after 4 s of poor tracking, or when the stage takes far longer than planned (slow creep)
        self.stage_time += self.frame_dt
        timed_out = self.auto_wait > self.STAGE_TIMEOUT or self.stage_time > 3.0 * seg_all + self.STAGE_TIMEOUT
        if timed_out:
            self.skipped.append({"t": round(self.sim_time, 2), "stage": b[0], "max_err_rad": round(float(np.max(np.abs(err))), 3)})
            self.auto_wait = 0.0
        # close enough to the waypoint; after running 1.5 s past plan, an arm holding a heavy load at relief
        # (that cannot quite reach the pose) is let go on instead of timing out
        tol = 0.06 if self.stage_time < seg_all + 1.5 else 0.15
        if waiting:
            self.skipped = self.skipped[:-1] if timed_out else self.skipped  # holding is not a stall
            return
        if timed_out or (self.auto_s >= 1.0 and np.max(np.abs(q_end - self.q)) < tol):
            if b[0] == "curl" and self.curl_from is not None:
                # learn how much the curl adds (< 0: the heap fell off), to curl that much short next time
                self.curl_gain += 0.5 * (self._bucket_fill() - self.curl_from - self.curl_gain)
            if b[0] == "dump" and self._bucket_fill() > self.SHAKE_IF * self.full_kg:
                self.shaking = 0.0  # sticky load still in the bucket: shake it out before swinging back
                self.shakes += 1
                return
            if b[0] == "dump":
                self._after_dump()
            self.auto_stage = (self.auto_stage + 1) % n
            self.auto_s = 0.0
            self.stage_time = 0.0
            self.seg_from = None
            self.low_prev, self.low_rate = None, 0.0
            if self.auto_stage == 0:
                face = self._faces.get(self.passes) if self.adaptive_face else None
                if face is not None and self.pass_fill < self.POOR_PASS * self.full_kg:
                    self.face_rest[(face["psi"], face["shift"])] = self.passes + self.POOR_REST  # little came out of it
                    for f in self.face_log[-2:]:
                        if f["pass"] == self.passes:
                            f["poor_kg"] = round(self.pass_fill)
                self.pass_fill = 0.0
                self.passes += 1
                self.ease = 0.0
                self.curl_out, self.curl_from = None, None
                if self.pit_done:  # nothing left within reach: stop, and send the carrier off with what it has
                    self.auto = False
                    self.stage_name = "pit dug out"
                    if self.haul and self.fleet.at_spot is not None:
                        self.dispatch_pending = True

    # -------------------------------------------------------------- hydraulics
    def link_frames(self, q):
        """Link frame origins and pitches in the house frame, for the cylinder pins."""
        p1, p2, b3 = chain(q)
        b1 = BOOM_PITCH + q[1]
        return {"house": (np.zeros(3), 0.0), "boom": (BOOM_ORIGIN, b1), "stick": (p1, b3 - BUCKET_PITCH - q[3]), "bucket": (p2, b3)}

    def nominal_vmax(self, sign=1.0):
        """Joint speeds at full lever, mid stroke, moving each joint in the direction of `sign`
        (for planning the auto cycle: a cylinder filling its rod side runs faster)."""
        mid = np.array([0.0] + [0.5 * (LIMITS[j][0] + LIMITS[j][1]) for j in JOINTS[1:]])
        _, _, J = self.linkage.state(mid)
        v = [hyd.cylinder_velocity(sign * np.eye(3)[k], J)[k] for k in range(3)]
        return np.array([SWING_VMAX, *np.abs(np.array(v) / J)])

    def _hydraulics(self):
        """Levers -> joint speed commands and torque limits for this frame, from the cylinders."""
        self.cyl_pts, self.cyl_L, self.cyl_J = self.linkage.state(self.q)
        spool = self.levers * LEVER_SIGN  # desired joint direction and opening
        omega, v_cyl = hyd.speeds(spool, self.cyl_J)  # swing and cylinders share the pumps
        J = np.where(np.abs(self.cyl_J) < hyd.J_MIN, np.copysign(hyd.J_MIN, self.cyl_J), self.cyl_J)
        v_cmd = np.concatenate([[omega], v_cyl / J])
        lo, hi = hyd.torque_limits(self.cyl_J, hyd.P_RELIEF * self.relief, spool[1:])
        tau = SWING_TAU * self.relief
        self.v_cmd.assign(v_cmd.astype(np.float32))
        self.tau_lo.assign(np.concatenate([[-tau], lo]).astype(np.float32))
        self.tau_hi.assign(np.concatenate([[tau], hi]).astype(np.float32))

    def part_poses(self):
        """Zone-frame poses (xyz, quaternion xyzw) of the cylinder barrels, rods and linkage plates."""
        psi = self.q[0]
        pts = {k: house_to_zone(psi, v) for k, v in self.cyl_pts.items()}
        axis_y = swing_vec(psi, np.array([0.0, -1.0, 0.0]))  # the house's +y (pin axes)
        frames = hyd.part_frames(pts, axis_y)
        return [(frames[name][0], quat_from_matrix(frames[name][1])) for name in PARTS]

    # -------------------------------------------------------------- stepping
    def _rigid_substeps(self):
        # even substep count: state_0/state_1 end where they started, so a captured graph stays valid
        for _ in range(RIGID_SUBSTEPS):
            self.state_0.clear_forces()
            wp.launch(compute_body_forces, dim=self.imp_ids.shape[0], inputs=[self.mpm_dt, self.imp_ids, self.imp, self.imp_pos, self.collider_body, self.state_0.body_q, self.model.body_com, self.state_0.body_f])
            self.sand_forces.assign(self.state_0.body_f)
            wp.launch(
                hydraulics_kernel,
                dim=4,
                inputs=[self.state_0.joint_q, self.state_0.joint_qd, self.v_cmd, self.q_ref, self.kp, self.kd, self.tau_lo, self.tau_hi, self.q_lo, self.q_hi, self.sim_dt],
                outputs=[self.control.joint_f, self.torque],
            )
            self.collision.collide(self.state_0, self.contacts)
            self.solver.step(self.state_0, self.state_1, self.control, self.contacts, self.sim_dt)
            self.state_0, self.state_1 = self.state_1, self.state_0

    @property
    def dumper(self):
        """The carrier serving the excavator: the one at the loading spot, else the next one due there."""
        f = self.fleet
        return f.at_spot or next((c for c in f if c.stage in ("reverse in", "from queue", "turn to load", "wait to load")), f.carriers[0])

    def dispatch(self):
        """Send the carrier at the loading spot off with its load (once the bucket is clear of it)."""
        if self.fleet.at_spot is not None:
            self.dispatch_pending = True

    def _haul(self):
        """Carriers: move the vessels, and hand each load over to and from the soil zone."""
        f = self.fleet
        f.gates(self.dozer.parked or not self.use_dozer)
        spot = f.at_spot
        if self.dispatch_pending and spot is not None and (not self.auto or self.stage_name not in ("swing to truck", "dump")) and self.q[0] > math.radians(-70):
            self.dispatch_pending = False
            spot.depart_requested = True
        q, qd = self.state_0.joint_q.numpy(), self.state_0.joint_qd.numpy()
        for d in f:
            if d.update(self.frame_dt) == "depart":
                self._take_load(d)
            if d.stage in ("tip", "dump") and d.counts is not None:
                self._release_load(d)
            if d.stage == "dump" and self.zone_frame % 15 == 0:
                d.residual_kg = self._in_vessel(d) * self.particle_mass  # what still sticks to the vessel
            if d.stage == "lower" and d.carry_counted != d.loads:  # once per trip, as the vessel comes down
                d.carry_counted = d.loads
                d.residual_kg = self._in_vessel(d) * self.particle_mass
                d.delivered_kg -= d.residual_kg  # carried back, not delivered
                if d.trip is not None:
                    d.trip["carry_back_kg"] = round(d.residual_kg)
            if d.parked and d.counts is None and self.zones.col_floor[self.zones.region_mask("truck")][0] < 0:
                self.zones.set_region_floor("truck", self.truck_floor)  # a carrier is back at the loading spot
            p, R, v, w = d.vessel_pose()
            q[d.vq:d.vq + 7] = [*p, *d.vessel_quat()]
            qd[d.vqd:d.vqd + 6] = [*v, *w]
        self.state_0.joint_q.assign(q)
        self.state_0.joint_qd.assign(qd)

    def _doze(self):
        """Dozer: spread what the carrier dumps; it works only while the carrier is away from the dump point,
        and the carrier waits at its turn point until the dozer has parked."""
        dz = self.dozer
        if self.zone_frame % 3 == 0:
            self._spoil_surface(self.doze_hm, DOZER_SEES_SPEED)
            self.doze_hm_np = self.doze_hm.numpy()
        may_work = self.fleet.dump_area_free()
        self.blade_force = self.sand_forces.numpy()[self.blade][:3].astype(float)
        dz.update(self.frame_dt, self._surface, self.blade_force, may_work)
        p, R = dz.blade_pose()
        q, qd = self.state_0.joint_q.numpy(), self.state_0.joint_qd.numpy()
        q[self.blade_q:self.blade_q + 7] = [*p, *dz.blade_quat()]
        qd[self.blade_qd:self.blade_qd + 6] = [*dz.v, *dz.w]
        self.state_0.joint_q.assign(q)
        self.state_0.joint_qd.assign(qd)

    def _spoil_surface(self, hm, max_speed):
        """Spoil-area heightmap: the dormant columns and the particles slower than max_speed (-0.95 where
        there is no soil)."""
        hm.fill_(-0.95)
        wp.launch(heightmap_kernel, dim=self.n_particles, inputs=[self.sand.particle_q, self.sand.particle_qd, max_speed, self.radius, SPOIL_HM[0], SPOIL_HM[1], HM_CELL, SPOIL_HM[2], SPOIL_HM[3], hm])
        self.zones.merge_heightmap(hm, SPOIL_HM[0], SPOIL_HM[1], HM_CELL, SPOIL_HM[2], SPOIL_HM[3])

    def _surface(self, xy):
        """Ground and soil height at points xy (n, 2): the spoil heightmap where it covers them, else the ground."""
        x0, y0, nx, ny = SPOIL_HM
        ix = np.floor((xy[:, 0] - x0) / HM_CELL).astype(int)
        iy = np.floor((xy[:, 1] - y0) / HM_CELL).astype(int)
        inside = (ix >= 0) & (ix < nx) & (iy >= 0) & (iy < ny)
        h = np.full(len(xy), GROUND_Z)
        h[inside] = np.maximum(self.doze_hm_np[iy[inside] * nx + ix[inside]], GROUND_Z)
        return h

    def _in_vessel(self, d):
        """Active particles inside carrier d's vessel (count)."""
        act = np.flatnonzero(self.zones.active_np)
        loc = self._vessel_local(d, self.sand.particle_q.numpy()[act])
        return int(((np.abs(loc[:, 0]) < BED_L / 2 + 0.05) & (np.abs(loc[:, 1]) < BED_W / 2 + 0.05) & (loc[:, 2] > -0.1) & (loc[:, 2] < BED_H + 1.5)).sum())

    @staticmethod
    def _vessel_local(d, pts):
        p, R, _, _ = d.vessel_pose()
        return (pts - p) @ R

    def _take_load(self, d):
        """Carrier d leaves the loading spot: its load (dormant columns at the loading spot and particles in the vessel)
        becomes counts per vessel cell, carried until it is dumped; the loading spot's region is switched
        off (nothing to rest on there now)."""
        z = self.zones
        x0, y0, nx, ny, _ = z.region_grid("truck")
        counts = z.take_region("truck")
        act = np.flatnonzero(z.active_np)
        loc = self._vessel_local(d, self.sand.particle_q.numpy()[act])
        inside = (np.abs(loc[:, 0]) < BED_L / 2 + 0.05) & (np.abs(loc[:, 1]) < BED_W / 2 + 0.05) & (loc[:, 2] > -0.1) & (loc[:, 2] < BED_H + 1.5)
        ix = np.clip(((loc[inside, 0] + BED_L / 2) / CELL).astype(int), 0, nx - 1)
        iy = np.clip(((loc[inside, 1] + BED_W / 2) / CELL).astype(int), 0, ny - 1)
        np.add.at(counts, (iy, ix), 1)
        z.remove(act[inside])
        z.set_region_floor("truck", -50.0)
        d.counts = counts
        d.payload_kg = d.load_kg = float(counts.sum()) * self.particle_mass
        d.loads += 1

    TAIL_LEN, TAIL_ANGLE = 0.6, math.radians(25.0)  # the vessel floor's rising rear end
    WAIT_POSE = ("wait for carrier", -50, 5.6, 6.5, -165)  # where the full bucket waits for an empty carrier
    RELEASE_FROM, RELEASE_TO = math.radians(25.0), math.radians(55.0)  # tip over which the load slides out
    RELEASE_PER_FRAME = 6000  # particles at most per frame (the pool and the MPM step stay bounded)

    def _release_load(self, d):
        """Tipping: the load leaves as particles, the columns at the rear (tail, -x in the vessel) first,
        moving with the vessel; they then slide out under MPM physics."""
        counts = d.counts
        frac = np.clip((d.tip - self.RELEASE_FROM) / (self.RELEASE_TO - self.RELEASE_FROM), 0.0, 1.0)
        if frac <= 0.0:
            return
        ny, nx = counts.shape
        cols = int(math.ceil(frac * nx))  # columns (along x, from the tail) due by now
        p, R, v, w = d.vessel_pose()
        pos = []
        budget = self.RELEASE_PER_FRAME
        for ix in range(cols):
            for iy in np.flatnonzero(counts[:, ix]):
                n = int(counts[iy, ix])
                k = np.arange(n)
                m = k % PER_LAYER
                x = -BED_L / 2 + (ix + 0.5) * CELL + ((m % 2) - 0.5) * SPACING
                y = -BED_W / 2 + (iy + 0.5) * CELL + ((m // 2) - 0.5) * SPACING
                zz = (k // PER_LAYER + 0.5) * SPACING * (1.0 + self.swell)  # the load is loose
                pos.append(np.stack([x, y, zz], axis=1))
                counts[iy, ix] = 0
                budget -= n
                if budget <= 0:
                    break
            if budget <= 0:
                break
        if pos:
            local = np.concatenate(pos)
            world = p + local @ R.T
            vel = v + np.cross(w, world - p)
            added, slots = self.zones.inject(world, vel)
            if added and self.bond is not None:  # dug and carried soil is broken up: remoulded, not intact
                wp.launch(set_bond_kernel, dim=added, inputs=[slots, self.bond[3], self.bond_state])
            if added < len(world):  # pool full: keep the rest in the vessel for later frames
                rest = local[added:]
                ix = np.clip(((rest[:, 0] + BED_L / 2) / CELL).astype(int), 0, nx - 1)
                iy = np.clip(((rest[:, 1] + BED_W / 2) / CELL).astype(int), 0, ny - 1)
                np.add.at(counts, (iy, ix), 1)
        remaining = float(counts.sum()) * self.particle_mass
        d.delivered_kg += d.payload_kg - remaining
        d.payload_kg = remaining
        if remaining <= 0.0:
            d.counts = None

    def step(self):
        if self.haul:
            self._haul()
        if self.use_dozer:
            self._doze()
        if self.auto:
            self._auto_control()
        self._hydraulics()

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

        if self.zone_frame % ZONE_EVERY == 0:
            self.zones.update([self.bucket_box(), self.dozer.blade_box()] if self.use_dozer else self.bucket_box())
        self.zone_frame += 1
        if self.zone_frame % self.mpm_every == 0:
            # the rigid frames in between reuse the last sand impulses as a constant force
            wp.launch(subtract_body_force, dim=self.model.body_count, inputs=[self.mpm_dt, self.state_0.body_q, self.state_0.body_qd, self.sand_forces, self.model.body_inv_inertia, self.model.body_inv_mass, self.sand.body_q, self.sand.body_qd])
            self.mpm.step(self.sand, self.sand, contacts=None, control=None, dt=self.mpm_dt)
            if self.bond is not None:
                wp.launch(soil_bond_kernel, dim=self.n_particles, inputs=[self.sand.mpm.particle_Jp, *self.bond, self.bond_state, self.sand_model.mpm.yield_stress, self.sand_model.mpm.tensile_yield_ratio])
            self._collect_impulses()
        wp.synchronize()
        ms = (time.perf_counter() - t0) * 1000
        self.step_ms = ms if self.sim_time == 0 else 0.9 * self.step_ms + 0.1 * ms
        self.sim_time += self.frame_dt

        qd_prev = self.qd
        self.q = self.state_0.joint_q.numpy()[:4].astype(float)
        self.qd = self.state_0.joint_qd.numpy()[:4].astype(float)
        self.tau = self.torque.numpy().astype(float)
        # house inertia about the swing axis, measured while the swing runs at relief (for the auto operator)
        acc = (self.qd[0] - qd_prev[0]) / self.frame_dt
        if abs(self.tau[0]) > 0.95 * SWING_TAU * self.relief and abs(acc) > 0.3 and acc * self.tau[0] > 0:
            self.swing_inertia += 0.1 * (abs(self.tau[0] / acc) - self.swing_inertia)
        self.cyl_p, self.cyl_head = hyd.pressures(self.tau[1:], self.cyl_J)
        # gauges: loaded-side pressure as a fraction of the nominal relief setting
        self.press = np.concatenate([[abs(self.tau[0]) / SWING_TAU], self.cyl_p / hyd.P_RELIEF])  # swing: of its 30.4 MPa relief

    BUCKET_LOCAL_BOX = (
        np.array([min(BUCKET_SHELL[:, 0].min(), TOOTH_TIP[0]) - PLATE, -BUCKET_W / 2 - PLATE, min(BUCKET_SHELL[:, 1].min(), TOOTH_TIP[1]) - PLATE]),
        np.array([max(BUCKET_SHELL[:, 0].max(), TOOTH_TIP[0]) + PLATE, BUCKET_W / 2 + PLATE, max(BUCKET_SHELL[:, 1].max(), TOOTH_TIP[1]) + PLATE]),
    )

    def bucket_box(self):
        """World AABB of the bucket (zone frame), for the soil zone."""
        bq = self.state_0.body_q.numpy()[self.links["bucket"]]
        lo, hi = self.BUCKET_LOCAL_BOX
        corners = np.array([[x, y, z] for x in (lo[0], hi[0]) for y in (lo[1], hi[1]) for z in (lo[2], hi[2])])
        x, y, z, w = bq[3:]
        R = np.array([
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ])
        pts = corners @ R.T + bq[:3]
        return pts.min(axis=0), pts.max(axis=0)

    # -------------------------------------------------------------- outputs
    def bucket_xform(self):
        bq = self.state_0.body_q.numpy()[self.links["bucket"]]
        return wp.transform(wp.vec3(*bq[:3]), wp.quat(*bq[3:]))

    def measure(self):
        sf = self.sand_forces.numpy()[self.links["bucket"]]
        self.force = sf[:3].astype(float)  # sand force on the bucket (world, zone frame)
        q = self.sand.particle_q
        self.hm.fill_(0.0)
        wp.launch(heightmap_kernel, dim=self.n_particles, inputs=[q, self.sand.particle_qd, SURFACE_MAX_SPEED, self.radius, HM_X0, HM_Y0, HM_CELL, HM_NX, HM_NY, self.hm])
        self.counter.zero_()
        self.zones.merge_heightmap(self.hm, HM_X0, HM_Y0, HM_CELL, HM_NX, HM_NY)
        wp.launch(stream_kernel, dim=self.n_particles, inputs=[q, self.sand.particle_qd, self.sand_model.particle_flags, MOVING_SPEED, wp.vec3(HM_X0, HM_Y0, 0.0), wp.vec3(HM_X0 + HM_NX * HM_CELL, HM_Y0 + HM_NY * HM_CELL, 0.0), wp.vec3(*BED_LO), wp.vec3(*BED_HI), MAX_SEND_PARTICLES, self.counter, self.moving])
        self.zones.stream_dormant(MAX_SEND_PARTICLES, self.counter, self.moving)
        # truck: load inside the bed, and the surface of the resting load
        self.truck_counter.zero_()
        wp.launch(bucket_fill_kernel, dim=self.n_particles, inputs=[q, wp.transform_identity(), wp.vec3(*BED_LO), wp.vec3(*BED_HI), self.truck_counter])
        self.truck_hm.fill_(BED_FLOOR_Z)
        wp.launch(heightmap_kernel, dim=self.n_particles, inputs=[q, self.sand.particle_qd, SURFACE_MAX_SPEED, self.radius, float(BED_LO[0]), float(BED_LO[1]), TRUCK_CELL, TRUCK_NX, TRUCK_NY, self.truck_hm])
        self.zones.merge_heightmap(self.truck_hm, float(BED_LO[0]), float(BED_LO[1]), TRUCK_CELL, TRUCK_NX, TRUCK_NY)
        self.fill_counter.zero_()
        # inside the shell profile, up to the mouth (struck load)
        wp.launch(bucket_profile_fill_kernel, dim=self.n_particles, inputs=[q, self.bucket_xform(), self.bucket_poly, BUCKET_W / 2, self.fill_counter])
        self.hm_np = self.hm.numpy()
        n_moving = min(int(self.counter.numpy()[0]), MAX_SEND_PARTICLES)
        moving = self.moving.numpy()[:n_moving]
        self.fill_kg = float(self.fill_counter.numpy()[0]) * self.particle_mass
        in_vessel = float(self.truck_counter.numpy()[0] + self.zones.dormant_in("truck")) * self.particle_mass
        # the soil in the vessel at the loading spot, plus the loads carried by the carriers away from it
        spot = self.fleet.at_spot
        self.spot_kg = in_vessel if spot is not None else 0.0
        self.truck_kg = self.spot_kg + sum(c.payload_kg for c in self.fleet if not c.parked)
        owner = self._load_owner()
        self.truck_hm_np = self.truck_hm.numpy() if owner is spot else self._carried_heightmap(owner)
        self._spoil_surface(self.spoil_hm, SURFACE_MAX_SPEED)  # no spoil: below the ground (the browser hides it)
        self.spoil_hm_np = self.spoil_hm.numpy()
        _, _, edge = fk(self.q)
        ix = int((edge[0] - HM_X0) / HM_CELL)
        iy = int((edge[1] - HM_Y0) / HM_CELL)
        ix_ahead = max(ix - 3, 0)  # the arm drags toward +x, so uncut ground is at lower x
        surface = float(self.hm_np[iy * HM_NX + ix_ahead]) if 0 <= ix < HM_NX and 0 <= iy < HM_NY else 0.0
        surface = min(surface, BED_TOP)
        self.cut_depth = max(surface - edge[2], 0.0)
        d = self.cut_depth
        self.fee_ref = (self.density * G * d * d * self.n_gamma + self.cohesion * d * self.n_c) * BUCKET_W * self.fee_scale
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
        out = {
            "house": [box(*bx) for bx in self.house_boxes],
            "boom": [box(*bx) for bx in self.boom_boxes],
            "stick": [box(*bx) for bx in self.stick_boxes],
            "bucket": [box(*bx) for bx in self.bucket_boxes],
            # the real machine: the browser puts each link's meshes from this glTF at the streamed link pose
            "model": {
                "file": "assets/machinery/opera_zx200.glb",
                "links": GLB_NODES,
                "parts": list(PARTS),
                "crawler": {"node": "track_link", "pos": list(self.to_three(SWING_POS - swing_vec(CRAWLER_YAW, CRAWLER_SWING))), "quat": list(self.quat_to_three(rot_z(CRAWLER_YAW)))},
            },
            # static parts in world (three.js) coordinates
            "tracks": [world_box((SWING_POS[0] + side * 1.2, 0, g + 0.45), (0.35, 2.3, 0.45)) for side in (-1, 1)]  # along y, as CRAWLER_YAW
            + [world_box((SWING_POS[0], 0, g + 0.75), (0.9, 1.4, 0.25))],
            # the crawler carrier moves (frame trailer "CAR1"): drawn from its glTF, or from these boxes
            "truck_bed": [],
            "truck_body": [],
            "truck_wheels": [],
            "carrier": {
                "file": "assets/machinery/opera_ic120.glb",
                "vessel_node": "vessel_pin_link",
                "hide_nodes": ["cylinder_head_link", "cylinder_rod_link"],  # the tipping cylinder (not posed)
                "vessel_boxes": [box(np.asarray(c) + dumper_mod.FLOOR_FROM_PIN, h, *a) for c, h, *a in self.vessel_boxes],  # pin frame
                "body_boxes": [box(c, h) for c, h in [((0.2, s * 1.0, 0.45), (3.1, 0.3, 0.45)) for s in (-1, 1)] + [((-0.3, 0.0, 0.85), (2.9, 1.1, 0.15)), ((2.2, -0.2, 1.9), (1.2, 1.1, 1.0))]],
                "parked_pin": self._carrier_three(dumper_mod.Dumper(BED_CENTER, GROUND_Z, HAUL_DUMP_X, HAUL_TURN_X, TRUCK_CAPACITY_KG))[1],
                "count": len(self.fleet),
                "stages": list(CARRIER_STAGES),
            },
            "truck_capacity_kg": TRUCK_CAPACITY_KG,
        }
        if self.use_dozer:  # the dozer moves (frame trailer "DOZ1"): its glTF, or these boxes, in URDF base_link frames
            out["dozer"] = {
                "file": "assets/machinery/opera_d37pxi.glb",
                "arm_node": "blade_arm_link", "cylinder_node": "blade_arm_cylinder_link", "piston_node": "blade_arm_piston_link",
                "arm_pivot": dozer_mod.ARM_PIVOT.tolist(), "blade_on_arm": dozer_mod.BLADE_ON_ARM.tolist(),
                "body_boxes": [{"center": c, "size": s} for c, s in [((0.0, s * 0.8, 0.35), (2.6, 0.5, 0.7)) for s in (-1, 1)] + [((-0.2, 0.0, 1.1), (2.6, 1.6, 0.9)), ((-0.6, 0.0, 2.2), (1.3, 1.4, 1.2))]],
                "blade_box": {"center": [0.2, 0.0, 0.25], "size": [0.2, dozer_mod.BLADE_W, 1.09]},
                "stages": dozer_mod.STAGES,
            }
        return out

    def hello(self):
        """First message to a client: machine, limits and scene geometry."""
        return {"type": "hello", "kind": "excavator", "particles": self.n_particles, "zone": True, "voxel": self.voxel, "joints": JOINTS, "limits": LIMITS, "vmax": dict(zip(JOINTS, self.vmax.round(3).tolist())), "relief_mpa": hyd.P_RELIEF / 1e6, "geometry": self.geometry(), "stages": [c[0] for c in self.CYCLE],
                "soil": {"name": self.soil, **soils.SOILS[self.soil], "options": [{"name": k, "label": v["label"]} for k, v in soils.SOILS.items()]}}

    def frame_bytes(self, frame_no, moving, parts=False):
        """One stream frame. parts=True returns it in three pieces (fixed-size head, grains, tail) for the
        replay recorder, which stores the head and tail as differences to the previous frame."""
        bq = self.state_0.body_q.numpy()
        links = b"".join(struct.pack("<3f4f", *self.to_three(bq[i][:3]), *self.quat_to_three(bq[i][3:])) for i in (self.links[name] for name in LINKS))
        f = self.force
        header = struct.pack(
            "<4sIfffI",
            b"EXC3", frame_no, self.sim_time, self.step_ms, min(self.frame_dt * 1000 / max(self.step_ms, 1e-3), 99.0), self.zones.n_active,
        )
        arm = struct.pack(
            "<4f4f4f3f6fB",
            *self.q, *self.qd, *self.press, f[0], f[2], -f[1],
            self.fill_kg, self.cut_depth, self.fee_ref, self.relief, self.truck_kg, TRUCK_CAPACITY_KG,
            self.auto_stage if self.auto else 255,
        )
        ox, oy, oz = PIT_CENTER_THREE
        hm = np.clip((self.hm_np + 1.0) * 1000.0, 0, 65535).astype(np.uint16)
        hm_header = struct.pack("<HHfff", HM_NX, HM_NY, ox + HM_X0 + HM_CELL / 2, oz - HM_Y0 - HM_CELL / 2, HM_CELL)
        thm = np.clip((self.truck_hm_np + 1.0) * 1000.0, 0, 65535).astype(np.uint16)
        thm_header = struct.pack("<HHfff", TRUCK_NX, TRUCK_NY, ox + BED_LO[0] + TRUCK_CELL / 2, oz - BED_LO[1] - TRUCK_CELL / 2, TRUCK_CELL)
        lo = np.array([ox - 4.0, oy - 0.5, oz - 12.0], dtype=np.float32)  # covers the pit, machine, haul road and spoil
        span = 40.0
        if len(moving):
            t3 = np.stack([moving[:, 0] + ox, moving[:, 2] + oy, oz - moving[:, 1]], axis=1)
            qp = np.clip((t3 - lo) / span * 65535.0, 0, 65535).astype(np.uint16)
        else:
            qp = np.zeros((0, 3), dtype=np.uint16)
        p_header = struct.pack("<I4f", len(qp), *lo, span)
        body = header + links + arm
        body += b"\0" * ((-len(body)) % 4)  # keep the heightmap 4-byte aligned
        # cylinders: barrel, rod and linkage plate poses, then loaded-side pressure (bar, + head side)
        parts = self.part_poses()
        body += struct.pack("<I", len(parts)) + b"".join(struct.pack("<3f4f", *self.to_three(p), *self.quat_to_three(qt)) for p, qt in parts)
        body += struct.pack("<3f", *(np.where(self.cyl_head, 1.0, -1.0) * self.cyl_p / 1e5))
        # trailer "CARS": the carriers (base_link and vessel pin poses, tip, payload, delivered, stage), which
        # of them the truck heightmap belongs to, then the spoil heightmap
        shm = np.clip((self.spoil_hm_np + 1.0) * 1000.0, 0, 65535).astype(np.uint16)
        shm_header = struct.pack("<HHfff", SPOIL_HM[2], SPOIL_HM[3], ox + SPOIL_HM[0] + HM_CELL / 2, oz - SPOIL_HM[1] - HM_CELL / 2, HM_CELL)
        owner = self._load_owner()
        car = struct.pack("<4sIi", b"CARS", len(self.fleet), -1 if owner is None else owner.index)
        for d in self.fleet:
            base, pin = self._carrier_three(d)
            car += struct.pack("<7f7f3fI", *base, *pin, d.tip, d.payload_kg, d.delivered_kg, CARRIER_STAGES.index(d.stage))
        head = body + hm_header + hm.tobytes() + thm_header + thm.tobytes()
        grains = p_header + qp.tobytes()
        tail = car + shm_header + shm.tobytes()
        if self.use_dozer:  # trailer "DOZ1": the dozer's base_link pose, lift angle, blade force (kN), stage
            dz = self.dozer
            tail += struct.pack("<4s7fffI", b"DOZ1", *self.to_three(dz.base_pos()), *self.quat_to_three(dz.base_quat()), dz.lift, dz.force / 1000, dozer_mod.STAGES.index(dz.stage))
        return (head, grains, tail) if parts else head + grains + tail

    def _load_owner(self):
        """The carrier whose load the truck heightmap shows: the one at the loading spot, else the one
        carrying a load (the most recent to leave), else None."""
        if self.fleet.at_spot is not None:
            return self.fleet.at_spot
        away = [c for c in self.fleet if c.counts is not None]
        return max(away, key=lambda c: c.trip["depart_t"] if c.trip else 0.0) if away else None

    def _carrier_three(self, d):
        """Carrier d's base_link and vessel pin poses in three.js coordinates ((pos, quat), (pos, quat))."""
        base = np.array([d.base[0], d.base[1], GROUND_Z])
        bq = dumper_mod.yaw_quat(d.yaw)
        p, R, _, _ = d.vessel_pose()
        pin = p - R @ dumper_mod.FLOOR_FROM_PIN
        return (list(self.to_three(base)) + list(self.quat_to_three(bq)), list(self.to_three(pin)) + list(self.quat_to_three(d.vessel_quat())))

    def _carried_heightmap(self, d):
        """Carrier d's carried load as the truck heightmap, in the vessel's parked frame (the browser moves it with
        the vessel): heights from the particle counts per vessel cell."""
        z = self.zones
        x0, y0, nx, ny, _ = z.region_grid("truck")
        hm = np.full(TRUCK_NX * TRUCK_NY, BED_FLOOR_Z, dtype=np.float32)
        if d is None or d.counts is None:
            return hm
        gx = BED_LO[0] + (np.arange(TRUCK_NX) + 0.5) * TRUCK_CELL
        gy = BED_LO[1] + (np.arange(TRUCK_NY) + 0.5) * TRUCK_CELL
        ix = np.clip(((gx - x0) / CELL).astype(int), 0, nx - 1)
        iy = np.clip(((gy - y0) / CELL).astype(int), 0, ny - 1)
        h = BED_FLOOR_Z + d.counts[np.ix_(iy, ix)] * LAYER_PER_PARTICLE * (1.0 + self.swell)  # loose
        return h.astype(np.float32).ravel()

    def stats(self):
        return {
            "t": round(self.sim_time, 2),
            "stage": self.stage_name,
            "soil": self.soil,
            "step_ms": round(self.step_ms, 2),
            "q_deg": [round(math.degrees(v), 1) for v in self.q],
            "qd": [round(v, 3) for v in self.qd],
            "pressure": [round(v, 2) for v in self.press],
            "cyl_bar": [round(float(v) / 1e5 * (1 if h else -1)) for v, h in zip(self.cyl_p, self.cyl_head)],
            "swing_bar": round(float(self.tau[0]) / hyd.SWING_DISP / 1e5),  # signed by torque direction
            "swing_inertia": round(self.swing_inertia),
            "swing_held_s": round(self.swing_held, 1),
            "arm_held_s": round(self.arm_held, 1),
            "shakes": self.shakes,
            "cyl_m": [round(float(v), 3) for v in self.cyl_L],
            "force_kN": [round(v / 1000, 1) for v in self.force],
            "fill_kg": round(self.fill_kg),
            "cut_m": round(self.cut_depth, 3),
            "mckyes_kN": round(self.fee_ref / 1000, 1),
            "edge": [round(v, 2) for v in fk(self.q)[2]],
            "truck_kg": round(self.truck_kg),
            "carrier": {"stage": self.dumper.stage, "payload_kg": round(self.dumper.payload_kg), "delivered_kg": round(self.fleet.delivered_kg),
                        "loads": sum(c.loads for c in self.fleet), "trips": self.fleet.trips[-3:], "excavator_wait_s": round(self.truck_wait, 1),
                        "stages": [c.stage for c in self.fleet]},
            "active": self.zones.n_active,
            "ease_m": round(self.ease, 2),
            "early_curls": self.early_curls,
            "payload_kg": round(self.payload_kg),
            "curl_gain_kg": round(self.curl_gain),
            "zone": dict(self.zones.counters),
            "face": self.face_log[-1] if self.face_log else None,
            "pit_done": self.pit_done,
        }


# ------------------------------------------------------------------ server
async def serve(sim, host, port):
    import websockets

    clients = set()
    pending_soil = []  # a soil change asked for by a client: the pit is rebuilt between frames

    async def handler(ws):
        clients.add(ws)
        await ws.send(json.dumps(sim.hello()))
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
                    sim.ease, sim.seg_from, sim.shaking = 0.0, None, None
                    if not sim.auto:
                        sim.set_levers(0, 0, 0, 0)
                elif kind == "dispatch":
                    sim.dispatch()  # send the carrier off with whatever it holds
                elif kind == "relief":
                    sim.relief = float(np.clip(cmd.get("value", 1.0), 0.1, 1.5))
                elif kind == "reset":
                    sim.reset()
                elif kind == "soil" and cmd.get("soil") in soils.SOILS:
                    pending_soil.append(cmd["soil"])
        finally:
            clients.discard(ws)

    async def sim_loop():
        nonlocal sim
        frame = 0
        next_send = 0.0
        wall0 = time.perf_counter()
        while True:
            if pending_soil:
                soil = pending_soil[-1]
                pending_soil.clear()
                if soil != sim.soil:
                    print(f"switching soil to {soil}", flush=True)
                    auto, relief = sim.auto, sim.relief
                    sim = ExcavatorSim(**{**sim.kwargs, "soil": soil})
                    sim.auto, sim.relief = auto, relief  # the operator's settings carry over to the new pit
                    next_send = 0.0
                    wall0 = time.perf_counter()
                    hello = json.dumps(sim.hello())
                    for ws in list(clients):
                        try:
                            await ws.send(hello)
                        except Exception:  # noqa: BLE001
                            clients.discard(ws)
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
    ap.add_argument("--iterations", type=int, default=None, help="MPM solver iterations (default 150 at 30 Hz, 60 at 60 Hz); too few let carried sand creep out of the bucket")
    ap.add_argument("--bench", type=float, default=None)
    ap.add_argument("--hold-test", action="store_true")
    ap.add_argument("--stall-test", action="store_true")
    ap.add_argument("--out", default=None)
    ap.add_argument("--trench", action="store_true", help="auto cycle digs one lane, deeper each pass")
    ap.add_argument("--pool", type=int, default=POOL, help="particle capacity of the soil zone")
    ap.add_argument("--mpm-every", type=int, default=MPM_EVERY, help="frames per MPM step (1: sand at 60 Hz)")
    ap.add_argument("--soil", choices=list(soils.SOILS), default=soils.DEFAULT_SOIL, help="soil preset (soils.py)")
    ap.add_argument("--carriers", type=int, default=1, choices=(1, 2), help="crawler carriers in the haul cycle")
    args = ap.parse_args()
    sim = ExcavatorSim(voxel=args.voxel, iterations=args.iterations, pool=args.pool, mpm_every=args.mpm_every, soil=args.soil, carriers=args.carriers)
    if args.trench:
        sim.set_trench()
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
