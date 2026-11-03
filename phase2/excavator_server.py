"""Phase 2b: a joint-driven excavator arm digging Newton MPM sand, streamed to the browser.

Replaces the kinematic bucket of soil_server.py with a 3-joint backhoe arm (boom, stick, bucket)
of a ~20 t excavator standing at the pit edge. Each joint is driven by a hydraulic-style actuator:

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
GROUND_Z = 2.35  # site ground next to the pit, zone frame
SWING_POS = np.array([7.0, 0.0, 3.35])  # swing bearing (upper structure pivot) on the undercarriage
BOOM_FOOT = np.array([-0.5, 0.0, 0.75])  # boom foot pin in the house frame -> 4.1 m above the pit floor
BASE = SWING_POS + BOOM_FOOT  # boom foot pin at swing angle 0
L_BOOM, L_STICK = 5.7, 2.9
BUCKET_D, BUCKET_H, BUCKET_W, PLATE = 1.0, 0.85, 1.3, 0.10  # outer depth along stick, mouth opening, width;
# plates are thicker than real steel so the 8 cm MPM grid can't leak sand through them. Inside: 0.9 x 1.2 x 0.8 m
# = 0.86 m3 struck (~1.38 t of sand at 1600 kg/m3), the rating of a 20 t excavator's general-purpose bucket
BUCKET_RATED_KG = 0.86 * 1600.0
MASS = {"house": 8000.0, "boom": 1800.0, "stick": 900.0, "bucket": 800.0}
JOINTS = ["swing", "boom", "stick", "bucket"]
LINKS = ["house", "boom", "stick", "bucket"]
# limits (rad), max speed (rad/s), relief torque (N m), servo gains
LIMITS = {"swing": (-3.0, 3.0), "boom": (-1.0, 1.2), "stick": (-2.7, -0.5), "bucket": (-2.3, 1.4)}
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


def fk(q):
    """FK for q = [swing, boom, stick, bucket]: stick pin, bucket pin and cutting edge (zone frame).
    The arm points outward (-x at swing 0); boom/stick/bucket angles are about the house +y axis and
    positive angles raise the link; positive swing turns the arm toward -y (the operator's left)."""
    psi = q[0]
    a1 = q[1]
    a2 = q[1] + q[2]
    a3 = q[1] + q[2] + q[3]
    foot = SWING_POS + swing_vec(psi, BOOM_FOOT)
    p1 = foot + swing_vec(psi, L_BOOM * np.array([-math.cos(a1), 0, math.sin(a1)]))
    p2 = p1 + swing_vec(psi, L_STICK * np.array([-math.cos(a2), 0, math.sin(a2)]))
    edge = p2 + swing_vec(psi, local_to_world(a3, np.array([-BUCKET_D, 0, -BUCKET_H])))
    return p1, p2, edge


def local_to_world(a, v):
    """Rotate a link-local vector by angle a about +y."""
    c, s = math.cos(a), math.sin(a)
    return np.array([v[0] * c + v[2] * s, v[1], -v[0] * s + v[2] * c])


def ik(psi, r, z, phi):
    """Joint angles [swing, boom, stick, bucket] that put the bucket pin at radius r (from the swing
    axis, along the arm) and height z, with absolute bucket angle phi, at swing angle psi."""
    dr, dz = r - (-BOOM_FOOT[0]), z - (SWING_POS[2] + BOOM_FOOT[2])
    d = min(math.hypot(dr, dz), L_BOOM + L_STICK - 1e-3)
    cos_q2 = (d * d - L_BOOM**2 - L_STICK**2) / (2 * L_BOOM * L_STICK)
    q2 = -math.acos(max(-1.0, min(1.0, cos_q2)))  # elbow up, stick folded under the boom
    target_ang = math.atan2(dz, dr)
    q1 = target_ang - math.atan2(L_STICK * math.sin(q2), L_BOOM + L_STICK * math.cos(q2))
    q3 = phi - q1 - q2
    return np.array([psi, q1, q2, q3])


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


# ------------------------------------------------------------------ simulation
class ExcavatorSim:
    # auto cycle: (stage, swing deg, bucket-pin radius from the swing axis, pin height, bucket angle deg
    # [, speed: fraction of full lever for moving into this stage])
    CYCLE = [
        ("approach", 0, 6.0, 2.6, -95),
        ("penetrate", 0, 7.9, 1.5, -60),
        ("drag", 0, 5.6, 1.1, -95),
        ("curl", 0, 5.2, 1.6, -175),
        ("lift", 0, 5.0, 6.6, -160),
        ("swing to truck", -90, 5.4, 6.6, -160),
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
        p1, p2, _ = fk(q0)
        a1, a2, a3 = q0[1], q0[1] + q0[2], q0[1] + q0[2] + q0[3]

        # upper structure (house, counterweight, cab): swings on the undercarriage; no contacts
        house = b.add_link(xform=wp.transform(wp.vec3(*SWING_POS)), mass=MASS["house"], com=wp.vec3(1.2, 0, 0.6), inertia=box_inertia(MASS["house"], 2.0, 1.3, 0.8))
        self.house_boxes = [((0.9, 0, 0.55), (1.8, 1.3, 0.55)), ((2.6, 0, 0.6), (0.5, 1.3, 0.5)), ((-0.2, 0.75, 1.35), (0.65, 0.5, 0.75))]
        for c, h in self.house_boxes:
            b.add_shape_box(house, xform=wp.transform(wp.vec3(*c)), hx=h[0], hy=h[1], hz=h[2], cfg=no_contact)
        boom = b.add_link(xform=wp.transform(wp.vec3(*BASE), rot_y(a1)), mass=MASS["boom"], com=wp.vec3(-L_BOOM / 2, 0, 0), inertia=box_inertia(MASS["boom"], L_BOOM / 2, 0.3, 0.35))
        b.add_shape_box(boom, xform=wp.transform(wp.vec3(-L_BOOM / 2, 0, 0)), hx=L_BOOM / 2, hy=0.3, hz=0.35, cfg=no_contact)
        stick = b.add_link(xform=wp.transform(wp.vec3(*p1), rot_y(a2)), mass=MASS["stick"], com=wp.vec3(-L_STICK / 2, 0, 0), inertia=box_inertia(MASS["stick"], L_STICK / 2, 0.2, 0.22))
        b.add_shape_box(stick, xform=wp.transform(wp.vec3(-L_STICK / 2, 0, 0)), hx=L_STICK / 2, hy=0.2, hz=0.22, cfg=stick_cfg)
        bucket = b.add_link(xform=wp.transform(wp.vec3(*p2), rot_y(a3)), mass=MASS["bucket"], com=wp.vec3(-BUCKET_D / 2, 0, -BUCKET_H / 2), inertia=box_inertia(MASS["bucket"], BUCKET_D / 2, BUCKET_W / 2, BUCKET_H / 2))
        D, H, W, T = BUCKET_D, BUCKET_H, BUCKET_W, PLATE
        self.bucket_boxes = [  # (center, half extents) in bucket frame: spine, floor, 2 sides, back
            ((-D / 2, 0, 0), (D / 2, W / 2, T / 2)),
            ((-D, 0, -H / 2), (T / 2, W / 2, H / 2)),
            ((-D / 2, W / 2, -H / 2), (D / 2, T / 2, H / 2)),
            ((-D / 2, -W / 2, -H / 2), (D / 2, T / 2, H / 2)),
            ((0, 0, -H / 2), (T / 2, W / 2, H / 2)),  # closed back: only the mouth is open
        ]
        for c, h in self.bucket_boxes:
            b.add_shape_box(bucket, xform=wp.transform(wp.vec3(*c)), hx=h[0], hy=h[1], hz=h[2], cfg=bucket_cfg)

        joint_kw = dict(axis=(0.0, 1.0, 0.0), armature=ARMATURE)
        j0 = b.add_joint_revolute(-1, house, parent_xform=wp.transform(wp.vec3(*SWING_POS)), axis=(0.0, 0.0, 1.0), armature=ARMATURE, limit_lower=LIMITS["swing"][0], limit_upper=LIMITS["swing"][1])
        j1 = b.add_joint_revolute(house, boom, parent_xform=wp.transform(wp.vec3(*BOOM_FOOT)), limit_lower=LIMITS["boom"][0], limit_upper=LIMITS["boom"][1], **joint_kw)
        j2 = b.add_joint_revolute(boom, stick, parent_xform=wp.transform(wp.vec3(-L_BOOM, 0, 0)), limit_lower=LIMITS["stick"][0], limit_upper=LIMITS["stick"][1], **joint_kw)
        j3 = b.add_joint_revolute(stick, bucket, parent_xform=wp.transform(wp.vec3(-L_STICK, 0, 0)), limit_lower=LIMITS["bucket"][0], limit_upper=LIMITS["bucket"][1], **joint_kw)
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

    DIG_STAGES = ("penetrate", "drag", "curl")
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
            _, pin, _ = fk(q)
            a3 = q[1] + q[2] + q[3]
            for x in (0.0, -BUCKET_D):
                for zz in (0.0, -BUCKET_H):
                    low = min(low, (pin + swing_vec(q[0], local_to_world(a3, np.array([x, 0.0, zz]))))[2])
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
        self.levers = np.clip(4.0 * err / self.vmax, -speed, speed)
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
        self.v_cmd.assign((self.levers * self.vmax).astype(np.float32))

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
        D, H, W, T = BUCKET_D, BUCKET_H, BUCKET_W, PLATE
        # inside faces of the plates; the open mouth is at z = -H
        wp.launch(bucket_fill_kernel, dim=self.n_particles, inputs=[q, self.bucket_xform(), wp.vec3(-D + T / 2, -W / 2 + T / 2, -H), wp.vec3(-T / 2, W / 2 - T / 2, -T / 2), self.fill_counter])
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
        def box(c, h):
            return {"center": [c[0], c[2], -c[1]], "size": [2 * h[0], 2 * h[2], 2 * h[1]]}

        def world_box(c, h):
            return {"center": list(self.to_three(c)), "size": [2 * h[0], 2 * h[2], 2 * h[1]]}

        cx, cy = BED_CENTER
        g = GROUND_Z
        return {
            "house": [box(c, h) for c, h in self.house_boxes] + [box((-0.5, 0, 0.75), (0.4, 0.45, 0.45))],
            "boom": [box((-L_BOOM / 2, 0, 0), (L_BOOM / 2, 0.3, 0.35))],
            "stick": [box((-L_STICK / 2, 0, 0), (L_STICK / 2, 0.2, 0.22))],
            "bucket": [box(c, h) for c, h in self.bucket_boxes],
            # static parts in world (three.js) coordinates
            "tracks": [world_box((SWING_POS[0] + 0.4, side * 1.2, g + 0.45), (2.3, 0.35, 0.45)) for side in (-1, 1)]
            + [world_box((SWING_POS[0] + 0.4, 0, g + 0.75), (1.4, 0.9, 0.25))],
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
