"""Phase 0 benchmark: a kinematic blade pushed through a sand bed with Newton's implicit MPM.

Measures wall-clock time per simulated frame, particle throughput, and the horizontal
resistance force on the blade, and compares that force with the McKyes/Reece
fundamental earthmoving equation for a vertical blade in cohesionless soil.
"""

import argparse
import json
import math
import time

import numpy as np
import warp as wp

import newton
from newton.solvers import SolverImplicitMPM

G = 9.81


def mckyes_force(rho, depth, width, phi, delta):
    """Passive resistance of a vertical blade (rake angle 90 deg), cohesionless, no surcharge."""
    alpha = math.pi / 2
    best = None
    for beta in np.linspace(math.radians(5), math.radians(85), 2000):
        denom = 2.0 * (math.cos(alpha + delta) + math.sin(alpha + delta) / math.tan(beta + phi))
        if denom <= 0:
            continue
        n_gamma = (1.0 / math.tan(alpha) + 1.0 / math.tan(beta)) / denom
        if best is None or n_gamma < best:
            best = n_gamma
    return rho * G * depth**2 * width * best, best


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--voxel", type=float, default=0.05)
    p.add_argument("--ppc", type=int, default=2, help="particles per voxel per axis")
    p.add_argument("--bed", type=float, nargs=3, default=[4.0, 2.0, 0.5], help="sand bed size x y z (m)")
    p.add_argument("--density", type=float, default=1600.0, help="bulk density kg/m3")
    p.add_argument("--friction", type=float, default=0.75, help="internal friction coefficient (tan phi)")
    p.add_argument("--blade-friction", type=float, default=0.4, help="soil-blade friction coefficient")
    p.add_argument("--depth", type=float, default=0.2, help="cutting depth (m)")
    p.add_argument("--width", type=float, default=1.0, help="blade width (m)")
    p.add_argument("--speed", type=float, default=0.5, help="blade speed (m/s)")
    p.add_argument("--fps", type=float, default=60.0)
    p.add_argument("--frames", type=int, default=150)
    p.add_argument("--iterations", type=int, default=50)
    p.add_argument("--out", type=str, default=None)
    args = p.parse_args()

    wp.init()
    dx = args.voxel
    bx, by, bz = args.bed

    # Rigid model: one kinematic blade
    builder = newton.ModelBuilder()
    blade_h = 0.8
    blade_t = 0.04
    x0 = -bx / 2 + 0.3
    blade_z = bz - args.depth + blade_h / 2
    blade = builder.add_body(
        xform=wp.transform(p=wp.vec3(x0, 0.0, blade_z), q=wp.quat_identity()),
        mass=500.0,
        is_kinematic=True,
    )
    builder.add_shape_box(blade, hx=blade_t / 2, hy=args.width / 2, hz=blade_h / 2)
    builder.add_ground_plane()
    model = builder.finalize()

    # Sand model
    sand_builder = newton.ModelBuilder()
    SolverImplicitMPM.register_custom_attributes(sand_builder)
    lo = np.array([-bx / 2, -by / 2, 0.0])
    hi = np.array([bx / 2, by / 2, bz])
    res = np.array(np.ceil(args.ppc * (hi - lo) / dx), dtype=int)
    cell = (hi - lo) / res
    radius = float(np.max(cell) * 0.5)
    mass = float(np.prod(cell) * args.density)
    sand_builder.add_particle_grid(
        pos=wp.vec3(lo),
        rot=wp.quat_identity(),
        vel=wp.vec3(0.0),
        dim_x=int(res[0]) + 1,
        dim_y=int(res[1]) + 1,
        dim_z=int(res[2]) + 1,
        cell_x=float(cell[0]),
        cell_y=float(cell[1]),
        cell_z=float(cell[2]),
        mass=mass,
        jitter=2.0 * radius,
        radius_mean=radius,
        custom_attributes={"mpm:friction": args.friction},
    )
    sand_model = sand_builder.finalize()
    n_particles = sand_model.particle_count

    cfg = SolverImplicitMPM.Config()
    cfg.voxel_size = dx
    cfg.max_iterations = args.iterations
    cfg.critical_fraction = 0.0
    cfg.strain_basis = "P0"
    solver = SolverImplicitMPM(sand_model, config=cfg)
    solver.setup_collider(model=model, collider_friction=[args.blade_friction, 0.5])

    state = model.state()
    sand_state = sand_model.state()
    sand_state.body_q = wp.clone(state.body_q)
    sand_state.body_qd = wp.zeros_like(state.body_qd)
    sand_state.body_f = wp.zeros_like(state.body_f)

    dt = 1.0 / args.fps
    q_np = sand_state.body_q.numpy()
    qd_np = np.zeros((model.body_count, 6), dtype=np.float32)
    qd_np[blade, 0] = args.speed  # spatial vector: linear first
    sand_state.body_qd.assign(qd_np)

    body_index = solver.collider_body_index.numpy()

    # Settle the bed for a few frames before cutting starts
    settle = 10
    forces = []
    frame_times = []
    for f in range(settle + args.frames):
        moving = f >= settle
        if moving:
            q_np[blade, 0] += args.speed * dt
            sand_state.body_q.assign(q_np)
        else:
            sand_state.body_qd.zero_()
        if f == settle:
            sand_state.body_qd.assign(qd_np)

        wp.synchronize()
        t0 = time.perf_counter()
        solver.step(sand_state, sand_state, contacts=None, control=None, dt=dt)
        wp.synchronize()
        t1 = time.perf_counter()

        imp, _pos, cid = solver.collect_collider_impulses(sand_state)
        imp = imp.numpy()
        cid = cid.numpy()
        valid = cid >= 0
        bids = np.full(cid.shape, -1)
        bids[valid] = body_index[np.clip(cid[valid], 0, len(body_index) - 1)]
        on_blade = bids == blade
        fx = float(imp[on_blade, 0].sum() / dt)
        fz = float(imp[on_blade, 2].sum() / dt)
        if moving:
            frame_times.append(t1 - t0)
            forces.append((fx, fz))

    frame_times = np.array(frame_times)
    forces = np.array(forces)
    # skip the first frames (JIT, entering the bed) and use the steady second half
    steady = forces[len(forces) // 2 :]
    timed = frame_times[5:]
    mean_ms = float(timed.mean() * 1000)
    phi = math.atan(args.friction)
    delta = math.atan(args.blade_friction)
    f_ref, n_gamma = mckyes_force(args.density, args.depth, args.width, phi, delta)

    result = {
        "voxel_m": dx,
        "ppc_per_axis": args.ppc,
        "particles": int(n_particles),
        "bed_m": args.bed,
        "iterations": args.iterations,
        "frame_dt_s": dt,
        "ms_per_frame": round(mean_ms, 2),
        "realtime_factor": round(dt / (mean_ms / 1000), 3),
        "particle_steps_per_s_millions": round(n_particles / (mean_ms / 1000) / 1e6, 2),
        "blade_force_x_N_mean_steady": round(float(np.abs(steady[:, 0]).mean()), 1),
        "blade_force_z_N_mean_steady": round(float(steady[:, 1].mean()), 1),
        "mckyes_force_N": round(f_ref, 1),
        "mckyes_N_gamma": round(n_gamma, 3),
        "force_ratio_sim_over_mckyes": round(float(np.abs(steady[:, 0]).mean()) / f_ref, 3),
        "travel_m": round(args.speed * args.frames * dt, 3),
    }
    print(json.dumps(result, indent=2))
    if args.out:
        with open(args.out, "w") as fh:
            json.dump(result, fh, indent=2)
        np.savez_compressed(
            args.out.replace(".json", ".npz"),
            particles=sand_state.particle_q.numpy(),
            forces=forces,
            frame_times=frame_times,
        )


if __name__ == "__main__":
    main()
