"""Self-contained reproducer: SolverImplicitMPM's rheology solve does not converge in a few hundred
Gauss-Seidel iterations on a soil-cutting problem, so the cutting force depends on max_iterations and on
the time step (a smaller step only starts each solve closer to the answer).

A 0.96 m wide vertical blade, kinematic, cuts 0.3 m deep at 0.5 m/s through a 3.2 x 1.36 x 0.8 m bed of
dry sand (friction 0.6, rigid-plastic, no dilatancy), between frictionless side walls, on a rough
floor, held by a rough end wall; 8 cm voxels, 4 cm particles. Prints the mean horizontal blade force
over the middle of the cut for each setting. Needs only newton and warp.

    python repro_mpm_convergence.py
"""

import time

import numpy as np
import warp as wp

import newton
from newton.solvers import SolverImplicitMPM

SPACING, VOXEL = 0.04, 0.08


def blade_force(dt, iterations, depth=0.3, speed=0.5, travel=1.0, width=0.96, bed=(3.2, 1.36, 0.8)):
    bx, by, bz = bed
    b = newton.ModelBuilder()
    slick = newton.ModelBuilder.ShapeConfig(mu=0.0)
    blade = b.add_body(xform=wp.transform(wp.vec3(0.4, by / 2, bz - depth + 0.5), wp.quat_identity()), mass=500.0, is_kinematic=True)
    b.add_shape_box(blade, hx=0.02, hy=width / 2, hz=0.5)
    for y in (-0.1, by + 0.1):
        b.add_shape_box(-1, xform=wp.transform(wp.vec3(bx / 2, y, 1.0), wp.quat_identity()), hx=bx, hy=0.1, hz=1.0, cfg=slick)
    b.add_shape_box(-1, xform=wp.transform(wp.vec3(bx + 0.1, by / 2, 1.0), wp.quat_identity()), hx=0.1, hy=by / 2, hz=1.0, cfg=newton.ModelBuilder.ShapeConfig(mu=1.0))
    b.add_ground_plane(cfg=newton.ModelBuilder.ShapeConfig(mu=1.0))
    model = b.finalize()

    sb = newton.ModelBuilder()
    SolverImplicitMPM.register_custom_attributes(sb)
    n = [int(round(v / SPACING)) for v in bed]
    sb.add_particle_grid(pos=wp.vec3(SPACING / 2, SPACING / 2, SPACING / 2), rot=wp.quat_identity(), vel=wp.vec3(0.0),
                         dim_x=n[0], dim_y=n[1], dim_z=n[2], cell_x=SPACING, cell_y=SPACING, cell_z=SPACING,
                         mass=SPACING**3 * 1600.0, jitter=0.0, radius_mean=SPACING / 2, custom_attributes={"mpm:friction": 0.6})
    sand = sb.finalize()
    cfg = SolverImplicitMPM.Config()
    cfg.voxel_size, cfg.max_iterations, cfg.critical_fraction, cfg.strain_basis = VOXEL, iterations, 0.0, "P0"
    mpm = SolverImplicitMPM(sand, config=cfg)
    mpm.setup_collider(model=model, collider_friction=[0.4, None])
    st = sand.state()
    rs = model.state()
    st.body_q, st.body_qd, st.body_f = wp.clone(rs.body_q), wp.zeros_like(rs.body_qd), wp.zeros_like(rs.body_f)

    body_index = mpm.collider_body_index.numpy()
    q = st.body_q.numpy()
    qd = np.zeros((model.body_count, 6), dtype=np.float32)
    for _ in range(int(0.5 / dt)):  # settle
        mpm.step(st, st, contacts=None, control=None, dt=dt)
    qd[blade, 0] = speed
    st.body_qd.assign(qd)
    steps = int(travel / speed / dt)
    forces = []
    for _ in range(steps):
        q[blade, 0] += speed * dt
        st.body_q.assign(q)
        mpm.step(st, st, contacts=None, control=None, dt=dt)
        imp, _pos, cid = mpm.collect_collider_impulses(st)
        imp, cid = imp.numpy(), cid.numpy()
        on = np.zeros(cid.shape, bool)
        ok = cid >= 0
        on[ok] = body_index[np.clip(cid[ok], 0, len(body_index) - 1)] == blade
        forces.append(abs(float(imp[on, 0].sum())) / dt)
    return float(np.mean(forces[int(0.3 * steps):int(0.8 * steps)]))


if __name__ == "__main__":
    wp.init()
    print(f"newton {newton.__version__}, warp {wp.__version__}")
    for dt, iterations in ((1 / 30, 150), (1 / 30, 600), (1 / 30, 2000), (1 / 60, 150), (1 / 120, 150), (1 / 240, 150)):
        t = time.perf_counter()
        f = blade_force(dt, iterations)
        print(f"dt = 1/{round(1 / dt):<4d} max_iterations = {iterations:<5d} blade force {f / 1e3:6.1f} kN   ({time.perf_counter() - t:.0f} s)", flush=True)
