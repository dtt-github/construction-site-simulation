"""Checks the soil presets (soils.py) against soil mechanics, on the excavator's MPM settings (8 cm
voxel, 4 cm particles, 30 Hz steps, 150 iterations).

    cut     a soil block with one unsupported vertical face, between frictionless side walls (plane
            strain), released under gravity: does a face of height H stand? Compared with the critical
            height range of the preset (soils.critical_height).
    blade   a vertical blade pushed through a bed at constant speed: horizontal force compared with
            McKyes' fundamental earthmoving equation with its cohesion term.

    ../phase0/.venv-newton/bin/python soil_calibrate.py cut --soil loam --heights 1 2 3
    ../phase0/.venv-newton/bin/python soil_calibrate.py blade --soil clay --depth 0.3
"""

import argparse
import json
import math

import numpy as np
import warp as wp

import newton
from newton.solvers import SolverImplicitMPM

import soils

SPACING, VOXEL, DT, ITERATIONS = 0.04, 0.08, 1.0 / 30.0, 150


def make_solver(model, particles, soil, attrs=None, collider_friction=None):
    sb = newton.ModelBuilder()
    SolverImplicitMPM.register_custom_attributes(sb)
    s = soils.SOILS[soil]
    mass = SPACING**3 * s["density"]
    for lo, dims in particles:
        sb.add_particle_grid(
            pos=wp.vec3(*lo), rot=wp.quat_identity(), vel=wp.vec3(0.0),
            dim_x=dims[0], dim_y=dims[1], dim_z=dims[2], cell_x=SPACING, cell_y=SPACING, cell_z=SPACING,
            mass=mass, jitter=0.0, radius_mean=SPACING / 2, custom_attributes=attrs or soils.mpm_attributes(soil),
        )
    sand = sb.finalize()
    cfg = SolverImplicitMPM.Config()
    cfg.voxel_size = VOXEL
    cfg.max_iterations = ITERATIONS
    cfg.critical_fraction = 0.0
    cfg.strain_basis = "P0"
    mpm = SolverImplicitMPM(sand, config=cfg)
    mpm.setup_collider(model=model, collider_friction=collider_friction)
    st = sand.state()
    rs = model.state()
    if model.body_count:
        st.body_q = wp.clone(rs.body_q)
        st.body_qd = wp.zeros_like(rs.body_qd)
        st.body_f = wp.zeros_like(rs.body_f)
    return sand, mpm, st


def cut_test(soil, height, seconds=4.0, width=0.32):
    """Block 2H long (rough back wall at x = 0, free face at x = 2H), H high, `width` wide between
    frictionless walls. Returns how far the crest moved and how much of the block moved > 10 cm."""
    length = max(2.0 * height, 1.0)
    b = newton.ModelBuilder()
    rough = newton.ModelBuilder.ShapeConfig(mu=1.0)
    slick = newton.ModelBuilder.ShapeConfig(mu=0.0)
    hz = height + 1.0
    b.add_shape_box(-1, xform=wp.transform(wp.vec3(-0.1, width / 2, hz / 2), wp.quat_identity()), hx=0.1, hy=width / 2 + 0.2, hz=hz / 2, cfg=rough)
    for y in (-0.1, width + 0.1):
        b.add_shape_box(-1, xform=wp.transform(wp.vec3(length, y, hz / 2), wp.quat_identity()), hx=length + 3.0, hy=0.1, hz=hz / 2, cfg=slick)
    b.add_ground_plane(cfg=rough)
    model = b.finalize()
    n = [int(round(length / SPACING)), int(round(width / SPACING)), int(round(height / SPACING))]
    lo = (SPACING / 2, SPACING / 2, SPACING / 2)
    sand, mpm, st = make_solver(model, [(lo, n)], soil)
    q0 = st.particle_q.numpy().copy()
    for _ in range(int(seconds / DT)):
        mpm.step(st, st, contacts=None, control=None, dt=DT)
    q = st.particle_q.numpy()
    d = np.linalg.norm(q - q0, axis=1)
    crest = (q0[:, 0] > length - 0.3) & (q0[:, 2] > height - 0.3)
    return {"soil": soil, "height_m": height, "crest_move_m": round(float(np.median(d[crest])), 3), "moved_frac": round(float((d > 0.1).mean()), 3),
            "face_toe_runout_m": round(float(q[:, 0].max() - length), 2)}


def blade_test(soil, depth, speed=0.5, travel=1.0, width=0.96, bed=(3.2, 1.36, 0.8), blade_friction=0.4):
    """Vertical blade cutting `depth` at `speed` through a bed between frictionless walls, held by a rough
    end wall (a cohesive bed would otherwise slide along the floor as a block); mean horizontal force
    from 0.3 to 0.8 of the travel, before the heap pushed ahead adds much surcharge."""
    b = newton.ModelBuilder()
    slick = newton.ModelBuilder.ShapeConfig(mu=0.0)
    bx, by, bz = bed
    x0 = 0.4
    blade = b.add_body(xform=wp.transform(wp.vec3(x0, by / 2, bz - depth + 0.5), wp.quat_identity()), mass=500.0, is_kinematic=True)
    b.add_shape_box(blade, hx=0.02, hy=width / 2, hz=0.5)
    for y in (-0.1, by + 0.1):
        b.add_shape_box(-1, xform=wp.transform(wp.vec3(bx / 2, y, 1.0), wp.quat_identity()), hx=bx, hy=0.1, hz=1.0, cfg=slick)
    b.add_shape_box(-1, xform=wp.transform(wp.vec3(bx + 0.1, by / 2, 1.0), wp.quat_identity()), hx=0.1, hy=by / 2, hz=1.0, cfg=newton.ModelBuilder.ShapeConfig(mu=1.0))
    b.add_ground_plane(cfg=newton.ModelBuilder.ShapeConfig(mu=1.0))
    model = b.finalize()
    n = [int(round(bx / SPACING)), int(round(by / SPACING)), int(round(bz / SPACING))]
    sand, mpm, st = make_solver(model, [((SPACING / 2, SPACING / 2, SPACING / 2), n)], soil, collider_friction=[blade_friction, None])
    body_index = mpm.collider_body_index.numpy()
    q_np = st.body_q.numpy()
    qd = np.zeros((model.body_count, 6), dtype=np.float32)
    for _ in range(15):  # settle
        mpm.step(st, st, contacts=None, control=None, dt=DT)
    qd[blade, 0] = speed
    st.body_qd.assign(qd)
    forces = []
    steps = int(travel / speed / DT)
    for k in range(steps):
        q_np[blade, 0] += speed * DT
        st.body_q.assign(q_np)
        mpm.step(st, st, contacts=None, control=None, dt=DT)
        imp, _pos, cid = mpm.collect_collider_impulses(st)
        imp, cid = imp.numpy(), cid.numpy()
        ok = cid >= 0
        on = np.zeros(cid.shape, bool)
        on[ok] = body_index[np.clip(cid[ok], 0, len(body_index) - 1)] == blade
        forces.append(float(imp[on, 0].sum() / DT))
    s = soils.SOILS[soil]
    f_sim = float(np.abs(np.array(forces[int(0.3 * steps):int(0.8 * steps)])).mean())
    f_ref, fac = soils.mckyes(s["density"], s["cohesion"], depth, width, math.atan(s["friction"]), math.atan(blade_friction))
    return {"soil": soil, "depth_m": depth, "force_N": round(f_sim), "mckyes_N": round(f_ref), "ratio": round(f_sim / f_ref, 2), **{k: round(v, 2) for k, v in fac.items()}}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("test", choices=("cut", "blade"))
    ap.add_argument("--soil", nargs="+", default=list(soils.SOILS))
    ap.add_argument("--heights", type=float, nargs="+", default=[0.5, 1.0, 2.0])
    ap.add_argument("--depth", type=float, nargs="+", default=[0.2])
    ap.add_argument("--out", default=None)
    args = ap.parse_args()
    rows = []
    for soil in args.soil:
        if args.test == "cut":
            lo, hi = soils.critical_height(soil)
            for h in args.heights:
                r = cut_test(soil, h)
                r["critical_height_m"] = [round(lo, 2), round(hi, 2)]
                rows.append(r)
                print(json.dumps(r), flush=True)
        else:
            for d in args.depth:
                r = blade_test(soil, d)
                rows.append(r)
                print(json.dumps(r), flush=True)
    if args.out:
        with open(args.out, "w") as fh:
            json.dump(rows, fh, indent=1)


if __name__ == "__main__":
    main()
