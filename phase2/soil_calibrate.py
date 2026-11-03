"""Checks the soil presets (soils.py) against soil mechanics, on the excavator's MPM settings (8 cm
voxel, 4 cm particles, 30 Hz steps, 150 iterations).

    cut     a soil block with one unsupported vertical face, between frictionless side walls (plane
            strain), released under gravity: does a face of height H stand? Compared with the critical
            height range of the preset (soils.critical_height).
    blade   a vertical blade pushed through a bed at constant speed: horizontal force compared with
            McKyes' fundamental earthmoving equation with its cohesion term.
    tine    the subsoiler of Makange et al. 2021 (Sci. Rep. 11:11082, CC BY 4.0) in their clay soil bin,
            scaled up lam times: its measured horizontal and vertical forces at 0.15, 0.20 and 0.30 m.
    column  plane-strain collapse of a granular column released from a wall: final runout and height
            against the 48 true-2D aluminium-rod experiments of Nguyen, Bui & Fukagawa (Failure mechanism
            of true 2D granular flows, arXiv 1512.05180), scaled up (the soil model has no length scale),
            and against Lube et al. 2005's quasi-2D sand experiments for the dry-sand preset.

    ../phase0/.venv-newton/bin/python soil_calibrate.py cut --soil loam --heights 1 2 3
    ../phase0/.venv-newton/bin/python soil_calibrate.py blade --soil clay --depth 0.3
    ../phase0/.venv-newton/bin/python soil_calibrate.py column --soil rods --ratios 0.5 1 2 4
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

# the aluminium-rod model ground of Nguyen et al. (rods 1.6 and 3.0 mm, 5 cm long): unit weight
# 20.4 kN/m3, phi 21.9 deg, c 0 (their Table 1); a test material, not a soil preset of the excavator
RODS = {"density": 20.4e3 / 9.81, "friction": math.tan(math.radians(21.9)), "cohesion": 0.0, "tensile": 0.0}


def material(soil):
    if isinstance(soil, dict):
        return soil
    return RODS if soil == "rods" else soils.SOILS[soil]


def attributes(m):
    """MPM attributes of a material dict (density, friction, cohesion, tensile), as soils.mpm_attributes."""
    attrs = {"mpm:friction": m["friction"]}
    if m["cohesion"] > 0.0 or m["tensile"] > 0.0:
        attrs.update({"mpm:yield_stress": m["cohesion"], "mpm:yield_pressure": soils.YIELD_PRESSURE,
                      "mpm:tensile_yield_ratio": m["tensile"] / soils.YIELD_PRESSURE})
    return attrs


# Makange, Ji, Torotwa, Fan & Huang 2021, "Prediction of precise subsoiling based on analytical method,
# discrete element simulation and experimental data from soil bin", Sci. Rep. 11:11082 (CC BY 4.0), Table 4:
# measured (speed km/h, depth m): (horizontal N, vertical N). Clay, 1668 kg/m3 mean bulk density (1464 at the
# top to 1787 below 0.3 m), 10.8-11.2 % moisture; a rectangular subsoiler blade, 260 mm effective width,
# at 23 deg to the ground (16 deg rake in their analytical model), on a 30 mm thick arc shank.
MAKANGE = {
    (1.0, 0.15): (1235.24, 490.22), (1.0, 0.20): (1905.15, 740.51), (1.0, 0.30): (3381.81, 1390.34),
    (2.0, 0.15): (1296.55, 500.23), (2.0, 0.20): (1934.67, 750.91), (2.0, 0.30): (3389.01, 1401.90),
    (2.5, 0.15): (1312.11, 510.50), (2.5, 0.20): (1941.32, 760.45), (2.5, 0.30): (3417.52, 1410.11),
}
# their Table 1, the bin soil by depth: (top of layer m, bulk density kg/m3, cone index kPa)
MAKANGE_LAYERS = [(0.00, 1463.7, 325), (0.05, 1576.8, 401), (0.10, 1665.4, 788), (0.15, 1718.4, 970),
                  (0.20, 1728.5, 1553), (0.25, 1734.2, 2326), (0.30, 1787.4, 3031)]
MAKANGE_CI = 1342.0  # kPa, their mean cone index
MAKANGE_DENSITY, TINE_W, TINE_ANGLE, SHANK_T = 1667.7, 0.26, math.radians(23.0), 0.03
TINE_L = 0.15  # m: blade length along its slope (not given in the paper; a typical subsoiler point)
SOIL_STEEL = 0.55  # their DEM soil-steel friction


def column_reference(soil, a):
    """Measured final runout (d_inf - d0)/d0 and final height h_inf/h0 at aspect ratio a = h0/d0.
    Rods (true 2D, Nguyen et al. eqs. 1-4): h_inf = h0 up to a = 0.65; then h0/h_inf = 1.42 a^(2/3);
    (d_inf - d0)/d0 = 3.25 a^0.96 for 0.65 <= a <= 1.5 and 3.80 a^0.73 above. Sand (quasi-2D, Lube et al.
    2005, Phys. Rev. E 72, 041301): (d_inf - d0)/d0 = 1.2 a below a = 1.8 and 1.9 a^(2/3) above 2.8."""
    if soil == "rods":
        run = 3.25 * a**0.96 if a <= 1.5 else 3.80 * a**0.73
        return run, (1.0 if a <= 0.65 else min(1.0, 1.0 / (1.42 * a ** (2 / 3))))
    if a < 1.8:
        return 1.2 * a, None
    if a > 2.8:
        return 1.9 * a ** (2 / 3), None
    return None, None


def make_solver(model, particles, soil, attrs=None, collider_friction=None):
    """`particles`: (corner, cell counts) blocks of `soil`, or (corner, cell counts, material dict) blocks."""
    sb = newton.ModelBuilder()
    SolverImplicitMPM.register_custom_attributes(sb)
    if attrs is None:
        attrs = attributes(material(soil)) if isinstance(soil, dict) or soil == "rods" else soils.mpm_attributes(soil)
    base_attrs = attrs
    for block in particles:
        lo, dims = block[0], block[1]
        m = block[2] if len(block) > 2 else material(soil)
        attrs = attributes(m) if len(block) > 2 else base_attrs
        mass = SPACING**3 * m["density"]
        sb.add_particle_grid(
            pos=wp.vec3(*lo), rot=wp.quat_identity(), vel=wp.vec3(0.0),
            dim_x=dims[0], dim_y=dims[1], dim_z=dims[2], cell_x=SPACING, cell_y=SPACING, cell_z=SPACING,
            mass=mass, jitter=0.0, radius_mean=SPACING / 2, custom_attributes=attrs,
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


def column_test(soil, a, d0=None, floor_mu=1.0, wall_mu=0.0, seconds=None, width=0.32, settle=1.0):
    """Column d0 wide, h0 = a d0 high, against a frictionless wall at x = 0, between frictionless side
    walls (plane strain), on a floor with friction floor_mu, held by a gate at x = d0 while it settles
    under its weight for `settle` s (the grid packs it a few per cent), then released by removing the
    gate at once, as in the experiments. h0 is the settled height. Final runout, from the wall: d_inf
    where the deposit stops being at least one particle thick, and d_far at its farthest connected
    particle (the thin sheet at the front, less than a particle thick, lies between); final height
    h_inf: the deposit's highest point."""
    if d0 is None:
        d0 = 4.0 if a < 1.0 else 2.0  # at least 12 cells high and 25 wide
    h0 = a * d0
    reach = d0 * (1.0 + 1.3 * (column_reference(soil, a)[0] or 4.0 * a)) + 2.0
    if seconds is None:
        seconds = max(3.0, 6.0 * math.sqrt(h0 / soils.G))  # collapses settle by t sqrt(g/h0) of about 3-4
    b = newton.ModelBuilder()
    slick = newton.ModelBuilder.ShapeConfig(mu=0.0)
    hz = h0 + 1.0
    b.add_shape_box(-1, xform=wp.transform(wp.vec3(-0.1, width / 2, hz / 2), wp.quat_identity()), hx=0.1, hy=width / 2 + 0.2, hz=hz / 2,
                    cfg=newton.ModelBuilder.ShapeConfig(mu=wall_mu))
    for y in (-0.1, width + 0.1):
        b.add_shape_box(-1, xform=wp.transform(wp.vec3(reach / 2, y, hz / 2), wp.quat_identity()), hx=reach / 2 + 0.5, hy=0.1, hz=hz / 2, cfg=slick)
    b.add_ground_plane(cfg=newton.ModelBuilder.ShapeConfig(mu=floor_mu))
    gate = b.add_body(xform=wp.transform(wp.vec3(d0 + 0.1, width / 2, hz / 2), wp.quat_identity()), mass=100.0, is_kinematic=True)
    b.add_shape_box(gate, hx=0.1, hy=width / 2 + 0.2, hz=hz / 2, cfg=slick)
    model = b.finalize()
    n = [int(round(d0 / SPACING)), int(round(width / SPACING)), int(round(h0 / SPACING))]
    sand, mpm, st = make_solver(model, [((SPACING / 2, SPACING / 2, SPACING / 2), n)], soil)
    for _ in range(int(settle / DT)):
        mpm.step(st, st, contacts=None, control=None, dt=DT)
    top = lambda q: float(np.percentile(q[:, 2], 99.9)) + SPACING / 2  # noqa: E731
    h0 = top(st.particle_q.numpy())
    a = h0 / d0
    ref_run, ref_h = column_reference(soil, a)
    q_b = st.body_q.numpy()
    q_b[gate, 2] += 100.0  # gate removed
    st.body_q.assign(q_b)
    history = []
    steps = int(seconds / DT)
    for k in range(steps):
        mpm.step(st, st, contacts=None, control=None, dt=DT)
        if k % 3 == 2 or k == steps - 1:
            q = st.particle_q.numpy()
            history.append((round((k + 1) * DT, 3), round(float(_front(q, width)), 3)))
    q = st.particle_q.numpy()
    d_inf, d_far, h_inf = _front(q, width), _front(q, width, layer=0.0), top(q)
    run, hh = (d_inf - d0) / d0, h_inf / h0
    # settled: the front moved less than one cell over the last tenth of the run
    tail = [x for t, x in history if t >= 0.9 * seconds]
    out = {"soil": soil, "a": round(a, 3), "d0_m": d0, "h0_m": round(h0, 2), "floor_mu": floor_mu, "wall_mu": wall_mu, "seconds": round(seconds, 2),
           "d_inf_m": round(d_inf, 2), "d_far_m": round(d_far, 2), "h_inf_m": round(h_inf, 2), "runout": round(run, 2),
           "runout_far": round((d_far - d0) / d0, 2), "height": round(hh, 3),
           "settled": bool(max(tail) - min(tail) < VOXEL), "front_history": history}
    if ref_run is not None:
        out.update({"runout_ref": round(ref_run, 2), "runout_err": round(run / ref_run - 1.0, 3),
                    "runout_far_err": round((d_far - d0) / d0 / ref_run - 1.0, 3)})
    if ref_h is not None:
        out.update({"height_ref": round(ref_h, 3), "height_err": round(hh / ref_h - 1.0, 3)})
    return out


def _front(q, width, dx=VOXEL, layer=SPACING):
    """Farthest x (m) from the wall up to which the deposit is continuously more than `layer` thick
    (layer 0: any particle in every cell-wide strip)."""
    nb = int(q[:, 0].max() / dx) + 2
    counts = np.bincount(np.clip((q[:, 0] / dx).astype(int), 0, nb - 1), minlength=nb)
    thick = counts * SPACING**3 / (dx * width)
    k = 0
    while k < nb and (thick[k] >= layer if layer > 0.0 else thick[k] > 0.0):
        k += 1
    return k * dx


def tine_test(cohesion, friction_deg, depth, lam=4.0, speed_kmh=1.0, tensile_ratio=0.15, mu_tool=SOIL_STEEL, length_lab=0.8, layered=False):
    """The Makange subsoiler scaled up lam times (lab cohesion `cohesion` Pa, friction angle `friction_deg`):
    the soil model has no length scale, so lengths scale with lam, cohesion with lam, speed with sqrt(lam)
    and forces with lam^3. Half the bin, with a frictionless symmetry wall through the tine. The tine
    enters the bed through its (standing) front face; forces are averaged once the blade is a blade length
    plus one depth inside, to the end of the run. Returns lab-scale horizontal and vertical forces (N).
    layered: the bin's measured profile (MAKANGE_LAYERS), density per layer and cohesion in proportion to
    the cone index (`cohesion` is then the value at the mean cone index)."""
    m = {"density": MAKANGE_DENSITY, "friction": math.tan(math.radians(friction_deg)), "cohesion": cohesion * lam,
         "tensile": cohesion * lam * tensile_ratio}
    d, w, L, t = depth * lam, TINE_W * lam, TINE_L * lam, max(SHANK_T * lam, 0.06)
    bx, by, bz = length_lab * lam + 0.6 * lam, 0.5 * lam, 0.5 * lam  # bed (half width by)
    v = speed_kmh / 3.6 * math.sqrt(lam)
    b = newton.ModelBuilder()
    slick = newton.ModelBuilder.ShapeConfig(mu=0.0)
    rough = newton.ModelBuilder.ShapeConfig(mu=1.0)
    tool_cfg = newton.ModelBuilder.ShapeConfig(mu=mu_tool)
    c, s_ = math.cos(TINE_ANGLE), math.sin(TINE_ANGLE)
    x_tip0 = -0.1  # blade tip starts just ahead of the bed's front face
    tine = b.add_body(xform=wp.transform(wp.vec3(x_tip0, 0.0, bz - d), wp.quat_identity()), mass=500.0, is_kinematic=True)
    # blade: a plate rising backward from the tip at TINE_ANGLE to the ground (body frame at the tip)
    pt = max(0.02 * lam, 0.06)
    rot = wp.quat_from_axis_angle(wp.vec3(0.0, 1.0, 0.0), TINE_ANGLE)  # +x tilted down: tip in front, low
    b.add_shape_box(tine, xform=wp.transform(wp.vec3(-0.5 * L * c, w / 4, 0.5 * L * s_ - 0.5 * pt), rot), hx=L / 2, hy=w / 4, hz=pt / 2, cfg=tool_cfg)
    # shank: from the blade's back edge up through the surface
    hz = (d + 0.5 * lam) / 2
    b.add_shape_box(tine, xform=wp.transform(wp.vec3(-L * c - t / 2, t / 2, L * s_ + hz), wp.quat_identity()), hx=t / 2, hy=t / 2, hz=hz, cfg=tool_cfg)
    b.add_shape_box(-1, xform=wp.transform(wp.vec3(bx / 2, -0.1, bz), wp.quat_identity()), hx=bx, hy=0.1, hz=bz, cfg=slick)  # symmetry
    b.add_shape_box(-1, xform=wp.transform(wp.vec3(bx / 2, by + 0.1, bz), wp.quat_identity()), hx=bx, hy=0.1, hz=bz, cfg=rough)
    b.add_shape_box(-1, xform=wp.transform(wp.vec3(bx + 0.1, by / 2, bz), wp.quat_identity()), hx=0.1, hy=by, hz=bz, cfg=rough)
    b.add_ground_plane(cfg=rough)
    model = b.finalize()
    n = [int(round(bx / SPACING)), int(round(by / SPACING)), int(round(bz / SPACING))]
    blocks = [((SPACING / 2, SPACING / 2, SPACING / 2), n)]
    if layered:
        blocks = []
        tops = [t for t, _, _ in MAKANGE_LAYERS] + [bz / lam]
        for k, (t0, rho, ci) in enumerate(MAKANGE_LAYERS):
            z_hi, z_lo = bz - t0 * lam, max(bz - tops[k + 1] * lam, 0.0)
            nz = int(round((z_hi - z_lo) / SPACING))
            if nz <= 0:
                continue
            ck = cohesion * lam * ci / MAKANGE_CI
            mk = {"density": rho, "friction": m["friction"], "cohesion": ck, "tensile": ck * tensile_ratio}
            blocks.append(((SPACING / 2, SPACING / 2, z_hi - nz * SPACING + SPACING / 2), [n[0], n[1], nz], mk))
    sand, mpm, st = make_solver(model, blocks, m)
    body_index = mpm.collider_body_index.numpy()
    q_np = st.body_q.numpy()
    qd = np.zeros((model.body_count, 6), dtype=np.float32)
    for _ in range(10):  # settle
        mpm.step(st, st, contacts=None, control=None, dt=DT)
    qd[tine, 0] = v
    st.body_qd.assign(qd)
    travel = length_lab * lam
    steps = int(travel / v / DT)
    xs, fx, fz = [], [], []
    for k in range(steps):
        q_np[tine, 0] += v * DT
        st.body_q.assign(q_np)
        mpm.step(st, st, contacts=None, control=None, dt=DT)
        imp, _pos, cid = mpm.collect_collider_impulses(st)
        imp, cid = imp.numpy(), cid.numpy()
        ok = cid >= 0
        on = np.zeros(cid.shape, bool)
        on[ok] = body_index[np.clip(cid[ok], 0, len(body_index) - 1)] == tine
        xs.append(q_np[tine, 0])
        fx.append(2.0 * float(imp[on, 0].sum()) / DT)  # both halves
        fz.append(2.0 * float(imp[on, 2].sum()) / DT)
    xs, fx, fz = np.array(xs), np.abs(np.array(fx)), np.array(fz)
    window = xs > L * c + d + 0.1 * lam
    scale = lam**3
    return {"depth_m": depth, "lam": lam, "speed_kmh": speed_kmh, "layered": layered, "cohesion_kPa": cohesion / 1e3, "friction_deg": friction_deg,
            "horizontal_N": round(float(fx[window].mean()) / scale, 1), "vertical_N": round(float(fz[window].mean()) / scale, 1),
            "horizontal_sd_N": round(float(fx[window].std()) / scale, 1), "samples": int(window.sum()),
            "series": [(round(float(x) / lam, 3), round(float(f) / scale), round(float(z) / scale)) for x, f, z in zip(xs[::max(1, len(xs) // 16)], fx[::max(1, len(xs) // 16)], fz[::max(1, len(xs) // 16)])],
            "particles": int(np.prod(n))}


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
    ap.add_argument("test", choices=("cut", "blade", "column", "tine"))
    ap.add_argument("--cohesion", type=float, default=10e3, help="tine: lab-scale cohesion (Pa)")
    ap.add_argument("--friction-deg", type=float, default=30.0, help="tine: friction angle")
    ap.add_argument("--lam", type=float, nargs="+", default=[4.0], help="tine: scale factors")
    ap.add_argument("--speed", type=float, nargs="+", default=[1.0], help="tine: lab speeds (km/h)")
    ap.add_argument("--soil", nargs="+", default=list(soils.SOILS))
    ap.add_argument("--ratios", type=float, nargs="+", default=[0.5, 1.0, 2.0, 4.0])
    ap.add_argument("--d0", type=float, default=None)
    ap.add_argument("--floor-mu", type=float, default=1.0)
    ap.add_argument("--heights", type=float, nargs="+", default=[0.5, 1.0, 2.0])
    ap.add_argument("--depth", type=float, nargs="+", default=[0.2])
    ap.add_argument("--out", default=None)
    args = ap.parse_args()
    rows = []
    if args.test == "tine":
        for lam in args.lam:
            for v in args.speed:
                for d in args.depth:
                    r = tine_test(args.cohesion, args.friction_deg, d, lam=lam, speed_kmh=v)
                    ref = MAKANGE.get((v, d))
                    if ref:
                        r.update({"measured_H_N": ref[0], "measured_V_N": ref[1], "H_err": round(r["horizontal_N"] / ref[0] - 1.0, 3)})
                    rows.append(r)
                    print(json.dumps(r), flush=True)
        args.soil = []
    for soil in args.soil:
        if args.test == "cut":
            lo, hi = soils.critical_height(soil)
            for h in args.heights:
                r = cut_test(soil, h)
                r["critical_height_m"] = [round(lo, 2), round(hi, 2)]
                rows.append(r)
                print(json.dumps(r), flush=True)
        elif args.test == "column":
            for a in args.ratios:
                r = column_test(soil, a, d0=args.d0, floor_mu=args.floor_mu)
                rows.append(r)
                print(json.dumps({k: v for k, v in r.items() if k != "front_history"}), flush=True)
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
