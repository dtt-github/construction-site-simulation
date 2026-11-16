"""Small crawler dozer with a power-angle-tilt blade (9 t class), built procedurally in Blender, exported as
assets/machinery/d37_hd.glb: a drop-in replacement for assets/machinery/opera_d37pxi.glb (PWRI OPERA D37PXi).

Same node names, link frames (read from that file) and joint extras:
  d37pxi (Z up -> Y up) / base_link / body_link / blade_arm_link (lift) / blade_angle_link (angle) /
  blade_link (tilt); blade_arm_link / blade_angle_piston_{left,right}_link (angle cylinders);
  body_link / blade_arm_cylinder_link / blade_arm_piston_link (lift cylinders)
The lift cylinders' rod eyes sit where the soil view's lift approximation (soil.js: cylinder turns 0.9 x lift,
rod extends 0.54 x lift) puts them on the C-frame. No manufacturer marks or livery.

    blender -b -t 4 --python assets/blender/d37_hd.py [-- --render out/prefix [--lift -0.3]]
"""

import math
import pathlib
import sys

import bmesh
import numpy as np
from mathutils import Matrix, Vector

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
import blendkit as K  # noqa: E402

ROOT = pathlib.Path(__file__).resolve().parents[2]
REF = ROOT / "assets/machinery/opera_d37pxi.glb"
OUT = ROOT / "assets/machinery/d37_hd.glb"
ARGS = sys.argv[sys.argv.index("--") + 1 :] if "--" in sys.argv else []
REFJ, _ = K.read_glb(REF)
REFN = {n["name"]: n for n in REFJ["nodes"]}


def ref_matrix(name):
    m = REFN[name].get("matrix")
    return Matrix(np.array(m).reshape(4, 4).T.tolist()) if m else Matrix.Identity(4)


def joint_axis(name):
    return Vector(REFN[name]["extras"]["urdf_joint"]["axis_local"]).normalized()


K.reset()
import bpy  # noqa: E402

M = {
    "paint": K.surface("paint_yellow", "#e2a91c", "paint", 1024, seed=31, coat=0.35),
    "blade": K.surface("blade_paint", "#e2a91c", "paint_worn", 1024, seed=32, coat=0.2),
    "frame": K.surface("frame_graphite", "#24272a", "paint_worn", 1024, seed=33),
    "track": K.surface("track_steel", "#38312a", "track", 1024, seed=34),
    "wear": K.surface("wear_steel", "#6a665f", "worn_edge", 512, seed=35),
    "cab": K.surface("cab_dark", "#2e3236", "paint", 512, seed=36, coat=0.25),
    "steel": K.material("steel_machined", "#9a9da0", rough=0.3, metal=1.0),
    "chrome": K.material("chrome_rod", "#e6e8ea", rough=0.08, metal=1.0),
    "glass": K.material("glass_tinted", "#16222a", rough=0.04, alpha=0.55),
    "plastic": K.material("plastic_black", "#0d0e0f", rough=0.55),
    "white": K.material("antenna_white", "#e8e8e4", rough=0.35),
    "lens": K.material("lamp_lens", "#f4f1e4", rough=0.1, emit="#fff4dc", emit_strength=0.6),
    "red": K.material("lamp_red", "#c4161c", rough=0.2, emit="#ff2010", emit_strength=0.4),
    "green": K.material("lamp_green", "#1e9a3c", rough=0.2, emit="#20ff60", emit_strength=0.4),
    "amber": K.material("lamp_amber", "#f0a020", rough=0.2, emit="#ffa000", emit_strength=0.5),
    "hazard_y": K.material("hazard_yellow", "#f0c010", rough=0.45, coat=0.2),
    "hazard_k": K.material("hazard_black", "#141414", rough=0.5, coat=0.2),
}

# ------------------------------------------------------------------ hierarchy (as the OPERA model)
base = K.empty("base_link")
body = K.empty("body_link", base, ref_matrix("body_link"))
arm = K.empty("blade_arm_link", body, ref_matrix("blade_arm_link"))
angle = K.empty("blade_angle_link", arm, ref_matrix("blade_angle_link"))
blade = K.empty("blade_link", angle, ref_matrix("blade_link"))
apist = {s: K.empty(f"blade_angle_piston_{s}_link", arm, ref_matrix(f"blade_angle_piston_{s}_link")) for s in ("left", "right")}
lcyl = K.empty("blade_arm_cylinder_link", body, ref_matrix("blade_arm_cylinder_link"))
lpis = K.empty("blade_arm_piston_link", lcyl, ref_matrix("blade_arm_piston_link"))

ARM_O = ref_matrix("blade_arm_link").translation
CYL_O = ref_matrix("blade_arm_cylinder_link").translation
LIFT_AXIS = joint_axis("blade_arm_piston_link") if "extras" in REFN["blade_arm_piston_link"] else Vector((1.0055, 0, -0.056785)).normalized()
LIFT_REACH, LIFT_Y = 0.98, 0.47
LIFT_EYE_ARM = CYL_O + LIFT_AXIS * LIFT_REACH - ARM_O  # lift cylinder rod eye on the C-frame (arm frame)
ANGLE_REACH = 1.0
BLADE_O = ref_matrix("blade_angle_link").translation  # in the arm frame

# ------------------------------------------------------------------ undercarriage (body frame, ground z = 0)
SPROCKET = ((-1.05, 0.46), 0.33)
IDLER = ((1.25, 0.37), 0.3)
ROLLERS = [((x, 0.2), 0.11) for x in (-0.72, -0.28, 0.16, 0.6, 0.98)]
GAUGE, SHOE_W = 1.7, 0.6


def build_shoe(bm, pitch):
    h = pitch * 0.5
    K.add_box(bm, (-h * 0.96, -SHOE_W / 2, 0.045), (h * 0.96, SHOE_W / 2, 0.062))
    K.add_prism_frame(bm, [(-0.012, 0.062), (0.012, 0.062), (0.006, 0.09), (-0.006, 0.09)], (0, -SHOE_W / 2, 0), (1, 0, 0), (0, 0, 1), 0, -SHOE_W)
    for s in (-1, 1):
        K.add_prism_frame(bm, [(-h, -0.03), (h, -0.03), (h * 1.02, 0.015), (h * 0.7, 0.045), (-h * 0.7, 0.045), (-h * 1.02, 0.015)], (0, s * 0.2 - 0.025, 0), (1, 0, 0), (0, 0, 1), 0, -0.05)
    K.add_cyl(bm, 0.028, (h, -0.18, 0), (h, 0.18, 0), 10)
    for bx in (-0.04, 0.04):
        for by in (-0.24, -0.16, 0.16, 0.24):
            K.add_cyl(bm, 0.01, (bx, by, 0.045), (bx, by, 0.037), 6)


def undercarriage():
    inner, length = K.belt_path([SPROCKET, IDLER] + ROLLERS, 600)
    n = int(round(length / 0.155))
    pitch = length / n
    shoe_mesh = None
    for side, yc in (("L", GAUGE / 2), ("R", -GAUGE / 2)):
        for k, (p, t, nrm) in enumerate(K.belt_frames(inner, length, n)):
            ob = K.make(f"track_shoe_{side}_{k:02d}", body, M["track"], lambda bm: build_shoe(bm, pitch), matrix=K.frame_matrix(p, t, nrm, yc), mesh=shoe_mesh)
            shoe_mesh = ob.data

        def wheels(bm, yc=yc):
            (ix, iz), ir = IDLER
            K.add_cyl(bm, ir - 0.05, (ix, yc - 0.22, iz), (ix, yc + 0.22, iz), 36)
            K.add_cyl(bm, ir - 0.02, (ix, yc - 0.05, iz), (ix, yc + 0.05, iz), 36)
            for (x, z), r in ROLLERS:
                for e in (-1, 1):
                    K.add_cyl(bm, r - 0.01, (x, yc + e * 0.13, z), (x, yc + e * 0.23, z), 20)
                    K.add_cyl(bm, r + 0.015, (x, yc + e * 0.23, z), (x, yc + e * 0.25, z), 20)
                K.add_cyl(bm, 0.05, (x, yc - 0.13, z), (x, yc + 0.13, z), 12)
            K.add_prism_y(bm, K.fillet([(-0.85, 0.3), (1.0, 0.3), (1.12, 0.42), (0.95, 0.58), (-0.8, 0.58), (-0.95, 0.45)], 0.05), yc - 0.17, yc + 0.17)
            K.add_box(bm, (0.75, yc - 0.12, 0.32), (1.12, yc + 0.12, 0.45))

        K.make(f"track_frame_{side}", body, M["frame"], wheels, bevel=0.01)

        def sprocket(bm, yc=yc):
            (sx, sz), sr = SPROCKET
            sgn = 1 if yc > 0 else -1
            K.add_gear(bm, (sx, sz), 0, sr - 0.04, sr + 0.025, 23, yc - 0.04, yc + 0.04)
            K.add_cyl(bm, 0.22, (sx, yc - 0.08, sz), (sx, yc + 0.08, sz), 32)
            K.add_cyl(bm, 0.28, (sx, yc - sgn * 0.08, sz), (sx, yc - sgn * 0.36, sz), 36)  # final drive
            for k in range(12):
                a = 2 * math.pi * k / 12
                K.add_cyl(bm, 0.013, (sx + 0.17 * math.cos(a), yc + sgn * 0.08, sz + 0.17 * math.sin(a)), (sx + 0.17 * math.cos(a), yc + sgn * 0.095, sz + 0.17 * math.sin(a)), 6)

        K.make(f"sprocket_{side}", body, M["frame"], sprocket, bevel=0.005)


undercarriage()

# ------------------------------------------------------------------ body: frame, fenders, hood, cab, rear


def body_parts():
    K.make("main_frame", body, M["frame"], lambda bm: (K.add_box(bm, (-1.45, -0.55, 0.32), (1.55, 0.55, 0.78)), K.add_box(bm, (ARM_O.x - 0.18, -0.72, 0.22), (ARM_O.x + 0.18, 0.72, 0.55))), bevel=0.02)
    for s in (-1, 1):
        K.make(f"fender_{'l' if s > 0 else 'r'}", body, M["paint"], lambda bm, s=s: K.add_prism_y(bm, [(-1.5, 0.78), (1.15, 0.78), (1.5, 0.66), (1.5, 0.7), (1.15, 0.82), (-1.5, 0.82)], s * 0.55, s * 1.16), bevel=0.008)
    # hood: narrow and sloped toward the blade so the operator sees its corners
    hood = K.fillet([(0.3, 0.8), (1.72, 0.8), (1.72, 1.2), (1.55, 1.38), (0.3, 1.6)], 0.08, 4)
    K.make("hood", body, M["paint"], lambda bm: K.add_prism_y(bm, hood, -0.38, 0.38), bevel=0.035, segs=3)

    def grille(bm):
        K.add_box(bm, (1.72, -0.3, 0.86), (1.73, 0.3, 1.16))
        for k in range(7):
            z = 0.88 + k * 0.04
            K.add_box(bm, (1.73, -0.28, z), (1.75, 0.28, z + 0.018))
        for s in (-1, 1):
            for k in range(6):
                x = 0.6 + k * 0.12
                K.add_obox(bm, (x, s * 0.385, 1.15), ((math.cos(0.4), 0, math.sin(0.4)), (0, 1, 0), (-math.sin(0.4), 0, math.cos(0.4))), (0.04, 0.008, 0.16))

    K.make("grilles", body, M["frame"], grille, bevel=0.002)
    K.make("lift_brackets", body, M["frame"], lambda bm: [K.add_prism_y(bm, K.circle_hull([((CYL_O.x, CYL_O.z), 0.08)], extra=[(CYL_O.x - 0.18, 0.8), (CYL_O.x + 0.18, 0.8)]), s * LIFT_Y + e * 0.07 - 0.015, s * LIFT_Y + e * 0.07 + 0.015) for s in (-1, 1) for e in (-1, 1)], bevel=0.006)
    K.make("lift_pins", body, M["steel"], lambda bm: [K.add_cyl(bm, 0.035, (CYL_O.x, s * LIFT_Y - 0.1, CYL_O.z), (CYL_O.x, s * LIFT_Y + 0.1, CYL_O.z), 14) for s in (-1, 1)])
    # cab
    x0, x1, y0, z0, z1 = -1.18, 0.3, 0.78, 0.82, 2.72
    xr = x1 - 0.1
    prof = K.fillet([(x0, z0), (x1, z0), (x1, 1.45), (xr, z1), (x0, z1)], 0.07, 4)
    K.make("cab_shell", body, M["paint"], lambda bm: K.add_prism_y(bm, prof, -y0, y0), bevel=0.025, segs=3)
    K.make("cab_roof", body, M["paint"], lambda bm: K.add_box(bm, (x0 - 0.06, -y0 - 0.06, z1 - 0.01), (xr + 0.1, y0 + 0.06, z1 + 0.07)), bevel=0.02)
    fx = lambda z: x1 if z <= 1.45 else x1 - (x1 - xr) * (z - 1.45) / (z1 - 1.45)  # noqa: E731

    def glass(bm):
        e = 0.006
        p0, p1 = Vector((fx(1.5), 0, 1.5)), Vector((fx(2.62), 0, 2.62))
        u = (p1 - p0).normalized()
        n = Vector((u.z, 0, -u.x))
        mid = (p0 + p1) / 2 + n * 0.003
        K.add_obox(bm, mid, (u, (0, 1, 0), n), ((p1 - p0).length / 2, y0 - 0.08, e / 2))
        K.add_box(bm, (x1, -y0 + 0.08, 0.95), (x1 + e, y0 - 0.08, 1.4))
        for s in (-1, 1):
            y = y0 if s > 0 else -y0 - e
            K.add_prism_y(bm, [(x0 + 0.08, 1.15), (x1 - 0.07, 1.15), (fx(2.62) - 0.07, 2.62), (x0 + 0.08, 2.62)], y, y + e)
        K.add_box(bm, (x0 - e, -y0 + 0.1, 1.3), (x0, y0 - 0.1, 2.6))

    K.make("cab_glass", body, M["glass"], glass, smooth=False)
    K.make("cab_interior", body, M["plastic"], lambda bm: (K.add_box(bm, (-0.95, -0.3, 0.82), (-0.65, 0.3, 1.75)), K.add_box(bm, (-0.45, -0.6, 0.82), (-0.15, -0.4, 1.3)), K.add_box(bm, (-0.45, 0.4, 0.82), (-0.15, 0.6, 1.3))), bevel=0.02)

    def pillars(bm):
        for s in (-1, 1):
            y = s * (y0 - 0.03)
            K.add_tube(bm, [(x1 + 0.004, y, 0.86), (x1 + 0.004, y, 1.45), (xr + 0.004, y, z1 - 0.04)], 0.03, 8)
            K.add_tube(bm, [(x0 - 0.004, y, 0.86), (x0 - 0.004, y, z1 - 0.04)], 0.03, 8)
            K.add_tube(bm, [(-0.3, y + s * 0.004, 0.86), (-0.3, y + s * 0.004, z1 - 0.04)], 0.02, 8)
            K.add_tube(bm, [(x1, s * (y0 + 0.02), 2.3), (x1 + 0.15, s * (y0 + 0.2), 2.35)], 0.012, 6)
            K.add_box(bm, (x1 + 0.13, s * (y0 + 0.2) - 0.08, 2.15), (x1 + 0.17, s * (y0 + 0.2) + 0.08, 2.4))
            K.add_tube(bm, [(-0.9, s * 1.17, 0.82), (-0.9, s * 1.17, 1.35), (0.0, s * 1.17, 1.35), (0.0, s * 1.17, 0.82)], 0.016, 8)  # grab rails

    K.make("cab_pillars", body, M["frame"], pillars)
    K.make("roof_lamps", body, M["lens"], lambda bm: [K.add_box(bm, (xr + 0.1, s * 0.5 - 0.07, z1 + 0.01), (xr + 0.105, s * 0.5 + 0.07, z1 + 0.06)) for s in (-1, 1)])
    # rear: fuel tank, lamps, drawbar
    rear = K.fillet([(-1.62, 0.78), (-1.15, 0.78), (-1.15, 1.6), (-1.5, 1.6), (-1.62, 1.45)], 0.08, 4)
    K.make("rear_tank", body, M["paint"], lambda bm: K.add_prism_y(bm, rear, -0.95, 0.95), bevel=0.035, segs=3)
    K.make("tail_lamps", body, M["red"], lambda bm: [K.add_box(bm, (-1.64, s * 0.75 - 0.06, 1.25), (-1.62, s * 0.75 + 0.06, 1.35)) for s in (-1, 1)])
    K.make("drawbar", body, M["frame"], lambda bm: (K.add_box(bm, (-1.85, -0.25, 0.42), (-1.45, 0.25, 0.55)), K.add_cyl(bm, 0.05, (-1.75, 0, 0.35), (-1.75, 0, 0.62), 16)), bevel=0.01)
    K.make("rear_chevron_band", body, M["hazard_k"], lambda bm: K.add_box(bm, (-1.63, -0.72, 0.94), (-1.622, 0.72, 1.12)))
    K.make("rear_chevrons", body, M["hazard_y"], lambda bm: [K.add_prism_frame(bm, [(y, 0), (y + 0.09, 0), (y + 0.16, 0.16), (y + 0.07, 0.16)], (-1.632, 0, 0.95), (0, -1, 0), (0, 0, 1), 0, 0.004) for y in np.arange(-0.62, 0.5, 0.2)], smooth=False)
    K.make("fuel_cap", body, M["frame"], lambda bm: K.add_cyl(bm, 0.07, (-1.38, 0.55, 1.6), (-1.38, 0.55, 1.66), 20), bevel=0.01)
    # exhaust and pre-cleaner on the hood
    K.make("exhaust", body, M["frame"], lambda bm: (K.add_tube(bm, K.catmull([(1.05, -0.2, 1.4), (1.05, -0.2, 1.95), (1.0, -0.2, 2.05), (0.9, -0.2, 2.08)], 4), 0.05, 14), K.add_cyl(bm, 0.07, (1.05, -0.2, 1.4), (1.05, -0.2, 1.48), 16)))
    K.make("pre_cleaner", body, M["plastic"], lambda bm: (K.add_cyl(bm, 0.04, (0.75, 0.18, 1.48), (0.75, 0.18, 1.75), 16), K.add_cyl(bm, 0.1, (0.75, 0.18, 1.75), (0.75, 0.18, 1.86), 24, r1=0.055)), bevel=0.01)
    # PWRI automation retrofit: GNSS antennas and a signal tower on the roof
    for y in (-0.5, 0.5):
        K.make(f"gnss_{'l' if y > 0 else 'r'}", body, M["white"], lambda bm, y=y: (K.add_cyl(bm, 0.02, (-0.9, y, z1 + 0.06), (-0.9, y, z1 + 0.24), 10), K.add_cyl(bm, 0.09, (-0.9, y, z1 + 0.24), (-0.9, y, z1 + 0.3), 32, r1=0.065)), bevel=0.01)
    for k, col in enumerate(("green", "amber", "red")):
        K.make(f"signal_{col}", body, M[col], lambda bm, k=k: (K.add_cyl(bm, 0.04, (-0.4, -0.15 + 0.15 * k, z1 + 0.06), (-0.4, -0.15 + 0.15 * k, z1 + 0.15), 20), K.add_sphere(bm, 0.04, (-0.4, -0.15 + 0.15 * k, z1 + 0.15), 20, 8)))


body_parts()

# ------------------------------------------------------------------ C-frame (blade_arm_link frame)


def c_frame():
    bx, bz = BLADE_O.x, BLADE_O.z
    ex, ez = LIFT_EYE_ARM.x, LIFT_EYE_ARM.z
    for s in (-1, 1):
        beam = K.fillet([(-0.12, -0.12), (bx - 0.15, bz - 0.12), (bx - 0.05, bz + 0.12), (0.35, 0.16), (0.0, 0.12)], 0.06)
        K.make(f"push_arm_{'l' if s > 0 else 'r'}", arm, M["paint"], lambda bm, s=s, beam=beam: K.add_prism_y(bm, beam, s * 0.45, s * 0.56), bevel=0.02)
        br = K.circle_hull([((ex, ez), 0.075)], extra=[(ex - 0.28, 0.08), (ex + 0.22, 0.04)])
        for e in (-1, 1):
            K.make(f"lift_ear_{'l' if s > 0 else 'r'}_{e + 1}", arm, M["paint"], lambda bm, s=s, e=e, br=br: K.add_prism_y(bm, br, s * LIFT_Y + e * 0.07 - 0.015, s * LIFT_Y + e * 0.07 + 0.015), bevel=0.006)
    K.make("c_frame_front", arm, M["paint"], lambda bm: K.add_box(bm, (bx - 0.32, -0.56, bz - 0.14), (bx - 0.1, 0.56, bz + 0.1)), bevel=0.02)
    K.make("trunnions", arm, M["frame"], lambda bm: [K.add_cyl(bm, 0.11, (0, s * 0.42, 0), (0, s * 0.62, 0), 24) for s in (-1, 1)], bevel=0.008)
    K.make("arm_pins", arm, M["steel"], lambda bm: [K.add_cyl(bm, 0.035, (ex, s * LIFT_Y - 0.1, ez), (ex, s * LIFT_Y + 0.1, ez), 14) for s in (-1, 1)] + [K.add_cyl(bm, 0.09, (bx - 0.1, -0.12, bz), (bx + 0.02, 0.12, bz), 20)])


c_frame()

# ------------------------------------------------------------------ cylinders


def cylinder(name, parent_barrel, parent_rod, origin, axis, ycs, r, rod_r, barrel_len, rod_from, reach, rod_origin=None):
    bb, br = bmesh.new(), bmesh.new()
    K.cylinder_meshes(bb, br, origin, axis, r, rod_r, barrel_len, rod_from, reach, eye_r=0.05, eye_w=0.1, ycs=ycs)
    if rod_origin is not None:  # rod node frame differs from the barrel's: move the rod's verts into it
        bmesh.ops.translate(br, verts=br.verts, vec=-Vector(rod_origin))
    for nm, bmx, parent, mat in ((f"{name}_barrel", bb, parent_barrel, M["paint"]), (f"{name}_rod", br, parent_rod, M["chrome"])):
        K.box_uv(bmx)
        me = bpy.data.meshes.new(nm)
        bmx.to_mesh(me)
        bmx.free()
        for p in me.polygons:
            p.use_smooth = True
        me.use_auto_smooth = True
        me.materials.append(mat)
        K.make(nm, parent, mat, None, mesh=me, bevel=0.004)


# lift: barrel on the cylinder link (origin at its pin), rod on the piston link (same frame at rest)
cylinder("lift_cyl", lcyl, lpis, (0, 0, 0), LIFT_AXIS, (-LIFT_Y, LIFT_Y), 0.065, 0.035, 0.62, 0.38, LIFT_REACH)
# angle: barrels on the C-frame from the piston links' origins, rods on the piston links
for s in ("left", "right"):
    o = ref_matrix(f"blade_angle_piston_{s}_link").translation
    ax = joint_axis(f"blade_angle_piston_{s}_link")
    cylinder(f"angle_cyl_{s}", arm, apist[s], o, ax, (0.0,), 0.055, 0.03, 0.55, 0.32, ANGLE_REACH, rod_origin=o)

# ------------------------------------------------------------------ blade (blade_link frame)
FACE = [(0.41, -0.29), (0.3, -0.12), (0.22, 0.08), (0.18, 0.3), (0.2, 0.52), (0.28, 0.7), (0.37, 0.79)]
BW = 1.6


def blade_parts():
    face = K.catmull(FACE, 6)
    back = K.offset_polyline(face, 0.035)
    if np.mean([p[0] for p in back]) > np.mean([p[0] for p in face]):
        back = K.offset_polyline(face, -0.035)
    K.make("moldboard", blade, M["blade"], lambda bm: K.add_prism_y(bm, face + back[::-1], -BW, BW), bevel=0.006)
    side = face + [(-0.08, 0.79), (-0.08, -0.25)]
    for s in (-1, 1):
        K.make(f"blade_end_{'l' if s > 0 else 'r'}", blade, M["blade"], lambda bm, s=s: K.add_prism_y(bm, side, s * (BW - 0.03), s * (BW + 0.015)), bevel=0.008)

    def frame(bm):
        K.add_box(bm, (-0.1, -BW + 0.05, -0.05), (0.15, BW - 0.05, 0.2))  # box beam
        K.add_box(bm, (-0.06, -BW + 0.05, 0.55), (0.18, BW - 0.05, 0.66))
        rib = [(x - 0.005, z) for x, z in back if -0.22 <= z <= 0.7]  # ribs end at the moldboard's back face
        for y in np.linspace(-BW + 0.2, BW - 0.2, 7):
            K.add_prism_y(bm, rib + [(-0.06, rib[-1][1]), (-0.08, rib[0][1])], y - 0.02, y + 0.02)
        K.add_box(bm, (-0.2, -0.28, -0.14), (0.02, 0.28, 0.18))  # ball socket and tilt mount
        for s in (-1, 1):  # angle cylinder ears
            for e in (-0.06, 0.06):
                K.add_prism_y(bm, K.circle_hull([((-0.08, 0.23), 0.06)], extra=[(0.05, 0.1), (0.05, 0.35)]), s * 0.5 + e - 0.012, s * 0.5 + e + 0.012)

    K.make("blade_frame", blade, M["blade"], frame, bevel=0.008)
    K.make("cutting_edge", blade, M["wear"], lambda bm: (K.add_prism_y(bm, [(0.37, -0.3), (0.45, -0.3), (0.38, -0.17), (0.3, -0.17)], -BW + 0.2, BW - 0.2), [K.add_prism_y(bm, [(0.37, -0.3), (0.47, -0.3), (0.4, -0.12), (0.3, -0.15)], s * (BW - 0.2), s * (BW + 0.02)) for s in (-1, 1)]), bevel=0.004)
    K.make("spill_lip", blade, M["blade"], lambda bm: K.add_prism_y(bm, [(0.3, 0.76), (0.45, 0.82), (0.43, 0.86), (0.28, 0.8)], -BW, BW), bevel=0.005)


blade_parts()

# ------------------------------------------------------------------ export
K.export_glb(OUT)
js, binchunk = K.read_glb(OUT)
roots = js["scenes"][0]["nodes"]
js["nodes"].append({"name": "d37pxi", "matrix": REFN["d37pxi"]["matrix"], "children": roots})
js["scenes"][0]["nodes"] = [len(js["nodes"]) - 1]
names = {n.get("name"): i for i, n in enumerate(js["nodes"])}
for name, ref in REFN.items():
    if name in names and ref.get("extras"):
        js["nodes"][names[name]]["extras"] = ref["extras"]
js["asset"]["copyright"] = (
    "Small crawler dozer with a power-angle-tilt blade, modelled procedurally in Blender (assets/blender/d37_hd.py) by the "
    "construction-site-simulation project to the joint frames of PWRI's OPERA D37PXi model (https://github.com/pwri-opera/d37pxi_ros2, Apache-2.0). Apache-2.0."
)
js["asset"]["extras"] = {"source": "assets/blender/d37_hd.py", "reference": "https://github.com/pwri-opera/d37pxi_ros2", "license": "Apache-2.0"}
K.write_glb(OUT, js, binchunk)
print(f"EXPORTED {OUT} nodes={len(js['nodes'])}")

if "--render" in ARGS:
    out = ARGS[ARGS.index("--render") + 1]
    lift = float(ARGS[ARGS.index("--lift") + 1]) if "--lift" in ARGS else 0.0
    # pose as soil.js does: arm turns by lift, cylinder by 0.9 x lift, rod out 0.54 x lift
    arm.matrix_basis = ref_matrix("blade_arm_link") @ Matrix.Rotation(lift, 4, "Y")
    lcyl.matrix_basis = ref_matrix("blade_arm_cylinder_link") @ Matrix.Rotation(0.9 * lift, 4, "Y")
    lpis.matrix_basis = Matrix.Translation(Vector((1.0055, 0, -0.056785)).normalized() * 0.54 * lift)
    sc = bpy.context.scene
    sc.render.engine = "BLENDER_EEVEE"
    ee = sc.eevee
    ee.taa_render_samples = 48
    ee.use_gtao = True
    ee.use_ssr = True
    ee.use_soft_shadows = True
    ee.shadow_cascade_size = "4096"
    sc.view_settings.view_transform = "AgX"
    sun = bpy.data.objects.new("sun", bpy.data.lights.new("sun", "SUN"))
    sc.collection.objects.link(sun)
    sun.data.energy = 4.0
    sun.rotation_euler = (math.radians(50), 0, math.radians(200))
    w = bpy.data.worlds.new("sky")
    sc.world = w
    w.use_nodes = True
    sky = w.node_tree.nodes.new("ShaderNodeTexSky")
    w.node_tree.links.new(sky.outputs["Color"], w.node_tree.nodes["Background"].inputs["Color"])
    w.node_tree.nodes["Background"].inputs["Strength"].default_value = 0.25
    b = bmesh.new()
    K.add_box(b, (-30, -30, -0.05), (30, 30, 0.0))
    me = bpy.data.meshes.new("ground")
    b.to_mesh(me)
    me.materials.append(K.material("dirt", "#8a7a62", rough=0.95))
    sc.collection.objects.link(bpy.data.objects.new("ground", me))
    sc.render.resolution_x, sc.render.resolution_y = 1600, 1000
    cam = bpy.data.objects.new("cam", bpy.data.cameras.new("cam"))
    sc.collection.objects.link(cam)
    sc.camera = cam
    cam.data.lens = 35
    for vname, loc in {"front_left": (6.0, 4.5, 2.6), "rear_right": (-5.5, -5.0, 3.0), "side": (0.4, -8.5, 1.1)}.items():
        cam.location = loc
        cam.rotation_euler = (Vector((0.3, 0, 0.8)) - Vector(loc)).to_track_quat("-Z", "Y").to_euler()
        sc.render.filepath = f"{out}_{vname}.png"
        bpy.ops.render.render(write_still=True)
