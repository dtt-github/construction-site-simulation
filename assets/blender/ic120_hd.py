"""Rubber-tracked crawler carrier (11 t payload class), built procedurally in Blender, exported as
assets/machinery/ic120_hd.glb: a drop-in replacement for assets/machinery/opera_ic120.glb (PWRI OPERA IC120).

Same node names, link frames (read from that file) and joint extras, so the simulator, the soil view and the
showcase take it unchanged:
  ic120 (Z up -> Y up) / base_link / body_link / vessel_pin_link (tip joint), cylinder_head_link /
  cylinder_rod_link (tip cylinder), left/right_sprocket_link; base_link / six wheel links (empty, as the source)
The vessel's floor, sides, headboard and tail lip follow the server's vessel boxes (phase2/excavator_server.py,
hello.geometry.carrier.vessel_boxes) so the load sits on the floor the physics uses. No manufacturer marks.

    blender -b -t 4 --python assets/blender/ic120_hd.py [-- --render out/prefix [--beauty] [--tip 0.6]]
"""

import math
import pathlib
import sys

import numpy as np
from mathutils import Matrix, Vector

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
import blendkit as K  # noqa: E402

ROOT = pathlib.Path(__file__).resolve().parents[2]
REF = ROOT / "assets/machinery/opera_ic120.glb"
OUT = ROOT / "assets/machinery/ic120_hd.glb"
ARGS = sys.argv[sys.argv.index("--") + 1 :] if "--" in sys.argv else []
REFJ, _ = K.read_glb(REF)
REFN = {n["name"]: n for n in REFJ["nodes"]}


def ref_matrix(name):
    m = REFN[name].get("matrix")
    return Matrix(np.array(m).reshape(4, 4).T.tolist()) if m else Matrix.Identity(4)


K.reset()
import bpy  # noqa: E402

M = {
    "paint": K.surface("paint_blue", "#2c5f8c", "paint", 1024, seed=21, coat=0.35),
    "vessel": K.surface("vessel_paint", "#2c5f8c", "paint_worn", 1024, seed=22, coat=0.2),
    "liner": K.surface("vessel_liner", "#5a5650", "steel", 1024, seed=23),
    "frame": K.surface("frame_graphite", "#24272a", "paint_worn", 1024, seed=24),
    "cab": K.surface("cab_grey", "#8a9095", "paint", 512, seed=25, coat=0.25),
    "rubber": K.surface("track_rubber", "#141414", "rubber", 1024, seed=26),
    "steel": K.material("steel_machined", "#9a9da0", rough=0.3, metal=1.0),
    "chrome": K.material("chrome_rod", "#e6e8ea", rough=0.08, metal=1.0),
    "glass": K.material("glass_tinted", "#16222a", rough=0.04, alpha=0.55),
    "plastic": K.material("plastic_black", "#0d0e0f", rough=0.55),
    "white": K.material("antenna_white", "#e8e8e4", rough=0.35),
    "lens": K.material("lamp_lens", "#f4f1e4", rough=0.1, emit="#fff4dc", emit_strength=0.6),
    "red": K.material("lamp_red", "#c4161c", rough=0.2, emit="#ff2010", emit_strength=0.4),
    "amber": K.material("lamp_amber", "#f0a020", rough=0.2, emit="#ffa000", emit_strength=0.5),
    "hazard_y": K.material("hazard_yellow", "#f0c010", rough=0.45, coat=0.2),
    "hazard_k": K.material("hazard_black", "#141414", rough=0.5, coat=0.2),
}

# ------------------------------------------------------------------ hierarchy (as the OPERA model)
base = K.empty("base_link")
body = K.empty("body_link", base, ref_matrix("body_link"))
for n in ("left_middle_wheel_link", "left_front_wheel_link", "left_rear_wheel_link", "right_middle_wheel_link", "right_front_wheel_link", "right_rear_wheel_link"):
    K.empty(n, base, ref_matrix(n))
spro = {s: K.empty(f"{s}_sprocket_link", body, ref_matrix(f"{s}_sprocket_link")) for s in ("left", "right")}
vessel = K.empty("vessel_pin_link", body, ref_matrix("vessel_pin_link"))
cyl_head = K.empty("cylinder_head_link", body, ref_matrix("cylinder_head_link"))
cyl_rod = K.empty("cylinder_rod_link", cyl_head, ref_matrix("cylinder_rod_link"))

# ------------------------------------------------------------------ undercarriage (body frame, ground z = 0)
SPROCKET = ((-2.35, 0.65), 0.33)  # at the sprocket links
IDLER = ((1.62, 0.47), 0.38)
ROLLERS = [((x, 0.24), 0.15) for x in (-1.9, -1.3, -0.7, -0.1, 0.5, 1.05)]
BELT_T, LUG_H, TW = 0.05, 0.04, 0.6  # belt thickness, lug height, track width


def tracks():
    inner, length = K.belt_path([SPROCKET, IDLER] + ROLLERS, 480)
    c = inner.mean(axis=0)
    outer = []
    for p, t, n in K.belt_frames(inner, length, len(inner)):
        outer.append(tuple(p + n * BELT_T))
    inner_l = [tuple(p) for p in inner]
    n_lugs = int(length / 0.14)
    for side, yc in (("l", 1.0), ("r", -1.0)):
        K.make(f"track_belt_{side}", body, M["rubber"], lambda bm, yc=yc: K.add_ring_y(bm, inner_l, outer, yc - TW / 2, yc + TW / 2), bevel=0.01)

        def lugs(bm, yc=yc):
            for k, (p, t, n) in enumerate(K.belt_frames(inner, length, n_lugs)):
                o = 0.11 if k % 2 else -0.11
                ctr = p + n * (BELT_T + LUG_H / 2)
                for dy, hw in ((o, 0.17), (-o * 1.9, 0.05)):
                    K.add_obox(bm, (ctr[0], yc + dy, ctr[1]), ((t[0], 0, t[1]), (0, 1, 0), (n[0], 0, n[1])), (0.035, hw, LUG_H / 2))

        K.make(f"track_lugs_{side}", body, M["rubber"], lugs, bevel=0.008)

        def wheels(bm, yc=yc):
            (ix, iz), ir = IDLER
            K.add_cyl(bm, ir - 0.02, (ix, yc - 0.27, iz), (ix, yc - 0.06, iz), 40)
            K.add_cyl(bm, ir - 0.02, (ix, yc + 0.06, iz), (ix, yc + 0.27, iz), 40)
            K.add_cyl(bm, 0.12, (ix, yc - 0.3, iz), (ix, yc + 0.3, iz), 20)
            for (x, z), r in ROLLERS:
                K.add_cyl(bm, r - 0.01, (x, yc - 0.25, z), (x, yc - 0.06, z), 24)
                K.add_cyl(bm, r - 0.01, (x, yc + 0.06, z), (x, yc + 0.25, z), 24)
                K.add_cyl(bm, 0.06, (x, yc - 0.28, z), (x, yc + 0.28, z), 12)
            # track frame between the wheels, and the tensioner housing behind the idler
            K.add_prism_y(bm, K.fillet([(-2.05, 0.36), (1.2, 0.36), (1.35, 0.5), (1.15, 0.72), (-1.95, 0.72), (-2.15, 0.55)], 0.06), yc - 0.2, yc + 0.2)
            K.add_box(bm, (1.0, yc - 0.15, 0.4), (1.4, yc + 0.15, 0.56))

        K.make(f"track_frame_{side}", body, M["frame"], wheels, bevel=0.012)

    for s, yc in (("left", 1.0), ("right", -1.0)):
        sgn = 1 if yc > 0 else -1

        def sprocket(bm, sgn=sgn):
            # in the sprocket frame (axle along y): gear between the rubber's guide lugs, hub, drive motor inboard
            K.add_gear(bm, (0.0, 0.0), 0, SPROCKET[1] - 0.05, SPROCKET[1] + 0.005, 13, -0.05, 0.05)
            K.add_cyl(bm, 0.2, (0, -0.16, 0), (0, 0.16, 0), 32)
            K.add_cyl(bm, 0.24, (0, -sgn * 0.16, 0), (0, -sgn * 0.42, 0), 32)
            for k in range(10):
                a = 2 * math.pi * k / 10
                K.add_cyl(bm, 0.014, (0.15 * math.cos(a), sgn * 0.16, 0.15 * math.sin(a)), (0.15 * math.cos(a), sgn * 0.175, 0.15 * math.sin(a)), 6)

        K.make(f"sprocket_{s}", spro[s], M["frame"], sprocket, bevel=0.006)


tracks()

# ------------------------------------------------------------------ chassis, cab, engine bay


def chassis():
    K.make("frame_rails", body, M["frame"], lambda bm: (K.add_box(bm, (-3.1, -0.62, 0.72), (3.3, -0.4, 1.0)), K.add_box(bm, (-3.1, 0.4, 0.72), (3.3, 0.62, 1.0)), K.add_box(bm, (-2.6, -1.0, 0.7), (-2.2, 1.0, 0.95)), K.add_box(bm, (0.2, -1.0, 0.7), (0.6, 1.0, 0.95))), bevel=0.015)
    K.make("deck", body, M["frame"], lambda bm: K.add_box(bm, (0.45, -1.32, 1.0), (3.3, 1.32, 1.1)), bevel=0.02)
    # front bumper with tow hook and lamps
    K.make("bumper", body, M["frame"], lambda bm: (K.add_box(bm, (3.25, -1.28, 0.72), (3.45, 1.28, 1.08)), K.add_tube(bm, [(3.45, 0.0 + 0.09 * math.sin(a), 0.84 - 0.09 * math.cos(a)) for a in np.linspace(0, math.pi, 9)], 0.022, 8)), bevel=0.02)
    for s in (-1, 1):
        K.make(f"bumper_chevrons_{'l' if s > 0 else 'r'}", body, M["hazard_y"], lambda bm, s=s: [K.add_prism_frame(bm, [(y0, 0), (y0 + 0.09, 0), (y0 + 0.09 + 0.07, 0.2), (y0 + 0.07, 0.2)], (3.452, 0, 0.8), (0, -1, 0), (0, 0, 1), 0, 0.005) for y0 in np.arange(0.35, 1.05, 0.18) * s if True], smooth=False)
    K.make("front_lamps", body, M["lens"], lambda bm: [K.add_box(bm, (3.45, s * 1.05 - 0.08, 0.95), (3.46, s * 1.05 + 0.08, 1.04)) for s in (-1, 1)])

    # engine bay (right): stepped hood, louvres, grille, exhaust, pre-cleaner
    eb = K.fillet([(0.5, 1.1), (3.25, 1.1), (3.25, 1.75), (3.05, 2.02), (0.5, 2.02)], 0.06, 3)
    K.make("engine_bay", body, M["paint"], lambda bm: K.add_prism_y(bm, eb, -1.32, 0.04), bevel=0.035, segs=3)

    def louvres(bm):
        y = -1.32
        for k in range(10):
            z = 1.25 + k * 0.055
            K.add_obox(bm, (1.6, y - 0.012, z), ((1, 0, 0), (0, -math.cos(0.5), -math.sin(0.5)), (0, -math.sin(0.5), math.cos(0.5))), (0.6, 0.016, 0.006))
        K.add_box(bm, (1.0, y - 0.004, 1.2), (2.2, y + 0.004, 1.85))
        for x in np.arange(0.75, 2.95, 0.09):  # top grille bars
            K.add_box(bm, (x, -1.15, 2.02), (x + 0.035, -0.15, 2.04))

    K.make("engine_grilles", body, M["frame"], louvres, bevel=0.003)
    K.make("exhaust", body, M["frame"], lambda bm: (K.add_tube(bm, K.catmull([(0.85, -0.95, 2.0), (0.85, -0.95, 2.55), (0.9, -0.95, 2.78), (1.05, -0.95, 2.85)], 4), 0.06, 16), K.add_cyl(bm, 0.085, (0.85, -0.95, 2.0), (0.85, -0.95, 2.1), 16)))
    K.make("pre_cleaner", body, M["plastic"], lambda bm: (K.add_cyl(bm, 0.045, (1.5, -1.0, 2.02), (1.5, -1.0, 2.35), 16), K.add_cyl(bm, 0.11, (1.5, -1.0, 2.35), (1.5, -1.0, 2.47), 24, r1=0.06)), bevel=0.01)

    def rails(bm):
        y = -1.27
        for x in (0.6, 1.5, 2.4, 3.1):
            K.add_tube(bm, [(x, y, 2.02), (x, y, 2.62)], 0.02, 10)
        K.add_tube(bm, [(0.6, y, 2.62), (3.1, y, 2.62)], 0.02, 10)
        K.add_tube(bm, [(0.6, y, 2.32), (3.1, y, 2.32)], 0.016, 10)

    K.make("handrails", body, M["hazard_y"], rails)

    # cab (left): raked windscreen, glazing, roof, door, mirrors, steps
    x0, x1, y0, y1, z0, z1 = 1.12, 3.02, 0.14, 1.3, 1.1, 2.78
    xr = x1 - 0.12
    prof = K.fillet([(x0, z0), (x1, z0), (x1, 1.55), (xr, z1), (x0, z1)], 0.07, 4)
    K.make("cab_shell", body, M["cab"], lambda bm: K.add_prism_y(bm, prof, y0, y1), bevel=0.025, segs=3)
    K.make("cab_roof", body, M["cab"], lambda bm: K.add_box(bm, (x0 - 0.04, y0 - 0.04, z1 - 0.01), (xr + 0.08, y1 + 0.04, z1 + 0.05)), bevel=0.015)
    fx = lambda z: x1 if z <= 1.55 else x1 - (x1 - xr) * (z - 1.55) / (z1 - 1.55)  # noqa: E731

    def glass(bm):
        e = 0.006
        p0, p1 = Vector((fx(1.6), 0, 1.6)), Vector((fx(2.7), 0, 2.7))
        u = (p1 - p0).normalized()
        n = Vector((u.z, 0, -u.x))
        mid = (p0 + p1) / 2 + n * 0.003
        mid.y = (y0 + y1) / 2
        K.add_obox(bm, mid, (u, (0, 1, 0), n), ((p1 - p0).length / 2, (y1 - y0) / 2 - 0.08, e / 2))
        K.add_box(bm, (x1, y0 + 0.08, 1.2), (x1 + e, y1 - 0.08, 1.5))
        K.add_prism_y(bm, [(1.95, 1.3), (fx(1.3) - 0.07, 1.3), (fx(2.7) - 0.07, 2.7), (1.95, 2.7)], y1, y1 + e)  # door
        K.add_prism_y(bm, [(1.2, 1.7), (1.85, 1.7), (1.85, 2.7), (1.2, 2.7)], y1, y1 + e)
        K.add_box(bm, (x0 - e, y0 + 0.1, 1.75), (x0, y1 - 0.1, 2.68))  # rear (over the headboard)

    K.make("cab_glass", body, M["glass"], glass, smooth=False)
    K.make("cab_interior", body, M["plastic"], lambda bm: (K.add_box(bm, (1.4, 0.45, 1.1), (1.75, 1.0, 2.0)), K.add_box(bm, (2.3, 0.3, 1.1), (2.85, 1.15, 1.6))), bevel=0.02)

    def cab_trim(bm):
        K.add_box(bm, (1.9, y1, 1.15), (1.92, y1 + 0.008, 2.74))
        K.add_box(bm, (2.55, y1 + 0.008, 1.95), (2.67, y1 + 0.03, 1.98))
        for y in (y0 + 0.035, y1 - 0.035):
            K.add_tube(bm, [(x1 + 0.004, y, 1.15), (x1 + 0.004, y, 1.55), (xr + 0.004, y, z1 - 0.04)], 0.022, 8)
        for s, y in ((1, y1), (-1, -1.32)):
            K.add_tube(bm, [(3.0, y, 2.2), (3.2, y + s * 0.15, 2.3), (3.25, y + s * 0.22, 2.35)], 0.014, 8)
            K.add_box(bm, (3.22, y + s * 0.22 - 0.1, 2.2), (3.26, y + s * 0.22 + 0.1, 2.5))
        for z in (0.55, 0.85):  # steps up to the door
            K.add_box(bm, (2.05, y1 + 0.0, z), (2.5, y1 + 0.17, z + 0.03))
        K.add_box(bm, (2.03, y1, 0.45), (2.06, y1 + 0.17, 1.05))
        K.add_box(bm, (2.49, y1, 0.45), (2.52, y1 + 0.17, 1.05))

    K.make("cab_trim", body, M["plastic"], cab_trim, bevel=0.004)

    # PWRI automation retrofit: GNSS antennas and whips on the cab roof, signal tower, sensor bar
    for x in (1.35, 2.6):
        K.make(f"gnss_{x:.0f}", body, M["white"], lambda bm, x=x: (K.add_cyl(bm, 0.02, (x, 0.72, z1 + 0.05), (x, 0.72, z1 + 0.25), 10), K.add_cyl(bm, 0.1, (x, 0.72, z1 + 0.25), (x, 0.72, z1 + 0.31), 32, r1=0.07)), bevel=0.01)
    K.make("whips", body, M["plastic"], lambda bm: [K.add_cyl(bm, 0.008, (x, 0.3, z1 + 0.05), (x, 0.3, z1 + 0.75), 6) for x in (1.6, 2.2)])
    for k, col in enumerate(("red", "amber")):
        K.make(f"signal_{col}", body, M[col], lambda bm, k=k: (K.add_cyl(bm, 0.045, (1.25, 1.0 + 0.12 * k, z1 + 0.05), (1.25, 1.0 + 0.12 * k, z1 + 0.15), 20), K.add_sphere(bm, 0.045, (1.25, 1.0 + 0.12 * k, z1 + 0.15), 20, 8)))
    K.make("tail_lamps", body, M["red"], lambda bm: [K.add_box(bm, (-3.12, s * 0.5 - 0.06, 0.8), (-3.1, s * 0.5 + 0.06, 0.9)) for s in (-1, 1)])

    # tip cylinder bracket on the chassis cross member
    hx, hz = ref_matrix("cylinder_head_link").translation.x, ref_matrix("cylinder_head_link").translation.z
    K.make("tip_cyl_bracket", body, M["frame"], lambda bm: [K.add_prism_y(bm, K.circle_hull([((hx, hz), 0.09)], extra=[(hx - 0.25, 0.72), (hx + 0.25, 0.72)]), y - 0.02, y + 0.02) for y in (-0.08, 0.08)], bevel=0.008)


chassis()

# ------------------------------------------------------------------ tip cylinder (frames of the source)
TIP_AXIS = Vector((-1.4989, 0, 0.51788)).normalized()  # cylinder_rod_prismatic_joint axis
TIP_REACH = 1.25  # rod eye along the axis at rest: under the vessel floor


def tip_cylinder():
    import bmesh

    bb, br = bmesh.new(), bmesh.new()
    K.cylinder_meshes(bb, br, (0, 0, 0), TIP_AXIS, 0.085, 0.048, 0.72, 0.42, TIP_REACH, eye_r=0.06, eye_w=0.12)
    for name, bmx, parent, mat in (("tip_cyl_barrel", bb, cyl_head, M["paint"]), ("tip_cyl_rod", br, cyl_rod, M["chrome"])):
        me = bpy.data.meshes.new(name)
        K.box_uv(bmx)
        bmx.to_mesh(me)
        bmx.free()
        for p in me.polygons:
            p.use_smooth = True
        me.use_auto_smooth = True
        me.materials.append(mat)
        K.make(name, parent, mat, None, mesh=me, bevel=0.004)


tip_cylinder()

# ------------------------------------------------------------------ vessel (vessel_pin_link frame; server boxes)
FLOOR_TOP, X0, X1, HW_IN, HW_OUT, SIDE_TOP, HEAD_TOP = 0.354, -0.405, 3.555, 1.175, 1.255, 1.154, 1.654


def vessel_parts():
    K.make("vessel_floor", vessel, M["liner"], lambda bm: K.add_box(bm, (X0, -HW_OUT, 0.314), (X1, HW_OUT, FLOOR_TOP)), bevel=0.008)
    for s in (-1, 1):
        K.make(f"vessel_side_{'l' if s > 0 else 'r'}", vessel, M["vessel"], lambda bm, s=s: K.add_prism_y(bm, [(X0 - 0.45, FLOOR_TOP + 0.2), (X0, FLOOR_TOP), (X1, FLOOR_TOP), (X1, SIDE_TOP), (X0 - 0.45, SIDE_TOP)], s * HW_IN, s * HW_OUT), bevel=0.01)
        K.make(f"vessel_side_liner_{'l' if s > 0 else 'r'}", vessel, M["liner"], lambda bm, s=s: K.add_box(bm, (X0, s * (HW_IN - 0.004), FLOOR_TOP), (X1 - 0.08, s * HW_IN, SIDE_TOP - 0.04)))

    def ribs(bm):
        for s in (-1, 1):
            for x in np.arange(0.0, 3.5, 0.58):
                K.add_box(bm, (x - 0.03, s * HW_OUT, FLOOR_TOP - 0.04), (x + 0.03, s * (HW_OUT + 0.05), SIDE_TOP - 0.02))
            K.add_tube(bm, [(X0 - 0.45, s * (HW_OUT - 0.04), SIDE_TOP), (X1, s * (HW_OUT - 0.04), SIDE_TOP)], 0.045, 10)  # top rail
            K.add_box(bm, (X0 - 0.4, s * HW_OUT, FLOOR_TOP - 0.06), (X1, s * (HW_OUT + 0.06), FLOOR_TOP + 0.04))  # bottom rail

    K.make("vessel_ribs", vessel, M["vessel"], ribs, bevel=0.008)
    # headboard, higher over the cab, with stiffeners
    K.make("vessel_headboard", vessel, M["vessel"], lambda bm: K.add_prism_y(bm, [(X1 - 0.08, FLOOR_TOP), (X1, FLOOR_TOP), (X1 + 0.15, 1.93), (X1 + 0.07, 1.93)], -HW_OUT, HW_OUT), bevel=0.012)
    K.make("vessel_headboard_ribs", vessel, M["vessel"], lambda bm: [K.add_prism_y(bm, [(X1, FLOOR_TOP + 0.05), (X1 + 0.12, 1.2), (X1 + 0.2, 1.9), (X1 + 0.15, 1.93), (X1 + 0.02, 1.0)], y - 0.03, y + 0.03) for y in (-0.9, -0.3, 0.3, 0.9)], bevel=0.006)
    # tail lip rising to the rear (the server's sloped tail box)
    a = math.radians(25)
    p0 = np.array([-0.325, 0.314])
    p1 = p0 + 0.6 * np.array([-math.cos(a), math.sin(a)])
    nrm = np.array([math.sin(a), math.cos(a)])
    K.make("vessel_tail", vessel, M["liner"], lambda bm: K.add_prism_y(bm, [tuple(p0), tuple(p1), tuple(p1 + nrm * 0.08), tuple(p0 + nrm * 0.08 + np.array([0.08, 0]))], -HW_OUT, HW_OUT), bevel=0.01)
    # underframe: two beams to the hinge, cross members, the tip cylinder's bracket
    eye = (ref_matrix("cylinder_head_link").translation + TIP_AXIS * TIP_REACH) - ref_matrix("vessel_pin_link").translation

    def under(bm):
        for y in (-0.78, 0.78):
            K.add_prism_y(bm, K.fillet([(-0.12, -0.08), (0.2, -0.1), (3.4, 0.22), (3.4, 0.314), (-0.3, 0.314)], 0.05), y - 0.07, y + 0.07)
        for x in (0.9, 1.9, 2.9):
            K.add_box(bm, (x - 0.05, -0.85, 0.22), (x + 0.05, 0.85, 0.314))
        for y in (-0.08, 0.08):
            K.add_prism_y(bm, K.circle_hull([((eye.x, eye.z), 0.08)], extra=[(eye.x - 0.25, 0.314), (eye.x + 0.25, 0.314)]), y - 0.02, y + 0.02)

    K.make("vessel_underframe", vessel, M["frame"], under, bevel=0.01)
    K.make("vessel_hinge", vessel, M["steel"], lambda bm: [K.add_cyl(bm, 0.07, (0, y - 0.1, 0), (0, y + 0.1, 0), 20) for y in (-0.78, 0.78)])
    K.make("vessel_chevrons", vessel, M["hazard_y"], lambda bm: [K.add_prism_frame(bm, [(y0, 0), (y0 + 0.1, 0), (y0 + 0.18, 0.18), (y0 + 0.08, 0.18)], (X0 - 0.47, 0, SIDE_TOP - 0.3), (0, 1, 0), (0, 0, 1), 0, -0.005) for y0 in np.arange(-1.1, 1.0, 0.24)], smooth=False)


vessel_parts()

# ------------------------------------------------------------------ export
K.export_glb(OUT)
js, binchunk = K.read_glb(OUT)
roots = js["scenes"][0]["nodes"]
js["nodes"].append({"name": "ic120", "matrix": REFN["ic120"]["matrix"], "children": roots})
js["scenes"][0]["nodes"] = [len(js["nodes"]) - 1]
names = {n.get("name"): i for i, n in enumerate(js["nodes"])}
for name, ref in REFN.items():
    if name in names and ref.get("extras"):
        js["nodes"][names[name]]["extras"] = ref["extras"]
js["asset"]["copyright"] = (
    "Rubber-tracked crawler carrier (11 t class), modelled procedurally in Blender (assets/blender/ic120_hd.py) by the "
    "construction-site-simulation project to the joint frames of PWRI's OPERA IC120 model (https://github.com/pwri-opera/ic120_ros2, Apache-2.0). Apache-2.0."
)
js["asset"]["extras"] = {"source": "assets/blender/ic120_hd.py", "reference": "https://github.com/pwri-opera/ic120_ros2", "license": "Apache-2.0"}
K.write_glb(OUT, js, binchunk)
print(f"EXPORTED {OUT} nodes={len(js['nodes'])}")

if "--render" in ARGS:
    out = ARGS[ARGS.index("--render") + 1]
    tip = float(ARGS[ARGS.index("--tip") + 1]) if "--tip" in ARGS else 0.0
    vessel.matrix_basis = ref_matrix("vessel_pin_link") @ Matrix.Rotation(-tip, 4, "Y")
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
    import bmesh as _bm

    b = _bm.new()
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
    for vname, loc in {"front_left": (8.5, 7.5, 4.0), "rear_right": (-9.0, -7.5, 4.5), "side": (0, -13, 1.6)}.items():
        cam.location = loc
        cam.rotation_euler = (Vector((0, 0, 1.2)) - Vector(loc)).to_track_quat("-Z", "Y").to_euler()
        sc.render.filepath = f"{out}_{vname}.png"
        bpy.ops.render.render(write_still=True)
