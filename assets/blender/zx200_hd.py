"""Detailed, articulated 20 t class hydraulic excavator built procedurally in Blender, exported as
assets/machinery/zx200_hd.glb.

It is a drop-in replacement for assets/machinery/opera_zx200.glb (PWRI OPERA ZX200 meshes): the same node
names, link frames (read from that file), joint extras and free parts, so the simulator (phase2), the soil
view (phase1/web/soil.js) and the showcase linkage (phase1/web/linkage.js) take it unchanged:

  zx200 (Z up -> Y up) / base_link / track_link / body_link / boom_link / arm_link / bucket_link / bucket_end_link
  free parts at the root, posed from the pins: boom_barrel, boom_rod, arm_barrel, arm_rod, bucket_barrel,
  bucket_rod, side_link, guide_link (mesh frames as phase2/hydraulics.py CYLINDER_MESH and LINK_MESH)

Dimensions follow the OPERA model (which follows the machine) and its pin table; the parts themselves are
modelled here: box-section boom and arm, bucket with teeth and wear strips, cylinders with glands, ports and
chrome rods, an undercarriage with individual track shoes, rollers, idlers and sprockets, the house with cab
glazing, counterweight, louvres, rails, mirrors, and PWRI's automation retrofit (GNSS antennas, signal tower).
No manufacturer marks or livery.

    blender -b -t 4 --python assets/blender/zx200_hd.py                       # build and export
    blender -b -t 4 --python assets/blender/zx200_hd.py -- --render out/prefix # also render review images
    blender -b -t 4 --python assets/blender/zx200_hd.py -- --pose 0.3,-0.6,2.1,2.2 --render out/prefix
"""

import math
import pathlib
import sys

import numpy as np
from mathutils import Matrix, Vector

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
import blendkit as K  # noqa: E402

ROOT = pathlib.Path(__file__).resolve().parents[2]
REF = ROOT / "assets/machinery/opera_zx200.glb"
OUT = ROOT / "assets/machinery/zx200_hd.glb"
ARGS = sys.argv[sys.argv.index("--") + 1 :] if "--" in sys.argv else []

REFJ, _ = K.read_glb(REF)
REFN = {n["name"]: n for n in REFJ["nodes"]}


def ref_matrix(name):
    m = REFN[name].get("matrix")
    return Matrix(np.array(m).reshape(4, 4).T.tolist()) if m else Matrix.Identity(4)


# ------------------------------------------------------------------ pins (phase2/hydraulics.py)
PINS = {
    "boom_head": ("house", 0.686, 0.247),
    "boom_rod": ("boom", 1.776, 2.249),
    "arm_head": ("boom", 2.276, 2.789),
    "arm_rod": ("stick", -0.778, -0.367),
    "bucket_head": ("stick", -0.248, 0.733),
    "guide": ("stick", 1.577, 1.878),
    "side": ("bucket", -0.328, 0.548),
}
GUIDE_LEN, SIDE_LEN = 0.561, 0.583
# mesh frames of the free parts: head eye at the origin, axis in x-z, pin-to-pin length as modelled
CYL = {
    #        axis dir            l_mesh  y centres        barrel r  rod r   barrel len rod len (strokes over the joint limits)
    "boom": ((0.3425, 0.9395), 2.73, (-0.32, 0.32), 0.085, 0.0425, 1.94, 1.60),  # L 2.07..3.35 m
    "arm": ((0.9486, 0.3165), 1.70, (0.0,), 0.095, 0.0475, 1.75, 1.72),  # L 1.88..3.35 m
    "bucket": ((0.6761, 0.7368), 2.16, (0.0,), 0.080, 0.040, 1.54, 1.35),  # L 1.67..2.71 m
}
LINK_MESH = {"side": ((0.0, 0.0), (-0.328, 0.482)), "guide": ((0.0, 0.0), (-0.338, -0.448))}
BOOM_FOOT, ARM_PIN = (0.0, 0.0), (4.84, 3.0023)  # boom frame
BUCKET_PIN = (1.9155, 2.1737)  # arm frame
BUCKET_TIP = (1.13012, 0.93856)  # bucket frame (bucket_end_link)

# ------------------------------------------------------------------ materials
K.reset()
M = {
    "paint": K.surface("paint_orange", "#e2741c", "paint", 1024, seed=1, coat=0.35),
    "paint_dark": K.surface("paint_graphite", "#2b2e31", "paint", 512, seed=2, coat=0.2),
    "cab": K.surface("paint_cab_grey", "#7b8186", "paint", 512, seed=3, coat=0.25),
    "frame": K.surface("steel_frame", "#1e2023", "paint_worn", 1024, seed=4),
    "track": K.surface("track_steel", "#38312a", "track", 1024, seed=5),
    "steel": K.material("steel_machined", "#9a9da0", rough=0.3, metal=1.0),
    "chrome": K.material("chrome_rod", "#e6e8ea", rough=0.08, metal=1.0),
    "bucket": K.surface("bucket_steel", "#2a2c2f", "steel", 1024, seed=6),
    "wear": K.surface("wear_steel", "#6a665f", "worn_edge", 512, seed=7),
    "glass": K.material("glass_tinted", "#16222a", rough=0.04, alpha=0.55),
    "rubber": K.surface("rubber_black", "#151515", "rubber", 512, seed=8),
    "plastic": K.material("plastic_black", "#0d0e0f", rough=0.55),
    "white": K.material("antenna_white", "#e8e8e4", rough=0.35),
    "lens": K.material("lamp_lens", "#f4f1e4", rough=0.1, emit="#fff4dc", emit_strength=0.6),
    "red": K.material("lamp_red", "#c4161c", rough=0.2, emit="#ff2010", emit_strength=0.4),
    "amber": K.material("lamp_amber", "#f0a020", rough=0.2, emit="#ffa000", emit_strength=0.4),
    "green": K.material("lamp_green", "#1e9a3c", rough=0.2, emit="#20ff60", emit_strength=0.4),
    "blue": K.material("lamp_blue", "#1c58c8", rough=0.2, emit="#2070ff", emit_strength=0.4),
    "hazard_y": K.material("hazard_yellow", "#f0c010", rough=0.45, coat=0.2),
    "hazard_k": K.material("hazard_black", "#141414", rough=0.5, coat=0.2),
}

# ------------------------------------------------------------------ hierarchy (as the OPERA model)
base = K.empty("base_link")
track = K.empty("track_link", base, ref_matrix("track_link"))
body = K.empty("body_link", track, ref_matrix("body_link"))
boom = K.empty("boom_link", body, ref_matrix("boom_link"))
arm = K.empty("arm_link", boom, ref_matrix("arm_link"))
bucket = K.empty("bucket_link", arm, ref_matrix("bucket_link"))
K.empty("bucket_end_link", bucket, ref_matrix("bucket_end_link"))
FREE = {n: K.empty(n) for n in ("boom_barrel", "boom_rod", "arm_barrel", "arm_rod", "bucket_barrel", "bucket_rod", "side_link", "guide_link")}


def pin_y(parent, name, mat, c, r, y0, y1, segs=24, bevel=0.006):
    """A cross pin or boss along y at (x, z) = c."""
    K.make(name, parent, mat, lambda bm: K.add_cyl(bm, r, (c[0], y0, c[1]), (c[0], y1, c[1]), segs), bevel=bevel)


def pin_with_caps(parent, name, c, r, half):
    """Steel pin through a joint, with retaining end plates and bolts."""

    def build(bm):
        K.add_cyl(bm, r, (c[0], -half, c[1]), (c[0], half, c[1]), 20)
        for s in (-1, 1):
            K.add_cyl(bm, r * 1.6, (c[0], s * half, c[1]), (c[0], s * (half + 0.018), c[1]), 20)
            K.add_cyl(bm, 0.012, (c[0] + r * 0.9, s * (half + 0.018), c[1]), (c[0] + r * 0.9, s * (half + 0.03), c[1]), 6)

    K.make(name, parent, M["steel"], build, bevel=0.003)


# ================================================================== undercarriage (track_link: ground z = 0)
GAUGE, SHOE_W = 2.2, 0.8
XS, XI, RP, ZP = -1.62, 1.75, 0.36, 0.10  # sprocket and idler x, pitch radius, bottom pitch line
ZC, ZT = ZP + RP, ZP + 2 * RP
LOOP = 2 * (XI - XS) + 2 * math.pi * RP
N_SHOES = 47
PITCH = LOOP / N_SHOES


def loop_pose(s):
    """Point, tangent and outward normal (x-z) along the track loop at arc length s."""
    s %= LOOP
    a = XI - XS
    arc = math.pi * RP
    if s < a:
        return (XS + s, ZP), (1, 0), (0, -1)
    s -= a
    if s < arc:
        f = s / RP
        return (XI + RP * math.sin(f), ZC - RP * math.cos(f)), (math.cos(f), math.sin(f)), (math.sin(f), -math.cos(f))
    s -= arc
    if s < a:
        return (XI - s, ZT), (-1, 0), (0, 1)
    s -= a
    f = s / RP
    return (XS - RP * math.sin(f), ZC + RP * math.cos(f)), (-math.cos(f), -math.sin(f)), (-math.sin(f), math.cos(f))


def build_shoe(bm):
    """One track shoe and its chain link, local frame: x along the track, y across, z outward."""
    h = PITCH * 0.5
    K.add_box(bm, (-h * 0.96, -SHOE_W / 2, 0.055), (h * 0.96, SHOE_W / 2, 0.075))  # plate
    for gx in (-0.055, 0.0, 0.055):  # triple grouser
        K.add_prism_frame(bm, [(gx - 0.012, 0.075), (gx + 0.012, 0.075), (gx + 0.006, 0.1), (gx - 0.006, 0.1)], (0, -SHOE_W / 2, 0), (1, 0, 0), (0, 0, 1), 0, -SHOE_W)
    for s in (-1, 1):  # link rails
        y = s * 0.25
        K.add_prism_frame(bm, [(-h, -0.03), (h, -0.03), (h * 1.02, 0.02), (h * 0.7, 0.055), (-h * 0.7, 0.055), (-h * 1.02, 0.02)], (0, y - 0.03, 0), (1, 0, 0), (0, 0, 1), 0, -0.06)
    K.add_cyl(bm, 0.034, (h, -0.22, 0), (h, 0.22, 0), 10)  # bushing
    for s in (-1, 1):
        K.add_cyl(bm, 0.022, (h, s * 0.28, 0), (h, s * 0.29, 0), 8)  # pin ends
    for bx in (-0.045, 0.045):  # shoe bolts
        for by in (-0.29, -0.21, 0.21, 0.29):
            K.add_cyl(bm, 0.011, (bx, by, 0.055), (bx, by, 0.045), 6)


def undercarriage():
    import bpy

    shoe_mesh = None
    shoes = {}
    for side, s in (("L", 1), ("R", -1)):
        yc = s * GAUGE / 2
        names = []
        for k in range(N_SHOES):
            (px, pz), (tx, tz), (nx, nz) = loop_pose(k * PITCH)
            X, Z = Vector((tx, 0, tz)), Vector((nx, 0, nz))
            Y = Z.cross(X)
            mat = Matrix.Identity(4)
            for c, v in enumerate((X, Y, Z)):
                mat[0][c], mat[1][c], mat[2][c] = v
            mat.translation = (px, yc, pz)
            name = f"track_shoe_{side}_{k:02d}"
            ob = K.make(name, track, M["track"], build_shoe, matrix=mat, mesh=shoe_mesh)
            shoe_mesh = ob.data
            names.append(name)
        shoes[side] = names

        # side frame: box section between the idler and sprocket, sloped top to shed spoil
        K.make(f"track_frame_{side}", track, M["frame"], lambda bm, yc=yc: K.add_prism_y(bm, K.fillet([(-1.42, 0.36), (1.42, 0.36), (1.62, 0.47), (1.48, 0.64), (-1.3, 0.64), (-1.46, 0.52)], 0.05), yc - 0.24, yc + 0.24), bevel=0.02)
        K.make(f"track_frame_top_{side}", track, M["frame"], lambda bm, yc=yc: K.add_prism_frame(bm, [(-1.25, 0.0), (1.38, 0.0), (1.38, 0.05), (-1.25, 0.05)], (0, yc - 0.25, 0.64), (1, 0, 0), (0, 0, 1), 0, -0.5), bevel=0.015)

        def rollers(bm, yc=yc):
            for x in np.linspace(-1.2, 1.3, 7):  # track rollers, double flange running on the rails
                zr = ZP + 0.055 + 0.09
                for e in (-1, 1):
                    K.add_cyl(bm, 0.09, (x, yc + e * 0.215, zr), (x, yc + e * 0.285, zr), 20)
                    K.add_cyl(bm, 0.112, (x, yc + e * 0.285, zr), (x, yc + e * 0.305, zr), 20)
                    K.add_cyl(bm, 0.112, (x, yc + e * 0.195, zr), (x, yc + e * 0.215, zr), 20)
                K.add_cyl(bm, 0.07, (x, yc - 0.195, zr), (x, yc + 0.195, zr), 16)
                K.add_box(bm, (x - 0.07, yc - 0.33, zr + 0.03), (x + 0.07, yc - 0.3, 0.37))  # mounting brackets
                K.add_box(bm, (x - 0.07, yc + 0.3, zr + 0.03), (x + 0.07, yc + 0.33, 0.37))
            for x in (-0.45, 0.75):  # carrier rollers under the top run
                zr = ZT - 0.055 - 0.075
                K.add_cyl(bm, 0.075, (x, yc - 0.13, zr), (x, yc + 0.13, zr), 20)
                K.add_cyl(bm, 0.09, (x, yc - 0.02, zr), (x, yc + 0.02, zr), 20)
                K.add_box(bm, (x - 0.05, yc - 0.16, 0.6), (x + 0.05, yc + 0.16, zr - 0.03))

        K.make(f"track_rollers_{side}", track, M["frame"], rollers, bevel=0.008)

        def idler(bm, yc=yc):
            K.add_cyl(bm, RP - 0.055, (XI, yc - 0.3, ZC), (XI, yc + 0.3, ZC), 40)
            K.add_cyl(bm, RP - 0.02, (XI, yc - 0.05, ZC), (XI, yc + 0.05, ZC), 40)
            K.add_cyl(bm, 0.1, (XI, yc - 0.34, ZC), (XI, yc + 0.34, ZC), 16)
            K.add_box(bm, (XI - 0.55, yc - 0.2, ZC - 0.08), (XI - 0.1, yc + 0.2, ZC + 0.08))  # yoke to the recoil spring

        K.make(f"track_idler_{side}", track, M["frame"], idler, bevel=0.01)

        def sprocket(bm, yc=yc, s=s):
            K.add_gear(bm, (XS, ZC), 0, RP - 0.035, RP + 0.03, 21, yc - 0.045, yc + 0.045)
            K.add_cyl(bm, 0.2, (XS, yc - 0.12, ZC), (XS, yc + 0.12, ZC), 32)
            # travel motor and reduction housing, inboard
            K.add_cyl(bm, 0.3, (XS, yc - s * 0.12, ZC), (XS, yc - s * 0.45, ZC), 40)
            K.add_cyl(bm, 0.22, (XS, yc - s * 0.45, ZC), (XS, yc - s * 0.6, ZC), 32)
            for k in range(16):
                a = 2 * math.pi * k / 16
                x, z = XS + 0.25 * math.cos(a), ZC + 0.25 * math.sin(a)
                K.add_cyl(bm, 0.016, (x, yc - s * 0.45, z), (x, yc - s * 0.47, z), 6)
            for k in range(12):  # sprocket bolts
                a = 2 * math.pi * k / 12
                x, z = XS + 0.16 * math.cos(a), ZC + 0.16 * math.sin(a)
                K.add_cyl(bm, 0.015, (x, yc + s * 0.12, z), (x, yc + s * 0.14, z), 6)

        K.make(f"track_sprocket_{side}", track, M["frame"], sprocket, bevel=0.006)

        # recoil spring housing cover behind the idler
        K.make(f"track_recoil_cover_{side}", track, M["frame"], lambda bm, yc=yc: K.add_box(bm, (0.95, yc - 0.22, 0.42), (1.45, yc + 0.22, 0.62)), bevel=0.01)

    def carbody(bm):
        K.add_prism_z(bm, K.fillet([(-1.05, -0.88), (1.05, -0.88), (1.05, 0.88), (-1.05, 0.88)], 0.1), 0.52, 0.84)
        K.add_prism_z(bm, K.fillet([(-0.7, -0.75), (0.7, -0.75), (0.7, 0.75), (-0.7, 0.75)], 0.25), 0.84, 0.94)
        K.add_cyl(bm, 0.92, (0, 0, 0.94), (0, 0, 1.04), 64)  # swing bearing outer race
        for k in range(36):
            a = 2 * math.pi * k / 36
            K.add_cyl(bm, 0.018, (0.82 * math.cos(a), 0.82 * math.sin(a), 0.94), (0.82 * math.cos(a), 0.82 * math.sin(a), 0.965), 6)
        K.add_cyl(bm, 0.18, (0, 0, 0.3), (0, 0, 0.55), 24)  # centre swivel joint

    K.make("carbody", track, M["frame"], carbody, bevel=0.02)
    return shoes


SHOES = undercarriage()
import bpy  # noqa: E402

# ================================================================== house (body_link: swing bearing top at z = 0)
HW = 1.33  # half width
CAB = dict(x0=-0.06, x1=1.62, y0=0.44, y1=HW, z0=0.2, z1=1.83)


def house():
    K.make("upper_race", body, M["frame"], lambda bm: K.add_cyl(bm, 0.9, (0, 0, -0.08), (0, 0, 0.0), 64), bevel=0.01)
    K.make("revolving_frame", body, M["frame"], lambda bm: K.add_prism_z(bm, K.fillet([(-1.95, -HW), (1.72, -HW), (1.78, -0.6), (1.78, 0.6), (1.72, HW), (-1.95, HW)], 0.08), 0.0, 0.2), bevel=0.02)

    # counterweight: rear face on the tail-swing radius, rubber-edged lower band, lifting eyes, tail lamps
    R = 2.72
    ys = np.linspace(HW, -HW, 28)
    cw = [(-1.9, -HW)] + [(-math.sqrt(R * R - y * y), y) for y in ys[::-1]] + [(-1.9, HW)]
    cw = cw[::-1]
    K.make("counterweight", body, M["paint"], lambda bm: K.add_prism_z(bm, cw, -0.07, 1.0), bevel=0.09, segs=4)
    band = [(x * 1.004, y * 1.004) for x, y in cw]
    K.make("counterweight_band", body, M["paint_dark"], lambda bm: K.add_prism_z(bm, band, -0.075, 0.1), bevel=0.02)

    def eyes(bm):
        for y in (-0.75, 0.75):
            K.add_tube(bm, [(-2.25 + 0.09 * math.cos(a), y, 1.0 + 0.09 * math.sin(a)) for a in np.linspace(0, math.pi, 10)], 0.018, 8)

    K.make("counterweight_eyes", body, M["steel"], eyes)
    for y, name in ((1.05, "tail_lamp_l"), (-1.05, "tail_lamp_r")):
        x = -math.sqrt(R * R - y * y)
        K.make(name, body, M["red"], lambda bm, x=x, y=y: K.add_obox(bm, (x - 0.0, y, 0.75), ((x, y, 0), (-y, x, 0), (0, 0, 1)), (0.02, 0.07, 0.035)), bevel=0.008)

    # engine and cooling compartment
    K.make("engine_room", body, M["paint"], lambda bm: K.add_box(bm, (-1.93, -HW, 0.2), (-0.08, HW, 1.0)), bevel=0.05, segs=3)
    K.make("engine_hood", body, M["paint_dark"], lambda bm: K.add_box(bm, (-1.85, -1.22, 1.0), (-0.32, 0.3, 1.13)), bevel=0.05, segs=3)

    def hood_ribs(bm):
        for y in (-0.95, -0.45, 0.05):
            K.add_box(bm, (-1.75, y - 0.02, 1.13), (-0.42, y + 0.02, 1.15))

    K.make("engine_hood_ribs", body, M["paint_dark"], hood_ribs, bevel=0.008)

    def louvres(bm):
        for side in (1, -1):
            y = side * HW
            for k in range(9):
                z = 0.38 + k * 0.055
                K.add_obox(bm, (-1.05, y + side * 0.012, z), ((1, 0, 0), (0, side * math.cos(0.5), -math.sin(0.5)), (0, side * math.sin(0.5), math.cos(0.5))), (0.5, 0.016, 0.006))
            K.add_box(bm, (-1.6, y - 0.004, 0.33), (-0.5, y + 0.004, 0.86))

    K.make("louvres", body, M["frame"], louvres, bevel=0.003)

    def seams(bm):  # door shut lines and hinges on both sides of the engine room
        for side in (1, -1):
            y = side * (HW + 0.002)
            for x in (-1.7, -0.45):
                K.add_box(bm, (x - 0.006, y - 0.004, 0.24), (x + 0.006, y + 0.004, 0.96))
            K.add_box(bm, (-1.7, y - 0.004, 0.955), (-0.45, y + 0.004, 0.965))
            for z in (0.35, 0.85):
                K.add_cyl(bm, 0.018, (-0.47, y, z - 0.06), (-0.47, y, z + 0.06), 10)
            K.add_box(bm, (-1.66, y - side * 0.0, 0.6), (-1.6, y + side * 0.025, 0.64))  # handle

    K.make("engine_room_seams", body, M["frame"], seams)

    # right front: hydraulic and fuel tanks under a stepped cover, with anti-slip plates and a filler cap
    tank = K.fillet([(-0.08, 0.2), (1.72, 0.2), (1.72, 0.78), (1.56, 0.98), (-0.08, 0.98)], 0.04)
    K.make("tank_cover", body, M["paint"], lambda bm: K.add_prism_y(bm, tank, -HW, -0.42), bevel=0.04, segs=3)

    def antislip(bm):
        for x0, x1 in ((0.0, 0.7), (0.78, 1.48)):
            K.add_box(bm, (x0, -1.26, 0.98), (x1, -0.5, 0.988))
            for x in np.arange(x0 + 0.04, x1 - 0.02, 0.08):
                for y in np.arange(-1.22, -0.52, 0.08):
                    K.add_box(bm, (x - 0.012, y - 0.012, 0.988), (x + 0.012, y + 0.012, 0.995))

    K.make("anti_slip", body, M["frame"], antislip)
    K.make("filler_cap", body, M["frame"], lambda bm: K.add_cyl(bm, 0.07, (-0.0 + 0.2, -0.95, 1.13 - 0.15), (0.2, -0.95, 1.03), 24), bevel=0.01)

    def steps(bm):
        for z in (0.36, 0.66):
            K.add_box(bm, (1.25, -HW - 0.16, z), (1.62, -HW - 0.0, z + 0.03))
            for x in np.arange(1.28, 1.6, 0.05):
                K.add_box(bm, (x, -HW - 0.15, z + 0.03), (x + 0.02, -HW - 0.01, z + 0.04))
        K.add_box(bm, (1.25, -HW - 0.02, 0.2), (1.28, -HW, 0.7))
        K.add_box(bm, (1.59, -HW - 0.02, 0.2), (1.62, -HW, 0.7))

    K.make("steps", body, M["frame"], steps, bevel=0.004)

    # handrails along the top of the tank cover and the hood (operators walk there for service)
    def rails(bm):
        y = -HW + 0.06
        posts = [(0.05, 0.98), (0.8, 0.98), (1.5, 0.98), (-0.4, 1.13), (-1.1, 1.13), (-1.75, 1.13)]
        for x, z in posts:
            K.add_tube(bm, [(x, y, z), (x, y, 1.7)], 0.02, 10)
        K.add_tube(bm, [(1.5, y, 1.7), (0.05, y, 1.7), (-0.4, y, 1.7), (-1.75, y, 1.7)], 0.02, 10)
        K.add_tube(bm, [(1.5, y, 1.35), (0.05, y, 1.35)], 0.016, 10)

    K.make("handrails", body, M["paint"], rails)

    # cab: frame, tinted glazing on every face, roof, wiper, door handle, mirrors, interior shadow box
    c = CAB
    # side profile: vertical lower front, windscreen raked back 0.1 m, rounded roof line
    XR = c["x1"] - 0.1  # front of the roof
    ZK = 0.56  # where the rake starts
    prof = K.fillet([(c["x0"], c["z0"]), (c["x1"], c["z0"]), (c["x1"], ZK), (XR, c["z1"]), (c["x0"], c["z1"])], 0.07, 4)
    K.make("cab_shell", body, M["cab"], lambda bm: K.add_prism_y(bm, prof, c["y0"], c["y1"]), bevel=0.025, segs=3)
    roof = K.fillet([(c["x0"] - 0.03, c["z1"] - 0.01), (XR + 0.07, c["z1"] - 0.01), (XR + 0.07, c["z1"] + 0.035), (c["x0"] - 0.03, c["z1"] + 0.045)], 0.015, 2)
    K.make("cab_roof", body, M["cab"], lambda bm: K.add_prism_y(bm, roof, c["y0"] - 0.03, c["y1"] + 0.03), bevel=0.012)

    def front_x(z):  # cab front at height z
        return c["x1"] if z <= ZK else c["x1"] - (c["x1"] - XR) * (z - ZK) / (c["z1"] - ZK)

    e = 0.006

    def glass(bm):
        # windscreen on the raked face
        z0, z1 = 0.62, 1.74
        p0, p1 = Vector((front_x(z0), 0, z0)), Vector((front_x(z1), 0, z1))
        u = (p1 - p0).normalized()
        n = Vector((u.z, 0, -u.x))
        mid = (p0 + p1) / 2 + n * 0.003
        mid.y = (c["y0"] + c["y1"]) / 2
        K.add_obox(bm, mid, (u, (0, 1, 0), n), ((p1 - p0).length / 2, (c["y1"] - c["y0"]) / 2 - 0.08, e / 2))
        K.add_box(bm, (c["x1"], c["y0"] + 0.08, 0.28), (c["x1"] + e, c["y1"] - 0.08, 0.52))  # lower front
        side_front = lambda z: front_x(z) - 0.07  # noqa: E731
        door = [(0.62, 0.58), (side_front(0.58), 0.58), (side_front(1.75), 1.75), (0.62, 1.75)]
        K.add_prism_y(bm, door, c["y1"], c["y1"] + e)
        K.add_prism_y(bm, [(0.04, 0.95), (0.54, 0.95), (0.54, 1.75), (0.04, 1.75)], c["y1"], c["y1"] + e)  # left rear quarter
        K.add_prism_y(bm, [(0.3, 0.95), (side_front(0.95), 0.95), (side_front(1.75), 1.75), (0.3, 1.75)], c["y0"] - e, c["y0"])  # right side
        K.add_box(bm, (c["x0"] - e, c["y0"] + 0.1, 1.1), (c["x0"], c["y1"] - 0.1, 1.72))  # rear
        K.add_box(bm, (0.45, c["y0"] + 0.15, c["z1"] + 0.035), (1.2, c["y1"] - 0.15, c["z1"] + 0.05))  # roof hatch

    K.make("cab_glass", body, M["glass"], glass, smooth=False)

    def cab_trim(bm):
        K.add_box(bm, (0.58, c["y1"], 0.25), (0.6, c["y1"] + 0.008, 1.78))  # door shut line
        K.add_box(bm, (1.2, c["y1"] + 0.008, 1.0), (1.32, c["y1"] + 0.03, 1.03))  # handle
        K.add_tube(bm, [(front_x(0.66) + 0.012, 0.62, 0.66), (front_x(1.62) + 0.012, 1.05, 1.62)], 0.008, 6)  # wiper
        K.add_box(bm, (front_x(0.6) + 0.004, 0.6, 0.6), (front_x(0.6) + 0.03, 0.66, 0.66))
        for y in (c["y0"] + 0.035, c["y1"] - 0.035):  # front pillars
            K.add_tube(bm, [(c["x1"] + 0.004, y, 0.24), (c["x1"] + 0.004, y, ZK), (XR + 0.004, y, c["z1"] - 0.04)], 0.022, 8)

    K.make("cab_trim", body, M["plastic"], cab_trim)

    def interior(bm):  # dark mass behind the glass: seat back, consoles
        K.add_box(bm, (0.25, 0.7, 0.3), (0.55, 1.1, 1.25))
        K.add_box(bm, (0.5, 0.55, 0.3), (1.0, 0.68, 0.85))
        K.add_box(bm, (0.5, 1.12, 0.3), (1.0, 1.25, 0.85))
        K.add_tube(bm, [(0.95, 0.62, 0.85), (1.0, 0.62, 1.0)], 0.018, 8)
        K.add_tube(bm, [(0.95, 1.18, 0.85), (1.0, 1.18, 1.0)], 0.018, 8)

    K.make("cab_interior", body, M["plastic"], interior, bevel=0.02)

    def mirrors(bm):
        K.add_tube(bm, [(1.6, HW + 0.02, 1.3), (1.72, HW + 0.12, 1.42), (1.72, HW + 0.2, 1.5)], 0.014, 8)
        K.add_box(bm, (1.69, HW + 0.12, 1.5), (1.73, HW + 0.32, 1.78))
        K.add_tube(bm, [(1.62, -HW + 0.08, 0.98), (1.66, -HW - 0.04, 1.3), (1.66, -HW - 0.08, 1.55)], 0.016, 8)
        K.add_box(bm, (1.64, -HW - 0.2, 1.55), (1.68, -HW + 0.0, 1.85))

    K.make("mirrors", body, M["plastic"], mirrors, bevel=0.01)

    def work_lamps(bm):
        for y in (0.6, 1.15):
            K.add_box(bm, (1.44, y - 0.07, c["z1"] + 0.035), (1.57, y + 0.07, c["z1"] + 0.13))

    K.make("work_lamps", body, M["plastic"], work_lamps, bevel=0.015)
    K.make("work_lamp_lenses", body, M["lens"], lambda bm: [K.add_box(bm, (1.57, y - 0.055, c["z1"] + 0.05), (1.575, y + 0.055, c["z1"] + 0.115)) for y in (0.6, 1.15)])

    # PWRI automation retrofit: four-colour signal tower on the cab roof, two GNSS antennas on the counterweight
    K.make("signal_bar", body, M["plastic"], lambda bm: K.add_box(bm, (0.05, 0.55, c["z1"] + 0.045), (0.2, 1.22, c["z1"] + 0.09)), bevel=0.01)
    for k, col in enumerate(("green", "blue", "red", "amber")):
        y = 0.65 + k * 0.17
        K.make(f"signal_{col}", body, M[col], lambda bm, y=y: (K.add_cyl(bm, 0.045, (0.125, y, c["z1"] + 0.09), (0.125, y, c["z1"] + 0.19), 20), K.add_sphere(bm, 0.045, (0.125, y, c["z1"] + 0.19), 20, 8)), bevel=0.0)
    for y in (-0.95, 0.95):
        K.make(f"gnss_mast_{'l' if y > 0 else 'r'}", body, M["frame"], lambda bm, y=y: K.add_tube(bm, [(-2.2, y, 1.0), (-2.2, y, 1.62)], 0.022, 10))
        K.make(f"gnss_antenna_{'l' if y > 0 else 'r'}", body, M["white"], lambda bm, y=y: K.add_cyl(bm, 0.1, (-2.2, y, 1.62), (-2.2, y, 1.68), 32, r1=0.07), bevel=0.012)

    # exhaust stack and pre-cleaner
    K.make("exhaust", body, M["frame"], lambda bm: (K.add_tube(bm, [(-1.25, -0.72, 1.12), (-1.25, -0.72, 1.45), (-1.27, -0.72, 1.55)], 0.055, 16), K.add_cyl(bm, 0.075, (-1.25, -0.72, 1.12), (-1.25, -0.72, 1.18), 16)), bevel=0.0)
    K.make("pre_cleaner", body, M["plastic"], lambda bm: (K.add_cyl(bm, 0.045, (-0.7, -1.0, 1.12), (-0.7, -1.0, 1.4), 16), K.add_cyl(bm, 0.11, (-0.7, -1.0, 1.4), (-0.7, -1.0, 1.52), 24, r1=0.06)), bevel=0.01)

    # boom foot brackets and the boom cylinder head brackets
    foot = K.fillet([(-0.55, 0.2), (0.45, 0.2), (0.33, 0.68), (0.2, 0.93), (-0.05, 0.99), (-0.3, 0.88), (-0.48, 0.55)], 0.08)
    for s in (-1, 1):
        K.make(f"boom_foot_bracket_{'l' if s > 0 else 'r'}", body, M["paint"], lambda bm, s=s: K.add_prism_y(bm, foot, s * 0.235, s * 0.3), bevel=0.012)
    pin_with_caps(body, "boom_foot_pin", (0.0547, 0.7249), 0.065, 0.34)

    def cyl_brackets(bm):
        hx, hz = PINS["boom_head"][1:]
        br = K.circle_hull([((hx, hz), 0.11)], extra=[(hx - 0.2, 0.2), (hx + 0.2, 0.2)])
        for yc in CYL["boom"][2]:
            for e in (-1, 1):
                y = yc + e * 0.07
                K.add_prism_y(bm, br, y - 0.015, y + 0.015)

    K.make("boom_cylinder_brackets", body, M["paint"], cyl_brackets, bevel=0.008)

    def head_pins(bm):
        hx, hz = PINS["boom_head"][1:]
        for yc in CYL["boom"][2]:
            K.add_cyl(bm, 0.04, (hx, yc - 0.1, hz), (hx, yc + 0.1, hz), 16)
            for e in (-1, 1):
                K.add_cyl(bm, 0.06, (hx, yc + e * 0.085, hz), (hx, yc + e * 0.1, hz), 16)

    K.make("boom_head_pins", body, M["steel"], head_pins, bevel=0.003)

    # hoses from the house to the boom: two pairs leaving the valve block beside the boom foot
    def hoses(bm):
        for y in (-0.16, -0.08, 0.08, 0.16):
            K.add_tube(bm, K.catmull([(0.6, y, 0.25), (0.45, y, 0.5), (0.25, y, 0.95), (0.4, y, 1.25)], 6), 0.022, 10)

    K.make("house_hoses", body, M["rubber"], hoses)


house()

# ================================================================== boom (boom_link)
BOOM_C = [(0.0, 0.0), (0.45, 1.05), (1.0, 1.85), (1.6, 2.35), (2.2, 2.68), (3.3, 2.84), (4.84, 3.0023)]
BOOM_D = [0.42, 0.62, 0.80, 0.70, 0.52, 0.42, 0.36]  # section depth at those points


def boom_outline():
    center = K.catmull(BOOM_C, 10)
    n = len(center)
    depth = np.interp(np.linspace(0, len(BOOM_D) - 1, n), np.arange(len(BOOM_D)), BOOM_D)
    top, bot = [], []
    for i, p in enumerate(center):
        t = np.array(center[min(i + 1, n - 1)]) - np.array(center[max(i - 1, 0)])
        t /= np.linalg.norm(t)
        nv = np.array([-t[1], t[0]])
        top.append(tuple(np.array(p) + nv * depth[i] / 2))
        bot.append(tuple(np.array(p) - nv * depth[i] / 2))
    t0 = np.array(center[1]) - np.array(center[0])
    t0 /= np.linalg.norm(t0)
    n0 = np.array([-t0[1], t0[0]])
    t1 = np.array(center[-1]) - np.array(center[-2])
    t1 /= np.linalg.norm(t1)
    n1 = np.array([-t1[1], t1[0]])
    r0, r1 = depth[0] / 2, depth[-1] / 2
    tip = [tuple(np.array(center[-1]) + r1 * (math.cos(f) * t1 + math.sin(f) * n1)) for f in np.linspace(-math.pi / 2, math.pi / 2, 14)]
    foot = [tuple(np.array(center[0]) + r0 * (math.cos(f) * -t0 + math.sin(f) * -n0)) for f in np.linspace(-math.pi / 2, math.pi / 2, 14)]
    return center, top, bot, tip, foot


def boom_parts():
    center, top, bot, tip, foot = boom_outline()
    n = len(center)
    cut = next(i for i, p in enumerate(center) if p[0] > 4.25)  # the nose narrows to fit inside the arm
    main = bot[: cut + 1] + top[: cut + 1][::-1] + foot
    nose = bot[cut:] + tip + top[cut:][::-1]
    K.make("boom_body", boom, M["paint"], lambda bm: K.add_prism_y(bm, main, -0.23, 0.23), bevel=0.03, segs=3)
    K.make("boom_nose", boom, M["paint"], lambda bm: K.add_prism_y(bm, nose, -0.145, 0.145), bevel=0.02, segs=3)

    # top and bottom flange plates proud of the side plates: the box-section look
    def flanges(bm):
        for edge, sgn in ((top, -1), (bot, 1)):
            e = edge[2 : cut + 1]
            inner = K.offset_polyline(e, sgn * 0.03 * (1 if edge is bot else 1))
            band = list(e) + inner[::-1]
            K.add_prism_y(bm, band, -0.245, 0.245)

    K.make("boom_flanges", boom, M["paint"], flanges, bevel=0.008)

    # side stiffener plates across the bend
    def stiffeners(bm):
        for s in (-1, 1):
            pts = K.fillet([(0.75, 1.25), (1.9, 2.1), (2.35, 2.85), (1.55, 2.55), (0.55, 1.7)], 0.12)
            K.add_prism_y(bm, pts, s * 0.23, s * 0.242)

    K.make("boom_stiffeners", boom, M["paint"], stiffeners, bevel=0.005)

    # bosses and brackets: boom cylinder rod pin (under the bend), arm cylinder head bracket (on top)
    cx, cz = PINS["boom_rod"][1:]
    K.make("boom_cyl_boss", boom, M["paint"], lambda bm: K.add_prism_y(bm, K.circle_hull([((cx, cz), 0.12)], extra=[(cx - 0.3, cz + 0.25), (cx + 0.25, cz + 0.32)]), -0.235, 0.235), bevel=0.015)
    pin_with_caps(boom, "boom_rod_pin", (cx, cz), 0.05, 0.4)
    ax, az = PINS["arm_head"][1:]
    ab = K.circle_hull([((ax, az), 0.12)], extra=[(ax - 0.35, az - 0.2), (ax + 0.3, az - 0.15)])
    for s in (-1, 1):
        K.make(f"arm_cyl_bracket_{'l' if s > 0 else 'r'}", boom, M["paint"], lambda bm, s=s: K.add_prism_y(bm, ab, s * 0.065, s * 0.105), bevel=0.01)
    pin_with_caps(boom, "arm_head_pin", (ax, az), 0.045, 0.125)
    K.make("boom_foot_boss", boom, M["paint"], lambda bm: K.add_cyl(bm, 0.19, (0, -0.235, 0), (0, 0.235, 0), 32), bevel=0.012)
    K.make("boom_tip_boss", boom, M["paint"], lambda bm: K.add_cyl(bm, 0.15, (ARM_PIN[0], -0.145, ARM_PIN[1]), (ARM_PIN[0], 0.145, ARM_PIN[1]), 32), bevel=0.01)
    pin_with_caps(boom, "arm_pin", ARM_PIN, 0.06, 0.215)

    # hydraulic lines along the top: two to the arm cylinder, two on to the bucket cylinder (over the nose)
    def lines(bm):
        e = top[3:]
        for y, end in ((-0.07, None), (0.07, None), (-0.16, 0), (0.16, 0)):
            pts = [(x, y, z + 0.045) for x, z in K.offset_polyline(e, 0.0)]
            if end is None:  # arm cylinder lines end at its head bracket
                pts = [p for p in pts if p[0] < ax - 0.25]
            else:
                pts = [p for p in pts if p[0] < 4.5]
            K.add_tube(bm, pts, 0.017, 8)
            for k in range(2, len(pts) - 1, 7):  # clamps
                x, yy, z = pts[k]
                K.add_box(bm, (x - 0.025, yy - 0.03, z - 0.03), (x + 0.025, yy + 0.03, z + 0.03))

    K.make("boom_lines", boom, M["steel"], lines)

    def boom_hoses(bm):
        p0 = [p for p in top if p[0] < ax - 0.25][-1]
        for y in (-0.07, 0.07):
            K.add_tube(bm, K.catmull([(p0[0], y, p0[1] + 0.045), (p0[0] + 0.15, y * 1.4, p0[1] + 0.18), (ax - 0.05, y * 1.2, az + 0.15)], 6), 0.022, 10)
        for y in (-0.16, 0.16):
            q = [p for p in top if p[0] < 4.5][-1]
            K.add_tube(bm, K.catmull([(q[0], y, q[1] + 0.045), (4.75, y, 3.35), (5.05, y * 0.6, 3.3)], 6), 0.022, 10)

    K.make("boom_hoses", boom, M["rubber"], boom_hoses)

    # work lamp on the left of the boom
    K.make("boom_lamp", boom, M["plastic"], lambda bm: K.add_box(bm, (1.25, 0.24, 1.65), (1.4, 0.32, 1.78)), bevel=0.012)
    K.make("boom_lamp_lens", boom, M["lens"], lambda bm: K.add_box(bm, (1.4, 0.25, 1.66), (1.405, 0.31, 1.77)))


boom_parts()


# ================================================================== details: hazard chevrons, weld beads, labels
def details():
    # yellow-black chevrons across the counterweight's rear face (it swings over people)
    R = 2.72
    z0, z1, w, lean = 0.48, 0.7, 0.11, 0.09

    def stripes(bm, parity):
        for k in range(-7, 7):
            if (k % 2 == 0) != parity:
                continue
            y0 = k * w
            q = [(y0, z0), (y0 + w, z0), (y0 + w + lean, z1), (y0 + lean, z1)]
            if max(abs(y) for y, _ in q) > 0.85:
                continue
            vs = []
            for r in (R - 0.004, R + 0.006):
                vs.append([bm.verts.new((-math.sqrt(r * r - y * y), y, z)) for y, z in q])
            a, b = vs
            bm.faces.new(a[::-1])
            bm.faces.new(b)
            for i in range(4):
                j = (i + 1) % 4
                bm.faces.new((a[i], a[j], b[j], b[i]))

    K.make("counterweight_chevrons_y", body, M["hazard_y"], lambda bm: stripes(bm, True), smooth=False)
    K.make("counterweight_chevrons_k", body, M["hazard_k"], lambda bm: stripes(bm, False), smooth=False)

    # weld beads where the boom's flange plates meet its side plates
    center, top, bot, tip, foot = boom_outline()
    cut = next(i for i, p in enumerate(center) if p[0] > 4.25)

    def welds(bm):
        for edge in (top, bot):
            e = edge[2 : cut + 1]
            inward = K.offset_polyline(e, 0.03)
            cen = np.mean(np.array(center), axis=0)
            if np.linalg.norm(np.mean(inward, axis=0) - cen) > np.linalg.norm(np.mean(e, axis=0) - cen):
                inward = K.offset_polyline(e, -0.03)
            for y in (-0.233, 0.233):
                K.add_tube(bm, [(x, y, z) for x, z in inward], 0.007, 6)

    K.make("boom_welds", boom, M["paint"], welds)

    # warning labels: crush zone on both boom sides, pinch points at the bucket linkage, no-step on the hood
    def label_plates(bm, which):
        spots = [(boom, 0.95, 1.55, 0.236), (boom, 0.95, 1.55, -0.236)]
        for _, x, z, y in spots:
            s = 1 if y > 0 else -1
            if which == "y":
                K.add_box(bm, (x - 0.09, y, z - 0.09), (x + 0.09, y + s * 0.003, z + 0.09))
            else:
                K.add_box(bm, (x - 0.1, y - s * 0.001, z - 0.1), (x + 0.1, y + s * 0.002, z + 0.1))
                K.add_prism_y(bm, [(x - 0.055, z - 0.045), (x + 0.055, z - 0.045), (x, z + 0.06)], y + s * 0.003, y + s * 0.0045)

    K.make("boom_labels_y", boom, M["hazard_y"], lambda bm: label_plates(bm, "y"), smooth=False)
    K.make("boom_labels_k", boom, M["hazard_k"], lambda bm: label_plates(bm, "k"), smooth=False)


details()

# ================================================================== arm (stick, arm_link)
R_PIN = PINS["arm_rod"][1:]
H_PIN = PINS["bucket_head"][1:]
G_PIN = PINS["guide"][1:]


def arm_parts():
    # the top runs about 0.2 m under the bucket cylinder's head pin, so the cylinder rides on top of the arm
    # on its bracket (H) instead of between the side plates
    hp = np.array(BUCKET_PIN) - np.array(H_PIN)
    up = np.array([-hp[1], hp[0]]) / np.linalg.norm(hp)
    top_pt = tuple(np.array(H_PIN) - 0.2 * up)
    outline = K.circle_hull([(R_PIN, 0.16), ((0.0, 0.0), 0.21), (BUCKET_PIN, 0.15)], extra=[top_pt], n=32)
    outline = K.fillet(outline, 0.04, 2)
    # box section: two side plates, cover plates around the edge except where the boom nose and the
    # cylinder eyes come in (they sit between the side plates)
    for s in (-1, 1):
        K.make(f"arm_side_{'l' if s > 0 else 'r'}", arm, M["paint"], lambda bm, s=s: K.add_prism_y(bm, outline, s * 0.155, s * 0.19), bevel=0.01)

    def covers(bm):
        P = outline
        n = len(P)
        keep = []
        for i in range(n):
            mid = ((P[i][0] + P[(i + 1) % n][0]) / 2, (P[i][1] + P[(i + 1) % n][1]) / 2)
            near = min(math.dist(mid, (0, 0)) - 0.42, math.dist(mid, R_PIN) - 0.3, math.dist(mid, H_PIN) - 0.3)
            keep.append(near > 0)
        # contiguous runs of kept edges -> bands
        runs, cur = [], []
        for i in range(n):
            if keep[i]:
                cur.append(i)
            elif cur:
                runs.append(cur)
                cur = []
        if cur:
            if runs and runs[0][0] == 0:
                runs[0] = cur + runs[0]
            else:
                runs.append(cur)
        for run in runs:
            pts = [P[i] for i in run] + [P[(run[-1] + 1) % n]]
            if len(pts) < 2:
                continue
            # inward = toward the outline centroid
            cxz = np.mean(np.array(P), axis=0)
            off = K.offset_polyline(pts, 0.03)
            if np.linalg.norm(np.mean(off, axis=0) - cxz) > np.linalg.norm(np.mean(pts, axis=0) - cxz):
                off = K.offset_polyline(pts, -0.03)
            K.add_prism_y(bm, pts + off[::-1], -0.19, 0.19)

    K.make("arm_covers", arm, M["paint"], covers, bevel=0.006)

    # bosses between the side plates, pins with retainers, tip boss for the bucket ears
    K.make("arm_bosses", arm, M["paint"], lambda bm: [K.add_cyl(bm, r, (x, -0.19, z), (x, 0.19, z), 28) for (x, z), r in ((R_PIN, 0.11), (G_PIN, 0.085))], bevel=0.008)
    hb = K.circle_hull([(H_PIN, 0.11)], extra=[tuple(np.array(top_pt) - 0.25 * hp / np.linalg.norm(hp) - 0.04 * up), tuple(np.array(top_pt) + 0.3 * hp / np.linalg.norm(hp) - 0.04 * up)])
    for s in (-1, 1):
        K.make(f"bucket_cyl_bracket_{'l' if s > 0 else 'r'}", arm, M["paint"], lambda bm, s=s: K.add_prism_y(bm, hb, s * 0.06, s * 0.1), bevel=0.01)
    K.make("arm_tip_boss", arm, M["paint"], lambda bm: K.add_cyl(bm, 0.14, (BUCKET_PIN[0], -0.255, BUCKET_PIN[1]), (BUCKET_PIN[0], 0.255, BUCKET_PIN[1]), 32), bevel=0.012)
    K.make("arm_foot_boss", arm, M["paint"], lambda bm: [K.add_cyl(bm, 0.2, (0, s * 0.155, 0), (0, s * 0.21, 0), 32) for s in (-1, 1)], bevel=0.01)
    pin_with_caps(arm, "arm_rod_pin", R_PIN, 0.045, 0.2)
    pin_with_caps(arm, "bucket_head_pin", H_PIN, 0.045, 0.115)
    pin_with_caps(arm, "guide_pin", G_PIN, 0.045, 0.26)
    pin_with_caps(arm, "bucket_pin", BUCKET_PIN, 0.055, 0.34)

    # grease lines to the tip pins, clipped along the side plates
    def lines(bm):
        for s in (-1, 1):
            y = s * 0.2
            a = np.array(top_pt) + 0.15 * hp / np.linalg.norm(hp) - 0.08 * up
            b = np.array(BUCKET_PIN) - 0.25 * hp / np.linalg.norm(hp) - 0.05 * up
            K.add_tube(bm, [(a[0], y, a[1]), (b[0], y, b[1])], 0.008, 6)
            for t in (0.3, 0.7):
                p = a + (b - a) * t
                K.add_box(bm, (p[0] - 0.015, y - 0.01 * s, p[1] - 0.015), (p[0] + 0.015, y + 0.012 * s, p[1] + 0.015))

    K.make("arm_grease_lines", arm, M["steel"], lines)


arm_parts()

# ================================================================== bucket (bucket_link)
SHELL = [(0.32, -0.06), (0.05, -0.08), (-0.18, 0.07), (-0.29, 0.42), (-0.28, 0.8), (-0.18, 1.03), (0.02, 1.15), (0.35, 1.17), (0.68, 1.08), (0.94, 0.985)]
BW = 0.53  # half width inside the side plates


def bucket_parts():
    outer = K.catmull(SHELL, 8)
    cen = np.array([0.4, 0.55])
    inner = K.offset_polyline(outer, 0.028)
    if np.linalg.norm(np.mean(inner, axis=0) - cen) > np.linalg.norm(np.mean(outer, axis=0) - cen):
        inner = K.offset_polyline(outer, -0.028)
    K.make("bucket_shell", bucket, M["bucket"], lambda bm: K.add_prism_y(bm, outer + inner[::-1], -BW, BW), bevel=0.006)
    side = outer + [outer[0]]
    for s in (-1, 1):
        K.make(f"bucket_side_{'l' if s > 0 else 'r'}", bucket, M["bucket"], lambda bm, s=s: K.add_prism_y(bm, side, s * BW, s * (BW + 0.03)), bevel=0.008)
        # side cutter along the mouth
        cut = [(0.32, -0.06), (0.94, 0.985), (0.97, 0.94), (0.36, -0.09)]
        K.make(f"bucket_side_cutter_{'l' if s > 0 else 'r'}", bucket, M["wear"], lambda bm, s=s, cut=cut: K.add_prism_y(bm, cut, s * (BW - 0.005), s * (BW + 0.04)), bevel=0.005)

    # cutting edge, tooth adapters and teeth pointing at the bucket tip frame
    tip = np.array(BUCKET_TIP)
    base = np.array([0.95, 0.975])
    d = (tip - base) / np.linalg.norm(tip - base)
    nrm = np.array([-d[1], d[0]])

    def edge(bm):
        a, b = base - d * 0.17, base + d * 0.01
        K.add_prism_y(bm, [tuple(a + nrm * 0.03), tuple(b + nrm * 0.025), tuple(b - nrm * 0.02), tuple(a - nrm * 0.03)], -BW - 0.03, BW + 0.03)

    K.make("bucket_cutting_edge", bucket, M["wear"], edge, bevel=0.005)

    def teeth(bm):
        L = np.linalg.norm(tip - base)
        for y in (-0.42, -0.21, 0.0, 0.21, 0.42):
            # adapter wrapping the lip
            a0 = base - d * 0.04
            K.add_obox(bm, (a0[0], y, a0[1]), ((d[0], 0, d[1]), (0, 1, 0), (nrm[0], 0, nrm[1])), (0.11, 0.055, 0.05))
            # tooth: tapered, flattening to a chisel at the tip
            v = []
            for t, hw, ht in ((0.0, 0.05, 0.045), (L * 0.55, 0.045, 0.03), (L, 0.035, 0.008)):
                p = base + d * t
                for sy, sz in ((-1, -1), (1, -1), (1, 1), (-1, 1)):
                    q = p + nrm * sz * ht
                    v.append(bm.verts.new((q[0], y + sy * hw, q[1])))
            for r in range(2):
                A, B = v[4 * r : 4 * r + 4], v[4 * r + 4 : 4 * r + 8]
                for k in range(4):
                    bm.faces.new((A[k], A[(k + 1) % 4], B[(k + 1) % 4], B[k]))
            bm.faces.new(v[0:4][::-1])
            bm.faces.new(v[8:12])

    K.make("bucket_teeth", bucket, M["wear"], teeth, bevel=0.004, angle=50)

    # wear strips under the floor
    def strips(bm):
        o = [p for p in outer if 0.3 < p[0] < 0.9]
        out = K.offset_polyline(o, -0.018)
        if np.linalg.norm(np.mean(out, axis=0) - cen) < np.linalg.norm(np.mean(o, axis=0) - cen):
            out = K.offset_polyline(o, 0.018)
        for y in (-0.36, 0.0, 0.36):
            K.add_prism_y(bm, o + out[::-1], y - 0.035, y + 0.035)

    K.make("bucket_wear_strips", bucket, M["wear"], strips, bevel=0.004)

    # ears: outer pair on the bucket pin (outside the arm tip), inner pair on the link pin (inside the side links)
    ears_o = K.circle_hull([((0, 0), 0.13)], extra=[(0.3, -0.05), (-0.17, 0.06)])
    ears_k = K.circle_hull([(PINS["side"][1:], 0.11)], extra=[(-0.27, 0.3), (-0.26, 0.75)])
    for s in (-1, 1):
        K.make(f"bucket_ear_pin_{'l' if s > 0 else 'r'}", bucket, M["bucket"], lambda bm, s=s: K.add_prism_y(bm, ears_o, s * 0.265, s * 0.31), bevel=0.01)
        K.make(f"bucket_ear_link_{'l' if s > 0 else 'r'}", bucket, M["bucket"], lambda bm, s=s: K.add_prism_y(bm, ears_k, s * 0.1, s * 0.145), bevel=0.01)
    pin_with_caps(bucket, "bucket_link_pin", PINS["side"][1:], 0.045, 0.21)

    # lifting eye on the back
    K.make("bucket_lift_eye", bucket, M["bucket"], lambda bm: K.add_tube(bm, [(-0.29 - 0.05 * math.sin(a), 0, 0.62 + 0.06 * math.cos(a)) for a in np.linspace(0, math.pi, 10)], 0.018, 8))


bucket_parts()

# ================================================================== free parts: cylinders and linkage


def cylinder_parts(name):
    (dx, dz), l_mesh, ycs, R, Rr, Lb, Lr = CYL[name]
    d = Vector((dx, 0, dz)).normalized()
    nv = Vector((-d.z, 0, d.x))  # perpendicular in the x-z plane
    Y = Vector((0, 1, 0))
    P = lambda s, y=0.0, o=0.0: d * s + Y * y + nv * o  # noqa: E731

    def barrel(bm):
        for yc in ycs:
            K.add_cyl(bm, 0.075, P(0, yc - 0.055), P(0, yc + 0.055), 24)  # head eye
            K.add_cyl(bm, R * 0.7, P(0.05, yc), P(0.12, yc), 24)
            K.add_cyl(bm, R + 0.014, P(0.1, yc), P(0.2, yc), 32)  # head cap
            K.add_cyl(bm, R, P(0.2, yc), P(Lb - 0.08, yc), 32)
            K.add_cyl(bm, R + 0.016, P(Lb - 0.08, yc), P(Lb - 0.01, yc), 32)  # gland
            for s in (0.28, Lb - 0.2):  # ports
                K.add_obox(bm, P(s, yc, R + 0.02), (d, Y, nv), (0.035, 0.03, 0.025))
            # steel line from the rod-end port back along the barrel to the head
            K.add_tube(bm, [P(Lb - 0.2, yc, R + 0.045), P(0.32, yc, R + 0.045), P(0.28, yc, R + 0.04)], 0.013, 8)
            for s in np.linspace(0.6, Lb - 0.4, 2):
                K.add_obox(bm, P(s, yc, R + 0.03), (d, Y, nv), (0.02, 0.025, 0.03))

    def rod(bm):
        for yc in ycs:
            K.add_cyl(bm, Rr, P(l_mesh - Lr, yc), P(l_mesh - 0.1, yc), 28)
            K.add_cyl(bm, Rr + 0.012, P(l_mesh - 0.11, yc), P(l_mesh - 0.06, yc), 28)
            K.add_cyl(bm, 0.07, P(l_mesh, yc - 0.05), P(l_mesh, yc + 0.05), 24)  # rod eye

    K.make(f"{name}_barrel_body", FREE[name + "_barrel"], M["paint"], barrel, bevel=0.006)

    def rod_steel(bm):
        for yc in ycs:
            K.add_cyl(bm, Rr, P(l_mesh - Lr, yc), P(l_mesh - 0.1, yc), 28)

    def rod_end(bm):
        for yc in ycs:
            K.add_cyl(bm, Rr + 0.012, P(l_mesh - 0.11, yc), P(l_mesh - 0.06, yc), 28)
            K.add_cyl(bm, 0.07, P(l_mesh, yc - 0.05), P(l_mesh, yc + 0.05), 24)
            K.add_obox(bm, P(l_mesh - 0.06, yc), (d, Y, nv), (0.04, 0.05, 0.055))

    K.make(f"{name}_rod_chrome", FREE[name + "_rod"], M["chrome"], rod_steel)
    K.make(f"{name}_rod_eye", FREE[name + "_rod"], M["paint"], rod_end, bevel=0.006)


for c in CYL:
    cylinder_parts(c)


def link_parts(name, ys, thick, r_end, pin_half=None):
    (p0, p1) = LINK_MESH[name]
    plate = K.circle_hull([(p0, r_end), (p1, r_end)], n=24)
    for y in ys:
        K.make(f"{name}_plate_{'l' if y > 0 else 'r'}", FREE[name + "_link"], M["paint"], lambda bm, y=y: K.add_prism_y(bm, plate, y - thick / 2, y + thick / 2), bevel=0.008)
    if pin_half:
        pin_with_caps(FREE[name + "_link"], f"{name}_link_pin", p0, 0.045, pin_half)


link_parts("side", (-0.175, 0.175), 0.04, 0.09, 0.24)  # carries the pin where the links meet the bucket cylinder
link_parts("guide", (-0.22, 0.22), 0.035, 0.095)  # its arm pin is the arm's guide_pin

# ================================================================== pose (for review renders only)


def Ry(q):
    return Matrix.Rotation(q, 4, "Y")


def pose(q):
    """Joint angles (swing, boom, arm, bucket), URDF sense, and place the free parts from the pins."""
    body.matrix_basis = ref_matrix("body_link") @ Matrix.Rotation(q[0], 4, "Z")
    boom.matrix_basis = ref_matrix("boom_link") @ Ry(q[1])
    arm.matrix_basis = ref_matrix("arm_link") @ Ry(q[2])
    bucket.matrix_basis = ref_matrix("bucket_link") @ Ry(q[3])
    bpy.context.view_layer.update()
    H = body.matrix_world.inverted()
    F = {"house": Matrix.Identity(4), "boom": H @ boom.matrix_world, "stick": H @ arm.matrix_world, "bucket": H @ bucket.matrix_world}
    P = lambda p: (F[p[0]] @ Vector((p[1], 0, p[2]))).to_3d()  # noqa: E731
    pts = {k: P(v) for k, v in PINS.items()}
    g, k = pts["guide"], pts["side"]
    dd = math.hypot(k.x - g.x, k.z - g.z)
    a = (GUIDE_LEN**2 - SIDE_LEN**2 + dd**2) / (2 * dd)
    h = math.sqrt(max(GUIDE_LEN**2 - a * a, 0))
    ux, uz = (k.x - g.x) / dd, (k.z - g.z) / dd
    pts["bucket_rod"] = Vector((g.x + a * ux - h * uz, g.y, g.z + a * uz + h * ux))
    Yax = Vector((0, 1, 0))

    def frame(p0, p1, dirx):
        u = (p1 - p0).normalized()
        w = (Yax - u * Yax.dot(u)).normalized()
        world = Matrix((u, w, u.cross(w))).transposed()
        dv = Vector((dirx[0], 0, dirx[1])).normalized()
        local = Matrix((dv, Yax, dv.cross(Yax))).transposed()
        return world @ local.transposed()

    def place(name, R3, p):
        m = R3.to_4x4()
        m.translation = p
        FREE[name].matrix_basis = body.matrix_world @ m

    for name in CYL:
        head, rod = pts[name + "_head"], pts[name + "_rod"]
        R3 = frame(head, rod, CYL[name][0])
        L = (rod - head).length
        place(name + "_barrel", R3, head)
        place(name + "_rod", R3, head + (rod - head) / L * (L - CYL[name][1]))
    for name, (m0, m1), (a0, a1) in (("side", LINK_MESH["side"], (pts["bucket_rod"], k)), ("guide", LINK_MESH["guide"], (g, pts["bucket_rod"]))):
        R3 = frame(a0, a1, (m1[0] - m0[0], m1[1] - m0[1]))
        place(name + "_link", R3, a0 - R3 @ Vector((m0[0], 0, m0[1])))
    bpy.context.view_layer.update()


# ================================================================== export
def export():
    q_rest = [0.0, 0.0, 0.0, 0.0]
    pose(q_rest)  # rest frames for the links
    for f in FREE.values():
        f.matrix_basis = Matrix.Identity(4)  # free parts at the root origin, as in the OPERA model
    K.export_glb(OUT)
    js, binchunk = K.read_glb(OUT)
    names = {n.get("name"): i for i, n in enumerate(js["nodes"])}
    # wrap the scene in the Y-up root and copy the joint extras of the reference model
    roots = js["scenes"][0]["nodes"]
    js["nodes"].append({"name": "zx200", "matrix": REFN["zx200"]["matrix"], "children": roots})
    js["scenes"][0]["nodes"] = [len(js["nodes"]) - 1]
    for name, ref in REFN.items():
        if name in names and ref.get("extras"):
            js["nodes"][names[name]]["extras"] = ref["extras"]
    js["nodes"][names["track_link"]]["extras"] = {
        "track": {
            "_note": "track loop in track_link (URDF) frame: bottom run at pitch line z, idler front, sprocket rear; shoes are spaced by pitch along it",
            "sprocket_x": XS, "idler_x": XI, "radius": RP, "pitch_line_z": ZP, "pitch": PITCH, "gauge": GAUGE,
            "shoes": SHOES,
        }
    }
    js["asset"]["copyright"] = (
        "20 t class hydraulic excavator, modelled procedurally in Blender (assets/blender/zx200_hd.py) by the "
        "construction-site-simulation project to the dimensions and pin geometry of PWRI's OPERA ZX200 model "
        "(https://github.com/pwri-opera/zx200_ros2, Apache-2.0). Apache-2.0."
    )
    js["asset"]["extras"] = {"source": "assets/blender/zx200_hd.py", "reference": "https://github.com/pwri-opera/zx200_ros2", "license": "Apache-2.0"}
    K.write_glb(OUT, js, binchunk)
    tris = sum(len(o.data.polygons) for o in bpy.data.objects if o.type == "MESH")
    print(f"EXPORTED {OUT} nodes={len(js['nodes'])} meshes={len(js.get('meshes', []))} faces(pre-bevel)={tris}")


if "--no-export" not in ARGS:
    export()

if "--print-free" in ARGS:  # where the free parts land in a pose (debugging the pin solve)
    pose([float(v) for v in ARGS[ARGS.index("--print-free") + 1].split(",")])
    for n in FREE:
        o = bpy.data.objects[n]
        print("FREE", n, [round(v, 3) for v in o.matrix_world.translation])
    for n in ("arm_link", "bucket_link"):
        print("LINK", n, [round(v, 3) for v in bpy.data.objects[n].matrix_world.translation])

# ================================================================== review renders
if "--render" in ARGS:
    out = ARGS[ARGS.index("--render") + 1]
    q = [0.3, -0.6, 2.1, 2.2]
    if "--pose" in ARGS:
        q = [float(v) for v in ARGS[ARGS.index("--pose") + 1].split(",")]
    pose(q)
    sc = bpy.context.scene
    # --beauty: EEVEE (this Blender build has no Cycles) with a physical sky, sun, soft shadows, AO and
    # screen-space reflections; otherwise Workbench with studio light for checking shapes
    engine = "BLENDER_EEVEE" if "--beauty" in ARGS else "BLENDER_WORKBENCH"
    sc.render.engine = engine
    if engine == "BLENDER_WORKBENCH":
        sc.display.shading.light = "STUDIO"
        sc.display.shading.color_type = "MATERIAL"
        sc.display.shading.show_cavity = True
        sc.display.shading.show_shadows = True
    else:
        ee = sc.eevee
        ee.taa_render_samples = 64
        ee.use_gtao = True
        ee.gtao_distance = 0.6
        ee.use_ssr = True
        ee.use_ssr_refraction = False
        ee.use_soft_shadows = True
        ee.shadow_cascade_size = "4096"
        ee.shadow_cube_size = "2048"
        sc.view_settings.view_transform = "AgX"
        sc.view_settings.look = "AgX - Medium High Contrast"
        sun = bpy.data.objects.new("sun", bpy.data.lights.new("sun", "SUN"))
        sc.collection.objects.link(sun)
        sun.data.energy = 4.0
        sun.data.angle = math.radians(1.5)
        sun.rotation_euler = (math.radians(50), 0, math.radians(200))
        sun.data.shadow_cascade_max_distance = 40
        world = bpy.data.worlds.new("sky")
        sc.world = world
        world.use_nodes = True
        nt = world.node_tree
        sky = nt.nodes.new("ShaderNodeTexSky")
        sky.sun_elevation = math.radians(40)
        sky.sun_rotation = math.radians(200)
        nt.links.new(sky.outputs["Color"], nt.nodes["Background"].inputs["Color"])
        nt.nodes["Background"].inputs["Strength"].default_value = 0.25
        ground = bpy.data.meshes.new("ground")
        import bmesh as _bm

        b = _bm.new()
        K.add_box(b, (-30, -30, -0.05), (30, 30, 0.0))
        b.to_mesh(ground)
        g = bpy.data.objects.new("ground", ground)
        sc.collection.objects.link(g)
        ground.materials.append(K.material("dirt", "#8a7a62", rough=0.95))
    sc.render.resolution_x, sc.render.resolution_y = 1600, 1000
    cam = bpy.data.objects.new("cam", bpy.data.cameras.new("cam"))
    sc.collection.objects.link(cam)
    sc.camera = cam
    target = Vector((2.2, 0, 2.3))
    views = {"front_left": (12.5, 10.5, 5.0), "rear_right": (-9.0, -10.5, 6.0), "side_left": (2.5, 16, 2.8), "side_right": (2.5, -16, 2.8), "track": (1.0, -4.2, 0.9), "cab": (4.5, 4.2, 2.6)}
    for vname, loc in views.items():
        if "--views" in ARGS and vname not in ARGS[ARGS.index("--views") + 1].split(","):
            continue
        cam.location = loc
        tgt = {"track": Vector((0.0, -1.1, 0.4)), "cab": Vector((0.6, 0.9, 2.0))}.get(vname, target)
        cam.rotation_euler = (tgt - Vector(loc)).to_track_quat("-Z", "Y").to_euler()
        cam.data.lens = 35
        sc.render.filepath = f"{out}_{vname}.png"
        bpy.ops.render.render(write_still=True)
