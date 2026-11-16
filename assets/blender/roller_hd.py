"""Single-drum vibratory soil roller (10.6 t class), built procedurally in Blender, exported as
assets/machinery/roller_hd.glb for the soil view (phase1/web/soil.js), which drew it from boxes.

Frame and dimensions from phase2/roller.py: base_link, Z up, x ahead, origin at the drum axle's height;
drum 2.13 m wide and 1.50 m across at DRUM_X = 1.45 m, rear tyres 1.50 m across at AXLE_X = -1.45 m.
The drum and the two rear wheels are their own nodes (drum, wheel_l, wheel_r) with their origin on the
axle, so the viewer can turn them as the roller moves. No manufacturer marks or livery.

    blender -b -t 4 --python assets/blender/roller_hd.py [-- --render out/prefix [--beauty]]
"""

import math
import pathlib
import sys

import numpy as np
from mathutils import Matrix, Vector

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
import blendkit as K  # noqa: E402

ROOT = pathlib.Path(__file__).resolve().parents[2]
OUT = ROOT / "assets/machinery/roller_hd.glb"
ARGS = sys.argv[sys.argv.index("--") + 1 :] if "--" in sys.argv else []

DRUM_W, DRUM_R, DRUM_X, AXLE_X, WHEEL_R = 2.13, 0.75, 1.45, -1.45, 0.75
GROUND = -0.75
TYRE_W, TYRE_Y = 0.6, 0.77

K.reset()
M = {
    "paint": K.surface("paint_yellow", "#e9b318", "paint", 1024, seed=11, coat=0.35),
    "frame": K.surface("frame_graphite", "#2b2d30", "paint_worn", 1024, seed=12),
    "drum": K.surface("drum_steel", "#6a665f", "worn_edge", 1024, seed=13),
    "steel": K.material("steel_machined", "#9a9da0", rough=0.3, metal=1.0),
    "rubber": K.surface("tyre_rubber", "#171717", "rubber", 512, seed=14),
    "rim": K.material("rim_paint", "#3a3c3f", rough=0.45, metal=0.3),
    "glass": K.material("glass_tinted", "#16222a", rough=0.04, alpha=0.55),
    "plastic": K.material("plastic_black", "#0d0e0f", rough=0.55),
    "lens": K.material("lamp_lens", "#f4f1e4", rough=0.1, emit="#fff4dc", emit_strength=0.6),
    "amber": K.material("lamp_amber", "#f0a020", rough=0.2, emit="#ffa000", emit_strength=0.5),
    "red": K.material("lamp_red", "#c4161c", rough=0.2, emit="#ff2010", emit_strength=0.4),
    "hazard_y": K.material("hazard_yellow", "#f0c010", rough=0.45, coat=0.2),
    "hazard_k": K.material("hazard_black", "#141414", rough=0.5, coat=0.2),
    "hose": K.surface("hose_rubber", "#111111", "rubber", 512, seed=15),
}

base = K.empty("base_link")
drum = K.empty("drum", base, Matrix.Translation((DRUM_X, 0, 0)))
wheels = {s: K.empty(f"wheel_{s}", base, Matrix.Translation((AXLE_X, (1 if s == "l" else -1) * TYRE_Y, 0))) for s in ("l", "r")}

# ------------------------------------------------------------------ drum (in the drum frame, axle along y)
hw = DRUM_W / 2


def drum_shell(bm):
    K.add_cyl(bm, DRUM_R - 0.004, (0, -hw, 0), (0, hw, 0), 64)
    for s in (-1, 1):  # rolled edges and end discs
        K.add_cyl(bm, DRUM_R, (0, s * (hw - 0.04), 0), (0, s * hw, 0), 64)


K.make("drum_shell", drum, M["drum"], drum_shell, bevel=0.012, segs=3)


def drum_ends(bm):
    for s in (-1, 1):
        K.add_cyl(bm, 0.6, (0, s * (hw - 0.01), 0), (0, s * (hw + 0.005), 0), 48)
        K.add_cyl(bm, 0.26, (0, s * hw, 0), (0, s * (hw + 0.06), 0), 32)  # hub
        for k in range(10):
            a = 2 * math.pi * k / 10
            K.add_cyl(bm, 0.016, (0.2 * math.cos(a), s * (hw + 0.06), 0.2 * math.sin(a)), (0.2 * math.cos(a), s * (hw + 0.075), 0.2 * math.sin(a)), 6)


K.make("drum_ends", drum, M["frame"], drum_ends, bevel=0.006)

# ------------------------------------------------------------------ rear wheels (wheel frame, axle along y)


def tyre(bm):
    K.add_cyl(bm, WHEEL_R - 0.03, (0, -TYRE_W / 2, 0), (0, TYRE_W / 2, 0), 48)
    # traction tread: chevron lugs round the tyre
    n = 26
    for k in range(n):
        a = 2 * math.pi * k / n
        for s in (-1, 1):
            c = Vector(((WHEEL_R - 0.03) * math.cos(a), s * TYRE_W * 0.24, (WHEEL_R - 0.03) * math.sin(a)))
            radial = Vector((math.cos(a), 0, math.sin(a)))
            tang = Vector((-math.sin(a), 0, math.cos(a)))
            along = (tang * 0.45 + Vector((0, s, 0))).normalized()
            K.add_obox(bm, c + radial * 0.02, (along, radial.cross(along), radial), (0.17, 0.035, 0.025))


K.make("tyre", None, M["rubber"], tyre, bevel=0.01)
import bpy  # noqa: E402

tyre_ob = bpy.data.objects["tyre"]


def rim(bm):
    for s in (-1, 1):
        K.add_cyl(bm, 0.43, (0, s * 0.18, 0), (0, s * 0.3, 0), 40)
    K.add_cyl(bm, 0.2, (0, -0.32, 0), (0, 0.32, 0), 32)
    for k in range(10):
        a = 2 * math.pi * k / 10
        for s in (-1, 1):
            K.add_cyl(bm, 0.018, (0.15 * math.cos(a), s * 0.32, 0.15 * math.sin(a)), (0.15 * math.cos(a), s * 0.34, 0.15 * math.sin(a)), 6)


for s, w in wheels.items():
    t = K.make(f"tyre_{s}", w, M["rubber"], None, mesh=tyre_ob.data, bevel=0.01)
    K.make(f"rim_{s}", w, M["rim"], rim, bevel=0.008)
bpy.data.objects.remove(tyre_ob)

# ------------------------------------------------------------------ front frame (yoke round the drum)


def yoke(bm):
    for s in (-1, 1):
        y0, y1 = s * (hw + 0.09), s * (hw + 0.15)
        side = K.fillet([(DRUM_X - 0.3, -0.18), (DRUM_X + 0.3, -0.18), (DRUM_X + 0.55, 0.3), (DRUM_X + 0.86, 0.62), (DRUM_X + 0.86, 0.92), (DRUM_X - 0.9, 0.92), (DRUM_X - 0.9, 0.62), (DRUM_X - 0.55, 0.3)], 0.12, 5)
        K.add_prism_y(bm, side, y0, y1)
    K.add_box(bm, (DRUM_X + 0.72, -hw - 0.15, 0.55), (DRUM_X + 0.92, hw + 0.15, 0.95))  # front cross beam
    K.add_box(bm, (DRUM_X - 0.95, -hw - 0.15, 0.55), (DRUM_X - 0.75, hw + 0.15, 0.95))  # rear cross beam
    K.add_box(bm, (0.25, -0.35, 0.25), (DRUM_X - 0.75, 0.35, 0.85))  # neck to the articulation


K.make("front_frame", base, M["paint"], yoke, bevel=0.03, segs=3)


def scrapers(bm):
    for x0, x1, z0, z1 in ((DRUM_X + 0.74, DRUM_X + 0.8, -0.05, 0.55), (DRUM_X - 0.8, DRUM_X - 0.74, -0.05, 0.55)):
        K.add_box(bm, (x0, -hw + 0.02, z0), (x1, hw - 0.02, z1))


K.make("scrapers", base, M["frame"], scrapers, bevel=0.008)


def front_details(bm):
    for s in (-1, 1):
        K.add_cyl(bm, 0.22, (DRUM_X, s * (hw + 0.15), 0), (DRUM_X, s * (hw + 0.32), 0), 32)  # drive motor / exciter housing
        K.add_box(bm, (DRUM_X + 0.85, s * 0.75 - 0.1, 0.95), (DRUM_X + 0.95, s * 0.75 + 0.1, 1.08))  # lamp housings


K.make("front_details", base, M["frame"], front_details, bevel=0.012)
K.make("front_lamps", base, M["lens"], lambda bm: [K.add_box(bm, (DRUM_X + 0.95, s * 0.75 - 0.08, 0.97), (DRUM_X + 0.955, s * 0.75 + 0.08, 1.06)) for s in (-1, 1)])

# hydraulic hoses to the drum drive motor and the exciter, from the neck
def drum_hoses(bm):
    for s in (-1, 1):
        for dy in (0.0, 0.06):
            y = s * (hw + 0.24 + dy)
            K.add_tube(bm, K.catmull([(0.55, s * 0.3, 0.75), (0.75, s * (hw - 0.1), 1.0), (DRUM_X - 0.6, y, 0.98), (DRUM_X - 0.15, y, 0.35), (DRUM_X - 0.05, s * (hw + 0.3), 0.12)], 6), 0.02, 10)


K.make("drum_hoses", base, M["hose"], drum_hoses)


# rear chevrons on the engine hood (the rear swings wide when it steers)
def chevrons(bm, parity):
    w, lean = 0.1, 0.08
    for k in range(-6, 6):
        if (k % 2 == 0) != parity:
            continue
        y0 = k * w
        K.add_prism_frame(bm, [(y0, 0.0), (y0 + w, 0.0), (y0 + w + lean, 0.18), (y0 + lean, 0.18)], (-2.855, 0, 0.98), (0, -1, 0), (0, 0, 1), 0.0, 0.006) if abs(y0) < 0.62 else None


K.make("rear_chevrons_y", base, M["hazard_y"], lambda bm: chevrons(bm, True), smooth=False)
K.make("rear_chevrons_k", base, M["hazard_k"], lambda bm: chevrons(bm, False), smooth=False)

# articulation joint and steering cylinders
K.make("articulation", base, M["frame"], lambda bm: (K.add_cyl(bm, 0.14, (0.15, 0, -0.1), (0.15, 0, 0.75), 24), K.add_box(bm, (-0.2, -0.3, -0.05), (0.25, 0.3, 0.7))), bevel=0.02)
K.make("steer_cyl", base, M["paint"], lambda bm: [K.add_cyl(bm, 0.055, (-0.3, s * 0.42, 0.35), (0.45, s * 0.38, 0.35), 20) for s in (-1, 1)], bevel=0.006)
K.make("steer_rod", base, M["steel"], lambda bm: [K.add_cyl(bm, 0.03, (0.45, s * 0.38, 0.35), (0.75, s * 0.36, 0.35), 16) for s in (-1, 1)])

# ------------------------------------------------------------------ rear frame, engine hood, cab


def rear_frame(bm):
    K.add_prism_y(bm, K.fillet([(-0.15, -0.3), (-2.6, -0.3), (-2.85, 0.0), (-2.85, 0.35), (-0.15, 0.35)], 0.06), -0.95, 0.95)


K.make("rear_frame", base, M["frame"], rear_frame, bevel=0.02)


def hood(bm):
    prof = K.fillet([(-1.05, 0.35), (-2.85, 0.35), (-2.85, 1.0), (-2.65, 1.25), (-1.05, 1.32)], 0.12, 4)
    K.add_prism_y(bm, prof, -0.88, 0.88)


K.make("engine_hood", base, M["paint"], hood, bevel=0.04, segs=3)


def hood_details(bm):
    for s in (-1, 1):
        y = s * 0.885
        for k in range(8):
            z = 0.55 + k * 0.06
            K.add_obox(bm, (-2.0, y + s * 0.01, z), ((1, 0, 0), (0, s * math.cos(0.5), -math.sin(0.5)), (0, s * math.sin(0.5), math.cos(0.5))), (0.45, 0.014, 0.005))
        K.add_box(bm, (-2.5, y - 0.004, 0.5), (-1.5, y + 0.004, 1.03))
    K.add_box(bm, (-2.87, -0.6, 0.45), (-2.85, 0.6, 0.95))  # rear grille panel
    for k in range(7):
        z = 0.5 + k * 0.065
        K.add_box(bm, (-2.885, -0.55, z), (-2.865, 0.55, z + 0.025))


K.make("hood_grilles", base, M["frame"], hood_details, bevel=0.003)
K.make("tail_lamps", base, M["red"], lambda bm: [K.add_box(bm, (-2.9, s * 0.72 - 0.06, 0.62), (-2.86, s * 0.72 + 0.06, 0.7)) for s in (-1, 1)], bevel=0.006)
K.make("exhaust", base, M["frame"], lambda bm: K.add_tube(bm, [(-2.2, 0.55, 1.28), (-2.2, 0.55, 1.75), (-2.25, 0.55, 1.82)], 0.045, 14))


def fenders(bm):
    for s in (-1, 1):
        y0, y1 = s * (TYRE_Y - TYRE_W / 2 - 0.03), s * (TYRE_Y + TYRE_W / 2 + 0.04)
        arc = [(AXLE_X + (WHEEL_R + 0.1) * math.cos(a), (WHEEL_R + 0.1) * math.sin(a)) for a in np.linspace(math.radians(15), math.radians(165), 14)]
        inner = [(AXLE_X + (WHEEL_R + 0.06) * math.cos(a), (WHEEL_R + 0.06) * math.sin(a)) for a in np.linspace(math.radians(165), math.radians(15), 14)]
        K.add_prism_y(bm, arc + inner, min(y0, y1), max(y0, y1))


K.make("fenders", base, M["frame"], fenders, bevel=0.01)

CAB = dict(x0=-1.05, x1=0.05, y0=-0.72, y1=0.72, z0=0.35, z1=2.18)


def cab():
    c = CAB
    K.make("cab_floor", base, M["frame"], lambda bm: K.add_box(bm, (-1.15, -0.95, 0.35), (0.25, 0.95, 0.45)), bevel=0.015)
    prof = K.fillet([(c["x0"], c["z0"]), (c["x1"], c["z0"]), (c["x1"] + 0.05, 1.1), (c["x1"] - 0.03, c["z1"]), (c["x0"], c["z1"])], 0.06, 4)
    K.make("cab_shell", base, M["frame"], lambda bm: K.add_prism_y(bm, prof, c["y0"], c["y1"]), bevel=0.025, segs=3)
    K.make("cab_roof", base, M["paint"], lambda bm: K.add_box(bm, (c["x0"] - 0.08, c["y0"] - 0.08, c["z1"]), (c["x1"] + 0.1, c["y1"] + 0.08, c["z1"] + 0.07)), bevel=0.025)

    def glass(bm):
        e = 0.006
        for s in (-1, 1):  # side doors
            y = c["y1"] if s > 0 else c["y0"] - e
            K.add_prism_y(bm, [(c["x0"] + 0.08, 0.7), (c["x1"] - 0.02, 0.7), (c["x1"] - 0.05, 2.1), (c["x0"] + 0.08, 2.1)], y, y + e)
        p0, p1 = Vector((c["x1"] + 0.05, 0, 1.15)), Vector((c["x1"] - 0.03, 0, 2.1))
        u = (p1 - p0).normalized()
        n = Vector((u.z, 0, -u.x))
        mid = (p0 + p1) / 2 + n * 0.003
        K.add_obox(bm, mid, (u, (0, 1, 0), n), ((p1 - p0).length / 2, c["y1"] - 0.08, e / 2))  # front
        K.add_box(bm, (c["x0"] - e, c["y0"] + 0.08, 1.15), (c["x0"], c["y1"] - 0.08, 2.1))  # rear

    K.make("cab_glass", base, M["glass"], glass, smooth=False)
    K.make("cab_interior", base, M["plastic"], lambda bm: (K.add_box(bm, (-0.75, -0.3, 0.45), (-0.45, 0.3, 1.35)), K.add_box(bm, (-0.25, -0.4, 0.45), (0.0, 0.4, 1.05))), bevel=0.02)

    def pillars(bm):
        for x in (c["x0"] + 0.03, c["x1"] - 0.0):
            for s in (-1, 1):
                top = c["x1"] - 0.03 if x > c["x0"] + 0.1 else x
                K.add_tube(bm, [(x, s * (c["y1"] - 0.02), 0.5), (x + (0.05 if x > c["x0"] + 0.1 else 0), s * (c["y1"] - 0.02), 1.1), (top, s * (c["y1"] - 0.02), c["z1"] - 0.02)], 0.03, 8)

    K.make("cab_pillars", base, M["frame"], pillars)

    def rails_mirrors(bm):
        for s in (-1, 1):
            K.add_tube(bm, [(-1.1, s * 0.93, 0.45), (-1.1, s * 0.93, 1.25), (-0.2, s * 0.93, 1.25), (-0.2, s * 0.93, 0.45)], 0.018, 8)
            K.add_tube(bm, [(c["x1"] + 0.05, s * (c["y1"] + 0.05), c["z1"] - 0.05), (c["x1"] + 0.15, s * (c["y1"] + 0.3), c["z1"] - 0.1)], 0.012, 6)
            K.add_box(bm, (c["x1"] + 0.12, s * (c["y1"] + 0.3) - 0.1, c["z1"] - 0.45), (c["x1"] + 0.16, s * (c["y1"] + 0.3) + 0.1, c["z1"] - 0.1))
        for k, z in enumerate((-0.05, 0.2)):  # entry steps on the left
            K.add_box(bm, (-0.95, 0.95, z), (-0.45, 1.12, z + 0.03))
        K.add_box(bm, (-0.96, 0.95, -0.1), (-0.93, 1.12, 0.35))
        K.add_box(bm, (-0.47, 0.95, -0.1), (-0.44, 1.12, 0.35))

    K.make("rails_steps_mirrors", base, M["frame"], rails_mirrors, bevel=0.004)
    K.make("beacon", base, M["amber"], lambda bm: (K.add_cyl(bm, 0.07, (-0.5, 0.45, c["z1"] + 0.07), (-0.5, 0.45, c["z1"] + 0.2), 24), K.add_sphere(bm, 0.07, (-0.5, 0.45, c["z1"] + 0.2), 24, 8)))
    K.make("roof_lamps", base, M["lens"], lambda bm: [K.add_box(bm, (c["x1"] + 0.1, s * 0.5 - 0.07, c["z1"] + 0.01), (c["x1"] + 0.105, s * 0.5 + 0.07, c["z1"] + 0.06)) for s in (-1, 1)])


cab()

# ------------------------------------------------------------------ export
K.export_glb(OUT)
js, binchunk = K.read_glb(OUT)
roots = js["scenes"][0]["nodes"]
js["nodes"].append({"name": "roller", "matrix": [1, 0, 0, 0, 0, 0, -1, 0, 0, 1, 0, 0, 0, 0, 0, 1], "children": roots})  # Z up -> Y up
js["scenes"][0]["nodes"] = [len(js["nodes"]) - 1]
names = {n.get("name"): i for i, n in enumerate(js["nodes"])}
js["nodes"][names["base_link"]]["extras"] = {"drum_x": DRUM_X, "drum_r": DRUM_R, "axle_x": AXLE_X, "wheel_r": WHEEL_R, "spin_nodes": {"drum": DRUM_R, "wheel_l": WHEEL_R, "wheel_r": WHEEL_R}, "_note": "spin_nodes turn about their local y by distance / radius as the roller moves along +x"}
js["asset"]["copyright"] = "Single-drum vibratory roller (10.6 t class), modelled procedurally in Blender (assets/blender/roller_hd.py) by the construction-site-simulation project. Apache-2.0."
js["asset"]["extras"] = {"source": "assets/blender/roller_hd.py", "license": "Apache-2.0"}
K.write_glb(OUT, js, binchunk)
print(f"EXPORTED {OUT} nodes={len(js['nodes'])}")

if "--render" in ARGS:
    out = ARGS[ARGS.index("--render") + 1]
    sc = bpy.context.scene
    if "--beauty" in ARGS:
        sc.render.engine = "BLENDER_EEVEE"
        ee = sc.eevee
        ee.taa_render_samples = 64
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
        K.add_box(b, (-20, -20, GROUND - 0.05), (20, 20, GROUND))
        me = bpy.data.meshes.new("ground")
        b.to_mesh(me)
        me.materials.append(K.material("dirt", "#8a7a62", rough=0.95))
        g = bpy.data.objects.new("ground", me)
        sc.collection.objects.link(g)
    else:
        sc.render.engine = "BLENDER_WORKBENCH"
        sc.display.shading.light = "STUDIO"
        sc.display.shading.color_type = "MATERIAL"
        sc.display.shading.show_cavity = True
    sc.render.resolution_x, sc.render.resolution_y = 1600, 1000
    cam = bpy.data.objects.new("cam", bpy.data.cameras.new("cam"))
    sc.collection.objects.link(cam)
    sc.camera = cam
    cam.data.lens = 35
    for vname, loc in {"front_left": (6.5, 5.5, 2.5), "rear_right": (-6.5, -5.5, 2.8), "side": (0, -8.5, 0.8)}.items():
        cam.location = loc
        cam.rotation_euler = (Vector((0, 0, 0.3)) - Vector(loc)).to_track_quat("-Z", "Y").to_euler()
        sc.render.filepath = f"{out}_{vname}.png"
        bpy.ops.render.render(write_still=True)
