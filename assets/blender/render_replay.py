"""Render a simulator replay (assets/replay/*.bin.gz, phase2/record_replay.py) in Blender with EEVEE: PNG frames
and an MP4. The scene is the one the browser builds (phase1/web/site.js + soil.js): the site terrain, the soil
surfaces from the heightmaps (pit, carrier load, spoil), the moving grains, and the machines from their glTF
models posed exactly as soil.js poses them (excavator links and cylinders, carriers and vessels, dozer blade and
lift cylinder, roller with turning drum and wheels).

Poses in a replay are in the viewer's world frame (Y up); the scene root turns them to Blender's Z up.

    blender -b -t 4 --python assets/blender/render_replay.py -- [options]
      --replay assets/replay/site_cycle.bin.gz   recording (default)
      --out out/dir                              frames, video (default renders/<recording name>)
      --from 0 --to 60                           simulated seconds to render
      --every 1                                  use every n-th recorded frame
      --camera overview|excavator|spread         camera preset
      --res 1920x1080 --samples 32 --fps 24      output
      --still                                    one frame only (the --from time), no video

The recording's frames are about 1/6 s of simulated time apart; at --fps 24 the video runs about 4x real time.
"""

import math
import pathlib
import re
import shutil
import subprocess
import sys

import bmesh
import bpy
import numpy as np
from mathutils import Matrix, Quaternion, Vector

HERE = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import blendkit as K  # noqa: E402
import replay_io as R  # noqa: E402

ROOT = HERE.parents[1]
ARGS = sys.argv[sys.argv.index("--") + 1 :] if "--" in sys.argv else []


def arg(name, default=None, cast=str):
    return cast(ARGS[ARGS.index(name) + 1]) if name in ARGS else default


REPLAY = ROOT / arg("--replay", "assets/replay/site_cycle.bin.gz")
OUT = pathlib.Path(arg("--out", str(ROOT / "renders" / REPLAY.name.split(".")[0])))
T_FROM, T_TO = arg("--from", 0.0, float), arg("--to", 1e9, float)
EVERY = arg("--every", 1, int)
CAMERA = arg("--camera", "overview")
W, H = (int(v) for v in arg("--res", "1920x1080").split("x"))
SAMPLES = arg("--samples", 32, int)
FPS = arg("--fps", 24, int)
STILL = "--still" in ARGS

# upgraded models (assets/blender), as soil.js loads them
UPGRADES = {
    "assets/machinery/opera_zx200.glb": "assets/machinery/zx200_hd.glb",
    "assets/machinery/opera_ic120.glb": "assets/machinery/ic120_hd.glb",
    "assets/machinery/opera_d37pxi.glb": "assets/machinery/d37_hd.glb",
}
CB = Matrix.Rotation(math.radians(90), 4, "X")  # viewer (Y up) -> Blender (Z up)
ZUP = Matrix.Rotation(math.radians(-90), 4, "X")  # soil.js wrapZup / the models' root: URDF Z up -> Y up


def T3(pos, quat):
    x, y, z, w = quat
    return Matrix.Translation(Vector(pos)) @ Quaternion((w, x, y, z)).to_matrix().to_4x4()


def place(pos, quat):
    """A soil.js group posed at (pos, quat) holding wrapZup(URDF frame) -> Blender world of that URDF frame."""
    return CB @ T3(pos, quat) @ ZUP


# ------------------------------------------------------------------ models


class Model:
    """An imported glTF with its mesh objects grouped by owner node; pose() sets every mesh from owner frames.

    The importer turns glTF Y up into Blender Z up, which with the models' Z-up -> Y-up root leaves each object's
    world matrix as (URDF world) @ ZUP; U(o) below is the URDF world of an object, rel = U(owner)^-1 U(mesh)."""

    def __init__(self, path, owners, skip=()):
        before = set(bpy.data.objects)
        bpy.ops.import_scene.gltf(filepath=str(path))
        bpy.context.view_layer.update()
        self.objs = [o for o in bpy.data.objects if o not in before]
        self.by = {re.sub(r"\.\d{3}$", "", o.name): o for o in self.objs}
        U = lambda o: o.matrix_world @ ZUP.inverted()  # noqa: E731
        self.U = {n: U(o) for n, o in self.by.items()}
        self.items = []
        for o in self.objs:
            if o.type != "MESH":
                continue
            p, owner, hidden = o, None, False
            while p is not None:
                n = re.sub(r"\.\d{3}$", "", p.name)
                if n in skip:
                    hidden = True
                    break
                if n in owners:
                    owner = n
                    break
                p = p.parent
            if hidden or owner is None:
                o.hide_render = o.hide_viewport = True
                continue
            self.items.append((o, owner, self.U[owner].inverted() @ U(o)))
        for o in self.objs:  # free the meshes from the hierarchy so each can take its own world matrix
            if o.type == "MESH":
                mw = o.matrix_world.copy()
                o.parent = None
                o.matrix_world = mw

    def clone(self):
        c = Model.__new__(Model)
        c.U, c.items = self.U, []
        for o, owner, rel in self.items:
            d = o.copy()
            bpy.context.scene.collection.objects.link(d)
            c.items.append((d, owner, rel))
        return c

    def rel(self, a, b):
        return self.U[a].inverted() @ self.U[b]

    def pose(self, frames):
        """frames: owner name -> Blender world matrix of its URDF frame."""
        for o, owner, rel in self.items:
            m = frames.get(owner)
            if m is None:
                o.hide_render = True
                continue
            o.hide_render = False
            o.matrix_world = m @ rel @ ZUP

    def hide(self):
        for o, _, _ in self.items:
            o.hide_render = True


# ------------------------------------------------------------------ static scene


def terrain_height(x, z):
    """phase1/web/site.js terrainHeight (viewer frame: x, z ground plane)."""

    def smooth(e0, e1, v):
        t = np.clip((v - e0) / (e1 - e0), 0, 1)
        return t * t * (3 - 2 * t)

    dx = np.maximum(np.maximum(-x - 30, x - 62), 0)
    dz = np.maximum(np.abs(z) - 22, 0)
    out = np.hypot(dx, dz)
    hills = 0.9 * np.sin(x * 0.11) * np.cos(z * 0.13) + 0.6 * np.sin(x * 0.27 + z * 0.19) + 0.35 * np.cos(x * 0.53 - z * 0.41)
    h = smooth(0, 14, out) * (3.5 + hills + 1.2)
    road = 1 - smooth(4, 7, np.abs(x + 11))
    h = np.where(z > 22, h * (1 - road) + road * np.minimum((z - 22) / 12, 4), h)
    sr = np.hypot(x + 22, z + 15)
    h = h + 3.2 * (1 - smooth(1.5, 6.5, sr))
    px = np.maximum(np.abs(x - 24) - 3, 0)
    pz = np.maximum(np.abs(z - 12) - 2.5, 0)
    h = h - np.maximum(2.5 - np.hypot(px, pz) / 1.5, 0)
    return h


def grid_mesh(name, nx, ny, xs, zs, heights, mat, colors=None):
    """Grid in the viewer frame (x, y = height, z); the object carries the frame change."""
    me = bpy.data.meshes.new(name)
    X, Z = np.meshgrid(xs, zs)
    verts = np.dstack([X, heights, Z]).reshape(-1, 3)
    idx = np.arange(nx * ny).reshape(ny, nx)
    a, b, c, d = idx[:-1, :-1].ravel(), idx[:-1, 1:].ravel(), idx[1:, 1:].ravel(), idx[1:, :-1].ravel()
    faces = np.stack([a, d, c, b], axis=1)
    me.from_pydata(verts.tolist(), [], faces.tolist())
    me.polygons.foreach_set("use_smooth", [True] * len(me.polygons))
    if colors is not None:
        attr = me.color_attributes.new("Col", "FLOAT_COLOR", "POINT")
        attr.data.foreach_set("color", np.hstack([colors, np.ones((len(colors), 1))]).astype(np.float32).ravel())
    me.materials.append(mat)
    ob = bpy.data.objects.new(name, me)
    bpy.context.scene.collection.objects.link(ob)
    ob.matrix_world = CB
    return ob


def set_heights(ob, heights):
    co = np.empty(len(ob.data.vertices) * 3, np.float32)
    ob.data.vertices.foreach_get("co", co)
    co = co.reshape(-1, 3)
    co[:, 1] = heights.ravel()
    ob.data.vertices.foreach_set("co", co.ravel())
    ob.data.update()


def ground_material(name, base_hex=None, vertex_colors=False, scale=6.0, bump=0.25):
    m = bpy.data.materials.new(name)
    m.use_nodes = True
    nt = m.node_tree
    p = nt.nodes["Principled BSDF"]
    p.inputs["Roughness"].default_value = 0.97
    noise = nt.nodes.new("ShaderNodeTexNoise")
    noise.inputs["Scale"].default_value = scale
    noise.inputs["Detail"].default_value = 12
    ramp = nt.nodes.new("ShaderNodeMath")
    ramp.operation = "MULTIPLY_ADD"
    ramp.inputs[1].default_value = 0.35
    ramp.inputs[2].default_value = 0.82
    nt.links.new(noise.outputs["Fac"], ramp.inputs[0])
    mix = nt.nodes.new("ShaderNodeMix")
    mix.data_type = "RGBA"
    mix.blend_type = "MULTIPLY"
    mix.inputs["Factor"].default_value = 1.0
    if vertex_colors:
        ca = nt.nodes.new("ShaderNodeVertexColor")
        ca.layer_name = "Col"
        nt.links.new(ca.outputs["Color"], mix.inputs[6])
    else:
        mix.inputs[6].default_value = (*K.srgb(base_hex), 1)
    nt.links.new(ramp.outputs[0], mix.inputs[7])
    nt.links.new(mix.outputs[2], p.inputs["Base Color"])
    b = nt.nodes.new("ShaderNodeBump")
    b.inputs["Strength"].default_value = bump
    b.inputs["Distance"].default_value = 0.02
    n2 = nt.nodes.new("ShaderNodeTexNoise")
    n2.inputs["Scale"].default_value = 60
    n2.inputs["Detail"].default_value = 8
    nt.links.new(n2.outputs["Fac"], b.inputs["Height"])
    nt.links.new(b.outputs["Normal"], p.inputs["Normal"])
    return m


def build_terrain():
    n, size = 256, 128.0
    xs = np.linspace(-size / 2, size / 2, n)
    X, Z = np.meshgrid(xs, xs)
    Hh = terrain_height(X, Z)
    pad, dirt, pit, grass = (np.array(K.srgb(c)) for c in ("#b3a68f", "#a3815c", "#7d5f44", "#8d9a5e"))
    outside = np.hypot(np.maximum(np.abs(X) - 30, 0), np.maximum(np.abs(Z) - 22, 0)) > 9
    col = np.where((Hh < -0.15)[..., None], pit, np.where((np.abs(Hh) < 0.05)[..., None], pad, np.where(outside[..., None], grass, dirt)))
    col = col * np.random.default_rng(1).uniform(0.94, 1.06, Hh.shape)[..., None] * 0.72  # the viewer's colours, toned for a physical sky
    gm = ground_material("ground", vertex_colors=True)
    grid_mesh("site_terrain", n, n, xs, xs, Hh, gm, col.reshape(-1, 3))
    # rolling country beyond the site out to the horizon (the site terrain is 128 m across)
    far = np.linspace(-700, 700, 141)
    FX, FZ = np.meshgrid(far, far)
    r = np.maximum(np.abs(FX), np.abs(FZ))
    FH = np.where(r < 60, -8.0, 3.6 + 2.5 * np.sin(FX * 0.013) * np.cos(FZ * 0.017) + 1.5 * np.sin(FX * 0.031 + FZ * 0.023))
    fcol = np.tile(np.array(K.srgb("#7f8c55")) * 0.72, (FX.size, 1))
    grid_mesh("country", 141, 141, far, far, FH, gm, fcol)


def setup_world():
    sc = bpy.context.scene
    sc.render.engine = "BLENDER_EEVEE"
    ee = sc.eevee
    ee.taa_render_samples = SAMPLES
    ee.use_gtao = True
    ee.gtao_distance = 0.8
    ee.use_ssr = True
    ee.use_soft_shadows = True
    ee.shadow_cascade_size = "4096"
    ee.use_bloom = False
    sc.view_settings.view_transform = "AgX"
    sc.view_settings.look = "AgX - Medium High Contrast"
    sc.render.resolution_x, sc.render.resolution_y = W, H
    sc.render.image_settings.file_format = "PNG"
    sun = bpy.data.objects.new("sun", bpy.data.lights.new("sun", "SUN"))
    sc.collection.objects.link(sun)
    sun.data.energy = 4.2
    sun.data.angle = math.radians(1.2)
    sun.rotation_euler = (math.radians(48), 0, math.radians(-35))
    sun.data.shadow_cascade_max_distance = 120
    sun.data.shadow_cascade_count = 4
    w = bpy.data.worlds.new("sky")
    sc.world = w
    w.use_nodes = True
    sky = w.node_tree.nodes.new("ShaderNodeTexSky")
    sky.sun_elevation = math.radians(42)
    sky.sun_rotation = math.radians(-35 + 180)
    sky.sun_disc = False
    w.node_tree.links.new(sky.outputs["Color"], w.node_tree.nodes["Background"].inputs["Color"])
    w.node_tree.nodes["Background"].inputs["Strength"].default_value = 0.22


CAMERAS = {
    # Blender frame: x as the viewer's x, y = -viewer z, z up
    "overview": ((20.0, -38.0, 21.0), (44.0, -8.0, 0.0), 28),
    "excavator": ((23.5, -1.0, 6.5), (30.5, -9.0, 1.4), 30),
    "spread": ((39.0, 13.0, 10.0), (53.0, -4.0, 0.5), 30),
}


def setup_camera():
    loc, tgt, lens = CAMERAS[CAMERA]
    cam = bpy.data.objects.new("cam", bpy.data.cameras.new("cam"))
    bpy.context.scene.collection.objects.link(cam)
    bpy.context.scene.camera = cam
    cam.location = loc
    cam.rotation_euler = (Vector(tgt) - Vector(loc)).to_track_quat("-Z", "Y").to_euler()
    cam.data.lens = lens
    cam.data.clip_end = 600


# ------------------------------------------------------------------ build


def main():
    print(f"REPLAY {REPLAY}")
    hello, raw = R.load(REPLAY)
    frames = [R.decode(b) for b in raw]
    geom = hello["geometry"]
    soil = hello.get("soil", {})
    K.reset()
    setup_world()
    setup_camera()
    build_terrain()

    soil_mat = ground_material("soil", soil.get("color", "#c9a774"), scale=14, bump=0.5)
    load_mat = ground_material("soil_load", soil.get("color", "#c9a774"), scale=14, bump=0.5)
    grain_mat = K.material("grains", soil.get("grain", "#b08a55"), rough=0.95)

    f0 = frames[0]
    hm = f0["hm"]
    xs = hm["x0"] + np.arange(hm["nx"]) * hm["cell"]
    zs = hm["z0"] - np.arange(hm["ny"]) * hm["cell"]
    pit = grid_mesh("soil_pit", hm["nx"], hm["ny"], xs, zs, hm["h"], soil_mat)
    truck = spoil = None
    if "truck_hm" in f0:
        t = f0["truck_hm"]
        truck = grid_mesh("soil_load", t["nx"], t["ny"], t["x0"] + np.arange(t["nx"]) * t["cell"], t["z0"] - np.arange(t["ny"]) * t["cell"], t["h"], load_mat)
    if "spoil_hm" in f0:
        t = f0["spoil_hm"]
        spoil = grid_mesh("soil_spoil", t["nx"], t["ny"], t["x0"] + np.arange(t["nx"]) * t["cell"], t["z0"] - np.arange(t["ny"]) * t["cell"], np.where(t["h"] > -3.4, t["h"], -0.4), soil_mat)
    grains = bpy.data.objects.new("grains", bpy.data.meshes.new("grains"))
    bpy.context.scene.collection.objects.link(grains)
    grains.matrix_world = CB
    grains.data.materials.append(grain_mat)
    ico = bmesh.new()
    bmesh.ops.create_icosphere(ico, subdivisions=1, radius=0.04)
    ico_v = np.array([v.co[:] for v in ico.verts])
    ico_f = np.array([[v.index for v in f.verts] for f in ico.faces])
    ico.free()

    # machines
    m = geom["model"]
    exc = Model(ROOT / UPGRADES.get(m["file"], m["file"]), set(m["links"].values()) | {m["crawler"]["node"]} | set(m.get("parts", [])))
    crawler = place(m["crawler"]["pos"], m["crawler"]["quat"])
    cars = []
    if "carrier" in geom:
        c = geom["carrier"]
        car = Model(ROOT / UPGRADES.get(c["file"], c["file"]), {"base_link", c["vessel_node"]}, skip=set(c.get("hide_nodes", [])))
        cars = [car] + [car.clone() for _ in range(c.get("count", 1) - 1)]
        parked_pin = T3(c["parked_pin"][:3], c["parked_pin"][3:])
    doz = None
    if "dozer" in geom:
        d = geom["dozer"]
        doz = Model(ROOT / UPGRADES.get(d["file"], d["file"]), {"base_link", d["arm_node"], d["cylinder_node"], d["piston_node"]})
        piston_axis = Vector((1.0055, 0, -0.056785)).normalized()  # soil.js pistonAxis (blade_arm_piston_joint)
    rols = []  # one model per roller ("ROL1", "ROL2" trailers), with its rolled distance per frame
    if "roller" in geom and (ROOT / "assets/machinery/roller_hd.glb").exists():
        first = Model(ROOT / "assets/machinery/roller_hd.glb", {"base_link", "drum", "wheel_l", "wheel_r"})
        n_rol = max([geom["roller"].get("count", 1)] + [len(f.get("rollers", [])) for f in frames])
        rols = [first] + [first.clone() for _ in range(n_rol - 1)]
        spin_r = {"drum": geom["roller"]["drum_r"], "wheel_l": geom["roller"]["wheel_r"], "wheel_r": geom["roller"]["wheel_r"]}
        # roll distance over the whole recording (the drum turns by distance / radius), as soil.js spinRoller
        travels = []
        for i in range(n_rol):
            travel, prev = [0.0], None
            for f in frames:
                rs = f.get("rollers", [])
                r = rs[i] if i < len(rs) else None
                d_ = 0.0
                if r is not None and prev is not None:
                    q = Quaternion((r["base"][1][3], *r["base"][1][:3]))
                    fwd = q @ Vector((1, 0, 0))
                    step = Vector(r["base"][0]) - prev
                    d_ = step.dot(fwd) if step.length < 2 else 0.0
                travel.append(travel[-1] + d_)
                prev = Vector(r["base"][0]) if r is not None else prev
            travels.append(travel[1:])

    sel = [k for k, f in enumerate(frames) if T_FROM <= f["sim_t"] <= T_TO][::EVERY]
    if STILL:
        sel = sel[:1]
    if not sel:
        raise SystemExit(f"no frames between {T_FROM} and {T_TO} s (recording runs {frames[0]['sim_t']:.1f}..{frames[-1]['sim_t']:.1f} s)")
    OUT.mkdir(parents=True, exist_ok=True)
    for old in OUT.glob("frame_*.png"):
        old.unlink()
    sc = bpy.context.scene
    print(f"RENDER {len(sel)} frames, sim {frames[sel[0]]['sim_t']:.1f}..{frames[sel[-1]]['sim_t']:.1f} s, camera {CAMERA}, {W}x{H}, {SAMPLES} samples")
    for i, k in enumerate(sel):
        f = frames[k]
        set_heights(pit, f["hm"]["h"])
        # excavator
        fr = {m["links"][n]: place(*f["links"][n]) for n in ("house", "boom", "stick", "bucket")}
        fr[m["crawler"]["node"]] = crawler
        for name, p in zip(m.get("parts", []), f.get("parts", [])):
            fr[name] = place(*p)
        exc.pose(fr)
        # carriers and the load in the one being loaded
        for u, cf in zip(cars, f.get("carriers", [])):
            u.pose({"base_link": place(*cf["base"]), c["vessel_node"]: place(*cf["pin"])})
        if truck is not None:
            owner = f.get("load_owner", -1)
            if f.get("carriers") and 0 <= owner < len(f["carriers"]):
                truck.hide_render = False
                set_heights(truck, f["truck_hm"]["h"])
                truck.matrix_world = CB @ T3(*f["carriers"][owner]["pin"]) @ parked_pin.inverted()
            else:
                truck.hide_render = True
        if spoil is not None and "spoil_hm" in f:
            h = f["spoil_hm"]["h"]
            set_heights(spoil, np.where(h > -3.4, h, -0.4))
        # dozer: blade arm turns about its lift joint, the lift cylinder follows (soil.js)
        if doz is not None:
            if "dozer" in f:
                z = f["dozer"]
                b = place(*z["base"])
                cyl = b @ doz.rel("base_link", d["cylinder_node"]) @ Matrix.Rotation(0.9 * z["lift"], 4, "Y")
                doz.pose({
                    "base_link": b,
                    d["arm_node"]: b @ doz.rel("base_link", d["arm_node"]) @ Matrix.Rotation(z["lift"], 4, "Y"),
                    d["cylinder_node"]: cyl,
                    d["piston_node"]: cyl @ Matrix.Translation(piston_axis * 0.54 * z["lift"]) @ doz.rel(d["cylinder_node"], d["piston_node"]),
                })
            else:
                doz.hide()
        for j, rol in enumerate(rols):
            rs = f.get("rollers", [])
            if j < len(rs):
                b = place(*rs[j]["base"])
                pose = {"base_link": b}
                for n, rr in spin_r.items():
                    pose[n] = b @ rol.rel("base_link", n) @ Matrix.Rotation(travels[j][k] / rr, 4, "Y")
                rol.pose(pose)
            else:
                rol.hide()
        # grains
        g = f["grains"]
        me = grains.data
        me.clear_geometry()
        if len(g):
            v = (g[:, None, :] + ico_v[None, :, :]).reshape(-1, 3)
            fc = (ico_f[None, :, :] + (np.arange(len(g)) * len(ico_v))[:, None, None]).reshape(-1, 3)
            me.from_pydata(v.tolist(), [], fc.tolist())
        me.update()
        sc.render.filepath = str(OUT / f"frame_{i:05d}.png")
        bpy.ops.render.render(write_still=True)
        if i % 10 == 0:
            print(f"FRAME {i + 1}/{len(sel)} sim {f['sim_t']:.1f} s", flush=True)
    if STILL:
        shutil.copy(OUT / "frame_00000.png", OUT / f"still_{CAMERA}_{frames[sel[0]]['sim_t']:.0f}s.png")
        print(f"STILL {OUT}")
        return
    ffmpeg = shutil.which("ffmpeg")
    if ffmpeg:
        video = OUT / f"{REPLAY.name.split('.')[0]}_{CAMERA}.mp4"
        subprocess.run([ffmpeg, "-y", "-loglevel", "error", "-framerate", str(FPS), "-i", str(OUT / "frame_%05d.png"), "-c:v", "libx264", "-pix_fmt", "yuv420p", "-crf", "18", "-movflags", "+faststart", str(video)], check=True)
        print(f"VIDEO {video}")


main()
