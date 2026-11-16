"""Small modelling kit for building machines procedurally in Blender (bpy + bmesh), shared by the machine
scripts in this folder. Everything is in metres, Z up, in the frame of whatever object a part is parented to.

Shapes are added to a bmesh (add_*), then turned into an object with make(); make() adds a bevel modifier
(applied on glTF export) and smooth shading with sharp edges kept by the auto-smooth angle.
"""

import json
import math
import struct

import bmesh
import bpy
import numpy as np
from mathutils import Matrix, Vector

# ------------------------------------------------------------------ scene


def reset():
    bpy.ops.wm.read_factory_settings(use_empty=True)


def coll():
    return bpy.context.scene.collection


def empty(name, parent=None, matrix=None, size=0.15):
    e = bpy.data.objects.new(name, None)
    coll().objects.link(e)
    e.empty_display_size = size
    if parent is not None:
        e.parent = parent
    if matrix is not None:
        e.matrix_basis = matrix
    return e


def srgb(hexstr):
    """'#rrggbb' -> linear RGB (Blender colour values are linear)."""
    h = hexstr.lstrip("#")
    c = [int(h[i : i + 2], 16) / 255 for i in (0, 2, 4)]
    return tuple(x / 12.92 if x <= 0.04045 else ((x + 0.055) / 1.055) ** 2.4 for x in c)


def material(name, color, rough=0.5, metal=0.0, coat=0.0, alpha=1.0, emit=None, emit_strength=2.0):
    m = bpy.data.materials.new(name)
    m.use_nodes = True
    p = m.node_tree.nodes["Principled BSDF"]
    p.inputs["Base Color"].default_value = (*srgb(color), 1)
    p.inputs["Roughness"].default_value = rough
    p.inputs["Metallic"].default_value = metal
    if coat:
        p.inputs["Coat Weight"].default_value = coat
        p.inputs["Coat Roughness"].default_value = 0.08
    if alpha < 1:
        p.inputs["Alpha"].default_value = alpha
        m.blend_method = "BLEND"
        m.shadow_method = "NONE"
    if emit:
        p.inputs["Emission Color"].default_value = (*srgb(emit), 1)
        p.inputs["Emission Strength"].default_value = emit_strength
    m.diffuse_color = (*srgb(color), alpha)  # what the Workbench preview shows
    m.metallic, m.roughness = metal, rough
    return m


UV_TILE = 2.0  # metres per texture tile


def box_uv(bm):
    """Box-projected UVs (each face takes the plane its normal faces most), one tile per UV_TILE metres, in the
    part's own frame, so the tiling textures keep a constant size on every part."""
    uv = bm.loops.layers.uv.verify()
    bm.normal_update()
    for f in bm.faces:
        n = f.normal
        ax = max(range(3), key=lambda k: abs(n[k]))
        for loop in f.loops:
            c = loop.vert.co
            u, v = ((c.y, c.z), (c.x, c.z), (c.x, c.y))[ax]
            loop[uv].uv = (u / UV_TILE, v / UV_TILE)


# ------------------------------------------------------------------ procedural textures (tileable, numpy)


def _fbm(n, beta, rng, lo=1.0):
    """Tileable fractal noise in [0, 1]: white noise shaped to a 1/f^beta spectrum."""
    w = np.fft.fft2(rng.standard_normal((n, n)))
    f = np.fft.fftfreq(n) * n
    r = np.sqrt(f[:, None] ** 2 + f[None, :] ** 2)
    r[0, 0] = 1.0
    h = np.real(np.fft.ifft2(w / np.maximum(r, lo) ** beta))
    return (h - h.min()) / (h.max() - h.min() + 1e-9)


def _blur(a, sigma):
    n = a.shape[0]
    f = np.fft.fftfreq(n)
    g = np.exp(-2 * (math.pi * sigma) ** 2 * (f[:, None] ** 2 + f[None, :] ** 2))
    return np.real(np.fft.ifft2(np.fft.fft2(a) * g))


def _scratches(n, count, rng, length=(0.008, 0.04), bias=None):
    """Thin random scratches (wrapping), as a [0, 1] mask."""
    m = np.zeros((n, n))
    for _ in range(count):
        x, y = rng.uniform(0, n, 2)
        a = rng.uniform(0, math.pi) if bias is None else bias + rng.normal(0, 0.25)
        L = rng.uniform(*length) * n
        t = np.arange(0, L, 0.5)
        xs = (x + t * math.cos(a)).astype(int) % n
        ys = (y + t * math.sin(a)).astype(int) % n
        m[ys, xs] = np.maximum(m[ys, xs], rng.uniform(0.4, 1.0))
    return np.clip(_blur(m, 0.6) * 2.5, 0, 1)


def _hex01(h):
    h = h.lstrip("#")
    return np.array([int(h[i : i + 2], 16) / 255 for i in (0, 2, 4)])


SURFACES = {
    # kind: colour variation, chipping (share of area), scratch count, grime, rough base and spread, metal, bump
    "paint": dict(var=0.05, chips=0.035, chip_color="#3b3936", chip_rough=0.55, chip_metal=0.6, scratches=45, grime=0.10, rough=(0.36, 0.08), metal=0.0, bump=0.35),
    "paint_worn": dict(var=0.07, chips=0.08, chip_color="#4a4744", chip_rough=0.5, chip_metal=0.7, scratches=70, grime=0.28, rough=(0.42, 0.1), metal=0.0, bump=0.5),
    "steel": dict(var=0.12, chips=0.0, scratches=160, grime=0.3, rough=(0.55, 0.15), metal=0.6, bump=0.8, rust="#5b3b24", rust_share=0.10),
    "track": dict(var=0.18, chips=0.0, scratches=60, grime=0.45, rough=(0.72, 0.1), metal=0.55, bump=1.0, rust="#6a4126", rust_share=0.3),
    "worn_edge": dict(var=0.1, chips=0.0, scratches=260, grime=0.1, rough=(0.38, 0.12), metal=0.9, bump=0.7),
    "rubber": dict(var=0.06, chips=0.0, scratches=15, grime=0.3, rough=(0.82, 0.06), metal=0.0, bump=0.6),
}


def texture_set(name, color, kind, size=1024, seed=0):
    """Albedo (sRGB), ORM (R 1, G roughness, B metalness) and normal map images for one material."""
    s = SURFACES[kind]
    rng = np.random.default_rng(seed)
    n = size
    base = _hex01(color)
    lo, mid, hi = _fbm(n, 2.4, rng), _fbm(n, 1.6, rng), _fbm(n, 1.0, rng)
    alb = np.ones((n, n, 3)) * base
    alb *= (1 + s["var"] * (lo - 0.5) * 2 + 0.3 * s["var"] * (hi - 0.5))[..., None]
    grime = np.clip((lo * 0.7 + mid * 0.3 - 0.45) * 2.2, 0, 1) * s["grime"]
    alb = alb * (1 - grime[..., None]) + _hex01("#4d4234") * grime[..., None]
    rough = s["rough"][0] + s["rough"][1] * (mid - 0.5) * 2 + 0.25 * grime
    metal = np.full((n, n), s["metal"])
    height = 0.15 * (hi - 0.5)  # orange peel / cast texture
    if s.get("rust"):
        rust = np.clip((_fbm(n, 1.8, rng) - (1 - s["rust_share"])) * 6, 0, 1)
        alb = alb * (1 - rust[..., None]) + _hex01(s["rust"]) * rust[..., None]
        rough = rough + 0.2 * rust
        metal = metal * (1 - 0.7 * rust)
        height += 0.3 * rust * hi
    if s["chips"]:
        cn = _fbm(n, 1.3, rng) * 0.6 + hi * 0.4
        chips = np.clip((cn - (1 - s["chips"])) * 25, 0, 1)
        alb = alb * (1 - chips[..., None]) + _hex01(s["chip_color"]) * chips[..., None]
        rough = rough * (1 - chips) + s["chip_rough"] * chips
        metal = metal * (1 - chips) + s["chip_metal"] * chips
        height -= 0.6 * chips
    sc = _scratches(n, s["scratches"], rng)
    alb = alb * (1 - 0.1 * sc[..., None]) + (base * 0.7 + 0.15) * 0.1 * sc[..., None]  # faint: mostly a gloss and bump change
    rough = rough * (1 - 0.5 * sc) + 0.25 * sc
    height -= 0.1 * sc
    h = height * s["bump"]
    gx = (np.roll(h, -1, 1) - np.roll(h, 1, 1)) * 0.5
    gy = (np.roll(h, -1, 0) - np.roll(h, 1, 0)) * 0.5
    nrm = np.dstack([-gx * 3.5, -gy * 3.5, np.ones_like(h)])
    nrm /= np.linalg.norm(nrm, axis=2, keepdims=True)
    imgs = {}
    for key, arr, srgb_space in (
        ("albedo", np.clip(alb, 0, 1), True),
        ("orm", np.dstack([np.ones_like(rough), np.clip(rough, 0.03, 1), np.clip(metal, 0, 1)]), False),
        ("normal", nrm * 0.5 + 0.5, False),
    ):
        img = bpy.data.images.new(f"{name}_{key}", n, n, alpha=False)
        img.colorspace_settings.name = "sRGB" if srgb_space else "Non-Color"
        rgba = np.dstack([arr, np.ones((n, n))]).astype(np.float32)
        img.pixels.foreach_set(rgba.ravel())
        img.file_format = "JPEG"
        img.pack()
        imgs[key] = img
    return imgs


def textured(mat, imgs):
    """Wire a texture set into a Principled material (glTF-exportable layout)."""
    nt = mat.node_tree
    p = nt.nodes["Principled BSDF"]
    a = nt.nodes.new("ShaderNodeTexImage")
    a.image = imgs["albedo"]
    nt.links.new(a.outputs["Color"], p.inputs["Base Color"])
    o = nt.nodes.new("ShaderNodeTexImage")
    o.image = imgs["orm"]
    sep = nt.nodes.new("ShaderNodeSeparateColor")
    nt.links.new(o.outputs["Color"], sep.inputs["Color"])
    nt.links.new(sep.outputs["Green"], p.inputs["Roughness"])
    nt.links.new(sep.outputs["Blue"], p.inputs["Metallic"])
    nm = nt.nodes.new("ShaderNodeTexImage")
    nm.image = imgs["normal"]
    nmap = nt.nodes.new("ShaderNodeNormalMap")
    nmap.inputs["Strength"].default_value = 1.0
    nt.links.new(nm.outputs["Color"], nmap.inputs["Color"])
    nt.links.new(nmap.outputs["Normal"], p.inputs["Normal"])
    return mat


def surface(name, color, kind, size=1024, seed=0, coat=0.0):
    """A textured material in one call."""
    s = SURFACES[kind]
    m = material(name, color, rough=s["rough"][0], metal=s["metal"], coat=coat)
    return textured(m, texture_set(name, color, kind, size, seed))


def make(name, parent, mat, build, bevel=0.0, segs=2, smooth=True, matrix=None, angle=40, mesh=None):
    """Object from a bmesh builder (or an existing mesh, shared). Bevel is a modifier, applied on export."""
    if mesh is None:
        bm = bmesh.new()
        build(bm)
        bmesh.ops.remove_doubles(bm, verts=bm.verts[:], dist=1e-6)
        bmesh.ops.recalc_face_normals(bm, faces=bm.faces[:])
        box_uv(bm)
        mesh = bpy.data.meshes.new(name)
        bm.to_mesh(mesh)
        bm.free()
        for p in mesh.polygons:
            p.use_smooth = smooth
        mesh.use_auto_smooth = True
        mesh.auto_smooth_angle = math.radians(35)
        mesh.materials.append(mat)
    ob = bpy.data.objects.new(name, mesh)
    coll().objects.link(ob)
    if parent is not None:
        ob.parent = parent
    if matrix is not None:
        ob.matrix_basis = matrix
    if bevel > 0:
        m = ob.modifiers.new("bevel", "BEVEL")
        m.width = bevel
        m.segments = segs
        m.limit_method = "ANGLE"
        m.angle_limit = math.radians(angle)
        m.harden_normals = True
        m.miter_outer = "MITER_ARC"
    return ob


# ------------------------------------------------------------------ shapes


def add_box(bm, lo, hi):
    lo, hi = Vector(lo), Vector(hi)
    c, s = (lo + hi) / 2, hi - lo
    bmesh.ops.create_cube(bm, size=1.0, matrix=Matrix.Translation(c) @ Matrix.Diagonal((*s, 1)))


def add_obox(bm, center, axes, half):
    """Oriented box: centre, three axis vectors (unit), half sizes along them."""
    M = Matrix.Identity(4)
    for k in range(3):
        a = Vector(axes[k]).normalized() * (2 * half[k])
        M[0][k], M[1][k], M[2][k] = a
    M.translation = Vector(center)
    bmesh.ops.create_cube(bm, size=1.0, matrix=M)


def add_cyl(bm, r, p0, p1, segs=32, r1=None):
    """Closed cylinder (or cone, radius r at p0 and r1 at p1) between two points."""
    p0, p1 = Vector(p0), Vector(p1)
    d = p1 - p0
    q = Vector((0, 0, 1)).rotation_difference(d.normalized())
    M = Matrix.Translation((p0 + p1) / 2) @ q.to_matrix().to_4x4()
    bmesh.ops.create_cone(bm, cap_ends=True, cap_tris=False, segments=segs, radius1=r, radius2=r if r1 is None else r1, depth=d.length, matrix=M)


def add_sphere(bm, r, c, segs=16, rings=8):
    bmesh.ops.create_uvsphere(bm, u_segments=segs, v_segments=rings, radius=r, matrix=Matrix.Translation(Vector(c)))


def _dedupe(pts, eps=1e-5):
    out = []
    for p in pts:
        if not out or math.dist(p, out[-1]) > eps:
            out.append(tuple(p))
    if len(out) > 2 and math.dist(out[0], out[-1]) <= eps:
        out.pop()
    return out


def add_prism_y(bm, pts, y0, y1):
    """Polygon in the x-z plane, extruded along y from y0 to y1."""
    pts = _dedupe(pts)
    a = [bm.verts.new((x, y0, z)) for x, z in pts]
    b = [bm.verts.new((x, y1, z)) for x, z in pts]
    bm.faces.new(a[::-1])
    bm.faces.new(b)
    n = len(pts)
    for i in range(n):
        j = (i + 1) % n
        bm.faces.new((a[i], a[j], b[j], b[i]))


def add_prism_z(bm, pts, z0, z1):
    """Polygon in the x-y plane, extruded along z."""
    pts = _dedupe(pts)
    a = [bm.verts.new((x, y, z0)) for x, y in pts]
    b = [bm.verts.new((x, y, z1)) for x, y in pts]
    bm.faces.new(a[::-1])
    bm.faces.new(b)
    n = len(pts)
    for i in range(n):
        j = (i + 1) % n
        bm.faces.new((a[i], a[j], b[j], b[i]))


def add_prism_frame(bm, pts, origin, u, v, w0, w1):
    """Polygon given in a plane spanned by u, v (2D coords), extruded along u x v from w0 to w1."""
    pts = _dedupe(pts)
    o, u, v = Vector(origin), Vector(u).normalized(), Vector(v).normalized()
    w = u.cross(v).normalized()
    a = [bm.verts.new(o + x * u + y * v + w0 * w) for x, y in pts]
    b = [bm.verts.new(o + x * u + y * v + w1 * w) for x, y in pts]
    bm.faces.new(a[::-1])
    bm.faces.new(b)
    n = len(pts)
    for i in range(n):
        j = (i + 1) % n
        bm.faces.new((a[i], a[j], b[j], b[i]))


def add_tube(bm, pts, r, segs=12, caps=True):
    """Sweep a circle along a polyline (parallel-transport frames)."""
    pts = [Vector(p) for p in _dedupe(pts)]
    rings, n_prev = [], None
    for i, p in enumerate(pts):
        t = (pts[min(i + 1, len(pts) - 1)] - pts[max(i - 1, 0)]).normalized()
        if n_prev is None:
            ref = Vector((0, 0, 1)) if abs(t.z) < 0.9 else Vector((1, 0, 0))
            n = t.cross(ref).normalized()
        else:
            n = (n_prev - t * n_prev.dot(t)).normalized()
        b = t.cross(n)
        n_prev = n
        rings.append([bm.verts.new(p + r * (math.cos(a) * n + math.sin(a) * b)) for a in (2 * math.pi * k / segs for k in range(segs))])
    for A, B in zip(rings, rings[1:]):
        for k in range(segs):
            bm.faces.new((A[k], A[(k + 1) % segs], B[(k + 1) % segs], B[k]))
    if caps:
        bm.faces.new(rings[0][::-1])
        bm.faces.new(rings[-1])


def add_gear(bm, center, axis_y_half, r_root, r_tip, teeth, y0, y1):
    """Spur-gear-like sprocket in the x-z plane at centre (x, z), extruded along y."""
    cx, cz = center
    pts = []
    for k in range(teeth):
        a0 = 2 * math.pi * k / teeth
        da = 2 * math.pi / teeth
        for f, r in ((0.0, r_root), (0.22, r_tip), (0.42, r_tip), (0.64, r_root)):
            a = a0 + f * da
            pts.append((cx + r * math.cos(a), cz + r * math.sin(a)))
    add_prism_y(bm, pts, y0, y1)


# ------------------------------------------------------------------ 2D helpers


def catmull(pts, n=8, closed=False):
    """Catmull-Rom spline through points (2D or 3D), n samples per segment."""
    P = [np.array(p, float) for p in pts]
    if closed:
        P = [P[-1]] + P + [P[0], P[1]]
    else:
        P = [2 * P[0] - P[1]] + P + [2 * P[-1] - P[-2]]
    out = []
    for i in range(1, len(P) - 2):
        p0, p1, p2, p3 = P[i - 1], P[i], P[i + 1], P[i + 2]
        for k in range(n):
            t = k / n
            out.append(0.5 * ((2 * p1) + (-p0 + p2) * t + (2 * p0 - 5 * p1 + 4 * p2 - p3) * t * t + (-p0 + 3 * p1 - 3 * p2 + p3) * t**3))
    if not closed:
        out.append(P[-2])
    return [tuple(x) for x in out]


def fillet(pts, r, n=5):
    """Round every corner of a closed polygon with a quadratic arc of up to radius r."""
    out = []
    N = len(pts)
    for i in range(N):
        p0, p, p1 = (np.array(pts[(i + k) % N], float) for k in (-1, 0, 1))
        e0, e1 = p0 - p, p1 - p
        a = p + e0 / np.linalg.norm(e0) * min(r, np.linalg.norm(e0) * 0.45)
        b = p + e1 / np.linalg.norm(e1) * min(r, np.linalg.norm(e1) * 0.45)
        for k in range(n + 1):
            t = k / n
            out.append(tuple((1 - t) ** 2 * a + 2 * (1 - t) * t * p + t**2 * b))
    return out


def hull(points):
    """Convex hull (counter-clockwise) of 2D points."""
    P = sorted(set((round(x, 6), round(y, 6)) for x, y in points))
    if len(P) < 3:
        return P

    def cross(o, a, b):
        return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])

    lo, up = [], []
    for p in P:
        while len(lo) >= 2 and cross(lo[-2], lo[-1], p) <= 0:
            lo.pop()
        lo.append(p)
    for p in reversed(P):
        while len(up) >= 2 and cross(up[-2], up[-1], p) <= 0:
            up.pop()
        up.append(p)
    return lo[:-1] + up[:-1]


def circle(c, r, n=24, a0=0.0, a1=2 * math.pi):
    return [(c[0] + r * math.cos(a0 + (a1 - a0) * k / n), c[1] + r * math.sin(a0 + (a1 - a0) * k / n)) for k in range(n + 1)]


def circle_hull(circles, extra=(), n=28):
    pts = list(extra)
    for c, r in circles:
        pts += circle(c, r, n)
    return hull(pts)


def offset_polyline(pts, d):
    """Offset an open 2D polyline by d along its left normal."""
    P = np.array(pts, float)
    out = []
    for i in range(len(P)):
        t = P[min(i + 1, len(P) - 1)] - P[max(i - 1, 0)]
        t /= np.linalg.norm(t)
        out.append(tuple(P[i] + d * np.array([-t[1], t[0]])))
    return out


# ------------------------------------------------------------------ glTF post-processing


def read_glb(path):
    b = open(path, "rb").read()
    n = struct.unpack("<I", b[12:16])[0]
    js = json.loads(b[20 : 20 + n])
    rest = b[20 + n :]
    binchunk = b""
    if len(rest) >= 8:
        ln = struct.unpack("<I", rest[:4])[0]
        binchunk = rest[8 : 8 + ln]
    return js, binchunk


def write_glb(path, js, binchunk):
    j = json.dumps(js, separators=(",", ":")).encode()
    j += b" " * (-len(j) % 4)
    binchunk += b"\0" * (-len(binchunk) % 4)
    total = 12 + 8 + len(j) + (8 + len(binchunk) if binchunk else 0)
    out = struct.pack("<III", 0x46546C67, 2, total) + struct.pack("<II", len(j), 0x4E4F534A) + j
    if binchunk:
        out += struct.pack("<II", len(binchunk), 0x004E4942) + binchunk
    open(path, "wb").write(out)


def export_glb(path):
    bpy.ops.export_scene.gltf(
        filepath=str(path),
        export_format="GLB",
        export_yup=False,  # keep URDF Z-up frames; the post-processing adds the Y-up root
        export_apply=True,
        export_extras=False,
        export_cameras=False,
        export_lights=False,
        export_animations=False,
        export_skins=False,
        export_morph=False,
        export_image_format="JPEG",
        export_jpeg_quality=88,
    )
