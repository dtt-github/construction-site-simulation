"""Build articulated GLB models of real construction machines from open ROS description packages.

Source: PWRI OPERA project (https://github.com/pwri-opera), Apache-2.0. Each package has Collada
meshes (metres, Z-up) per moving part and a URDF/xacro with the joints. This script:

  1. downloads the package zip into assets/_downloads/ (git-ignored),
  2. reads the URDF (evaluating simple ${...} xacro expressions),
  3. builds one GLB per machine: a node per link, placed at its joint origin, holding that link's
     meshes; Z-up is converted to glTF Y-up at the root; textures are limited to 2048 px,
  4. writes each moving node's joint into node extras, in the same format the site uses:
     extras.joints = [{name, type, parent, pivot, axis, min, max, unit}] with pivot and axis in
     model space at rest (Y-up), plus extras.urdf_joint with the joint in the node's own frame,
  5. records the source and license in the glTF asset block.

    phase1/.venv/bin/python assets/build_machines.py            # all machines
    phase1/.venv/bin/python assets/build_machines.py zx200      # one machine
"""

import io
import json
import math
import pathlib
import re
import struct
import sys
import urllib.request
import xml.etree.ElementTree as ET
import zipfile

import numpy as np
import trimesh
from PIL import Image

ROOT = pathlib.Path(__file__).resolve().parent
DOWNLOADS = ROOT / "_downloads"
OUT = ROOT / "machinery"
MAX_TEXTURE = 2048

MACHINES = {
    "zx200": {
        "repo": "pwri-opera/zx200_ros2",
        "urdf": "zx200_description/urdf/zx200.xacro",
        "package": "zx200_description",
        "name": "Hitachi ZX200 hydraulic excavator (20 t class)",
    },
    "ic120": {
        "repo": "pwri-opera/ic120_ros2",
        "urdf": "ic120_description/urdf/ic120_gazebo.urdf",
        "package": "ic120_description",
        "name": "Hitachi IC120 crawler dump carrier",
    },
    "d37pxi": {
        "repo": "pwri-opera/d37pxi_ros2",
        "urdf": "d37pxi_description/urdf/d37pxi24.urdf",
        "package": "d37pxi_description",
        "name": "Komatsu D37PXi-24 bulldozer",
    },
}


def fetch(repo):
    name = repo.split("/")[1]
    zpath = DOWNLOADS / f"{name}.zip"
    if not zpath.exists():
        DOWNLOADS.mkdir(parents=True, exist_ok=True)
        url = f"https://github.com/{repo}/archive/refs/heads/main.zip"
        print(f"downloading {url}")
        with urllib.request.urlopen(url, timeout=300) as r:
            zpath.write_bytes(r.read())
    out = DOWNLOADS / name
    if not out.exists():
        with zipfile.ZipFile(zpath) as z:
            z.extractall(out)
    return next(out.iterdir())  # <repo>-main/


def safe_math(expr):
    """Evaluate a xacro ${...} expression: numbers, + - * /, unary minus, pi and radians() only.
    Downloaded files are data, so nothing else is accepted (no names, calls or attributes)."""
    import ast
    import operator

    ops = {ast.Add: operator.add, ast.Sub: operator.sub, ast.Mult: operator.mul, ast.Div: operator.truediv, ast.USub: operator.neg, ast.UAdd: operator.pos}

    def ev(node):
        if isinstance(node, ast.Expression):
            return ev(node.body)
        if isinstance(node, ast.Constant) and isinstance(node.value, (int, float)):
            return float(node.value)
        if isinstance(node, ast.Name) and node.id == "pi":
            return math.pi
        if isinstance(node, ast.BinOp) and type(node.op) in ops:
            return ops[type(node.op)](ev(node.left), ev(node.right))
        if isinstance(node, ast.UnaryOp) and type(node.op) in ops:
            return ops[type(node.op)](ev(node.operand))
        if isinstance(node, ast.Call) and isinstance(node.func, ast.Name) and node.func.id == "radians" and len(node.args) == 1:
            return math.radians(ev(node.args[0]))
        raise ValueError(f"unsupported xacro expression: {expr!r}")

    return ev(ast.parse(expr.strip(), mode="eval"))


def read_urdf(path):
    text = path.read_text()
    text = re.sub(r"\$\{([^}]*)\}", lambda m: repr(safe_math(m.group(1))), text)
    root = ET.fromstring(text)
    links = {}
    for link in root.findall("link"):
        meshes = []
        for vis in link.findall("visual"):
            mesh = vis.find("geometry/mesh")
            if mesh is None:
                continue
            o = vis.find("origin")
            meshes.append((mesh.get("filename"), origin_matrix(o)))
        links[link.get("name")] = meshes
    joints = []
    for j in root.findall("joint"):
        lim = j.find("limit")
        ax = j.find("axis")
        joints.append(
            {
                "name": j.get("name"),
                "type": j.get("type"),
                "parent": j.find("parent").get("link"),
                "child": j.find("child").get("link"),
                "origin": origin_matrix(j.find("origin")),
                "axis": [float(v) for v in ax.get("xyz").split()] if ax is not None else [1.0, 0.0, 0.0],
                "lower": float(lim.get("lower")) if lim is not None and lim.get("lower") is not None else None,
                "upper": float(lim.get("upper")) if lim is not None and lim.get("upper") is not None else None,
            }
        )
    return links, joints


def origin_matrix(o):
    if o is None:
        return np.eye(4)
    xyz = [float(v) for v in (o.get("xyz") or "0 0 0").split()]
    rpy = [float(v) for v in (o.get("rpy") or "0 0 0").split()]
    m = trimesh.transformations.euler_matrix(*rpy, axes="sxyz")
    m[:3, 3] = xyz
    return m


def resolve_mesh(pkg_dir, filename):
    """package://<pkg>/meshes/x.dae or package://meshes/x.dae -> a file inside the package (never outside)."""
    rel = filename.replace("package://", "")
    rel = rel[len(pkg_dir.name) + 1 :] if rel.startswith(pkg_dir.name + "/") else rel
    path = (pkg_dir / rel).resolve()
    if not path.exists() or pkg_dir.resolve() not in path.parents:
        matches = list(pkg_dir.rglob(pathlib.Path(rel).name))
        if not matches:
            raise FileNotFoundError(filename)
        path = matches[0]
    return path


def shrink_textures(geom):
    mat = getattr(geom.visual, "material", None)
    for attr in ("baseColorTexture", "image"):
        img = getattr(mat, attr, None)
        if isinstance(img, Image.Image) and max(img.size) > MAX_TEXTURE:
            img = img.copy()
            img.thumbnail((MAX_TEXTURE, MAX_TEXTURE))
            setattr(mat, attr, img)


def build(key):
    cfg = MACHINES[key]
    src = fetch(cfg["repo"])
    pkg_dir = src / cfg["package"]
    links, joints = read_urdf(src / cfg["urdf"])
    children = {}
    for j in joints:
        children.setdefault(j["parent"], []).append(j)
    root_link = next(name for name in links if all(j["child"] != name for j in joints))

    scene = trimesh.Scene()
    z_to_y = trimesh.transformations.rotation_matrix(-math.pi / 2, [1, 0, 0])
    scene.graph.update(frame_from=scene.graph.base_frame, frame_to=key, matrix=z_to_y)
    world = {root_link: np.eye(4)}  # link frames in the Z-up model space
    joint_extras = {}
    tri_count = 0

    def add_link(name, parent_node, matrix):
        nonlocal tri_count
        scene.graph.update(frame_from=parent_node, frame_to=name, matrix=matrix)
        for filename, vis_origin in links.get(name, []):
            mesh_path = resolve_mesh(pkg_dir, filename)
            part = trimesh.load(mesh_path, force="scene")
            for gname, geom in part.geometry.items():
                shrink_textures(geom)
                tri_count += len(geom.faces)
                for node in part.graph.geometry_nodes.get(gname, []):
                    tf = part.graph.get(node)[0]
                    scene.add_geometry(geom, node_name=f"{name}/{gname}/{node}", geom_name=f"{name}_{gname}", parent_node_name=name, transform=vis_origin @ tf)
        for j in children.get(name, []):
            world[j["child"]] = world[name] @ j["origin"]
            if j["type"] in ("revolute", "continuous", "prismatic"):
                to_y = z_to_y[:3, :3]
                pivot = (z_to_y @ np.append(world[j["child"]][:3, 3], 1.0))[:3]
                axis = to_y @ (world[j["child"]][:3, :3] @ np.array(j["axis"]))
                rev = j["type"] != "prismatic"
                joint_extras[j["child"]] = {
                    "joints": [
                        {
                            "name": j["name"],
                            "type": "revolute" if rev else "prismatic",
                            "parent": j["parent"],
                            "pivot": [round(float(v), 6) for v in pivot],
                            "axis": [round(float(v), 6) for v in axis],
                            **({"min": round(math.degrees(j["lower"]), 3), "max": round(math.degrees(j["upper"]), 3)} if rev and j["lower"] is not None and j["type"] != "continuous" else {}),
                            **({"min": j["lower"], "max": j["upper"]} if not rev and j["lower"] is not None else {}),
                            "unit": "degrees" if rev else "meters",
                        }
                    ],
                    "urdf_joint": {"name": j["name"], "type": j["type"], "axis_local": j["axis"], "lower": j["lower"], "upper": j["upper"]},
                }
            add_link(j["child"], name, j["origin"])

    add_link(root_link, key, np.eye(4))
    OUT.mkdir(parents=True, exist_ok=True)
    glb = scene.export(file_type="glb")
    glb = add_extras(glb, joint_extras, cfg)
    out = OUT / f"opera_{key}.glb"
    out.write_bytes(glb)
    bounds = scene.bounds
    print(json.dumps({"machine": key, "file": str(out.relative_to(ROOT.parent)), "kb": len(glb) // 1024, "triangles": tri_count, "joints": list(joint_extras), "size_m_yup": (bounds[1] - bounds[0]).round(2).tolist()}))


def add_extras(glb, joint_extras, cfg):
    """Insert node extras and asset copyright into the GLB's JSON chunk."""
    json_len = struct.unpack_from("<I", glb, 12)[0]
    doc = json.loads(glb[20 : 20 + json_len])
    rest = glb[20 + json_len :]
    for node in doc.get("nodes", []):
        if node.get("name") in joint_extras:
            node.setdefault("extras", {}).update(joint_extras[node["name"]])
    doc["asset"]["copyright"] = f"{cfg['name']}: Public Works Research Institute (PWRI), OPERA project, https://github.com/{cfg['repo']}, Apache-2.0. Converted to glTF by the construction-site-simulation project (changed: format, axes, texture size, node layout)."
    doc["asset"]["extras"] = {"source": f"https://github.com/{cfg['repo']}", "license": "Apache-2.0"}
    blob = json.dumps(doc, separators=(",", ":")).encode()
    blob += b" " * ((-len(blob)) % 4)
    total = 12 + 8 + len(blob) + len(rest)
    return struct.pack("<III", 0x46546C67, 2, total) + struct.pack("<II", len(blob), 0x4E4F534A) + blob + rest


if __name__ == "__main__":
    for k in sys.argv[1:] or list(MACHINES):
        build(k)
