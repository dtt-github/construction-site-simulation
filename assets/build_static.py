"""Convert single-mesh open models (Gazebo Fuel, CC0) to GLB for the site.

For each model: download the Fuel zip into assets/_downloads/, confirm its license through the Fuel
API (refuse anything that isn't CC0 / CC-BY), load the Collada mesh, convert it to glTF Y-up metres,
limit textures to 2048 px and write assets/site/<id>.glb with the source and license in the asset block.

    phase1/.venv/bin/python assets/build_static.py
"""

import io
import json
import math
import pathlib
import struct
import urllib.parse
import urllib.request
import xml.etree.ElementTree as ET
import zipfile

import trimesh
from PIL import Image

ROOT = pathlib.Path(__file__).resolve().parent
DOWNLOADS = ROOT / "_downloads"
OUT = ROOT / "site"
OK_LICENSES = {"https://creativecommons.org/publicdomain/zero/1.0/": "CC0-1.0", "https://creativecommons.org/licenses/by/4.0/": "CC-BY-4.0"}

FUEL = {
    "gazebo_tower_crane": ("OpenRobotics", "Tower crane", "meshes/tower_crane.dae"),
    "gazebo_construction_cone": ("OpenRobotics", "Construction Cone", "meshes/construction_cone.dae"),
    "gazebo_construction_barrel": ("OpenRobotics", "Construction Barrel", "meshes/construction_barrel.dae"),
    "gazebo_jersey_barrier": ("OpenRobotics", "Jersey Barrier", "meshes/jersey_barrier.dae"),
}


def fuel_info(owner, name):
    url = f"https://fuel.gazebosim.org/1.0/{owner}/models/{urllib.parse.quote(name)}"
    with urllib.request.urlopen(url, timeout=60) as r:
        return json.load(r), url


def fuel_zip(owner, name):
    zpath = DOWNLOADS / f"{name.replace(' ', '_')}.zip"
    if not zpath.exists():
        DOWNLOADS.mkdir(parents=True, exist_ok=True)
        with urllib.request.urlopen(f"https://fuel.gazebosim.org/1.0/{owner}/models/{urllib.parse.quote(name)}.zip", timeout=300) as r:
            zpath.write_bytes(r.read())
    out = DOWNLOADS / zpath.stem
    if not out.exists():
        with zipfile.ZipFile(zpath) as z:
            z.extractall(out)
    return out


def up_axis(dae_path):
    root = ET.parse(dae_path).getroot()
    el = root.find(".//{*}asset/{*}up_axis")
    return el.text.strip() if el is not None else "Y_UP"


def unit_metres(dae_path):
    """Collada <unit meter="..."> (these Gazebo meshes are authored in inches)."""
    el = ET.parse(dae_path).getroot().find(".//{*}asset/{*}unit")
    return float(el.get("meter", 1.0)) if el is not None else 1.0


def sdf_scale(folder):
    """Scale applied to the visual mesh in model.sdf (Gazebo applies it on top of the mesh units)."""
    el = ET.parse(folder / "model.sdf").getroot().find(".//visual//mesh/scale")
    return [float(v) for v in el.text.split()] if el is not None else [1.0, 1.0, 1.0]


def convert(asset_id):
    owner, name, mesh_rel = FUEL[asset_id]
    info, page = fuel_info(owner, name)
    license_id = OK_LICENSES.get(info.get("license_url"))
    if not license_id:
        raise SystemExit(f"{asset_id}: license {info.get('license_name')} is not allowed")
    folder = fuel_zip(owner, name)
    dae = folder / mesh_rel
    resolver = trimesh.resolvers.FilePathResolver(str(dae), allow_anywhere=True)  # textures live in ../materials
    scene = trimesh.load(dae, force="scene", resolver=resolver)
    for geom in scene.geometry.values():
        mat = getattr(geom.visual, "material", None)
        for attr in ("baseColorTexture", "image"):
            img = getattr(mat, attr, None)
            if isinstance(img, Image.Image) and max(img.size) > 2048:
                img = img.copy()
                img.thumbnail((2048, 2048))
                setattr(mat, attr, img)
    sx, sy, sz = (v * unit_metres(dae) for v in sdf_scale(folder))
    if up_axis(dae) == "Z_UP":
        scale = trimesh.transformations.scale_matrix(1.0)
        scale[:3, :3] = [[sx, 0, 0], [0, sy, 0], [0, 0, sz]]
        scene.apply_transform(trimesh.transformations.rotation_matrix(-math.pi / 2, [1, 0, 0]) @ scale)
    else:
        scale = trimesh.transformations.scale_matrix(1.0)
        scale[:3, :3] = [[sx, 0, 0], [0, sz, 0], [0, 0, sy]]
        scene.apply_transform(scale)
    # sit on the ground
    scene.apply_translation([0, -scene.bounds[0][1], 0])
    glb = scene.export(file_type="glb")
    json_len = struct.unpack_from("<I", glb, 12)[0]
    doc = json.loads(glb[20 : 20 + json_len])
    authors = info.get("authors") or info.get("owner")
    doc["asset"]["copyright"] = f"\"{name}\" from Gazebo Fuel ({page}), {license_id}. Converted to glTF Y-up by the construction-site-simulation project."
    doc["asset"]["extras"] = {"source": page, "license": license_id}
    blob = json.dumps(doc, separators=(",", ":")).encode()
    blob += b" " * ((-len(blob)) % 4)
    rest = glb[20 + json_len :]
    glb = struct.pack("<III", 0x46546C67, 2, 20 + len(blob) + len(rest)) + struct.pack("<II", len(blob), 0x4E4F534A) + blob + rest
    OUT.mkdir(parents=True, exist_ok=True)
    out = OUT / f"{asset_id}.glb"
    out.write_bytes(glb)
    size = scene.bounds[1] - scene.bounds[0]
    print(json.dumps({"id": asset_id, "license": license_id, "page": page, "kb": len(glb) // 1024, "triangles": sum(len(g.faces) for g in scene.geometry.values()), "size_m": [round(float(v), 2) for v in size], "authors": authors}))


if __name__ == "__main__":
    for k in FUEL:
        convert(k)
