"""Read KHR_physics_rigid_bodies data and node extras from a GLB file into a summary dict."""

import json
import struct
import sys


def glb_json(path):
    with open(path, "rb") as fh:
        magic, _version, _length = struct.unpack("<III", fh.read(12))
        assert magic == 0x46546C67, "not a GLB"
        chunk_len, chunk_type = struct.unpack("<II", fh.read(8))
        assert chunk_type == 0x4E4F534A
        return json.loads(fh.read(chunk_len))


def summary(path):
    g = glb_json(path)
    ext = g.get("extensions", {}).get("KHR_physics_rigid_bodies", {})
    bodies, joints, colliders, hulls = [], 0, 0, 0
    for i, node in enumerate(g.get("nodes", [])):
        e = node.get("extensions", {}).get("KHR_physics_rigid_bodies", {})
        x = node.get("extras", {})
        if "motion" in e:
            bodies.append({"node": node.get("name", str(i)), "mass": e["motion"].get("mass"), "extras_mass": x.get("mass"), "density": x.get("density")})
        if "collider" in e:
            colliders += 1
        if "joint" in e:
            joints += 1
        hulls += len(x.get("collisionHulls", []) or [])
    return {
        "extensionsUsed": g.get("extensionsUsed", []),
        "materials": ext.get("physicsMaterials", []),
        "bodies": bodies,
        "total_mass_kg": round(sum(b["mass"] or 0 for b in bodies), 3),
        "colliders": colliders,
        "joints": joints,
        "hulls": hulls,
    }


if __name__ == "__main__":
    print(json.dumps(summary(sys.argv[1]), indent=1))
