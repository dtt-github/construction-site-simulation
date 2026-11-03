"""Replace coarse GLB collision hulls with a CoACD convex decomposition of the USD render mesh.

Usage: decompose_colliders.py <asset> [--threshold 0.04] [--max-hulls 24]

Reads every rigid body's meshes from the SimReady USD (Z-up), converts them to the GLB's
Y-up frame, expresses them in the matching GLB body node's local frame (the frame the
GLB stores hulls in), runs CoACD per body and writes web/hulls/<asset>.json:
{ "<body node name>": [[x, y, z, x, y, z, ...], ...] }.
"""

import argparse
import glob
import json
import os

import coacd
import numpy as np
from pxr import Usd, UsdGeom, UsdPhysics

from read_glb_physics import glb_json


def trs_matrix(node):
    m = np.eye(4)
    if "matrix" in node:
        return np.array(node["matrix"]).reshape(4, 4).T
    t = node.get("translation", [0, 0, 0])
    x, y, z, w = node.get("rotation", [0, 0, 0, 1])
    s = node.get("scale", [1, 1, 1])
    r = np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ]
    )
    m[:3, :3] = r * np.array(s)
    m[:3, 3] = t
    return m


def glb_world_matrices(g):
    parent = {}
    for i, n in enumerate(g["nodes"]):
        for c in n.get("children", []):
            parent[c] = i
    cache = {}

    def world(i):
        if i not in cache:
            local = trs_matrix(g["nodes"][i])
            cache[i] = world(parent[i]) @ local if i in parent else local
        return cache[i]

    return {g["nodes"][i].get("name"): world(i) for i in range(len(g["nodes"]))}


def usd_body_meshes(stage):
    """Map body prim name -> (vertices Nx3 in USD world, faces Mx3)."""
    xf = UsdGeom.XformCache()
    out = {}
    for prim in stage.Traverse(Usd.TraverseInstanceProxies(Usd.PrimDefaultPredicate)):
        if not prim.HasAPI(UsdPhysics.RigidBodyAPI):
            continue
        verts, faces = [], []
        base = 0
        for p in Usd.PrimRange(prim, Usd.TraverseInstanceProxies(Usd.PrimDefaultPredicate)):
            if p != prim and p.HasAPI(UsdPhysics.RigidBodyAPI):
                continue  # nested body
            if not p.IsA(UsdGeom.Mesh):
                continue
            purpose = UsdGeom.Imageable(p).ComputePurpose()
            if purpose not in (UsdGeom.Tokens.default_, UsdGeom.Tokens.render):
                continue
            mesh = UsdGeom.Mesh(p)
            pts = mesh.GetPointsAttr().Get()
            counts = mesh.GetFaceVertexCountsAttr().Get()
            idx = mesh.GetFaceVertexIndicesAttr().Get()
            if not pts or not counts:
                continue
            m = np.array(xf.GetLocalToWorldTransform(p)).T  # row-major Gf -> column vectors
            P = np.c_[np.array(pts, dtype=float), np.ones(len(pts))] @ m.T
            verts.append(P[:, :3])
            k = 0
            for n in counts:
                for t in range(1, n - 1):
                    faces.append([base + idx[k], base + idx[k + t], base + idx[k + t + 1]])
                k += n
            base += len(pts)
        if verts:
            out[prim.GetName()] = (np.vstack(verts), np.array(faces, dtype=np.int64))
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("asset")
    ap.add_argument("--threshold", type=float, default=0.04)
    ap.add_argument("--max-hulls", type=int, default=24)
    ap.add_argument("--bodies", nargs="*", default=None, help="only keep these body names")
    args = ap.parse_args()

    root = glob.glob(f"usd/{args.asset}/simready_usd/*.usd")[0]
    stage = Usd.Stage.Open(root)
    g = glb_json(f"glb/{args.asset}.glb")
    glb_world = glb_world_matrices(g)

    # USD Z-up -> glTF Y-up: (x, y, z) -> (x, z, -y)
    z_to_y = np.array([[1, 0, 0], [0, 0, 1], [0, -1, 0]], dtype=float)

    result = {}
    report = {}
    for name, (V, F) in usd_body_meshes(stage).items():
        if name not in glb_world:
            report[name] = "no matching GLB node"
            continue
        Vy = V @ z_to_y.T
        inv = np.linalg.inv(glb_world[name])
        Vl = (np.c_[Vy, np.ones(len(Vy))] @ inv.T)[:, :3]
        parts = coacd.run_coacd(
            coacd.Mesh(Vl, F),
            threshold=args.threshold,
            max_convex_hull=args.max_hulls,
            preprocess_mode="auto",
            resolution=2000,
        )
        # CoACD hulls overshoot the mesh slightly; clamp them to the part's real bounds so,
        # e.g., a chassis does not reach below its wheels
        lo, hi = Vl.min(0), Vl.max(0)
        hulls = [np.round(np.clip(np.asarray(v, dtype=float), lo, hi), 5).reshape(-1).tolist() for v, _f in parts]
        result[name] = hulls
        report[name] = f"{len(F)} tris -> {len(hulls)} hulls"
    if args.bodies:
        result = {k: v for k, v in result.items() if k in args.bodies}
    os.makedirs("web/hulls", exist_ok=True)
    with open(f"web/hulls/{args.asset}.json", "w") as fh:
        json.dump(result, fh)
    print(json.dumps({"asset": args.asset, **report}, indent=1))


if __name__ == "__main__":
    main()
