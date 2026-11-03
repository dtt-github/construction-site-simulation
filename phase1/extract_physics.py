"""Extract UsdPhysics data from a SimReady asset into a JSON sidecar.

Usage: extract_physics.py <asset_root.usd> <out.json> [--variant PhysX|Newton|MuJoCo]

Reads the asset's physics variant, then records for every rigid body its mass,
center of mass, inertia and bounds; for every collider its body, approximation
and physics material (friction, restitution, density); and every joint with its
bodies, frames, axis, limits and drives. Units are SI (metres, kilograms);
positions are in the asset's own frame and up axis, which is recorded.
"""

import argparse
import json

import numpy as np
from scipy.spatial import ConvexHull

from pxr import Gf, Usd, UsdGeom, UsdPhysics, UsdShade

JOINT_TYPES = [
    (UsdPhysics.RevoluteJoint, "revolute"),
    (UsdPhysics.PrismaticJoint, "prismatic"),
    (UsdPhysics.SphericalJoint, "spherical"),
    (UsdPhysics.DistanceJoint, "distance"),
    (UsdPhysics.FixedJoint, "fixed"),
]


def vec(v):
    return None if v is None else [round(float(x), 6) for x in v]


def quat(q):
    if q is None:
        return None
    im = q.GetImaginary()
    return [round(float(q.GetReal()), 6), round(float(im[0]), 6), round(float(im[1]), 6), round(float(im[2]), 6)]


def get(attr):
    return attr.Get() if attr and attr.HasAuthoredValue() else None


def collider_volume(prim, xf_cache):
    """World-scaled convex hull volume of a mesh collider (m^3); boxes/spheres analytic."""
    try:
        world = xf_cache.GetLocalToWorldTransform(prim)
        if prim.IsA(UsdGeom.Mesh):
            pts = UsdGeom.Mesh(prim).GetPointsAttr().Get()
            if not pts or len(pts) < 4:
                return None
            arr = np.array([world.Transform(Gf.Vec3d(*p)) for p in pts])
            mesh = UsdGeom.Mesh(prim)
            counts = mesh.GetFaceVertexCountsAttr().Get()
            idx = mesh.GetFaceVertexIndicesAttr().Get()
            hull = float(ConvexHull(arr).volume)
            if not counts or not idx:
                return {"mesh": None, "hull": round(hull, 8)}
            # fan-triangulate faces and sum signed tetrahedron volumes (exact for closed meshes)
            vol = 0.0
            k = 0
            for n in counts:
                a = arr[idx[k]]
                for t in range(1, n - 1):
                    b, c = arr[idx[k + t]], arr[idx[k + t + 1]]
                    vol += float(np.dot(a, np.cross(b, c))) / 6.0
                k += n
            return {"mesh": round(abs(vol), 8), "hull": round(hull, 8)}
        if prim.IsA(UsdGeom.Cube):
            size = UsdGeom.Cube(prim).GetSizeAttr().Get()
            sc = [Gf.Vec3d(*world.GetRow3(i)).GetLength() for i in range(3)]
            v = round(float(size**3 * sc[0] * sc[1] * sc[2]), 8)
            return {"mesh": v, "hull": v}
    except Exception:  # noqa: BLE001
        return None
    return None


def owning_body(prim):
    p = prim
    while p and p.IsValid():
        if p.HasAPI(UsdPhysics.RigidBodyAPI):
            return p
        p = p.GetParent()
    return None


def physics_material(prim):
    binding = UsdShade.MaterialBindingAPI(prim)
    mat, _ = binding.ComputeBoundMaterial("physics")
    if not mat:
        return None
    mp = mat.GetPrim()
    if not mp.HasAPI(UsdPhysics.MaterialAPI):
        return {"path": str(mp.GetPath())}
    api = UsdPhysics.MaterialAPI(mp)
    return {
        "path": str(mp.GetPath()),
        "static_friction": get(api.GetStaticFrictionAttr()),
        "dynamic_friction": get(api.GetDynamicFrictionAttr()),
        "restitution": get(api.GetRestitutionAttr()),
        "density": get(api.GetDensityAttr()),
    }


def semantic_labels(prim):
    labels = {}
    for attr in prim.GetAttributes():
        name = attr.GetName()
        if name.startswith("semantic") or name.startswith("semantics:"):
            val = attr.Get()
            if val is not None:
                labels[name] = val if isinstance(val, (str, int, float)) else str(val)
    return labels


def extract(path, variant):
    stage = Usd.Stage.Open(path)
    default = stage.GetDefaultPrim() or stage.GetPseudoRoot()
    if variant:
        for prim in [default, *default.GetChildren()]:
            vs = prim.GetVariantSets()
            for engine in ("PhysX", "Newton", "MuJoCo"):
                if vs.HasVariantSet(engine):
                    vs.GetVariantSet(engine).SetVariantSelection("Enabled" if engine == variant else "Disabled")

    bbox = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_, UsdGeom.Tokens.render])
    xf_cache = UsdGeom.XformCache()

    out = {
        "source": path,
        "variant": variant,
        "up_axis": UsdGeom.GetStageUpAxis(stage),
        "meters_per_unit": UsdGeom.GetStageMetersPerUnit(stage),
        "kilograms_per_unit": UsdPhysics.GetStageKilogramsPerUnit(stage),
        "default_prim": str(default.GetPath()),
        "labels": semantic_labels(default),
        "articulation_roots": [],
        "bodies": [],
        "colliders": [],
        "joints": [],
        "materials": {},
    }

    for prim in stage.Traverse(Usd.TraverseInstanceProxies(Usd.PrimDefaultPredicate)):
        if prim.HasAPI(UsdPhysics.ArticulationRootAPI):
            out["articulation_roots"].append(str(prim.GetPath()))

        if prim.HasAPI(UsdPhysics.RigidBodyAPI):
            rb = UsdPhysics.RigidBodyAPI(prim)
            mass_api = UsdPhysics.MassAPI(prim) if prim.HasAPI(UsdPhysics.MassAPI) else None
            world = xf_cache.GetLocalToWorldTransform(prim)
            box = bbox.ComputeWorldBound(prim).ComputeAlignedRange()
            out["bodies"].append(
                {
                    "path": str(prim.GetPath()),
                    "kinematic": bool(get(rb.GetKinematicEnabledAttr()) or False),
                    "enabled": bool(get(rb.GetRigidBodyEnabledAttr()) if get(rb.GetRigidBodyEnabledAttr()) is not None else True),
                    "mass": get(mass_api.GetMassAttr()) if mass_api else None,
                    "density": get(mass_api.GetDensityAttr()) if mass_api else None,
                    "center_of_mass": vec(get(mass_api.GetCenterOfMassAttr())) if mass_api else None,
                    "diagonal_inertia": vec(get(mass_api.GetDiagonalInertiaAttr())) if mass_api else None,
                    "position": vec(world.ExtractTranslation()),
                    "orientation_wxyz": quat(world.ExtractRotationQuat()),
                    "bounds_min": None if box.IsEmpty() else vec(box.GetMin()),
                    "bounds_max": None if box.IsEmpty() else vec(box.GetMax()),
                    "labels": semantic_labels(prim),
                }
            )

        if prim.HasAPI(UsdPhysics.CollisionAPI):
            col = UsdPhysics.CollisionAPI(prim)
            approx = None
            if prim.HasAPI(UsdPhysics.MeshCollisionAPI):
                approx = get(UsdPhysics.MeshCollisionAPI(prim).GetApproximationAttr())
            body = owning_body(prim)
            mat = physics_material(prim) or (physics_material(body) if body else None)
            vol = collider_volume(prim, xf_cache)
            if mat:
                out["materials"][mat["path"]] = mat
            out["colliders"].append(
                {
                    "path": str(prim.GetPath()),
                    "type": prim.GetTypeName(),
                    "enabled": bool(get(col.GetCollisionEnabledAttr()) if get(col.GetCollisionEnabledAttr()) is not None else True),
                    "approximation": approx,
                    "body": str(body.GetPath()) if body else None,
                    "material": mat["path"] if mat else None,
                    "density": get(UsdPhysics.MassAPI(prim).GetDensityAttr()) if prim.HasAPI(UsdPhysics.MassAPI) else None,
                    "mesh_volume_m3": vol["mesh"] if vol else None,
                    "hull_volume_m3": vol["hull"] if vol else None,
                }
            )

        for cls, name in JOINT_TYPES:
            if prim.IsA(cls):
                j = cls(prim)
                b0 = j.GetBody0Rel().GetTargets()
                b1 = j.GetBody1Rel().GetTargets()
                rec = {
                    "path": str(prim.GetPath()),
                    "type": name,
                    "body0": str(b0[0]) if b0 else None,
                    "body1": str(b1[0]) if b1 else None,
                    "local_pos0": vec(get(j.GetLocalPos0Attr())),
                    "local_rot0_wxyz": quat(get(j.GetLocalRot0Attr())),
                    "local_pos1": vec(get(j.GetLocalPos1Attr())),
                    "local_rot1_wxyz": quat(get(j.GetLocalRot1Attr())),
                    "enabled": bool(get(j.GetJointEnabledAttr()) if get(j.GetJointEnabledAttr()) is not None else True),
                    "exclude_from_articulation": bool(get(j.GetExcludeFromArticulationAttr()) or False),
                }
                if name in ("revolute", "prismatic"):
                    rec["axis"] = get(j.GetAxisAttr())
                    rec["lower_limit"] = get(j.GetLowerLimitAttr())
                    rec["upper_limit"] = get(j.GetUpperLimitAttr())
                    rec["limit_unit"] = "degrees" if name == "revolute" else "meters"
                    drive_name = "angular" if name == "revolute" else "linear"
                    drive = UsdPhysics.DriveAPI.Get(prim, drive_name)
                    if drive:
                        rec["drive"] = {
                            "type": get(drive.GetTypeAttr()),
                            "stiffness": get(drive.GetStiffnessAttr()),
                            "damping": get(drive.GetDampingAttr()),
                            "max_force": get(drive.GetMaxForceAttr()),
                            "target_position": get(drive.GetTargetPositionAttr()),
                            "target_velocity": get(drive.GetTargetVelocityAttr()),
                        }
                out["joints"].append(rec)
                break

    # Mass as a physics engine would compute it: explicit body mass wins, otherwise
    # sum of collider volume x density (collider MassAPI density, then body density,
    # then physics material density, then the PhysX default of 1000 kg/m^3).
    total = 0.0
    for b in out["bodies"]:
        if b["mass"]:
            b["effective_mass"] = b["mass"]
            b["mass_source"] = "explicit"
        else:
            m = 0.0
            for c in out["colliders"]:
                v = c["mesh_volume_m3"] or c["hull_volume_m3"]
                if c["body"] != b["path"] or not v:
                    continue
                mat = out["materials"].get(c["material"] or "", {})
                dens = c["density"] or b["density"] or mat.get("density") or 1000.0
                m += v * dens
            b["effective_mass"] = round(m, 4)
            b["mass_source"] = "mesh_volume_x_density"
        total += b["effective_mass"]
    out["summary"] = {
        "bodies": len(out["bodies"]),
        "colliders": len(out["colliders"]),
        "joints": len(out["joints"]),
        "drives": sum(1 for j in out["joints"] if "drive" in j),
        "total_mass_kg": round(total, 3),
        "bodies_without_explicit_mass": sum(1 for b in out["bodies"] if not b["mass"]),
        "approximations": sorted({c["approximation"] or "none" for c in out["colliders"]}),
    }
    return out


def main():
    p = argparse.ArgumentParser()
    p.add_argument("usd")
    p.add_argument("out")
    p.add_argument("--variant", default="PhysX", choices=["PhysX", "Newton", "MuJoCo"])
    args = p.parse_args()
    data = extract(args.usd, args.variant)
    with open(args.out, "w") as fh:
        json.dump(data, fh, indent=1, default=str)
    print(json.dumps({"asset": args.usd.split("/")[-1], **data["summary"]}))


if __name__ == "__main__":
    main()
