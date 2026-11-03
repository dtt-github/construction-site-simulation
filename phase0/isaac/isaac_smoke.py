"""Isaac Sim 6.2 headless smoke test on the DGX Spark.

1. PhysX GPU rigid-body throughput: 4,096 boxes dropped on a ground plane.
2. Load a SimReady asset (forklift) with its PhysX payload and check it settles.
3. RTX render of one camera frame to PNG.
Writes /work/logs/isaac_smoke.json.
"""

import json
import os
import time
import traceback

from isaacsim import SimulationApp

app = SimulationApp({"headless": True, "width": 1280, "height": 720})

import carb  # noqa: E402
import numpy as np  # noqa: E402
import omni.usd  # noqa: E402
from omni.physx import get_physx_simulation_interface  # noqa: E402
from pxr import Gf, PhysxSchema, Sdf, UsdGeom, UsdLux, UsdPhysics  # noqa: E402

OUT = "/work/isaac/out"
os.makedirs(OUT, exist_ok=True)
result = {}


def new_stage():
    omni.usd.get_context().new_stage()
    stage = omni.usd.get_context().get_stage()
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    scene = UsdPhysics.Scene.Define(stage, "/physicsScene")
    scene.CreateGravityDirectionAttr(Gf.Vec3f(0, 0, -1))
    scene.CreateGravityMagnitudeAttr(9.81)
    px = PhysxSchema.PhysxSceneAPI.Apply(scene.GetPrim())
    px.CreateEnableGPUDynamicsAttr(True)
    px.CreateBroadphaseTypeAttr("GPU")
    ground = UsdGeom.Cube.Define(stage, "/ground")
    ground.CreateSizeAttr(1.0)
    ground.AddScaleOp().Set(Gf.Vec3f(200, 200, 1))
    ground.AddTranslateOp().Set(Gf.Vec3f(0, 0, -0.5))
    UsdPhysics.CollisionAPI.Apply(ground.GetPrim())
    return stage


def step(n, dt=1.0 / 60.0):
    sim = get_physx_simulation_interface()
    for _ in range(n):
        sim.simulate(dt, 0.0)
        sim.fetch_results()


# 1. Rigid-body throughput
def rigid_test(gpu, update_usd):
    carb.settings.get_settings().set("/physics/updateToUsd", update_usd)
    stage = new_stage()
    PhysxSchema.PhysxSceneAPI.Get(stage, "/physicsScene").GetEnableGPUDynamicsAttr().Set(gpu)
    PhysxSchema.PhysxSceneAPI.Get(stage, "/physicsScene").GetBroadphaseTypeAttr().Set("GPU" if gpu else "MBP")
    n_side = 16
    count = 0
    for i in range(n_side):
        for j in range(n_side):
            for k in range(16):
                path = f"/boxes/b_{i}_{j}_{k}"
                cube = UsdGeom.Cube.Define(stage, path)
                cube.CreateSizeAttr(0.4)
                cube.AddTranslateOp().Set(Gf.Vec3f(i * 0.5 - 4, j * 0.5 - 4, 0.5 + k * 0.45))
                UsdPhysics.CollisionAPI.Apply(cube.GetPrim())
                UsdPhysics.RigidBodyAPI.Apply(cube.GetPrim())
                UsdPhysics.MassAPI.Apply(cube.GetPrim()).CreateMassAttr(10.0)
                count += 1
    app.update()
    return stage, count


def run_rigid(gpu, update_usd):
    stage, count = rigid_test(gpu, update_usd)
    step(10)
    t0 = time.perf_counter()
    step(300)
    dt_wall = time.perf_counter() - t0
    xf = UsdGeom.Xformable(stage.GetPrimAtPath("/boxes/b_8_8_15"))
    z = xf.ComputeLocalToWorldTransform(0).ExtractTranslation()[2]
    carb.settings.get_settings().set("/physics/updateToUsd", True)
    app.update()
    z = UsdGeom.Xformable(stage.GetPrimAtPath("/boxes/b_8_8_15")).ComputeLocalToWorldTransform(0).ExtractTranslation()[2]
    return {
        "gpu_dynamics": gpu,
        "write_back_to_usd": update_usd,
        "bodies": count,
        "steps": 300,
        "ms_per_step": round(dt_wall / 300 * 1000, 3),
        "realtime_factor_at_60Hz": round((300 / 60) / dt_wall, 2),
        "top_box_final_z_m": round(float(z), 3),
    }


result["rigid"] = []
for gpu, upd in [(True, True), (True, False), (False, False)]:
    try:
        result["rigid"].append(run_rigid(gpu, upd))
    except Exception:
        result["rigid"].append({"gpu_dynamics": gpu, "error": traceback.format_exc()[-1500:]})
print(json.dumps(result, indent=2), flush=True)

# 2. SimReady forklift
try:
    stage = new_stage()
    prim = stage.DefinePrim("/forklift", "Xform")
    prim.GetReferences().AddReference("/assets/vehicle_forklift_orange_a01/simready_usd/sm_vehicle_forklift_orange_a01_01.usd")
    UsdGeom.Xformable(prim).AddTranslateOp().Set(Gf.Vec3f(0, 0, 0.3))
    app.update()
    bodies = [p for p in stage.Traverse() if p.HasAPI(UsdPhysics.RigidBodyAPI)]
    joints = [p for p in stage.Traverse() if p.IsA(UsdPhysics.Joint)]
    variants = {}
    for p in stage.Traverse():
        vs = p.GetVariantSets()
        for name in vs.GetNames():
            variants[name] = vs.GetVariantSet(name).GetVariantSelection()
        if variants:
            break
    step(5)
    first = bodies[0] if bodies else None
    z0 = UsdGeom.Xformable(first).ComputeLocalToWorldTransform(0).ExtractTranslation() if first else None
    step(300)
    z1 = UsdGeom.Xformable(first).ComputeLocalToWorldTransform(0).ExtractTranslation() if first else None
    result["forklift"] = {
        "rigid_bodies": len(bodies),
        "joints": len(joints),
        "variant_selection": variants,
        "first_body": str(first.GetPath()) if first else None,
        "first_body_z_start_m": round(float(z0[2]), 3) if z0 is not None else None,
        "first_body_z_after_5s_m": round(float(z1[2]), 3) if z1 is not None else None,
        "ok": bool(bodies) and z1 is not None and abs(float(z1[2])) < 5.0,
    }
except Exception:
    result["forklift"] = {"ok": False, "error": traceback.format_exc()[-1500:]}
print(json.dumps(result.get("forklift"), indent=2), flush=True)

# 3. RTX render
try:
    import omni.replicator.core as rep
    from PIL import Image

    stage = omni.usd.get_context().get_stage()
    light = UsdLux.DomeLight.Define(stage, "/dome")
    light.CreateIntensityAttr(1000)
    cam = UsdGeom.Camera.Define(stage, "/cam")
    UsdGeom.XformCommonAPI(cam).SetTranslate(Gf.Vec3d(5, -5, 3))
    UsdGeom.XformCommonAPI(cam).SetRotate(Gf.Vec3f(65, 0, 45))
    rp = rep.create.render_product("/cam", (1280, 720))
    rgb = rep.AnnotatorRegistry.get_annotator("rgb")
    rgb.attach([rp])
    t0 = time.perf_counter()
    for _ in range(60):
        app.update()
    rep.orchestrator.step(rt_subframes=8)
    data = rgb.get_data()
    for _ in range(10):
        if np.asarray(data).ndim == 3:
            break
        app.update()
        rep.orchestrator.step()
        data = rgb.get_data()
    img = np.asarray(data)[:, :, :3]
    Image.fromarray(img).save(f"{OUT}/isaac_forklift.png")
    result["render"] = {
        "ok": img.size > 0 and float(img.mean()) > 1.0,
        "shape": list(img.shape),
        "mean_pixel": round(float(img.mean()), 1),
        "seconds_first_frames": round(time.perf_counter() - t0, 1),
        "png": "isaac/out/isaac_forklift.png",
    }
except Exception:
    result["render"] = {"ok": False, "error": traceback.format_exc()[-1500:]}
print(json.dumps(result.get("render"), indent=2), flush=True)

with open(f"{OUT}/isaac_smoke.json", "w") as fh:
    json.dump(result, fh, indent=2)
app.close()
