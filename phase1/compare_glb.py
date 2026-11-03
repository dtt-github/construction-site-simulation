"""Compare the USD physics sidecars against the physics stored in the codepawl GLBs."""

import json
import sys

from read_glb_physics import summary

rows = []
for name in open("data/site_assets.txt").read().split():
    usd = json.load(open(f"sidecars/{name}.json"))
    glb = summary(f"glb/{name}.glb")
    um = usd["summary"]["total_mass_kg"]
    gm = glb["total_mass_kg"]
    mats = list(usd["materials"].values())
    dens = sorted({m.get("density") for m in mats if m.get("density")})
    fr_u = sorted({round(m["dynamic_friction"], 3) for m in mats if m.get("dynamic_friction") is not None})
    fr_g = sorted({round(m.get("dynamicFriction", -1), 3) for m in glb["materials"]})
    rows.append({
        "asset": name,
        "bodies usd/glb": f'{usd["summary"]["bodies"]}/{len(glb["bodies"])}',
        "joints usd/glb": f'{usd["summary"]["joints"]}/{glb["joints"]}',
        "mass usd kg": um,
        "mass glb kg": gm,
        "mass ratio": round(gm / um, 3) if um else None,
        "friction usd": fr_u,
        "friction glb": fr_g,
        "densities usd": dens,
        "glb hulls": glb["hulls"],
        "collider approx usd": usd["summary"]["approximations"],
    })
json.dump(rows, open("data/glb_vs_usd.json", "w"), indent=1)
w = sys.stdout.write
w("| asset | bodies usd/glb | joints usd/glb | mass usd kg | mass glb kg | ratio | friction usd | friction glb | densities |\n|---|---|---|---|---|---|---|---|---|\n")
for r in rows:
    w(f'| {r["asset"]} | {r["bodies usd/glb"]} | {r["joints usd/glb"]} | {r["mass usd kg"]} | {r["mass glb kg"]} | {r["mass ratio"]} | {r["friction usd"]} | {r["friction glb"]} | {r["densities usd"]} |\n')
