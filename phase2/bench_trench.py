"""Trench test on the 2 m sand fill: the auto cycle digs one lane, deeper every pass.

Reports per pass the truck delivery, the dig level and the frame cost, and at the end the trench cross
section (sand surface across the lane, from the streamed heightmap): depth, width and side slope.

    ../phase0/.venv-newton/bin/python bench_trench.py              # 8 passes
    ../phase0/.venv-newton/bin/python bench_trench.py 10 --out logs/trench.json
"""

import argparse
import json
import math
import time

import numpy as np

import excavator_server as ex

ap = argparse.ArgumentParser()
ap.add_argument("passes", type=int, nargs="?", default=8)
ap.add_argument("--out", default=None)
ap.add_argument("--pool", type=int, default=ex.POOL)
ap.add_argument("--soil", choices=list(ex.soils.SOILS), default=ex.soils.DEFAULT_SOIL)
args = ap.parse_args()

sim = ex.ExcavatorSim(pool=args.pool, soil=args.soil)
sim.set_trench()
sim.auto = True
rows, ms, peak, truck = [], [], 0, 0.0
t0 = time.perf_counter()
pass_t0, pass_ms = sim.sim_time, []
while sim.passes < args.passes and sim.sim_time < 60.0 * args.passes:
    passes = sim.passes
    t = time.perf_counter()
    sim.step()
    pass_ms.append((time.perf_counter() - t) * 1000)
    peak = max(peak, sim.zones.n_active)
    if sim.passes != passes:
        sim.measure()
        dig_z = sim._stage_z(sim.CYCLE[2])  # drag pin height used this pass
        rows.append({"pass": passes + 1, "s": round(sim.sim_time - pass_t0, 1), "truck_kg": round(sim.truck_kg - truck), "drag_pin_z": round(dig_z, 2),
                     "frame_ms": round(float(np.mean(pass_ms[10:])), 1), "active_peak": peak, "skipped": len(sim.skipped), "ease_m": round(sim.ease, 2), "early_curls": sim.early_curls})
        print(json.dumps(rows[-1]), flush=True)
        truck, pass_t0, pass_ms, peak = sim.truck_kg, sim.sim_time, [], 0
wall = time.perf_counter() - t0
sim.measure()
# cross sections across the lane (constant x), from the pit heightmap
hm = sim.hm_np.reshape(ex.HM_NY, ex.HM_NX)
ys = ex.HM_Y0 + (np.arange(ex.HM_NY) + 0.5) * ex.HM_CELL
sections = []
for x in (0.0, 1.0, 2.0):
    ix = int((x - ex.HM_X0) / ex.HM_CELL)
    prof = hm[:, ix]
    lane = np.abs(ys) < 2.4
    bottom = float(prof[lane].min())
    depth = ex.BED_TOP - bottom
    half = ex.BED_TOP - depth / 2
    wide = ys[lane][prof[lane] < half]
    width = float(wide.max() - wide.min() + ex.HM_CELL) if len(wide) else 0.0
    # side slope between a quarter and three quarters of the depth, on both sides
    angles = []
    for side in (1, -1):
        sel = lane & (np.sign(ys) == side)
        y_sel, z_sel = ys[sel], prof[sel]
        band = (z_sel > bottom + depth / 4) & (z_sel < ex.BED_TOP - depth / 4)
        if band.sum() >= 2:
            k = np.polyfit(y_sel[band], z_sel[band], 1)[0]
            angles.append(round(math.degrees(math.atan(abs(k))), 1))
    sections.append({"x": x, "depth_m": round(depth, 2), "width_at_half_depth_m": round(width, 2), "side_slope_deg": angles,
                     "profile": [[round(float(y), 2), round(float(z), 2)] for y, z in zip(ys[lane][::2], prof[lane][::2])]})
res = {"pool": args.pool, "passes": rows, "sim_s": round(sim.sim_time, 1), "realtime_factor": round(sim.sim_time / wall, 3), "truck_kg": round(sim.truck_kg),
       "zone": sim.zones.counters, "skipped": sim.skipped, "sections": sections}
print(json.dumps({k: v for k, v in res.items() if k != "sections"}))
for s_ in sections:
    print(json.dumps({k: v for k, v in s_.items() if k != "profile"}))
if args.out:
    with open(args.out, "w") as fh:
        json.dump(res, fh, indent=1)
