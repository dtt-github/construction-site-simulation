"""Real-time check of the moving soil zone: auto dig-swing-load for N simulated seconds.

Reports the real-time factor, frame cost (all frames and dig stages), active particles, and what each
pass delivered to the truck.

    ../phase0/.venv-newton/bin/python bench_zone.py              # 130 s, default margins
    ../phase0/.venv-newton/bin/python bench_zone.py 130 0.8 1.1  # seconds, wake and sleep margins (m)
"""

import json
import sys
import time

import numpy as np

import excavator_server as ex
from soil_zone import SoilZones

secs = float(sys.argv[1]) if len(sys.argv) > 1 else 130.0
if len(sys.argv) > 3:
    SoilZones.WAKE_MARGIN, SoilZones.SLEEP_MARGIN = float(sys.argv[2]), float(sys.argv[3])
sim = ex.ExcavatorSim()
sim.auto = True
ms, stages, active, truck_at_pass = [], [], [], []
t0 = time.perf_counter()
while sim.sim_time < secs:
    t = time.perf_counter()
    passes = sim.passes
    sim.step()
    ms.append((time.perf_counter() - t) * 1000)
    stages.append(sim.stage_name)
    active.append(sim.zones.n_active)
    if sim.passes != passes:
        sim.measure()
        truck_at_pass.append(sim.truck_kg)
wall = time.perf_counter() - t0
sim.measure()
ms, st, ac = np.array(ms[60:]), np.array(stages[60:]), np.array(active[60:])  # skip the compile frames
dig = np.isin(st, ex.ExcavatorSim.DIG_STAGES)
per_pass = np.diff([0.0, *truck_at_pass])
print(json.dumps({
    "wake_margin": SoilZones.WAKE_MARGIN, "sleep_margin": SoilZones.SLEEP_MARGIN,
    "sim_s": round(sim.sim_time, 1), "realtime_factor": round(sim.sim_time / wall, 3),
    "frame_ms_mean": round(float(ms.mean()), 2), "frame_ms_dig_mean": round(float(ms[dig].mean()), 2), "frame_ms_p95": round(float(np.percentile(ms, 95)), 2),
    "active_dig_mean": int(ac[dig].mean()), "active_max": int(ac.max()),
    "passes": len(truck_at_pass), "per_pass_kg": [round(float(v)) for v in per_pass], "mean_pass_kg": round(float(per_pass.mean())) if len(per_pass) else 0,
    "truck_kg": round(sim.truck_kg), "zone": sim.zones.counters, "skipped": sim.skipped,
}))
