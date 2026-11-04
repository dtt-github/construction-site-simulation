"""Haul-cycle benchmark: the autopilot loads the crawler carrier, which hauls each load to the spoil area,
dumps it and comes back, for N simulated seconds. Reports what a site engineer would look at: loads, load
mass, passes per load, loading time, the carrier's time away and its stages, the excavator's waiting time,
and tonnes per hour delivered; the dozer's passes, blade forces and how flat it left the spread layer, and
how long the carrier waited for it; plus the real-time factor.

    ../phase0/.venv-newton/bin/python bench_haul.py 600                 # dry sand
    SOIL=loam ../phase0/.venv-newton/bin/python bench_haul.py 600
"""

import json
import os
import sys
import time

import numpy as np

import excavator_server as ex

secs = float(sys.argv[1]) if len(sys.argv) > 1 else 600.0
soil = os.environ.get("SOIL", ex.soils.DEFAULT_SOIL)
sim = ex.ExcavatorSim(soil=soil)
sim.auto = True
d = sim.dumper
loads, start_load, passes_at = [], 0.0, 0
t0 = time.perf_counter()
prev_stage = d.stage
while sim.sim_time < secs:
    sim.step()
    if d.stage != prev_stage:
        if prev_stage == "loading":  # left with a load
            loads.append({"departed_s": round(sim.sim_time, 1), "load_kg": round(d.payload_kg), "loading_s": round(sim.sim_time - start_load, 1),
                          "passes": sim.passes - passes_at})
        if d.stage == "loading":  # back, empty
            start_load, passes_at = sim.sim_time, sim.passes
        prev_stage = d.stage
wall = time.perf_counter() - t0
sim.measure()
trips = d.trips
delivered_t = d.delivered_kg / 1000.0
out = {
    "soil": soil,
    "sim_s": secs,
    "realtime_factor": round(secs / wall, 3),
    "loads": loads,
    "trips": trips,
    "delivered_t": round(delivered_t, 2),
    "t_per_hour": round(delivered_t / secs * 3600, 1),
    "mean_load_kg": round(float(np.mean([x["load_kg"] for x in loads])), 0) if loads else None,
    "mean_loading_s": round(float(np.mean([x["loading_s"] for x in loads])), 1) if loads else None,
    "carry_back_kg": [t.get("carry_back_kg") for t in trips],
    "mean_away_s": round(float(np.mean([x["away_s"] for x in trips])), 1) if trips else None,
    "excavator_wait_s": round(sim.truck_wait, 1),
    "passes": sim.passes,
    "spoil_settled_t": round(sim.zones.dormant_in("spoil") * sim.particle_mass / 1000, 2),
    "lost_particles": sim.zones.counters["lost"],
    "pool_short": sim.zones.counters["pool_short"],
    "skipped": sim.skipped,
    "carrier_wait_for_dozer_s": round(d.dozer_wait, 1),
    "dozer": {**{k: round(v, 2) for k, v in sim.dozer.counters.items()}, "mean_push_kN": sim.dozer.push_force},
    "spread": sim.dozer.spread_report(sim._surface),
}
print(json.dumps(out))
