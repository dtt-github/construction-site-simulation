"""Haul-cycle benchmark: the autopilot loads the crawler carrier, which hauls each load to the spoil area,
dumps it and comes back, for N simulated seconds. Reports what a site engineer would look at: loads, load
mass, passes per load, loading time, the carrier's time away and its stages, the excavator's waiting time,
and tonnes per hour delivered; the dozer's passes, blade forces and how flat it left the spread layer, and
how long the carrier waited for it; plus the real-time factor.

    ../phase0/.venv-newton/bin/python bench_haul.py 600                 # dry sand
    SOIL=loam ../phase0/.venv-newton/bin/python bench_haul.py 600
    SOIL=loam CARRIERS=2 ../phase0/.venv-newton/bin/python bench_haul.py 600   # two carriers sharing the road
"""

import json
import os
import sys
import time

import numpy as np

import excavator_server as ex

secs = float(sys.argv[1]) if len(sys.argv) > 1 else 600.0
soil = os.environ.get("SOIL", ex.soils.DEFAULT_SOIL)
n_carriers = int(os.environ.get("CARRIERS", "1"))
sim = ex.ExcavatorSim(soil=soil, carriers=n_carriers)
sim.auto = True
fleet = sim.fleet
loads, start_load, passes_at = [], {}, {}
t0 = time.perf_counter()
prev = {c.index: c.stage for c in fleet}
idle_spot = 0.0
while sim.sim_time < secs:
    sim.step()
    if fleet.at_spot is None:
        idle_spot += sim.frame_dt  # no carrier at the loading spot
    for c in fleet:
        if c.stage != prev[c.index]:
            if prev[c.index] == "loading":  # left with a load
                loads.append({"carrier": c.index, "departed_s": round(sim.sim_time, 1), "load_kg": round(c.payload_kg),
                              "loading_s": round(sim.sim_time - start_load.get(c.index, 0.0), 1), "passes": sim.passes - passes_at.get(c.index, 0)})
            if c.stage == "loading":  # at the spot, empty
                start_load[c.index], passes_at[c.index] = sim.sim_time, sim.passes
            prev[c.index] = c.stage
wall = time.perf_counter() - t0
sim.measure()
trips = fleet.trips
delivered_t = fleet.delivered_kg / 1000.0
out = {
    "soil": soil,
    "carriers": n_carriers,
    "sim_s": secs,
    "realtime_factor": round(secs / wall, 3),
    "loads": loads,
    "trips": trips,
    "delivered_t": round(delivered_t, 2),
    # what the excavator loaded: the loads that left plus the soil in the vessel at the spot at the end
    "loaded_t": round((sum(x["load_kg"] for x in loads) + sim.spot_kg) / 1000, 2),
    "loaded_t_per_hour": round((sum(x["load_kg"] for x in loads) + sim.spot_kg) / 1000 / secs * 3600, 1),
    "mean_departure_interval_s": round(float(np.mean(np.diff([x["departed_s"] for x in loads]))), 1) if len(loads) > 1 else None,
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
    "carrier_wait_at_dump_s": round(sum(c.dozer_wait for c in fleet), 1),
    "carrier_wait_to_leave_s": round(sum(c.depart_wait for c in fleet), 1),
    "carrier_wait_to_load_s": round(sum(c.queue_wait for c in fleet), 1),
    "spot_empty_s": round(idle_spot, 1),
    "pit_left_t": round(float((sim.zones.count_np * sim._pit_cols).sum()) * sim.particle_mass / 1000, 1),
    "pit_done": sim.pit_done,
    "faces": [(f["psi"], f["shift"]) for f in sim.face_log],
    "dozer": {**{k: round(v, 2) for k, v in sim.dozer.counters.items()}, "mean_push_kN": sim.dozer.push_force},
    "spread": sim.dozer.spread_report(sim._surface),
}
print(json.dumps(out))
