"""Haul-cycle benchmark: the autopilot loads the crawler carrier, which hauls each load to the spoil area,
dumps it and comes back, for N simulated seconds. Reports what a site engineer would look at: loads, load
mass, passes per load, loading time, the carrier's time away and its stages, the excavator's waiting time,
and tonnes per hour delivered; the dozer's passes, blade forces and how flat it left the spread layer, and
how long the carrier waited for it; plus the real-time factor. Each pass is logged too (`pass_log`): its strip,
the most the bucket held while digging, what it held at the swing, what it carried to the carrier and what it
left after the dump, so one run shows where a soil's loads go.

A quick check stops after a number of passes instead (PASSES=25 is about 500 s): enough to compare two
versions of the autopilot pass by pass, in a quarter of the time. Progress goes to stderr every 100 s.

    ../phase0/.venv-newton/bin/python bench_haul.py 600                 # dry sand
    SOIL=loam ../phase0/.venv-newton/bin/python bench_haul.py 600
    SOIL=loam CARRIERS=2 ../phase0/.venv-newton/bin/python bench_haul.py 600   # two carriers sharing the road
    SOIL=loam CARRIERS=3 HAUL_M=150 ../phase0/.venv-newton/bin/python bench_haul.py 1500   # 150 m more road each way
    SOIL=clay PASSES=25 ../phase0/.venv-newton/bin/python bench_haul.py 1500   # quick check: the first 25 passes
"""

import json
import os
import sys
import time

import numpy as np

import dumper as dumper_mod

import excavator_server as ex

secs = float(sys.argv[1]) if len(sys.argv) > 1 else 600.0
soil = os.environ.get("SOIL", ex.soils.DEFAULT_SOIL)
n_carriers = int(os.environ.get("CARRIERS", "1"))
haul_m = float(os.environ.get("HAUL_M", "0"))  # the off-screen two-lane road, each way (dumper.py)
relocate_m = float(os.environ.get("RELOCATE_M", "0"))  # move on along the bank when the face is dug out
max_passes = int(os.environ.get("PASSES", "0")) or None
roller = os.environ.get("ROLLER", "1") != "0"
sim = ex.ExcavatorSim(soil=soil, carriers=n_carriers, haul_m=haul_m, relocate_m=relocate_m, roller=roller)
sim.auto = True
fleet = sim.fleet
loads, start_load, passes_at = [], {}, {}
t0 = time.perf_counter()
prev = {c.index: c.stage for c in fleet}
idle_spot = 0.0
pass_log, cur, prev_stage, next_report = [], None, None, 100.0
DIG = ("penetrate", "drag", "drag-curl", "curl")
while sim.sim_time < secs and (max_passes is None or sim.passes < max_passes):
    sim.step()
    if sim.stage_name != prev_stage:
        st, fill = sim.stage_name, round(sim._bucket_fill())
        if st == "approach" and cur is not None:
            pass_log.append(cur)
            cur = None
        if cur is None:
            n = sim.passes + (st == "approach")  # the count goes up as the approach ends
            f = sim._faces.get(n) if sim.adaptive_face else None
            cur = {"pass": n, "t": round(sim.sim_time, 1), "face": None if f is None else [f["psi"], f["shift"], round(f["avail_m3"], 2)],
                   "dig_max_kg": 0, "early_curl": False, "skips": len(sim.skipped)}
        if prev_stage in DIG:
            cur["dig_max_kg"] = max(cur["dig_max_kg"], fill)
        if st == "curl" and prev_stage not in ("drag-curl", "curl"):
            cur["early_curl"] = True  # curled before the planned curl point (the load was there)
        if st == "swing to truck":
            cur["at_swing_kg"] = fill
        elif st == "dump":
            cur["carried_kg"] = fill
        elif st == "swing back":
            cur["left_kg"] = fill
        prev_stage = st
    if cur is not None and prev_stage in DIG:
        cur["dig_max_kg"] = max(cur["dig_max_kg"], round(sim.auto_fill))
    if sim.sim_time >= next_report:
        print(f"{sim.sim_time:.0f} s: {sim.passes} passes, {sum(x['load_kg'] for x in loads) / 1000:.1f} t in {len(loads)} loads", file=sys.stderr, flush=True)
        next_report += 100.0
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
if cur is not None and "left_kg" in cur:
    pass_log.append(cur)
for x, nxt in zip(pass_log, pass_log[1:] + [{"skips": len(sim.skipped)}]):
    x["skips"] = nxt["skips"] - x["skips"]  # stages skipped in this pass
secs = sim.sim_time  # a quick check stops early
done = [x for x in pass_log if "carried_kg" in x]
trips = fleet.trips
delivered_t = fleet.delivered_kg / 1000.0
out = {
    "soil": soil,
    "carriers": n_carriers,
    "haul_m": haul_m,
    "relocate_m": relocate_m,
    "relocations": sim.relocations,
    "sim_s": round(secs, 1),
    "stopped_after_passes": max_passes,
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
    "curl_gain_kg": round(sim.curl_gain),
    "mean_carried_kg": round(float(np.mean([x["carried_kg"] for x in done]))) if done else None,
    "mean_dumped_kg": round(float(np.mean([x["carried_kg"] - x["left_kg"] for x in done]))) if done else None,
    "thin_passes": sum(1 for x in done if x["at_swing_kg"] < 0.5 * sim.full_kg),
    "pass_log": pass_log,
    "dozer": {**{k: round(v, 2) for k, v in sim.dozer.counters.items()}, "mean_push_kN": sim.dozer.push_force},
    "spread": sim.dozer.spread_report(sim._surface),
    "tracks": {"dozer_sinkage_cm": round(sim.dozer.sinkage * 100, 2), "dozer_pull_kN": round(sim.dozer.f_traction / 1000, 1),
               "dozer_slip": round(sim.dozer.counters["slip_m"] / max(sim.dozer.counters["slip_m"] + sim.dozer.counters["push_m"], 1e-9), 3),
               "carrier_sinkage_cm": [round(sim.carrier_track.sinkage(m) * 100, 2) for m in (dumper_mod.MASS_KG, dumper_mod.MASS_KG + 11000)],
               **sim.track_counters},
    "compaction": {r: sim.zones.compaction(r) for r in ("spoil", "road")},
    # per fill cell (under the roller's lanes), and whether the dozer is spreading it at the end
    "cell_compaction": [{**sim.zones.compaction("spoil", sim.roller.cell_points(k)), "spreading": k == sim.dozer.cell}
                        for k in range(len(sim.roller.cell_lanes))] if sim.use_roller else None,
    "roller": {**{k: round(v, 2) for k, v in sim.roller.counters.items()}, **sim.roller_counters} if sim.use_roller else None,
}
print(json.dumps(out))
