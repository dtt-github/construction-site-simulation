"""Payload experiments: run one dig-swing-dump pass per cycle variant on a fresh sand bed and measure
how much sand is in the bucket at each stage and how much reaches the truck.

Usage: python tune_payload.py [variant ...]   (default: all)   -> logs/payload_<variant>.json + summary table
"""

import json
import math
import sys
import time

import numpy as np
import warp as wp

wp.config.quiet = True
import excavator_server as ex  # noqa: E402

ITERATIONS = int(__import__("os").environ.get("MPM_ITER", "60"))

TAIL = [
    ("dump", -90, 5.4, 6.4, -40),
    ("raise", -90, 5.4, 7.0, -110),
    ("swing back", 0, 6.0, 6.6, -95),
    ("lower", 0, 6.0, 2.6, -95),
]

VARIANTS = {
    # the cycle before this round (shorter, shallower drag; bucket relaxes to -160 deg for the carry)
    "baseline": [
        ("approach", 0, 6.0, 2.6, -95),
        ("penetrate", 0, 7.9, 1.5, -60),
        ("drag", 0, 5.6, 1.1, -95),
        ("curl", 0, 5.2, 1.6, -175),
        ("lift", 0, 5.0, 6.6, -160),
        ("swing to truck", -90, 5.4, 6.6, -160),
        *TAIL,
    ],
    # longer and deeper drag, curl while still dragging, keep full curl through the carry
    "long_drag_curl": [
        ("approach", 0, 6.2, 2.6, -95),
        ("penetrate", 0, 8.3, 1.3, -55),
        ("drag", 0, 5.7, 1.0, -100),
        ("drag-curl", 0, 4.9, 1.1, -140),
        ("curl", 0, 4.8, 1.5, -180),
        ("lift", 0, 5.0, 6.6, -180),
        ("swing to truck", -90, 5.4, 6.6, -180),
        *TAIL,
    ],
    # same, plus a gentler loaded lift and swing (half lever)
    "long_drag_curl_gentle": [
        ("approach", 0, 6.2, 2.6, -95),
        ("penetrate", 0, 8.3, 1.3, -55),
        ("drag", 0, 5.7, 1.0, -100),
        ("drag-curl", 0, 4.9, 1.1, -140),
        ("curl", 0, 4.8, 1.5, -180),
        ("lift", 0, 5.0, 6.6, -180, 0.6),
        ("swing to truck", -90, 5.4, 6.6, -180, 0.5),
        *TAIL,
    ],
    # baseline path, but full curl through the carry and a gentle loaded swing (isolates the swing effect)
    "baseline_fullcurl_gentle": [
        ("approach", 0, 6.0, 2.6, -95),
        ("penetrate", 0, 7.9, 1.5, -60),
        ("drag", 0, 5.6, 1.1, -95),
        ("curl", 0, 5.2, 1.6, -180),
        ("lift", 0, 5.0, 6.6, -180, 0.6),
        ("swing to truck", -90, 5.4, 6.6, -180, 0.5),
        *TAIL,
    ],
    # baseline drag-then-curl, starting from the long drag's farther reach (longer cut, no curl while dragging)
    "baseline_long": [
        ("approach", 0, 6.2, 2.6, -95),
        ("penetrate", 0, 8.3, 1.3, -55),
        ("drag", 0, 5.6, 1.1, -95),
        ("curl", 0, 5.2, 1.6, -175),
        ("lift", 0, 5.0, 6.6, -160),
        ("swing to truck", -90, 5.4, 6.6, -160),
        *TAIL,
    ],
    # cut deeper and curl earlier while still dragging low, so the bucket fills toward the top
    "deep_drag_early_curl": [
        ("approach", 0, 6.2, 2.6, -95),
        ("penetrate", 0, 8.3, 1.2, -55),
        ("drag", 0, 6.6, 0.95, -105),
        ("drag-curl", 0, 4.9, 1.0, -155),
        ("curl", 0, 4.8, 1.5, -180),
        ("lift", 0, 5.0, 6.6, -180),
        ("swing to truck", -90, 5.4, 6.6, -180),
        *TAIL,
    ],
    # same, curling while dragging at a slower rate (more cutting per degree of curl)
    "deep_drag_early_slow_curl": [
        ("approach", 0, 6.2, 2.6, -95),
        ("penetrate", 0, 8.3, 1.2, -55),
        ("drag", 0, 6.6, 0.95, -105),
        ("drag-curl", 0, 4.9, 1.0, -155, 0.6),
        ("curl", 0, 4.8, 1.5, -180),
        ("lift", 0, 5.0, 6.6, -180),
        ("swing to truck", -90, 5.4, 6.6, -180),
        *TAIL,
    ],
    # gentle, and a slower curl so the bucket fills before it closes
    "long_drag_slow_curl_gentle": [
        ("approach", 0, 6.2, 2.6, -95),
        ("penetrate", 0, 8.3, 1.3, -55),
        ("drag", 0, 5.7, 1.0, -100),
        ("drag-curl", 0, 4.9, 1.1, -140, 0.6),
        ("curl", 0, 4.8, 1.5, -180, 0.6),
        ("lift", 0, 5.0, 6.6, -180, 0.6),
        ("swing to truck", -90, 5.4, 6.6, -180, 0.5),
        *TAIL,
    ],
}


FLOOR_CLEARANCE = 0.08  # m: lowest bucket point stays this far above the hard pit floor (z = 0)
DIG = ("penetrate", "drag", "drag-curl", "curl")


def lowest_point(w_from, w_to, steps=11):
    """Lowest outer-box corner of the bucket along the straight pin path from w_from to w_to."""
    low = 1e9
    for s in np.linspace(0, 1, steps):
        psi, r, z, phi = [w_from[k] + s * (w_to[k] - w_from[k]) for k in range(1, 5)]
        q = ex.ik(math.radians(psi), r, z, math.radians(phi))
        _, pin, _ = ex.fk(q)
        a3 = q[1] + q[2] + q[3]
        for x in (0.0, -ex.BUCKET_D):
            for zz in (0.0, -ex.BUCKET_H):
                low = min(low, (pin + ex.swing_vec(q[0], ex.local_to_world(a3, np.array([x, 0.0, zz]))))[2])
    return low


def with_floor_clearance(cycle):
    """Raise dig waypoints until the bucket clears the floor along the path into them."""
    out = [list(w) for w in cycle]
    for i, w in enumerate(out):
        if w[0] not in DIG:
            continue
        prev = out[i - 1]
        while lowest_point(prev, w) < FLOOR_CLEARANCE:
            w[3] = round(w[3] + 0.01, 3)
    return [tuple(w) for w in out]


def check_limits(cycle):
    bad = []
    for w in cycle:
        q = ex.ik(math.radians(w[1]), w[2], w[3], math.radians(w[4]))
        for k, j in enumerate(ex.JOINTS):
            lo, hi = ex.LIMITS[j]
            if not lo <= q[k] <= hi:
                bad.append(f"{w[0]}:{j}={math.degrees(q[k]):.0f}")
    return bad


def run_pass(sim, cycle, max_sim_s=60.0):
    sim.reset()
    sim.CYCLE = cycle
    sim.DIG_STAGES = DIG
    sim.auto = True
    stage_log = []
    prev = sim.auto_stage
    t_wall = time.perf_counter()
    end_stage = next(i for i, w in enumerate(cycle) if w[0] == "swing back")
    peak_fill = 0.0
    while sim.sim_time < max_sim_s:
        sim.step()
        if int(round(sim.sim_time * 60)) % 6 == 0:
            sim.measure()
            peak_fill = max(peak_fill, sim.fill_kg)
        if sim.auto_stage != prev:
            sim.measure()
            stage_log.append({"reached": cycle[sim.auto_stage][0], "t": round(sim.sim_time, 2), "bucket_kg": round(sim.fill_kg), "truck_kg": round(sim.truck_kg), "pressure": [round(p, 2) for p in sim.press]})
            prev = sim.auto_stage
            if sim.auto_stage == end_stage:
                break
    # let the dumped load settle, then weigh the truck
    sim.auto = False
    sim.set_levers(0, 0, 0, 0)
    for _ in range(90):
        sim.step()
    sim.measure()
    return {
        "truck_kg": round(sim.truck_kg),
        "peak_bucket_kg": round(peak_fill),
        "rated_kg": ex.BUCKET_RATED_KG,
        "cycle_s": round(stage_log[-1]["t"], 1) if stage_log else None,
        "wall_s": round(time.perf_counter() - t_wall, 1),
        "stages": stage_log,
        "skipped_stages": sim.skipped,
    }


def main():
    names = sys.argv[1:] or list(VARIANTS)
    sim = ex.ExcavatorSim(iterations=ITERATIONS)
    rows = []
    for name in names:
        cycle = with_floor_clearance(VARIANTS[name])
        bad = check_limits(cycle)
        if bad:
            print(f"{name}: joint limits violated {bad}", flush=True)
            continue
        res = run_pass(sim, cycle)
        with open(f"logs/payload_{name}.json", "w") as fh:
            json.dump({"variant": name, "cycle": cycle, **res}, fh, indent=1)
        by_stage = {s["reached"]: s["bucket_kg"] for s in res["stages"]}
        # bucket_at[stage] = interior load on arriving at that stage's waypoint
        rows.append((name, res["truck_kg"], res["peak_bucket_kg"], by_stage.get("curl"), by_stage.get("lift"), by_stage.get("swing to truck"), res["cycle_s"]))
        print(json.dumps({"variant": name, **{k: v for k, v in res.items() if k != "stages"}, "bucket_at": by_stage}), flush=True)
    print("\n| variant | truck kg (pass) | % of 1.38 t | peak in bucket | after curl | after lift | after swing | cycle s |")
    print("|---|---|---|---|---|---|---|---|")
    for name, truck, peak, curl, lift, dump, cyc in rows:
        print(f"| {name} | {truck} | {100 * truck / ex.BUCKET_RATED_KG:.0f}% | {peak} | {curl} | {lift} | {dump} | {cyc} |")


if __name__ == "__main__":
    main()
