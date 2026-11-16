"""Time study of a simulated site, in the form a field time study takes (stopwatch or video: each machine's
work cycle split into elements, timed cycle by cycle). The benchmark feeds it each machine's stage every frame;
it maps stages to elements, cuts cycles where a machine starts its cycle again, and reports per element the
mean, spread and share of the cycle, plus where each machine's time went over the whole run.

Elements follow the usual breakdown, so field data can be set beside it directly:

- excavator: dig (bucket into the face, drag and curl), swing loaded, dump, swing empty (back to the face),
  wait for a carrier; a cycle starts as the bucket goes in;
- carrier: load (at the loading spot), haul loaded (to the dump point), dump (back up, tip, lower),
  return (back to the loading spot), wait (at the dump point, for the loading spot, on the road); a cycle
  starts as loading starts;
- dozer: push (blade down and pushing), return (blade raised, back and to the next lane), wait (at a lane's
  end for a carrier to tip and go), parked;
  a cycle starts as the blade lowers;
- roller: roll, travel (to and from lanes), parked.
"""

import numpy as np

ELEMENTS = {
    "excavator": {"approach": "swing empty", "penetrate": "dig", "drag": "dig", "drag-curl": "dig", "curl": "dig",
                  "swing to truck": "swing loaded", "wait for carrier": "wait", "dump": "dump", "swing back": "swing empty",
                  "relocate": "move on", "pit dug out": "idle", "manual": "idle"},
    "carrier": {"loading": "load", "drive out": "haul loaded", "turn to dump": "haul loaded", "haul out": "haul loaded",
                "reverse to dump": "dump", "tip": "dump", "dump": "dump", "lower": "dump",
                "return": "return", "turn to load": "return", "haul back": "return", "reverse in": "return",
                "to queue": "return", "from queue": "return", "to dump": "haul loaded", "to lane": "return",
                "wait at dump": "wait", "wait to load": "wait", "queue on road": "wait"},
    "dozer": {"lower": "push", "push": "push", "raise": "return", "back": "return", "to lane": "return",
              "to park": "return", "parked": "parked", "wait clear": "wait"},
    "roller": {"roll": "roll", "to lane": "travel", "to park": "travel", "parked": "parked"},
}
CYCLE_START = {"excavator": "dig", "carrier": "load", "dozer": "push", "roller": None}
# elements that are not part of a work cycle: a cycle holding one of these is reported, but kept out of the
# cycle statistics (a carrier waiting 300 s for the dozer is a delay, not a slow haul)
DELAYS = {"excavator": ("move on", "idle"), "carrier": (), "dozer": ("parked", "wait"), "roller": ("parked",)}


class Machine:
    def __init__(self, kind):
        self.kind, self.map = kind, ELEMENTS[kind]
        self.totals, self.cycles, self.cur, self.prev = {}, [], None, None

    def add(self, stage, dt):
        el = self.map.get(stage, stage)
        self.totals[el] = self.totals.get(el, 0.0) + dt
        start = CYCLE_START[self.kind]
        if start is not None and el == start and self.prev != start:
            if self.cur is not None:
                self.cycles.append(self.cur)
            self.cur = {}
        if self.cur is not None:
            self.cur[el] = self.cur.get(el, 0.0) + dt
        self.prev = el

    def report(self):
        total = sum(self.totals.values())
        out = {"time_share": {k: round(v / total, 3) for k, v in sorted(self.totals.items(), key=lambda kv: -kv[1])},
               "total_s": round(total, 1)}
        if CYCLE_START[self.kind] is None:
            return out
        cyc = [c for c in self.cycles if not any(d in c for d in DELAYS[self.kind])]
        out["cycles"] = len(self.cycles)
        out["cycles_timed"] = len(cyc)  # without the ones that held a delay
        if not cyc:
            return out
        els = sorted({e for c in cyc for e in c}, key=lambda e: -np.mean([c.get(e, 0.0) for c in cyc]))
        tot = np.array([sum(c.values()) for c in cyc])
        out["cycle_s"] = _stats(tot)
        out["elements"] = {e: {**_stats(np.array([c.get(e, 0.0) for c in cyc])),
                               "share": round(float(np.mean([c.get(e, 0.0) for c in cyc]) / tot.mean()), 3)} for e in els}
        return out


def _stats(v):
    return {"mean": round(float(v.mean()), 2), "sd": round(float(v.std(ddof=1)), 2) if len(v) > 1 else 0.0,
            "median": round(float(np.median(v)), 2), "p10": round(float(np.percentile(v, 10)), 2),
            "p90": round(float(np.percentile(v, 90)), 2)}


class TimeStudy:
    def __init__(self):
        self.machines = {}

    def add(self, name, kind, stage, dt):
        if name not in self.machines:
            self.machines[name] = Machine(kind)
        self.machines[name].add(stage, dt)

    def report(self):
        return {name: m.report() for name, m in self.machines.items()}
