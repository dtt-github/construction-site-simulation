"""Machine-to-machine clearance on the site: the carriers, the dozer and the roller never drive into each other.

The mobile machines are scripted (kinematic), so nothing in the physics keeps them apart: two of them could
pass through each other. On a site they don't: operators keep a clearance and stop, or wait, when another
machine is in the way (site traffic rules, e.g. HSE HSG144 "The safe use of vehicles on construction
sites": keep vehicles and plant apart, and give way). Here each machine has a plan-view outline (a rectangle
round its tracks, wheels, blade, drum and body, from the models drawn in the viewer), and a machine whose
next step would bring its outline within CLEARANCE_M of another machine's stays where it is for that frame:
it waits until the other has moved on. Every machine yields this way (the soil never pushes a machine aside,
as the outlines never touch), so a wait can block another wait; the site's own rules (the dump gate, the
dozer's and roller's cells) keep that from happening, and the counters show if it does.

Outlines (base_link frame, x ahead): x range and half width (m).
  carrier  IC120 class: vessel tail 3.5 m behind the base (3.3 m lowered; tipped, the tailgate swings
           back), cab and idlers 2.3 m ahead; tracks 1.0 m half gauge + 0.6 m shoes -> 1.3 m half width.
  dozer    D37PXi-24: blade cutting edge 2.47 m ahead at full raise (dozer.py), blade 3.21 m wide; tracks
           and sprocket 1.7 m behind the base.
  roller   BW 211 D-5 class: drum 1.45 m ahead (radius 0.75), rear tyres 1.45 m behind (radius 0.75),
           engine hood to 2.25 m behind; drum 2.13 m wide plus its frame.
"""

import math

import numpy as np

CLEARANCE_M = 0.3  # gap kept between two machines' outlines
OUTLINE = {
    "carrier": (-3.5, 2.3, 1.3),
    "dozer": (-1.7, 2.65, 1.61),
    "roller": (-2.25, 2.25, 1.17),
}
STUCK_S = 60.0  # a wait longer than this is counted as stuck (two machines waiting on each other)


def corners(kind, x, y, yaw, grow=0.0):
    """The outline's four corners (4, 2) in the site frame, grown by grow on every side."""
    x0, x1, hw = OUTLINE[kind]
    local = np.array([[x0 - grow, -hw - grow], [x1 + grow, -hw - grow], [x1 + grow, hw + grow], [x0 - grow, hw + grow]])
    c, s = math.cos(yaw), math.sin(yaw)
    return local @ np.array([[c, s], [-s, c]]) + np.array([x, y])


def overlap(a, b):
    """Whether two convex polygons (n, 2) overlap (separating axis test)."""
    for poly in (a, b):
        edges = np.roll(poly, -1, axis=0) - poly
        for nx, ny in np.stack([-edges[:, 1], edges[:, 0]], 1):
            pa, pb = a @ (nx, ny), b @ (nx, ny)
            if pa.max() <= pb.min() or pb.max() <= pa.min():
                return False
    return True


def near(kind_a, poses_a, kind_b, poses_b, gap=CLEARANCE_M):
    """Whether any pose (x, y, yaw) of one machine comes within gap of any pose of the other: used to lay out
    routes that keep two machines apart (the roller's lanes beside the dozer's pushes)."""
    b = [corners(kind_b, *p) for p in poses_b]
    return any(overlap(corners(kind_a, *p, gap), q) for p in poses_a for q in b)


class Machine:
    """How the traffic check reads and restores one machine's pose."""

    def __init__(self, name, kind, obj):
        self.name, self.kind, self.obj = name, kind, obj
        self.wait_s, self.waited_s, self.yields, self.stuck = 0.0, 0.0, 0, 0

    def pose(self):
        o = self.obj
        if self.kind == "carrier":
            return float(o.base[0]), float(o.base[1]), o.yaw
        return o.x, o.y, o.yaw

    def present(self):
        return not (self.kind == "carrier" and self.obj.hidden)

    def save(self):
        o = self.obj
        keep = {"yaw": o.yaw, "speed": o.speed, "plan": list(o.plan), "stage": o.stage}
        if self.kind == "carrier":
            keep["base"] = o.base.copy()
        else:
            keep["x"], keep["y"] = o.x, o.y
        return keep

    def restore(self, keep):
        o = self.obj
        for k, v in keep.items():
            setattr(o, k, v)
        o.speed = 0.0


class Traffic:
    def __init__(self):
        self.machines = []
        self.enabled = True  # False: the machines drive through each other (collisions are still counted)

    def add(self, name, kind, obj):
        self.machines.append(Machine(name, kind, obj))

    def reset(self):
        for m in self.machines:
            m.wait_s, m.waited_s, m.yields, m.stuck = 0.0, 0.0, 0, 0

    def outline(self, m, grow=0.0):
        return corners(m.kind, *m.pose(), grow)

    def blocked_by(self, m):
        """The machines whose outline the machine's outline (grown by the clearance) now overlaps."""
        mine = self.outline(m, CLEARANCE_M)
        return [o for o in self.machines if o is not m and o.present() and overlap(mine, self.outline(o))]

    def step(self, m, update, dt):
        """Advance one machine by update() unless that brings it closer than the clearance to another
        machine it was clear of; then it keeps its pose and waits. Returns update()'s result."""
        if not self.enabled or not m.present():
            return update()
        x0, y0, _ = m.pose()
        before = {id(o): math.hypot(o.pose()[0] - x0, o.pose()[1] - y0) for o in self.blocked_by(m)}
        keep = m.save()
        result = update()
        if not m.present():
            return result
        x1, y1, _ = m.pose()
        # a machine it was clear of, or one it was already close to and now comes closer to
        now = [o for o in self.blocked_by(m) if id(o) not in before or math.hypot(o.pose()[0] - x1, o.pose()[1] - y1) < before[id(o)] - 1e-6]
        if now:
            m.restore(keep)
            if m.wait_s == 0.0:
                m.yields += 1
            m.wait_s += dt
            m.waited_s += dt
            if m.wait_s >= STUCK_S and m.wait_s - dt < STUCK_S:
                m.stuck += 1
        else:
            m.wait_s = 0.0
        return result

    def overlaps(self):
        """Pairs of machines whose outlines overlap now (without the clearance): a collision."""
        ms = [m for m in self.machines if m.present()]
        return [(a.name, b.name) for i, a in enumerate(ms) for b in ms[i + 1:] if overlap(self.outline(a), self.outline(b))]

    def counters(self):
        return {m.name: {"yields": m.yields, "waited_s": round(m.waited_s, 1), "stuck": m.stuck} for m in self.machines}
