"""Track-soil contact: how far a crawler's tracks sink into the soil, the ruts they press, and how much they
can pull before they slip (classical terramechanics, Bekker and Janosi-Hanamoto as in Wong, Theory of Ground
Vehicles).

Sinkage (Bekker): a track of width b pressing the soil with a uniform pressure p sinks
    z0 = (p / (kc / b + kphi)) ** (1 / n)
and pushing that rut ahead of it costs the compaction resistance, per track,
    Rc = b (kc / b + kphi) z0 ** (n + 1) / (n + 1).
Thrust (Janosi and Hanamoto): the soil under a track of contact length L shears by up to i L at slip i, and
the track's thrust is
    F(i) = (A c + W tan(phi)) (1 - K / (i L) (1 - exp(-i L / K)))
with A the contact area of both tracks, W the weight on them and K the shear deformation modulus. A crawler
pulling a load P slips at the i where F(i) = P + Rc; when P + Rc exceeds F(1) its tracks spin in place.

Pressure-sinkage values (n, kc in kN/m^(n+1), kphi in kN/m^(n+2)) are the commonly cited Bekker-Wong set
for dry sand, sandy loam (15 % moisture) and clayey soil (38 %), as tabulated in NASA TM-20250006958 (an
overview of tyre-ground contact models), which attributes them to Wong. They describe natural ground, not
freshly dumped fill, so the sinkage of loose spoil is if anything understated. The shear strength (c, phi)
is that of the MPM soil itself (soils.py), at its remoulded cohesion where the soil has been dug, so the
tracks and the blade see the same soil. K is not in that table: 2.5 cm for sands and loams and 1 cm for
clay are assumed, values of the order Wong reports for such soils.

Machine data:
- Komatsu D37PX-24: 610 mm (24 in) shoes, 2.235 m (7 ft 4 in) of track on the ground, 30 kPa (4.33 psi)
  (dealer specification); the 9.5 t used here gives 35 kPa.
- IHI/Kato IC120: 14.1 t operating weight (31,080 lb), 32.5 kPa empty and 58.6 kPa with 11 t on board
  (4.71 and 8.5 psi, dealer listing), so 4.26 m^2 of track on the ground; with 600 mm shoes (assumed) that
  is 3.55 m per track.
"""

import math

import numpy as np

import soils

G = 9.81

# Bekker pressure-sinkage values per soil preset: n, kc (kN/m^(n+1)), kphi (kN/m^(n+2)), and the assumed
# shear deformation modulus K (m)
TERRAIN = {
    "dry-sand": {"source": "dry sand", "n": 1.1, "kc": 0.99, "kphi": 1528.43, "K": 0.025},
    "moist-sand": {"source": "dry sand", "n": 1.1, "kc": 0.99, "kphi": 1528.43, "K": 0.025},
    "loam": {"source": "sandy loam, 15 % moisture", "n": 0.7, "kc": 5.27, "kphi": 1515.04, "K": 0.025},
    "clay": {"source": "clayey soil, 38 % moisture", "n": 0.5, "kc": 13.19, "kphi": 692.15, "K": 0.01},
}


class Track:
    """A pair of tracks: shoe width b, contact length L (m each), on one soil preset."""

    def __init__(self, b, length, soil, loose=True):
        self.b, self.L = b, length
        self.area = 2 * b * length
        t, s = TERRAIN[soil], soils.SOILS[soil]
        self.n, self.K = t["n"], t["K"]
        self.k = (t["kc"] / b + t["kphi"]) * 1e3  # N/m^(n+2)
        c = s["cohesion"] * (s.get("remoulded", 1.0) if loose else 1.0)
        self.c, self.tan_phi = c, s["friction"]

    def pressure(self, mass_kg):
        return mass_kg * G / self.area

    def sinkage(self, mass_kg):
        """Static sinkage (m) under the machine's weight."""
        return (self.pressure(mass_kg) / self.k) ** (1.0 / self.n)

    def resistance(self, mass_kg):
        """Compaction resistance of both tracks (N)."""
        z = self.sinkage(mass_kg)
        return 2 * self.b * self.k * z ** (self.n + 1) / (self.n + 1)

    def thrust(self, mass_kg, slip):
        """Thrust (N) at slip (0..1]."""
        f_max = self.area * self.c + mass_kg * G * self.tan_phi
        j = max(slip, 1e-6) * self.L / self.K
        return f_max * (1.0 - (1.0 - math.exp(-j)) / j)

    def slip(self, mass_kg, pull):
        """Slip at which the tracks deliver pull (N) on top of their compaction resistance; 1.0 when they
        cannot (they spin in place)."""
        need = pull + self.resistance(mass_kg)
        if need >= self.thrust(mass_kg, 1.0):
            return 1.0
        lo, hi = 0.0, 1.0
        for _ in range(40):
            mid = 0.5 * (lo + hi)
            lo, hi = (mid, hi) if self.thrust(mass_kg, mid) < need else (lo, mid)
        return hi

    def max_pull(self, mass_kg, slip=1.0):
        """Drawbar pull (N) at slip: thrust less compaction resistance."""
        return self.thrust(mass_kg, slip) - self.resistance(mass_kg)


def footprint(x, y, yaw, centre_x, half_gauge, b, length, step=0.04):
    """Points (n, 2) covering both tracks' contact patches of a machine at (x, y, yaw); centre_x: the
    patches' middle along the machine, ahead of (x, y)."""
    u = np.arange(-length / 2 + step / 2, length / 2, step) + centre_x
    v = np.arange(-b / 2 + step / 2, b / 2, step)
    pts = []
    for s in (-1, 1):
        gu, gv = np.meshgrid(u, s * half_gauge + v)
        pts.append(np.stack([gu.ravel(), gv.ravel()], 1))
    p = np.concatenate(pts)
    c, s = math.cos(yaw), math.sin(yaw)
    return np.stack([x + c * p[:, 0] - s * p[:, 1], y + s * p[:, 0] + c * p[:, 1]], 1)
