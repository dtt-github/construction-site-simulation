"""Soil presets for the MPM soil, with the published values they stand for, and the soil mechanics used
to check them: the critical height of an unsupported vertical cut and McKyes' fundamental earthmoving
equation with its cohesion and adhesion terms.

Newton's implicit MPM yields on a Drucker-Prager cone, sqrt(J2) <= yield_stress + friction * (p + p_t)
(p the mean pressure, compression positive; p_t = tensile_yield_ratio * yield_pressure, the tensile
strength). With friction taken as tan(phi), as the sand calibration does, yield_stress is the cohesion c.
Each preset's c is checked against the vertical-cut test in soil_calibrate.py.

    dry sand     cohesionless, 37 deg (the calibrated sand of phases 0-2)
    moist sand   capillary ("apparent") cohesion of a few kPa between grains; stands in steep faces
    loam         sandy clay loam, firm: drained c' about 10-15 kPa, phi' about 28-30 deg
    clay         firm clay, undrained (short-term, as it is dug): cu 40-75 kPa (ISO 14688-2, BS 5930)
"""

import math

import numpy as np

G = 9.81
YIELD_PRESSURE = 1.0e8  # Pa: compressive cap, far above any pressure in the pit; sets the tensile scale

SOILS = {
    "dry-sand": {"color": "#c9a774", "grain": "#b08a55", "label": "Dry sand", "density": 1600.0, "friction": 0.75, "cohesion": 0.0, "tensile": 0.0},
    "moist-sand": {"color": "#9c7f58", "grain": "#87693f", "label": "Moist sand", "density": 1750.0, "friction": 0.75, "cohesion": 2.0e3, "tensile": 0.5e3, "mpm_cohesion": 0.9e3, "mpm_tensile": 0.2e3, "remoulded": 0.7},
    "loam": {"color": "#7b5b3e", "grain": "#684a30", "label": "Loam", "density": 1850.0, "friction": 0.55, "cohesion": 12.0e3, "tensile": 2.0e3, "remoulded": 0.2},
    "clay": {"color": "#8f6a52", "grain": "#7a5640", "label": "Clay", "density": 1950.0, "friction": 0.05, "cohesion": 45.0e3, "tensile": 8.0e3, "remoulded": 0.3},
}
DEFAULT_SOIL = "dry-sand"
# plastic volume opening (Jp - 1) over which a cohesive soil loosened by digging falls to its remoulded
# strength ("remoulded": fraction kept; 1 / sensitivity, typically 1/2-1/4 for clays): it breaks up rather
# than drawing out into strands (MPM's perfectly plastic tension cap would let it stretch without limit).
# Pressed together again (Jp back to 1) it regains strength.
CRACK_STRAIN = 0.1


def mpm_attributes(soil):
    """Per-particle Newton MPM material attributes for a preset."""
    s = SOILS[soil]
    c, t = s.get("mpm_cohesion", s["cohesion"]), s.get("mpm_tensile", s["tensile"])
    attrs = {"mpm:friction": s["friction"]}
    if c > 0.0 or t > 0.0:
        attrs.update({"mpm:yield_stress": c, "mpm:yield_pressure": YIELD_PRESSURE, "mpm:tensile_yield_ratio": t / YIELD_PRESSURE})
    return attrs


def critical_height(soil):
    """Range (m) of the critical height of an unsupported vertical cut: 2.67 c/gamma tan(45 + phi/2) with
    tension cracks (Terzaghi) to 3.83 c/gamma tan(45 + phi/2) without (limit analysis upper bound)."""
    s = SOILS[soil]
    k = s["cohesion"] / (s["density"] * G) * math.tan(math.pi / 4 + math.atan(s["friction"]) / 2)
    return 2.67 * k, 3.83 * k


def mckyes(density, cohesion, depth, width, phi, delta, adhesion=0.0, rake=math.pi / 2):
    """Fundamental earthmoving equation (McKyes 1985) for a flat tool at `rake` (rad from the horizontal)
    cutting `depth` in flat ground: F = (gamma d^2 N_gamma + c d N_c + c_a d N_ca) w, minimised over the
    failure plane angle. Returns the horizontal force (N) and the factors at the minimum."""
    a = rake
    best = None
    for beta in np.linspace(math.radians(3), math.radians(87), 3000):
        den = math.cos(a + delta) + math.sin(a + delta) / math.tan(beta + phi)
        if den <= 0.0:
            continue
        n_g = (1.0 / math.tan(a) + 1.0 / math.tan(beta)) / (2.0 * den)
        n_c = (1.0 + 1.0 / (math.tan(beta) * math.tan(beta + phi))) / den
        n_ca = (1.0 - 1.0 / (math.tan(a) * math.tan(beta + phi))) / den
        f = (density * G * depth**2 * n_g + cohesion * depth * n_c + adhesion * depth * n_ca) * width
        if best is None or f < best[0]:
            best = (f, n_g, n_c, n_ca, beta)
    f, n_g, n_c, n_ca, beta = best
    return f * math.sin(a + delta), {"N_gamma": n_g, "N_c": n_c, "N_ca": n_ca, "beta_deg": math.degrees(beta)}
