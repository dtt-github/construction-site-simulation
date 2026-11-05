"""Soil presets for the MPM soil, with the published values they stand for, and the soil mechanics used
to check them: the critical height of an unsupported vertical cut and McKyes' fundamental earthmoving
equation with its cohesion and adhesion terms.

Newton's implicit MPM yields on a Drucker-Prager cone. In its orthonormal stress vector (normal part
r_N = tr(sigma)/sqrt(3), shear part |r_T| = |dev sigma| = sqrt(2 J2)) it is |r_T| <= yield_stress +
(friction / sqrt(3/2)) (r_N + tensile), that is sqrt(J2) <= friction p + yield_stress / sqrt(2) (p the mean
pressure, compression positive), with a flat cap at half the yield pressure. Its plastic flow has no
dilatancy, so in plane strain it matches Mohr-Coulomb (phi, c) at friction = sin(phi) and yield_stress =
sqrt(2) c cos(phi) (soils.drucker_prager). The presets instead set friction = tan(phi) and yield_stress = c,
a cone stronger than their phi and c, which the excavator's unconverged rheology solve (150 iterations at
1/30 s) softens again: the values are effective ones for that solver setting, checked there against the
vertical-cut and blade tests in soil_calibrate.py (RESULTS.md, "Why cutting forces change with the MPM step").

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
    "dry-sand": {"color": "#c9a774", "grain": "#b08a55", "label": "Dry sand", "density": 1600.0, "friction": 0.75, "cohesion": 0.0, "tensile": 0.0, "fill_factor": 1.0, "swell": 0.12},
    "moist-sand": {"color": "#9c7f58", "grain": "#87693f", "label": "Moist sand", "density": 1750.0, "friction": 0.75, "cohesion": 2.0e3, "tensile": 0.5e3, "mpm_cohesion": 0.9e3, "mpm_tensile": 0.2e3, "remoulded": 0.7, "fill_factor": 1.0, "swell": 0.15},
    "loam": {"color": "#7b5b3e", "grain": "#684a30", "label": "Loam", "density": 1850.0, "friction": 0.55, "cohesion": 12.0e3, "tensile": 2.0e3, "remoulded": 0.2, "fill_factor": 1.05, "swell": 0.25},
    "clay": {"color": "#8f6a52", "grain": "#7a5640", "label": "Clay", "density": 1950.0, "friction": 0.05, "cohesion": 45.0e3, "tensile": 8.0e3, "remoulded": 0.3, "fill_factor": 1.0, "swell": 0.35},
}
DEFAULT_SOIL = "dry-sand"
# what a bucket carries: "fill_factor" of the heaped (SAE) rating as loose soil (excavator bucket fill factors
# of the Caterpillar Performance Handbook: sand and gravel 95-110 %, moist loam and sandy clay 100-110 %,
# hard tough clay 80-90 %; Komatsu's handbook rates clay higher, 110-120 %: clay takes 1.0, between the
# two), and "swell" from bank to loose volume (typical: sand 10-15 %, loam 20-30 %, clay 30-40 %)
# plastic volume opening (Jp - 1) over which a cohesive soil loosened by digging falls to its remoulded
# strength ("remoulded": fraction kept; 1 / sensitivity, typically 1/2-1/4 for clays): it breaks up rather
# than drawing out into strands (MPM's perfectly plastic tension cap would let it stretch without limit).
# Pressed together again (Jp back to 1) it regains strength.
CRACK_STRAIN = 0.1


def drucker_prager(phi, c):
    """Newton MPM friction and yield_stress matching Mohr-Coulomb (phi rad, c Pa) in plane strain with no
    dilatancy (converged solve)."""
    return math.sin(phi), math.sqrt(2.0) * c * math.cos(phi)


def mpm_attributes(soil, dilatancy=0.0):
    """Per-particle Newton MPM material attributes for a preset. dilatancy: the flow rule's dilation (normal
    velocity = dilatancy x friction x shear velocity): sheared soil opens up, and with the solver's packing
    limit it settles back only as far as its particle volumes allow (bulking, soil_zone.py)."""
    s = SOILS[soil]
    c, t = s.get("mpm_cohesion", s["cohesion"]), s.get("mpm_tensile", s["tensile"])
    attrs = {"mpm:friction": s["friction"]}
    if dilatancy > 0.0:
        attrs["mpm:dilatancy"] = dilatancy
    if c > 0.0 or t > 0.0:
        attrs.update({"mpm:yield_stress": c, "mpm:yield_pressure": YIELD_PRESSURE, "mpm:tensile_yield_ratio": t / YIELD_PRESSURE})
    return attrs


def payload_kg(soil, heaped_m3):
    """Mass a bucket of heaped rating `heaped_m3` typically carries in this soil."""
    s = SOILS[soil]
    return heaped_m3 * s["fill_factor"] * s["density"] / (1.0 + s["swell"])


def critical_height(soil):
    """Range (m) of the critical height of an unsupported vertical cut: 2.67 c/gamma tan(45 + phi/2) with
    tension cracks (Terzaghi) to 3.83 c/gamma tan(45 + phi/2) without (limit analysis upper bound)."""
    s = SOILS[soil]
    k = s["cohesion"] / (s["density"] * G) * math.tan(math.pi / 4 + math.atan(s["friction"]) / 2)
    return 2.67 * k, 3.83 * k


def resting_friction(soil):
    """Friction for moving soil on resting (dormant) soil. The soil zone's resting soil is a rigid collider,
    and a collider has friction only, no cohesion; a soil's own friction then lets an undrained clay
    (friction 0.05) slide on clay as on ice. Use the larger of the friction and the remoulded cohesion over
    the vertical stress under 1 m of soil (c_r / gamma 1 m): clay 0.71; the other presets keep their friction."""
    s = SOILS[soil]
    return max(s["friction"], s["cohesion"] * s.get("remoulded", 1.0) / (s["density"] * G * 1.0))


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
