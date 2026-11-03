"""Phase 0 benchmark: the same blade-through-sand test in Project Chrono CRM (GPU SPH soil).

Same geometry as newton_blade_bench.py: a 1 m wide vertical blade cutting 0.2 m deep
through a 4 x 2 x 0.5 m bed, driven at constant speed by a linear motor. Reports wall
time per simulated second, real-time factor, particle count and the soil force on the
blade, and compares the force with the McKyes/Reece earthmoving equation.
"""

import argparse
import json
import math
import time

import numpy as np
import pychrono as chrono
import pychrono.fsi as fsi
import pychrono.vehicle as veh


def mckyes_force(rho, depth, width, phi, delta, g=9.81):
    alpha = math.pi / 2
    best = None
    for beta in np.linspace(math.radians(5), math.radians(85), 2000):
        denom = 2.0 * (math.cos(alpha + delta) + math.sin(alpha + delta) / math.tan(beta + phi))
        if denom <= 0:
            continue
        n_gamma = (1.0 / math.tan(alpha) + 1.0 / math.tan(beta)) / denom
        if best is None or n_gamma < best:
            best = n_gamma
    return rho * g * depth**2 * width * best


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--spacing", type=float, default=0.03, help="SPH initial particle spacing (m)")
    p.add_argument("--bed", type=float, nargs=3, default=[4.0, 2.0, 0.5])
    p.add_argument("--density", type=float, default=1600.0)
    p.add_argument("--friction", type=float, default=0.75)
    p.add_argument("--blade-friction", type=float, default=0.4)
    p.add_argument("--depth", type=float, default=0.2)
    p.add_argument("--width", type=float, default=1.0)
    p.add_argument("--speed", type=float, default=0.5)
    p.add_argument("--duration", type=float, default=1.0, help="simulated cutting time (s)")
    p.add_argument("--settle", type=float, default=0.2, help="settling time before the blade moves (s)")
    p.add_argument("--step", type=float, default=2.5e-4, help="CFD step (s)")
    p.add_argument("--active-domain", action="store_true", help="only simulate SPH near the blade")
    p.add_argument("--out", type=str, default=None)
    args = p.parse_args()

    bx, by, bz = args.bed
    sys = chrono.ChSystemNSC()
    sys.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
    sys.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -9.81))

    ground = chrono.ChBody()
    ground.SetFixed(True)
    sys.AddBody(ground)

    # Blade
    blade_h, blade_t = 0.8, 0.04
    x0 = -bx / 2 + 0.3
    z_center = bz - args.depth + blade_h / 2
    blade = chrono.ChBody()
    blade.SetMass(500)
    blade.SetInertiaXX(chrono.ChVector3d(50, 50, 50))
    blade.SetPos(chrono.ChVector3d(x0, 0, z_center))
    sys.AddBody(blade)

    motor = chrono.ChLinkMotorLinearSpeed()
    motor.SetGuideConstraint(chrono.ChLinkMotorLinear.GuideConstraint_PRISMATIC)
    speed_fn = chrono.ChFunctionSetpoint()
    motor.SetSpeedFunction(speed_fn)
    # Motor acts along Z of its frame; rotate so Z points along world +X
    motor.Initialize(blade, ground, chrono.ChFramed(chrono.ChVector3d(x0, 0, z_center), chrono.QuatFromAngleY(math.pi / 2)))
    sys.Add(motor)

    terrain = veh.CRMTerrain(sys, args.spacing)
    terrain.SetVerbose(False)
    terrain.SetGravitationalAcceleration(chrono.ChVector3d(0, 0, -9.81))
    terrain.SetStepSizeCFD(args.step)

    soil = fsi.SoilProperties()
    soil.density = args.density
    soil.Young_modulus = 1e6
    soil.Poisson_ratio = 0.3
    soil.mu_I0 = 0.04
    soil.mu_fric_s = args.friction
    soil.mu_fric_2 = args.friction
    soil.average_diam = 0.005
    soil.cohesion_coeff = 0.0
    terrain.SetCrmSPH(soil)

    sph = fsi.SPHParameters()
    sph.integration_scheme = fsi.IntegrationScheme_RK2
    sph.initial_spacing = args.spacing
    sph.d0_multiplier = 1.0
    sph.free_surface_threshold = 0.8
    sph.artificial_viscosity = 0.5
    sph.viscosity_method = fsi.ViscosityMethod_ARTIFICIAL_BILATERAL
    sph.boundary_method = fsi.BoundaryMethod_ADAMI
    sph.use_variable_time_step = False
    terrain.SetSPHParameters(sph)
    terrain.SetOutputLevel(fsi.OutputLevel_STATE)

    geom = chrono.ChBodyGeometry()
    geom.materials.push_back(chrono.ChContactMaterialData())
    geom.coll_boxes.push_back(chrono.BoxShape(chrono.VNULL, chrono.QUNIT, chrono.ChVector3d(blade_t, args.width, blade_h)))
    terrain.AddRigidBody(blade, geom, False)
    if args.active_domain:
        terrain.SetActiveDomain(chrono.ChVector3d(1.2, args.width + 0.6, 1.2))

    terrain.Construct(
        chrono.ChVector3d(bx, by, bz),
        chrono.ChVector3d(0, 0, 0),
        fsi.BoxSide_ALL & ~fsi.BoxSide_Z_POS,
    )
    terrain.Initialize()
    n_particles = terrain.GetNumSPHParticles()
    aabb = terrain.GetSPHBoundingBox()
    print(f"SPH particles: {n_particles}  AABB z: {aabb.min.z:.3f}..{aabb.max.z:.3f}", flush=True)

    exchange = 5 * args.step
    t = 0.0
    forces = []
    wall = 0.0
    timed_sim = 0.0
    while t < args.settle + args.duration:
        moving = t >= args.settle
        speed_fn.SetSetpoint(args.speed if moving else 0.0, t)
        t0 = time.perf_counter()
        terrain.DoStepDynamics(exchange)
        t1 = time.perf_counter()
        t += exchange
        if moving:
            wall += t1 - t0
            timed_sim += exchange
            f = terrain.GetFsiBodyForce(blade)
            forces.append((f.x, f.z, motor.GetMotorForce()))

    forces = np.array(forces)
    steady = forces[len(forces) // 2 :]
    f_ref = mckyes_force(args.density, args.depth, args.width, math.atan(args.friction), math.atan(args.blade_friction))
    fx = float(np.abs(steady[:, 0]).mean())
    result = {
        "spacing_m": args.spacing,
        "particles": int(n_particles),
        "active_domain": args.active_domain,
        "cfd_step_s": args.step,
        "wall_s_per_sim_s": round(wall / timed_sim, 2),
        "realtime_factor": round(timed_sim / wall, 4),
        "blade_force_x_N_mean_steady": round(fx, 1),
        "blade_force_z_N_mean_steady": round(float(steady[:, 1].mean()), 1),
        "motor_force_N_mean_steady": round(float(np.abs(steady[:, 2]).mean()), 1),
        "mckyes_force_N": round(f_ref, 1),
        "force_ratio_sim_over_mckyes": round(fx / f_ref, 3),
        "blade_travel_m": round(blade.GetPos().x - x0, 3),
    }
    print(json.dumps(result, indent=2))
    if args.out:
        with open(args.out, "w") as fh:
            json.dump(result, fh, indent=2)


if __name__ == "__main__":
    main()
