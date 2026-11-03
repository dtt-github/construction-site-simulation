"""Phase 2 soil server: Newton implicit-MPM sand in the excavation pit, dug by a bucket, streamed to the browser.

The sand bed fills the pit floor of the Phase 1 site (6 x 5 m, 0.5 m deep). A steel bucket
(kinematic, four plates) is driven toward targets sent by the browser, or through an automatic
dig cycle. Each frame the server derives a 10 cm surface heightmap from the particles, collects
the particles that are moving, the bucket's soil force and fill, and streams them over a
WebSocket as one binary message.

Frames: the simulation runs in a Newton Z-up frame centred on the pit floor ("zone" frame).
Everything streamed is converted to the site's three.js Y-up frame:
    three = (x + 24, z - 2.5, 12 - y)

Usage:
    python soil_server.py                 # serve ws://127.0.0.1:8765
    python soil_server.py --bench 30      # headless: 30 s of automatic digging, print stats
"""

import argparse
import asyncio
import json
import math
import struct
import time

import numpy as np
import warp as wp

import newton
from newton.solvers import SolverImplicitMPM

# ------------------------------------------------------------------ site geometry
PIT_CENTER_THREE = (24.0, -2.5, 12.0)  # pit floor centre in site (three.js) coordinates
ZONE_HALF = (3.0, 2.5)  # pit floor half-size, zone x and y (m)
BED_DEPTH = 0.5
WALL_HEIGHT = 1.6

# bucket: 0.9 m wide, 0.7 m deep, 0.6 m tall, open toward +x and +z, origin at its centre
BUCKET_W, BUCKET_L, BUCKET_H, PLATE = 0.9, 0.7, 0.6, 0.03
BUCKET_MAX_SPEED = 0.6  # m/s
BUCKET_MAX_ROT = math.radians(50)  # rad/s

# heightmap streamed to the client
HM_CELL = 0.1
HM_NX = int(round(2 * ZONE_HALF[0] / HM_CELL))
HM_NY = int(round(2 * ZONE_HALF[1] / HM_CELL))
MAX_SEND_PARTICLES = 40000
MOVING_SPEED = 0.05  # m/s: particles faster than this are streamed as grains
SURFACE_MAX_SPEED = 0.3  # m/s: only slower particles build the ground surface

G = 9.81


def mckyes_n_gamma(phi, delta):
    """Minimum N_gamma of the fundamental earthmoving equation for a vertical tool."""
    alpha = math.pi / 2
    best = None
    for beta in np.linspace(math.radians(5), math.radians(85), 2000):
        denom = 2.0 * (math.cos(alpha + delta) + math.sin(alpha + delta) / math.tan(beta + phi))
        if denom > 0:
            n = (1.0 / math.tan(alpha) + 1.0 / math.tan(beta)) / denom
            best = n if best is None else min(best, n)
    return best


# ------------------------------------------------------------------ kernels
@wp.kernel
def heightmap_kernel(
    q: wp.array[wp.vec3],
    qd: wp.array[wp.vec3],
    max_speed: float,
    radius: float,
    x0: float,
    y0: float,
    cell: float,
    nx: int,
    ny: int,
    hm: wp.array[float],
):
    i = wp.tid()
    if wp.length(qd[i]) > max_speed:
        return  # flying or carried sand is drawn as grains, not as ground
    p = q[i]
    ix = int(wp.floor((p[0] - x0) / cell))
    iy = int(wp.floor((p[1] - y0) / cell))
    if ix >= 0 and ix < nx and iy >= 0 and iy < ny:
        wp.atomic_max(hm, iy * nx + ix, p[2] + radius)


@wp.kernel
def compact_moving_kernel(
    q: wp.array[wp.vec3],
    qd: wp.array[wp.vec3],
    min_speed: float,
    max_out: int,
    counter: wp.array[int],
    out: wp.array[wp.vec3],
):
    i = wp.tid()
    if wp.length(qd[i]) > min_speed:
        slot = wp.atomic_add(counter, 0, 1)
        if slot < max_out:
            out[slot] = q[i]


@wp.kernel
def bucket_fill_kernel(
    q: wp.array[wp.vec3],
    xf: wp.transform,
    half_l: float,
    half_w: float,
    half_h: float,
    counter: wp.array[int],
):
    i = wp.tid()
    local = wp.transform_point(wp.transform_inverse(xf), q[i])
    if wp.abs(local[0]) < half_l and wp.abs(local[1]) < half_w and local[2] > -half_h and local[2] < half_h:
        wp.atomic_add(counter, 0, 1)


# ------------------------------------------------------------------ simulation
class SoilSim:
    def __init__(self, voxel=0.08, ppc=2, density=1600.0, friction=0.75, iterations=30, fps=60.0):
        wp.init()
        self.dt = 1.0 / fps
        self.voxel = voxel
        self.density = density
        self.friction = friction
        self.iterations = iterations

        # rigid model: kinematic bucket + static pit walls
        b = newton.ModelBuilder()
        steel = newton.ModelBuilder.ShapeConfig(mu=0.4)
        wall = newton.ModelBuilder.ShapeConfig(mu=0.6)
        self.bucket = b.add_body(xform=wp.transform(p=wp.vec3(-2.2, 0.0, 1.2)), mass=400.0, is_kinematic=True)
        hl, hw, hh = BUCKET_L / 2, BUCKET_W / 2, BUCKET_H / 2
        b.add_shape_box(self.bucket, xform=wp.transform(p=wp.vec3(-hl, 0, 0)), hx=PLATE / 2, hy=hw, hz=hh, cfg=steel)  # back
        b.add_shape_box(self.bucket, xform=wp.transform(p=wp.vec3(0, 0, -hh)), hx=hl, hy=hw, hz=PLATE / 2, cfg=steel)  # bottom
        b.add_shape_box(self.bucket, xform=wp.transform(p=wp.vec3(0, hw, 0)), hx=hl, hy=PLATE / 2, hz=hh, cfg=steel)  # side
        b.add_shape_box(self.bucket, xform=wp.transform(p=wp.vec3(0, -hw, 0)), hx=hl, hy=PLATE / 2, hz=hh, cfg=steel)  # side
        zx, zy = ZONE_HALF
        t = 0.2
        for px, py, hx, hy in [(zx + t, 0, t, zy + 2 * t), (-zx - t, 0, t, zy + 2 * t), (0, zy + t, zx, t), (0, -zy - t, zx, t)]:
            b.add_shape_box(-1, xform=wp.transform(p=wp.vec3(px, py, WALL_HEIGHT / 2)), hx=hx, hy=hy, hz=WALL_HEIGHT / 2, cfg=wall)
        b.add_ground_plane(cfg=wall)
        self.model = b.finalize()

        # sand bed
        sb = newton.ModelBuilder()
        SolverImplicitMPM.register_custom_attributes(sb)
        lo = np.array([-zx, -zy, 0.0])
        hi = np.array([zx, zy, BED_DEPTH])
        res = np.array(np.ceil(ppc * (hi - lo) / voxel), dtype=int)
        cell = (hi - lo) / res
        self.radius = float(np.max(cell) * 0.5)
        self.particle_mass = float(np.prod(cell) * density)
        sb.add_particle_grid(
            pos=wp.vec3(lo),
            rot=wp.quat_identity(),
            vel=wp.vec3(0.0),
            dim_x=int(res[0]) + 1,
            dim_y=int(res[1]) + 1,
            dim_z=int(res[2]) + 1,
            cell_x=float(cell[0]),
            cell_y=float(cell[1]),
            cell_z=float(cell[2]),
            mass=self.particle_mass,
            jitter=2.0 * self.radius,
            radius_mean=self.radius,
            custom_attributes={"mpm:friction": friction},
        )
        self.sand_model = sb.finalize()
        self.n_particles = self.sand_model.particle_count

        cfg = SolverImplicitMPM.Config()
        cfg.voxel_size = voxel
        cfg.max_iterations = iterations
        cfg.critical_fraction = 0.0
        cfg.strain_basis = "P0"
        self.solver = SolverImplicitMPM(self.sand_model, config=cfg)
        self.solver.setup_collider(model=self.model)
        self.collider_body = self.solver.collider_body_index.numpy()

        self.n_gamma = mckyes_n_gamma(math.atan(friction), math.atan(0.4))

        # buffers
        self.hm = wp.zeros(HM_NX * HM_NY, dtype=float)
        self.moving = wp.zeros(MAX_SEND_PARTICLES, dtype=wp.vec3)
        self.counter = wp.zeros(1, dtype=int)
        self.fill_counter = wp.zeros(1, dtype=int)

        self.reset()

    # -------------------------------------------------------------- state
    def reset(self):
        rigid_state = self.model.state()
        self.state = self.sand_model.state()
        self.state.body_q = wp.clone(rigid_state.body_q)
        self.state.body_qd = wp.zeros_like(rigid_state.body_qd)
        self.state.body_f = wp.zeros_like(rigid_state.body_f)
        self.pos = np.array([-2.2, 0.0, 1.2])
        self.curl = 0.0  # rad; positive = mouth tipped up (holding), negative = teeth down / dumping
        self.target_pos = self.pos.copy()
        self.target_curl = self.curl
        self.sim_time = 0.0
        self.step_ms = 0.0
        self.force = np.zeros(3)
        self.fill_kg = 0.0
        self.cut_depth = 0.0
        self.fee_ref = 0.0
        self.auto = False
        self.auto_t = 0.0
        self.hm_np = np.zeros(HM_NX * HM_NY, dtype=np.float32)
        self._apply_bucket(np.zeros(3), 0.0)

    def set_target(self, x=None, y=None, z=None, curl_deg=None):
        if x is not None:
            self.target_pos[0] = float(np.clip(x, -2.6, 2.6))
        if y is not None:
            self.target_pos[1] = float(np.clip(y, -1.9, 1.9))
        if z is not None:
            self.target_pos[2] = float(np.clip(z, 0.25, 2.2))
        if curl_deg is not None:
            self.target_curl = math.radians(float(np.clip(curl_deg, -110, 80)))

    # automatic backhoe-style cycle in the zone frame: (t, x, y, z, curl_deg)
    CYCLE = [
        (0.0, -2.2, 0.0, 1.2, -20),
        (2.0, -2.0, 0.0, 0.62, -35),  # penetrate
        (6.5, 0.6, 0.0, 0.55, -15),  # drag toward the machine (+x)
        (8.0, 0.8, 0.0, 0.62, 55),  # curl
        (10.0, 1.0, 0.0, 1.6, 55),  # lift
        (12.0, 2.2, 0.0, 1.6, 55),  # swing to dump spot
        (14.0, 2.2, 0.0, 1.4, -100),  # dump
        (16.0, 0.0, 0.0, 1.6, -20),
        (18.0, -2.2, 0.0, 1.2, -20),  # back to start
    ]

    def _auto_targets(self):
        period = self.CYCLE[-1][0]
        t = self.auto_t % period
        for a, b in zip(self.CYCLE, self.CYCLE[1:]):
            if a[0] <= t <= b[0]:
                s = (t - a[0]) / (b[0] - a[0])
                vals = [a[k] + s * (b[k] - a[k]) for k in range(1, 5)]
                self.set_target(*vals)
                return

    # -------------------------------------------------------------- stepping
    def _apply_bucket(self, lin_vel, ang_vel):
        q = wp.quat_from_axis_angle(wp.vec3(0.0, 1.0, 0.0), -self.curl)
        body_q = self.state.body_q.numpy()
        body_q[self.bucket] = [*self.pos, q[0], q[1], q[2], q[3]]
        self.state.body_q.assign(body_q)
        body_qd = np.zeros((self.model.body_count, 6), dtype=np.float32)
        body_qd[self.bucket, :3] = lin_vel
        body_qd[self.bucket, 3:] = [0.0, -ang_vel, 0.0]
        self.state.body_qd.assign(body_qd)

    def step(self):
        if self.auto:
            self.auto_t += self.dt
            self._auto_targets()
        # move bucket toward its target with speed limits (kinematic, like a stiff hydraulic arm)
        d = self.target_pos - self.pos
        dist = float(np.linalg.norm(d))
        max_d = BUCKET_MAX_SPEED * self.dt
        delta = d if dist <= max_d else d * (max_d / dist)
        dc = self.target_curl - self.curl
        max_c = BUCKET_MAX_ROT * self.dt
        dcurl = float(np.clip(dc, -max_c, max_c))
        self.pos = self.pos + delta
        self.curl += dcurl
        self._apply_bucket(delta / self.dt, dcurl / self.dt)

        wp.synchronize()
        t0 = time.perf_counter()
        self.solver.step(self.state, self.state, contacts=None, control=None, dt=self.dt)
        wp.synchronize()
        self.step_ms = 0.9 * self.step_ms + 0.1 * (time.perf_counter() - t0) * 1000 if self.sim_time else (time.perf_counter() - t0) * 1000
        self.sim_time += self.dt


    # -------------------------------------------------------------- outputs
    def measure(self):
        # soil force on the bucket (impulses from the last MPM step)
        imp, _pos, cid = self.solver.collect_collider_impulses(self.state)
        imp = imp.numpy()
        cid = cid.numpy()
        valid = (cid >= 0) & (cid < len(self.collider_body))
        on_bucket = np.zeros(len(cid), dtype=bool)
        on_bucket[valid] = self.collider_body[cid[valid]] == self.bucket
        self.force = imp[on_bucket].sum(axis=0) / self.dt if on_bucket.any() else np.zeros(3)

        q = self.state.particle_q
        self.hm.fill_(0.0)
        wp.launch(
            heightmap_kernel,
            dim=self.n_particles,
            inputs=[q, self.state.particle_qd, SURFACE_MAX_SPEED, self.radius, -ZONE_HALF[0], -ZONE_HALF[1], HM_CELL, HM_NX, HM_NY, self.hm],
        )
        self.counter.zero_()
        wp.launch(
            compact_moving_kernel,
            dim=self.n_particles,
            inputs=[q, self.state.particle_qd, MOVING_SPEED, MAX_SEND_PARTICLES, self.counter, self.moving],
        )
        self.fill_counter.zero_()
        xf = wp.transform(wp.vec3(*self.pos), wp.quat_from_axis_angle(wp.vec3(0.0, 1.0, 0.0), -self.curl))
        wp.launch(
            bucket_fill_kernel,
            dim=self.n_particles,
            inputs=[q, xf, BUCKET_L / 2 - PLATE, BUCKET_W / 2 - PLATE, BUCKET_H / 2, self.fill_counter],
        )
        self.hm_np = self.hm.numpy()
        n_moving = min(int(self.counter.numpy()[0]), MAX_SEND_PARTICLES)
        moving = self.moving.numpy()[:n_moving]
        self.fill_kg = float(self.fill_counter.numpy()[0]) * self.particle_mass

        # cutting depth at the bucket's cutting edge, and the McKyes reference force for it
        edge_local = np.array([BUCKET_L / 2, 0.0, -BUCKET_H / 2])
        c, s = math.cos(self.curl), math.sin(self.curl)
        # rotation about y by -curl
        edge = self.pos + np.array([c * edge_local[0] - s * edge_local[2], 0.0, s * edge_local[0] + c * edge_local[2]])
        ix = int((edge[0] + ZONE_HALF[0]) / HM_CELL)
        iy = int((edge[1] + ZONE_HALF[1]) / HM_CELL)
        # surface just ahead of the edge (the uncut ground); the cell under the bucket is already cut
        ix_ahead = min(ix + 3, HM_NX - 1)
        surface = float(self.hm_np[iy * HM_NX + ix_ahead]) if 0 <= ix < HM_NX and 0 <= iy < HM_NY else 0.0
        # the reference has no surcharge term, so measure depth from the original bed level at most
        surface = min(surface, BED_DEPTH)
        self.cut_depth = max(surface - edge[2], 0.0)
        self.fee_ref = self.density * G * self.cut_depth**2 * BUCKET_W * self.n_gamma
        return moving

    def frame_bytes(self, frame_no, moving):
        ox, oy, oz = PIT_CENTER_THREE
        # bucket pose in three.js coordinates
        bx, by, bz = self.pos
        bucket_three = (bx + ox, bz + oy, oz - by)
        # rotation about zone +y by -curl  ->  about three -z by -curl  ==  about three +z by +curl
        half = self.curl / 2
        quat_three = (0.0, 0.0, math.sin(half), math.cos(half))
        f = self.force
        force_three = (f[0], f[2], -f[1])
        header = struct.pack(
            "<4sIfffI3f4f3ffff",
            b"SOIL",
            frame_no,
            self.sim_time,
            self.step_ms,
            min(self.dt * 1000 / max(self.step_ms, 1e-3), 99.0),
            self.n_particles,
            *bucket_three,
            *quat_three,
            *force_three,
            self.fill_kg,
            self.cut_depth,
            self.fee_ref,
        )
        # heightmap: zone (ix, iy) -> three (x0 + ix*cell, z0 - iy*cell); heights in mm above (oy - 1)
        hm = np.clip((self.hm_np + 1.0) * 1000.0, 0, 65535).astype(np.uint16)
        hm_header = struct.pack("<HHfff", HM_NX, HM_NY, ox - ZONE_HALF[0] + HM_CELL / 2, oz + ZONE_HALF[1] - HM_CELL / 2, HM_CELL)
        # moving particles quantised in a box around the zone (three coords)
        lo = np.array([ox - 4.0, oy - 0.5, oz - 4.0], dtype=np.float32)
        span = 8.0
        if len(moving):
            t3 = np.stack([moving[:, 0] + ox, moving[:, 2] + oy, oz - moving[:, 1]], axis=1)
            qp = np.clip((t3 - lo) / span * 65535.0, 0, 65535).astype(np.uint16)
        else:
            qp = np.zeros((0, 3), dtype=np.uint16)
        p_header = struct.pack("<I4f", len(qp), *lo, span)
        return header + hm_header + hm.tobytes() + p_header + qp.tobytes()

    def stats(self):
        return {
            "sim_time_s": round(self.sim_time, 2),
            "particles": self.n_particles,
            "step_ms": round(self.step_ms, 2),
            "realtime_factor": round(self.dt * 1000 / max(self.step_ms, 1e-3), 2),
            "bucket": [round(float(v), 3) for v in self.pos],
            "curl_deg": round(math.degrees(self.curl), 1),
            "force_N": [round(float(v), 1) for v in self.force],
            "fill_kg": round(self.fill_kg, 1),
            "cut_depth_m": round(self.cut_depth, 3),
            "mckyes_N": round(self.fee_ref, 1),
        }


# ------------------------------------------------------------------ server
async def serve(sim, host, port):
    import websockets

    clients = set()

    async def handler(ws):
        clients.add(ws)
        await ws.send(json.dumps({"type": "hello", "particles": sim.n_particles, "voxel": sim.voxel, "particle_mass": sim.particle_mass}))
        try:
            async for msg in ws:
                try:
                    cmd = json.loads(msg)
                except ValueError:
                    continue
                kind = cmd.get("type")
                if kind == "target":
                    sim.auto = False
                    sim.set_target(cmd.get("x"), cmd.get("y"), cmd.get("z"), cmd.get("curl"))
                elif kind == "auto":
                    sim.auto = bool(cmd.get("on", True))
                    sim.auto_t = 0.0
                elif kind == "reset":
                    sim.reset()
        finally:
            clients.discard(ws)

    async def sim_loop():
        frame = 0
        next_send = 0.0
        wall0 = time.perf_counter()
        while True:
            sim.step()
            if sim.sim_time < next_send - 1.0:  # simulation was reset: restart the clocks
                next_send = 0.0
                wall0 = time.perf_counter() - sim.sim_time
            # keep the simulation at or below real time
            ahead = sim.sim_time - (time.perf_counter() - wall0)
            if ahead > 0:
                await asyncio.sleep(ahead)
            elif ahead < -0.5:
                wall0 = time.perf_counter() - sim.sim_time  # fell behind: don't try to catch up
            if sim.sim_time >= next_send:
                next_send = sim.sim_time + 1 / 30
                moving = sim.measure()
                if clients:
                    data = sim.frame_bytes(frame, moving)
                    frame += 1
                    for ws in list(clients):
                        if ws.transport is not None and ws.transport.get_write_buffer_size() > 4_000_000:
                            continue  # slow client: drop this frame for it
                        try:
                            await ws.send(data)
                        except Exception as err:  # noqa: BLE001
                            print(f"dropping client: {err!r}", flush=True)
                            clients.discard(ws)
                if clients and frame % 150 == 0:
                    print(json.dumps(sim.stats()), flush=True)
            await asyncio.sleep(0)

    async with websockets.serve(handler, host, port, max_size=2**20, compression=None):
        print(f"soil server on ws://{host}:{port}  particles={sim.n_particles}", flush=True)
        await sim_loop()


def bench(sim, seconds):
    sim.auto = True
    log = []
    t_wall = time.perf_counter()
    steps = int(seconds / sim.dt)
    for k in range(steps):
        sim.step()
        if k % 6 == 0:
            sim.measure()
            log.append({"t": round(sim.sim_time, 2), "x": round(sim.pos[0], 2), "z": round(sim.pos[2], 2), "curl": round(math.degrees(sim.curl)), "Fx": round(float(sim.force[0])), "Fz": round(float(sim.force[2])), "fill_kg": round(sim.fill_kg, 1), "cut_m": round(sim.cut_depth, 3), "mckyes_N": round(sim.fee_ref)})
    wall = time.perf_counter() - t_wall
    moving = sim.measure()
    frame = sim.frame_bytes(0, moving)
    return {
        "particles": sim.n_particles,
        "voxel_m": sim.voxel,
        "sim_s": round(sim.sim_time, 2),
        "wall_s": round(wall, 2),
        "realtime_factor_incl_outputs": round(sim.sim_time / wall, 3),
        "mpm_step_ms": round(sim.step_ms, 2),
        "frame_bytes": len(frame),
        "moving_particles_last": len(moving),
        "log": log,
    }


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=8765)
    ap.add_argument("--voxel", type=float, default=0.08)
    ap.add_argument("--ppc", type=int, default=2)
    ap.add_argument("--iterations", type=int, default=30)
    ap.add_argument("--bench", type=float, default=None, help="headless benchmark seconds")
    ap.add_argument("--out", default=None)
    args = ap.parse_args()
    sim = SoilSim(voxel=args.voxel, ppc=args.ppc, iterations=args.iterations)
    if args.bench:
        res = bench(sim, args.bench)
        if args.out:
            with open(args.out, "w") as fh:
                json.dump(res, fh, indent=1)
        summary = {k: v for k, v in res.items() if k != "log"}
        print(json.dumps(summary, indent=1))
        return
    asyncio.run(serve(sim, args.host, args.port))


if __name__ == "__main__":
    main()
