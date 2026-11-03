"""Simulated sensors on the excavator: a spinning lidar and a depth camera on the cab, and an IMU in the
upper structure. Rays are cast on the GPU against everything the simulator has:
    - soil: the resting (dormant) soil columns, and the active MPM particles (spheres) near the bucket,
      in the air and in the truck
    - ground: the pit floor, the pit sides and walls, the site ground
    - machine: the upper structure, boom, stick and bucket collision boxes, and the crawler tracks
    - truck: the bed plates, chassis, cab and wheels
Every return carries a semantic label (LABELS), which real sensors do not give, for training perception.

The geometry is the simulator's collision geometry: boxes for the machine and truck (close to the real
shapes in size, not in detail) and the soil at its 4 cm particle / 8 cm column resolution. Ranges are exact
(no noise model, no beam divergence or multiple returns); add noise in your own pipeline if needed.

    import sensors
    rig = sensors.SensorRig(sim)           # sim: excavator_server.ExcavatorSim
    scan = rig.lidar()                     # dict: points (N, 3) in the lidar frame, ranges, labels, ...
    image = rig.depth_camera()             # dict: depth (H, W) in m along the optical axis, labels
    imu = rig.imu()                        # dict: angular_velocity, linear_acceleration (specific force)

Frames: the house frame has x forward (along the arm), y left, z up, origin at the swing joint; the lidar
frame is the house frame moved to the cab roof. The camera frame is OpenCV (x right, y down, z forward).
"""

import math

import numpy as np
import warp as wp

import excavator_server as ex
from soil_zone import ACTIVE, CELL, SPACING

LABELS = {0: "none", 1: "ground", 2: "soil", 3: "machine", 4: "truck"}
NONE, GROUND, SOIL, MACHINE, TRUCK = 0, 1, 2, 3, 4

# mounts in the house frame (m): the cab box is centred at (0.6, 0.75, 1.35), half size (0.55, 0.5, 0.75)
LIDAR_MOUNT = np.array([1.1, 0.9, 2.6])  # on a 0.5 m mast at the cab's front left corner, clear of the roof
CAMERA_MOUNT = np.array([1.25, 0.75, 1.95])  # just ahead of the cab front, near its top
CAMERA_PITCH = math.radians(25.0)  # down from the horizontal
IMU_MOUNT = np.array([0.0, 0.0, 0.3])
STEP = 0.5 * SPACING  # ray-march step through soil (m)


@wp.func
def ray_box(o: wp.vec3, d: wp.vec3, xf: wp.transform, half: wp.vec3):
    """Distance along the ray to an oriented box (1e30: miss)."""
    q = wp.transform_get_rotation(xf)
    ol = wp.quat_rotate_inv(q, o - wp.transform_get_translation(xf))
    dl = wp.quat_rotate_inv(q, d)
    t0 = float(-1.0e30)
    t1 = float(1.0e30)
    for k in range(3):
        if wp.abs(dl[k]) < 1.0e-9:
            if ol[k] < -half[k] or ol[k] > half[k]:
                return 1.0e30
        else:
            a = (-half[k] - ol[k]) / dl[k]
            b = (half[k] - ol[k]) / dl[k]
            t0 = wp.max(t0, wp.min(a, b))
            t1 = wp.min(t1, wp.max(a, b))
    if t1 < wp.max(t0, 0.0):
        return 1.0e30
    return wp.max(t0, 0.0)


@wp.func
def soil_top(p: wp.vec3, surf: wp.array[float], x0: float, y0: float, nx: int, ny: int):
    ix = int(wp.floor((p[0] - x0) / CELL))
    iy = int(wp.floor((p[1] - y0) / CELL))
    if ix < 0 or iy < 0 or ix >= nx or iy >= ny:
        return -1.0e30
    return surf[iy * nx + ix]


@wp.func
def in_particles(p: wp.vec3, grid: wp.uint64, pq: wp.array[wp.vec3], flags: wp.array[wp.int32], r: float):
    query = wp.hash_grid_query(grid, p, r)
    i = int(0)
    while wp.hash_grid_query_next(query, i):
        if (flags[i] & ACTIVE) != 0:
            if wp.length(pq[i] - p) < r:
                return True
    return False


@wp.kernel
def raycast_kernel(
    origin: wp.array[wp.vec3],
    direction: wp.array[wp.vec3],
    max_range: float,
    box_xf: wp.array[wp.transform],
    box_half: wp.array[wp.vec3],
    box_label: wp.array[int],
    surf: wp.array[float],
    sx0: float,
    sy0: float,
    snx: int,
    sny: int,
    grid: wp.uint64,
    pq: wp.array[wp.vec3],
    flags: wp.array[wp.int32],
    p_radius: float,
    p_lo: wp.vec3,
    p_hi: wp.vec3,
    dist: wp.array[float],
    label: wp.array[int],
):
    k = wp.tid()
    o = origin[k]
    d = direction[k]
    best = max_range
    lab = int(NONE)
    for b in range(box_xf.shape[0]):
        t = ray_box(o, d, box_xf[b], box_half[b])
        if t < best:
            best = t
            lab = box_label[b]
    if d[2] < 0.0:  # pit floor plane z = 0
        t = -o[2] / d[2]
        if t > 0.0 and t < best:
            best = t
            lab = int(GROUND)
    # march for soil up to the nearest solid hit
    t = float(0.0)
    while t < best:
        p = o + t * d
        top = soil_top(p, surf, sx0, sy0, snx, sny)
        hit = p[2] <= top
        if not hit and p[0] > p_lo[0] and p[1] > p_lo[1] and p[2] > p_lo[2] and p[0] < p_hi[0] and p[1] < p_hi[1] and p[2] < p_hi[2]:
            hit = in_particles(p, grid, pq, flags, p_radius)
        if hit:
            # refine between the last free point and this one
            lo = wp.max(t - STEP, 0.0)
            hi = t
            for _it in range(5):
                mid = 0.5 * (lo + hi)
                pm = o + mid * d
                h = pm[2] <= soil_top(pm, surf, sx0, sy0, snx, sny) or in_particles(pm, grid, pq, flags, p_radius)
                if h:
                    hi = mid
                else:
                    lo = mid
            best = hi
            lab = int(SOIL)
            break
        t += STEP
    dist[k] = wp.where(lab == int(NONE), 0.0, best)
    label[k] = lab


def quat_matrix(q):
    x, y, z, w = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def compose(p, q, lp, lq):
    """World pose of a shape at local (lp, lq) on a body at (p, q); quaternions (x, y, z, w)."""
    R = quat_matrix(q)
    x1, y1, z1, w1 = q
    x2, y2, z2, w2 = lq
    qq = np.array([w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2, w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
                   w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2, w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2])
    return p + R @ lp, qq


def house_pose(sim):
    """Upper structure (URDF body_link) pose in the zone frame: position and rotation matrix."""
    bq = sim.state_0.body_q.numpy()[sim.links["house"]]
    return bq[:3].astype(float), quat_matrix(bq[3:])


class SensorRig:
    def __init__(self, sim, lidar_channels=32, lidar_columns=1024, lidar_fov=(-45.0, 45.0), lidar_range=40.0,
                 camera_size=(320, 240), camera_hfov=90.0, camera_range=30.0):
        self.sim = sim
        m = sim.model
        self.shape_type = m.shape_type.numpy()
        self.shape_body = m.shape_body.numpy()
        self.shape_xf = m.shape_transform.numpy()
        self.shape_half = m.shape_scale.numpy()
        boxes = np.flatnonzero(self.shape_type == 7)  # newton.GeoType.BOX
        self.boxes = boxes
        label = []
        for i in boxes:
            c = self.shape_xf[i][:3]
            if self.shape_body[i] >= 0:
                label.append(MACHINE)
            elif ex.BED_LO[0] - 0.3 < c[0] < ex.BED_HI[0] + 0.3 and ex.BED_LO[1] - 0.3 < c[1] < ex.BED_HI[1] + 0.3 and c[2] > ex.BED_FLOOR_Z - 0.3:
                label.append(TRUCK)
            else:
                label.append(GROUND)
        # parts that the browser draws but the physics does not have: crawler tracks, truck chassis, cab, wheels
        g, (cx, cy) = ex.GROUND_Z, ex.BED_CENTER
        extra = [((ex.SWING_POS[0] + s * 1.2, 0.0, g + 0.45), (0.35, 2.3, 0.45), MACHINE) for s in (-1, 1)]
        extra += [((ex.SWING_POS[0], 0.0, g + 0.75), (0.9, 1.4, 0.25), MACHINE)]
        extra += [((cx + 0.9, cy, 3.6), (3.4, 0.5, 0.3), TRUCK), ((cx + ex.BED_L / 2 + 1.15, cy, g + 1.7), (0.95, 1.2, 1.1), TRUCK)]
        extra += [((cx + ax, cy + s * (ex.BED_W / 2 - 0.25), g + 0.55), (0.55, 0.225, 0.55), TRUCK) for ax in (-1.3, 0.0, ex.BED_L / 2 + 1.1) for s in (-1, 1)]
        self.extra = extra
        self.box_label = wp.array(label + [e[2] for e in extra], dtype=int)
        self.box_half = wp.array(np.concatenate([self.shape_half[boxes], np.array([e[1] for e in extra])]).astype(np.float32), dtype=wp.vec3)
        n_boxes = len(boxes) + len(extra)
        self.box_xf_np = np.zeros((n_boxes, 7), dtype=np.float32)
        for j, e in enumerate(extra):
            self.box_xf_np[len(boxes) + j] = [*e[0], 0, 0, 0, 1]
        self.box_xf = wp.zeros(n_boxes, dtype=wp.transform)
        # dormant soil surface on the zone's column grid (the ground region covers the pit, the machine and the truck)
        _, self.sx0, self.sy0, self.snx, self.sny, _ = ex.GROUND_REGION
        self.surf = wp.zeros(self.snx * self.sny, dtype=float)
        self.grid = wp.HashGrid(128, 128, 64)
        self.p_radius = 0.6 * SPACING
        # lidar beams (lidar frame) and camera rays (camera frame, OpenCV)
        el = np.radians(np.linspace(lidar_fov[0], lidar_fov[1], lidar_channels))
        az = np.linspace(0.0, 2 * math.pi, lidar_columns, endpoint=False)
        E, A = np.meshgrid(el, az, indexing="ij")
        self.lidar_dirs = np.stack([np.cos(E) * np.cos(A), np.cos(E) * np.sin(A), np.sin(E)], axis=-1).reshape(-1, 3)
        self.lidar_shape, self.lidar_range = (lidar_channels, lidar_columns), lidar_range
        w, h = camera_size
        f = (w / 2) / math.tan(math.radians(camera_hfov) / 2)
        self.camera_K = np.array([[f, 0, w / 2], [0, f, h / 2], [0, 0, 1]])
        u, v = np.meshgrid(np.arange(w) + 0.5, np.arange(h) + 0.5)
        rays = np.stack([(u - w / 2) / f, (v - h / 2) / f, np.ones_like(u)], axis=-1).reshape(-1, 3)
        self.camera_rays_cv = rays  # unnormalised: z = 1, so depth along the axis = distance / |ray|
        self.camera_shape, self.camera_range = (h, w), camera_range
        # camera (OpenCV) axes in the house frame: z forward and pitched down, x right (-y house), y down
        cp, sp = math.cos(CAMERA_PITCH), math.sin(CAMERA_PITCH)
        self.R_house_cam = np.array([[0.0, -sp, cp], [-1.0, 0.0, 0.0], [0.0, -cp, -sp]])
        self.imu_prev = None

    # ------------------------------------------------------------------ scene update
    def house_pose(self):
        return house_pose(self.sim)

    def _refresh(self):
        sim = self.sim
        bq = sim.state_0.body_q.numpy()
        for j, i in enumerate(self.boxes):
            lp, lq = self.shape_xf[i][:3], self.shape_xf[i][3:]
            b = self.shape_body[i]
            if b >= 0:
                p, q = compose(bq[b][:3], bq[b][3:], lp, lq)
            else:
                p, q = lp, lq
            self.box_xf_np[j] = [*p, *q]
        self.box_xf.assign(self.box_xf_np)
        self.surf.fill_(-1.0e30)
        sim.zones.merge_heightmap(self.surf, self.sx0, self.sy0, CELL, self.snx, self.sny)
        q = sim.sand.particle_q
        self.grid.build(points=q, radius=self.p_radius)
        act = sim.zones.active_np
        if act.any():
            pts = q.numpy()[act]
            self.p_lo, self.p_hi = pts.min(0) - self.p_radius, pts.max(0) + self.p_radius
        else:
            self.p_lo, self.p_hi = np.zeros(3), np.zeros(3)

    def _cast(self, origins, dirs, max_range):
        n = len(dirs)
        o = wp.array(np.broadcast_to(origins, (n, 3)).astype(np.float32), dtype=wp.vec3)
        d = wp.array(dirs.astype(np.float32), dtype=wp.vec3)
        dist, label = wp.zeros(n, dtype=float), wp.zeros(n, dtype=int)
        wp.launch(raycast_kernel, dim=n, inputs=[o, d, max_range, self.box_xf, self.box_half, self.box_label, self.surf,
                                                 self.sx0, self.sy0, self.snx, self.sny, self.grid.id, self.sim.sand.particle_q,
                                                 self.sim.sand_model.particle_flags, self.p_radius, wp.vec3(*self.p_lo), wp.vec3(*self.p_hi), dist, label])
        return dist.numpy(), label.numpy()

    # ------------------------------------------------------------------ sensors
    def lidar(self):
        """One full revolution, all beams at once. Returns points in the lidar frame (hits only), ranges
        (channels x columns, 0 = no return), labels (same shape) and the lidar pose in the zone frame."""
        self._refresh()
        p, R = self.house_pose()
        origin = p + R @ LIDAR_MOUNT
        dist, label = self._cast(origin, self.lidar_dirs @ R.T, self.lidar_range)
        hit = label > 0
        return {"points": (self.lidar_dirs[hit] * dist[hit, None]).astype(np.float32), "point_labels": label[hit].astype(np.uint8),
                "ranges": dist.reshape(self.lidar_shape).astype(np.float32), "labels": label.reshape(self.lidar_shape).astype(np.uint8),
                "position": origin, "rotation": R}

    def depth_camera(self):
        """Depth image (m along the optical axis, 0 = nothing within range), label image, intrinsics K and
        the camera pose (OpenCV axes) in the zone frame."""
        self._refresh()
        p, R = self.house_pose()
        origin = p + R @ CAMERA_MOUNT
        Rc = R @ self.R_house_cam
        norm = np.linalg.norm(self.camera_rays_cv, axis=1)
        dist, label = self._cast(origin, (self.camera_rays_cv / norm[:, None]) @ Rc.T, self.camera_range)
        depth = np.where(label > 0, dist / norm, 0.0)
        return {"depth": depth.reshape(self.camera_shape).astype(np.float32), "labels": label.reshape(self.camera_shape).astype(np.uint8),
                "K": self.camera_K, "position": origin, "rotation": Rc}

    def imu(self, dt=None):
        """Angular velocity (rad/s) and specific force (m/s2, gravity included as an accelerometer reads it)
        in the house frame at IMU_MOUNT; acceleration by differencing the mount's velocity since the last call
        (dt: time since then; first call: zero acceleration)."""
        sim = self.sim
        h = sim.links["house"]
        p, R = self.house_pose()
        qd = sim.state_0.body_qd.numpy()[h]
        com = p + R @ sim.model.body_com.numpy()[h]
        w = qd[3:6].astype(float)
        mount = p + R @ IMU_MOUNT
        v = qd[:3].astype(float) + np.cross(w, mount - com)
        t = sim.sim_time
        a = np.zeros(3) if self.imu_prev is None else (v - self.imu_prev[1]) / max(dt or (t - self.imu_prev[0]), 1e-9)
        self.imu_prev = (t, v)
        f = a - np.array([0.0, 0.0, -ex.G])
        return {"angular_velocity": (R.T @ w).astype(np.float32), "linear_acceleration": (R.T @ f).astype(np.float32), "time": t}
