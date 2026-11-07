"""Moving particle zone + heightfield soil for real-time MPM digging.

Resting soil away from the bucket is not simulated: it is stored per 8 cm column as a particle count
(mass-conserving) and acts on the active sand as a rigid heightfield collider. Near the bucket, columns
"wake": their count is turned back into MPM particles on a 4 cm lattice. Columns whose particles are at
rest and away from the bucket "sleep": their particles are counted back into the column and parked.

    soil region   grid of columns over a soil-bearing area (the pit with its slopes and the ground
                  around it, the truck bed); each column has a floor height (the lowest point of the
                  ground under it)
    count[c]      dormant particles in column c; its top is floor + count * layer[c], where layer is
                  LAYER_PER_PARTICLE for soil in place (bank) and (1 + swell) times that in the regions
                  holding dug, loose soil (the vessel, the haul road, the spoil area)
    pool          fixed-size particle array; inactive slots are parked in a compact storage block far
                  away (Newton's sparse grid allocates voxels for every particle, so parked particles
                  are packed ~26 per voxel to keep that cost small)
    collider      one Warp mesh over all regions: a top quad per column plus vertical walls between
                  neighbouring columns and at region borders. Columns without dormant soil sit at their
                  floor (at or below the real ground), so the mesh is a wall exactly at the zone boundary
                  and a floor under dormant soil. Where the mesh is inside the ground it does nothing:
                  Newton's MPM takes the most-penetrated collider.

Waking is partial: a column becomes particles only from its top down to just below the bucket (rising
1:1 away from it), and the soil underneath stays dormant as a base, so a deep bed costs little more than
a shallow one. Dormant soil standing steeper than a 0.2 m drop to a neighbour wakes on its own
(avalanche), so cut faces slump to the angle of repose instead of standing as rigid walls.

Bulking: dug soil takes more room than soil in place (swell: sand about 12 %, loam 25 %, clay 35 %). A
particle woken from a bank column is in place; once it has moved DUG_M from where it woke, it is dug, and
its MPM volume grows by (1 + swell), so with the solver's packing limit (critical fraction) the loose soil
spreads out at lower density, mass unchanged. Loose particles keep that volume when they sleep in a loose
region and wake again; one sleeping in the pit is counted back as soil in place.

Dormant columns are as rigid as the pit walls. That is the approximation: soil outside the zone does not
fail, so the zone has to cover the bucket's failure wedge (WAKE_MARGIN).
"""

import math

import numpy as np
import warp as wp

import newton

SPACING = 0.04  # particle lattice (m): 2 x 2 columns per 8 cm cell
CELL = 2 * SPACING
PER_LAYER = 4
LAYER_PER_PARTICLE = SPACING / PER_LAYER  # column rise per particle (m)
TOP_LIMIT = 4.0  # m above the floor: particles higher than this never sleep into a column (a dumped cohesive load stands over 2 m)
BELOW_FLOOR = 0.3  # m: particles this far under their column floor belong to no column
FELL_THROUGH = 0.6  # m: particles this far under the floor of every region above them fell through the ground
PARK_SPACING = 0.027  # parked particles: about 26 per 8 cm voxel
PARK_ORIGIN = (-30.0, -30.0, -30.0)

ACTIVE = int(newton.ParticleFlags.ACTIVE)


@wp.struct
class Regions:
    lo: wp.array[wp.vec2]  # grid origin x0, y0
    nx: wp.array[int]
    ny: wp.array[int]
    offset: wp.array[int]
    floor: wp.array[float]  # per column


@wp.func
def column_of(p: wp.vec3, regions: Regions):
    """Column holding p, or -1. Regions are tried in order, so a raised region (the truck bed) listed
    first takes precedence over the ground below it."""
    for r in range(regions.nx.shape[0]):
        lo = regions.lo[r]
        ix = int(wp.floor((p[0] - lo[0]) / CELL))
        iy = int(wp.floor((p[1] - lo[1]) / CELL))
        if ix >= 0 and ix < regions.nx[r] and iy >= 0 and iy < regions.ny[r]:
            c = regions.offset[r] + iy * regions.nx[r] + ix
            f = regions.floor[c]
            if p[2] > f - BELOW_FLOOR and p[2] < f + TOP_LIMIT:
                return c
    return -1


@wp.kernel
def swell_dug_kernel(
    q: wp.array[wp.vec3],
    flags: wp.array[wp.int32],
    woke_at: wp.array[wp.vec3],
    loose: wp.array[int],
    dug_m: float,
    loose_volume: float,
    volume: wp.array[float],
):
    """Particles in place that have moved dug_m from where they woke are dug: loose, at the larger volume."""
    i = wp.tid()
    if (flags[i] & ACTIVE) == 0 or loose[i] != 0:
        return
    if wp.length(q[i] - woke_at[i]) > dug_m:
        loose[i] = 1
        volume[i] = loose_volume


@wp.kernel
def mark_spawned_kernel(
    slots: wp.array[int],
    q: wp.array[wp.vec3],
    is_loose: wp.array[int],
    loose_volume: float,
    woke_at: wp.array[wp.vec3],
    loose: wp.array[int],
    volume: wp.array[float],
):
    """Newly woken or injected particles: remember where they started, and whether they are loose."""
    k = wp.tid()
    i = slots[k]
    woke_at[i] = q[i]
    loose[i] = is_loose[k]
    if is_loose[k] != 0:
        volume[i] = loose_volume


@wp.func
def fell_through(p: wp.vec3, regions: Regions):
    """True if p lies under at least one region and well below the floor of every region it lies under:
    it was pushed through the ground (under a raised region's collider, as under the truck bed)."""
    under = int(0)
    for r in range(regions.nx.shape[0]):
        lo = regions.lo[r]
        ix = int(wp.floor((p[0] - lo[0]) / CELL))
        iy = int(wp.floor((p[1] - lo[1]) / CELL))
        if ix >= 0 and ix < regions.nx[r] and iy >= 0 and iy < regions.ny[r]:
            if p[2] > regions.floor[regions.offset[r] + iy * regions.nx[r] + ix] - FELL_THROUGH:
                return False
            under = 1
    return under == 1


@wp.kernel
def column_stats_kernel(
    q: wp.array[wp.vec3],
    qd: wp.array[wp.vec3],
    flags: wp.array[wp.int32],
    regions: Regions,
    v_sleep: float,
    col_n: wp.array[int],
    col_fast: wp.array[int],
    col_vmax: wp.array[float],
    col_ztop: wp.array[float],
):
    i = wp.tid()
    if (flags[i] & ACTIVE) == 0:
        return
    c = column_of(q[i], regions)
    if c >= 0:
        v = wp.length(qd[i])
        wp.atomic_add(col_n, c, 1)
        if v >= v_sleep:
            wp.atomic_add(col_fast, c, 1)
        wp.atomic_max(col_vmax, c, v)
        wp.atomic_max(col_ztop, c, q[i][2])


@wp.func
def deactivate(
    i: int,
    flags: wp.array[wp.int32],
    mpm_flags: wp.array[wp.int32],
    mpm_material_flags: wp.array[wp.int32],
    mpm_material_volume: wp.array[float],
    q: wp.array[wp.vec3],
    qd: wp.array[wp.vec3],
    park: wp.array[wp.vec3],
    freed: wp.array[int],
    freed_n: wp.array[int],
):
    off = flags[i] & ~ACTIVE
    flags[i] = off
    mpm_flags[i] = off
    mpm_material_flags[i] = off
    mpm_material_volume[i] = 0.0
    q[i] = park[i]
    qd[i] = wp.vec3(0.0)
    freed[wp.atomic_add(freed_n, 0, 1)] = i


@wp.kernel
def absorb_kernel(
    q: wp.array[wp.vec3],
    qd: wp.array[wp.vec3],
    regions: Regions,
    sleep: wp.array[int],
    min_z: float,
    park: wp.array[wp.vec3],
    flags: wp.array[wp.int32],
    mpm_flags: wp.array[wp.int32],
    mpm_material_flags: wp.array[wp.int32],
    mpm_material_volume: wp.array[float],
    count: wp.array[int],
    freed: wp.array[int],
    freed_n: wp.array[int],
    lost: wp.array[int],
):
    """Sleep: particles in sleeping columns are added to the column count and parked. Particles that
    fell through the ground (below min_z, or well under every region above them) are removed and counted
    as lost."""
    i = wp.tid()
    if (flags[i] & ACTIVE) == 0:
        return
    p = q[i]
    if p[2] < min_z or fell_through(p, regions):
        wp.atomic_add(lost, 0, 1)
        deactivate(i, flags, mpm_flags, mpm_material_flags, mpm_material_volume, q, qd, park, freed, freed_n)
        return
    c = column_of(p, regions)
    if c < 0 or sleep[c] == 0:
        return
    wp.atomic_add(count, c, 1)
    deactivate(i, flags, mpm_flags, mpm_material_flags, mpm_material_volume, q, qd, park, freed, freed_n)


@wp.kernel
def remove_kernel(
    slots: wp.array[int],
    flags: wp.array[wp.int32],
    mpm_flags: wp.array[wp.int32],
    mpm_material_flags: wp.array[wp.int32],
    mpm_material_volume: wp.array[float],
    q: wp.array[wp.vec3],
    qd: wp.array[wp.vec3],
    park: wp.array[wp.vec3],
    freed: wp.array[int],
    freed_n: wp.array[int],
):
    """Take the given active particles out of the simulation (they are parked)."""
    i = slots[wp.tid()]
    deactivate(i, flags, mpm_flags, mpm_material_flags, mpm_material_volume, q, qd, park, freed, freed_n)


@wp.kernel
def set_velocity_kernel(slots: wp.array[int], vel: wp.array[wp.vec3], qd: wp.array[wp.vec3]):
    k = wp.tid()
    qd[slots[k]] = vel[k]


@wp.kernel
def spawn_kernel(
    slots: wp.array[int],
    pos: wp.array[wp.vec3],
    volume: float,
    flags: wp.array[wp.int32],
    mpm_flags: wp.array[wp.int32],
    mpm_material_flags: wp.array[wp.int32],
    mpm_material_volume: wp.array[float],
    q: wp.array[wp.vec3],
    qd: wp.array[wp.vec3],
    qd_grad: wp.array[wp.mat33],
    elastic_strain: wp.array[wp.mat33],
    jp: wp.array[float],
    stress: wp.array[wp.mat33],
    transform: wp.array[wp.mat33],
):
    """Wake: put fresh, unstressed particles into the given pool slots."""
    k = wp.tid()
    i = slots[k]
    on = flags[i] | ACTIVE
    flags[i] = on
    mpm_flags[i] = on
    mpm_material_flags[i] = on
    mpm_material_volume[i] = volume
    q[i] = pos[k]
    qd[i] = wp.vec3(0.0)
    qd_grad[i] = wp.mat33(0.0)
    elastic_strain[i] = wp.identity(3, float)
    jp[i] = 1.0
    stress[i] = wp.mat33(0.0)
    transform[i] = wp.identity(3, float)


@wp.kernel
def column_top_kernel(count: wp.array[int], floor_z: wp.array[float], layer: wp.array[float], points: wp.array[wp.vec3]):
    """Collider: each column's top quad at its dormant soil height (the floor when it has none)."""
    c = wp.tid()
    z = floor_z[c] + float(count[c]) * layer[c]
    for k in range(4):
        p = points[4 * c + k]
        points[4 * c + k] = wp.vec3(p[0], p[1], z)


@wp.kernel
def merge_heightmap_kernel(
    hm: wp.array[float],
    x0: float,
    y0: float,
    cell: float,
    nx: int,
    count: wp.array[int],
    layer: wp.array[float],
    regions: Regions,
):
    """Display heightmap: the higher of the particle surface and the dormant column top."""
    k = wp.tid()
    px = x0 + (float(k % nx) + 0.5) * cell
    py = y0 + (float(k // nx) + 0.5) * cell
    for r in range(regions.nx.shape[0]):
        lo = regions.lo[r]
        ix = int(wp.floor((px - lo[0]) / CELL))
        iy = int(wp.floor((py - lo[1]) / CELL))
        if ix >= 0 and ix < regions.nx[r] and iy >= 0 and iy < regions.ny[r]:
            c = regions.offset[r] + iy * regions.nx[r] + ix
            n = count[c]
            if n > 0:
                hm[k] = wp.max(hm[k], regions.floor[c] + float(n) * layer[c])
            return


@wp.kernel
def dormant_grains_kernel(
    count: wp.array[int],
    col_xy: wp.array[wp.vec2],
    floor_z: wp.array[float],
    layer: wp.array[float],
    show: wp.array[int],
    max_out: int,
    counter: wp.array[int],
    out: wp.array[wp.vec3],
):
    """Dormant soil that no display heightmap covers (spill on slopes and ground) is drawn as grains,
    one per particle on its column lattice, as the resting particles were before they slept."""
    c = wp.tid()
    n = count[c]
    if show[c] == 0 or n == 0:
        return
    base = wp.atomic_add(counter, 0, n)
    for k in range(n):
        slot = base + k
        if slot < max_out:
            m = k % 4
            out[slot] = wp.vec3(
                col_xy[c][0] + (float(m % 2) - 0.5) * SPACING,
                col_xy[c][1] + (float(m // 2) - 0.5) * SPACING,
                floor_z[c] + (float(k // 4) + 0.5) * SPACING * layer[c] / LAYER_PER_PARTICLE,
            )


def column_mesh(spec, floor):
    """Fixed-topology collider mesh: 4 own vertices per column (top quad), vertical quads between
    neighbouring columns and at region borders (to fixed vertices at the column floor). Each wall quad
    is wound so that its normal points toward the lower side, whichever side that is."""
    pts, tris = [], []
    for x0, y0, nx, ny, off in spec:  # column vertices first: column c owns vertices 4c..4c+3
        for iy in range(ny):
            for ix in range(nx):
                xa, ya, z = x0 + ix * CELL, y0 + iy * CELL, floor[off + iy * nx + ix]
                pts += [(xa, ya, z), (xa + CELL, ya, z), (xa + CELL, ya + CELL, z), (xa, ya + CELL, z)]

    def x_wall(a1, a2, b1, b2):  # shared edge at constant x; A on the -x side
        tris.extend([(a1, b2, a2), (a1, b1, b2)])

    def y_wall(a1, a2, b1, b2):  # shared edge at constant y; A on the -y side
        tris.extend([(a1, a2, b2), (a1, b2, b1)])

    def fixed(i):
        pts.append(pts[i])
        return len(pts) - 1

    for x0, y0, nx, ny, off in spec:
        for iy in range(ny):
            for ix in range(nx):
                c = 4 * (off + iy * nx + ix)
                v00, v10, v11, v01 = c, c + 1, c + 2, c + 3
                tris += [(v00, v10, v11), (v00, v11, v01)]
                if ix + 1 < nx:
                    n = 4 * (off + iy * nx + ix + 1)
                    x_wall(v10, v11, n, n + 3)
                else:
                    x_wall(v10, v11, fixed(v10), fixed(v11))
                if iy + 1 < ny:
                    n = 4 * (off + (iy + 1) * nx + ix)
                    y_wall(v01, v11, n, n + 1)
                else:
                    y_wall(v01, v11, fixed(v01), fixed(v11))
                if ix == 0:
                    x_wall(fixed(v00), fixed(v01), v00, v01)  # the outside is the -x (A) side
                if iy == 0:
                    y_wall(fixed(v00), fixed(v10), v00, v10)
    return np.array(pts, dtype=np.float32), np.array(tris, dtype=np.int32)


class SoilZones:
    """Dormant soil columns over a set of regions, with wake/sleep around the bucket."""

    WAKE_MARGIN = 0.5  # m around the bucket's footprint: columns here wake (the failure wedge)
    SLEEP_MARGIN = 0.6  # m: columns outside this (and at rest) sleep; > WAKE_MARGIN for hysteresis
    WAKE_DZ = 0.6  # m: only columns whose top is within this below the bucket's lowest point wake
    WAKE_DEPTH = 0.05  # m below the bucket box (which has 0.1 m of plate margin): woken columns become particles down to here
    MIN_AWAKE = 8  # updates (0.5 s): a woken column stays awake at least this long, so it can flow
    SLEEP_DZ = 0.9
    V_SLEEP = 0.04  # m/s: a column sleeps only if its particles are slower than this, but for
    FAST_FRAC = 0.03  # this fraction of them (MPM jitter on a heap's surface would keep it awake for good)
    V_CREEP = 0.25  # m/s: ... and none faster than this (soil falling or flowing onto it)
    LOST_Z = -0.5  # m: particles below this fell through the ground and are removed
    REPOSE_SLOPE = 0.75  # tan of the angle of repose (sand friction 0.75, 37 deg)
    AVALANCHE_DROP = 0.2  # m to a neighbour: faces this high slump (smaller steps are below MPM's 8 cm resolution)

    DUG_M = 0.3  # m: a particle that has moved this far from where it woke has been dug (it bulks)

    def __init__(self, regions, pool, particle_volume, swell=0.0, loose_regions=()):
        """regions: list of (name, x0, y0, nx, ny, ground) with ground(x, y) -> ground height (numpy),
        raised regions first; pool: particle capacity; swell: bank to loose volume increase of dug soil;
        loose_regions: the regions whose dormant soil is loose (dug)."""
        self.swell = swell
        self.names = [r[0] for r in regions]
        self.ground = [r[5] for r in regions]
        spec, cx, cy, fz, off = [], [], [], [], 0
        for _, x0, y0, nx, ny, ground in regions:
            spec.append((x0, y0, nx, ny, off))
            off += nx * ny
            gx, gy = np.meshgrid(x0 + (np.arange(nx) + 0.5) * CELL, y0 + (np.arange(ny) + 0.5) * CELL)
            gx, gy = gx.ravel(), gy.ravel()
            # floor: the lowest ground under the column, so its top quad never sticks out of a slope
            corners = [ground(gx + dx * CELL / 2, gy + dy * CELL / 2) for dx in (-1, 1) for dy in (-1, 1)]
            cx.append(gx)
            cy.append(gy)
            fz.append(np.min(np.broadcast_arrays(*corners, ground(gx, gy)), axis=0))
        self.spec = spec
        self.n_cols = off
        self.pool = pool
        self.particle_volume = particle_volume
        self.col_x, self.col_y = np.concatenate(cx), np.concatenate(cy)
        self.col_floor = np.concatenate(fz).astype(np.float64)
        self.col_region = np.concatenate([np.full(s[2] * s[3], k) for k, s in enumerate(spec)])
        self.col_loose = np.isin(self.col_region, [self.names.index(n) for n in loose_regions])
        self.col_layer_np = np.where(self.col_loose, LAYER_PER_PARTICLE * (1.0 + swell), LAYER_PER_PARTICLE)
        self.col_layer = wp.array(self.col_layer_np.astype(np.float32), dtype=float)
        self.loose_volume = particle_volume * (1.0 + swell)
        self.floor_z = wp.array(self.col_floor.astype(np.float32), dtype=float)
        self.col_xy = wp.array(np.stack([self.col_x, self.col_y], axis=1).astype(np.float32), dtype=wp.vec2)

        self.regions = Regions()
        self.regions.lo = wp.array([wp.vec2(s[0], s[1]) for s in spec], dtype=wp.vec2)
        self.regions.nx = wp.array([s[2] for s in spec], dtype=int)
        self.regions.ny = wp.array([s[3] for s in spec], dtype=int)
        self.regions.offset = wp.array([s[4] for s in spec], dtype=int)
        self.regions.floor = self.floor_z

        self.count = wp.zeros(self.n_cols, dtype=int)
        self.col_n = wp.zeros(self.n_cols, dtype=int)
        self.col_vmax = wp.zeros(self.n_cols, dtype=float)
        self.col_fast = wp.zeros(self.n_cols, dtype=int)
        self.col_ztop = wp.zeros(self.n_cols, dtype=float)
        self.sleep = wp.zeros(self.n_cols, dtype=int)
        self.show = wp.zeros(self.n_cols, dtype=int)
        self.freed = wp.zeros(pool, dtype=int)
        self.woke_at = wp.zeros(pool, dtype=wp.vec3)  # where each particle woke (or was injected)
        self.loose = wp.zeros(pool, dtype=int)  # 1: dug, at the loose volume
        self.freed_n = wp.zeros(1, dtype=int)
        self.lost = wp.zeros(1, dtype=int)

        k = math.ceil(pool ** (1 / 3))
        g = np.stack(np.meshgrid(*[np.arange(k)] * 3, indexing="ij"), -1).reshape(-1, 3)[:pool]
        self.park_np = (g * PARK_SPACING + np.array(PARK_ORIGIN)).astype(np.float32)
        self.park = wp.array(self.park_np, dtype=wp.vec3)

        pts, tris = column_mesh(spec, self.col_floor)
        self.mesh_points = wp.array(pts, dtype=wp.vec3)
        self.mesh = wp.Mesh(points=self.mesh_points, indices=wp.array(tris.ravel(), dtype=int), velocities=wp.zeros(len(pts), dtype=wp.vec3))
        self.rng = np.random.default_rng(1)
        self.counters = {"woken": 0, "slept": 0, "avalanche": 0, "lost": 0, "pool_short": 0}
        self.on_spawn = None  # callback(slots: wp.array[int]) after dormant soil is woken into those particles

    def region_mask(self, name):
        return self.col_region == self.names.index(name)

    def set_display(self, show):
        """show: per-column mask of dormant soil to stream as grains (where no heightmap shows it)."""
        self.show.assign(show.astype(np.int32))

    def attach(self, sand_model, sand_state, mpm):
        self.model, self.state, self.mpm = sand_model, sand_state, mpm

    def reset(self, counts):
        """counts: dormant particles per column (all soil starts dormant); every pool slot is parked."""
        self.count_np = counts.astype(np.int64)
        self.count.assign(self.count_np.astype(np.int32))
        mm = self.mpm._mpm_model
        off = (self.model.particle_flags.numpy() & ~ACTIVE).astype(np.int32)
        for arr in (self.model.particle_flags, mm.particle_flags, mm.material_particle_flags):
            arr.assign(off)
        mm.material_particle_volume.zero_()
        self.state.particle_q.assign(self.park_np)
        self.state.particle_qd.zero_()
        self.active_np = np.zeros(self.pool, dtype=bool)
        self.tick = 0
        self.woken_tick = np.full(self.n_cols, -(10**9), dtype=np.int64)
        self.lost.zero_()
        self.counters = {k: 0 for k in self.counters}
        self._refit()

    @property
    def n_active(self):
        return int(self.active_np.sum())

    def dormant_in(self, name):
        return int(self.count_np[self.region_mask(name)].sum())

    # ------------------------------------------------------------------ moving soil between regions and
    # things the zone does not know about (a vehicle carrying a load)
    def region_grid(self, name):
        """(x0, y0, nx, ny, column offset) of a region."""
        i = self.names.index(name)
        return self.spec[i]

    def set_region_floor(self, name, floor):
        """Move a region's floor (array per column, or one height). A region far below the ground is
        switched off: no particle maps to its columns and its collider surface is out of the way."""
        m = self.region_mask(name)
        self.col_floor[m] = floor
        self.floor_z.assign(self.col_floor.astype(np.float32))
        self._refit()

    def take_region(self, name):
        """Remove a region's dormant soil; returns its particle counts as a (ny, nx) grid."""
        x0, y0, nx, ny, off = self.region_grid(name)
        counts = self.count_np[off : off + nx * ny].reshape(ny, nx).copy()
        self.count_np[off : off + nx * ny] = 0
        self.count.assign(self.count_np.astype(np.int32))
        self._refit()
        return counts

    def settle(self, mask):
        """Put the active particles over the columns in mask to sleep at once, wherever they are moving."""
        st, mm = self.state, self.mpm._mpm_model
        self.sleep.assign(mask.astype(np.int32))
        self.freed_n.zero_()
        wp.launch(
            absorb_kernel, dim=self.pool,
            inputs=[st.particle_q, st.particle_qd, self.regions, self.sleep, self.LOST_Z, self.park, self.model.particle_flags, mm.particle_flags, mm.material_particle_flags, mm.material_particle_volume, self.count, self.freed, self.freed_n, self.lost],
        )
        nf = int(self.freed_n.numpy()[0])
        if nf:
            self.active_np[self.freed.numpy()[:nf]] = False
        self.count_np = self.count.numpy().astype(np.int64)
        self._refit()

    def set_counts(self, counts):
        """Replace the dormant particle counts of every column."""
        self.count_np = counts.astype(np.int64)
        self.count.assign(self.count_np.astype(np.int32))
        self._refit()

    def remove(self, slots):
        """Take active particles (pool slots) out of the simulation."""
        slots = np.asarray(slots, dtype=np.int32)
        if not len(slots):
            return
        st, mm = self.state, self.mpm._mpm_model
        self.freed_n.zero_()
        wp.launch(remove_kernel, dim=len(slots), inputs=[wp.array(slots, dtype=int), self.model.particle_flags, mm.particle_flags, mm.material_particle_flags, mm.material_particle_volume, st.particle_q, st.particle_qd, self.park, self.freed, self.freed_n])
        self.active_np[slots] = False

    def inject(self, pos, vel, loose=True):
        """Add fresh, unstressed particles at pos with velocities vel (as many as the pool has room for),
        loose (dug) or in place; returns how many were added and their pool slots."""
        free = np.flatnonzero(~self.active_np)
        n = min(len(free), len(pos))
        if n < len(pos):
            self.counters["pool_short"] += 1
        if n == 0:
            return 0, None
        st, mm, ext = self.state, self.mpm._mpm_model, self.state.mpm
        slots = free[:n]
        self.active_np[slots] = True
        slots_wp = wp.array(slots.astype(np.int32), dtype=int)
        wp.launch(
            spawn_kernel, dim=n,
            inputs=[slots_wp, wp.array(np.asarray(pos[:n], dtype=np.float32), dtype=wp.vec3), self.particle_volume, self.model.particle_flags, mm.particle_flags, mm.material_particle_flags, mm.material_particle_volume, st.particle_q, st.particle_qd, ext.particle_qd_grad, ext.particle_elastic_strain, ext.particle_Jp, ext.particle_stress, ext.particle_transform],
        )
        wp.launch(set_velocity_kernel, dim=n, inputs=[slots_wp, wp.array(np.asarray(vel[:n], dtype=np.float32), dtype=wp.vec3), st.particle_qd])
        self._mark(slots_wp, np.full(n, int(loose), dtype=np.int32))
        if self.on_spawn is not None:
            self.on_spawn(slots_wp)
        return n, slots_wp

    def _mark(self, slots_wp, is_loose):
        st, mm = self.state, self.mpm._mpm_model
        wp.launch(mark_spawned_kernel, dim=len(is_loose), inputs=[slots_wp, st.particle_q, wp.array(is_loose, dtype=int), self.loose_volume, self.woke_at, self.loose, mm.material_particle_volume])

    def _refit(self):
        wp.launch(column_top_kernel, dim=self.n_cols, inputs=[self.count, self.floor_z, self.col_layer, self.mesh_points])
        self.mesh.refit()

    def _near(self, box, margin, dz, top):
        lo, hi = box
        return (
            (self.col_x > lo[0] - margin) & (self.col_x < hi[0] + margin)
            & (self.col_y > lo[1] - margin) & (self.col_y < hi[1] + margin)
            & (top > lo[2] - dz)
        )

    def _lowest_neighbour(self, surface):
        """Lowest of the 4 neighbours' surfaces, per column (within its region)."""
        out = np.full(self.n_cols, np.inf)
        for x0, y0, nx, ny, off in self.spec:
            g = np.pad(surface[off : off + nx * ny].reshape(ny, nx), 1, constant_values=np.inf)
            out[off : off + nx * ny] = np.minimum.reduce([g[:-2, 1:-1], g[2:, 1:-1], g[1:-1, :-2], g[1:-1, 2:]]).ravel()
        return out

    def _spawn_positions(self, cols, keep):
        pos = []
        for col, k0 in zip(cols, keep):
            k = np.arange(k0, self.count_np[col])  # lattice slots above the dormant base
            m = k % PER_LAYER
            x = self.col_x[col] + ((m % 2) - 0.5) * SPACING
            y = self.col_y[col] + ((m // 2) - 0.5) * SPACING
            # stack each lattice sub-column on the real ground under it (it differs from the column
            # floor on slopes), so no particle starts inside the ground
            z = self.ground[self.col_region[col]](x, y) + (k // PER_LAYER + 0.5) * SPACING * self.col_layer_np[col] / LAYER_PER_PARTICLE
            pos.append(np.stack(np.broadcast_arrays(x, y, z), axis=1))
        pos = np.concatenate(pos)
        pos[:, :2] += self.rng.uniform(-0.1, 0.1, (len(pos), 2)) * SPACING  # break the lattice symmetry
        return pos

    def update(self, box):
        """box: (lo, hi) world AABB of the bucket, or a list of such boxes (the bucket first, then other
        tools such as a dozer blade). Sleeps resting columns away from them, wakes dormant columns near
        them. Returns (woken, slept) column counts."""
        boxes = box if isinstance(box, list) else [box]
        st, mm = self.state, self.mpm._mpm_model
        if self.swell > 0.0:  # particles moved DUG_M from where they woke are dug: they bulk
            wp.launch(swell_dug_kernel, dim=self.pool, inputs=[st.particle_q, self.model.particle_flags, self.woke_at, self.loose, self.DUG_M, self.loose_volume, mm.material_particle_volume])
        self.col_n.zero_()
        self.col_vmax.zero_()
        self.col_fast.zero_()
        self.col_ztop.fill_(-1.0e6)
        wp.launch(column_stats_kernel, dim=self.pool, inputs=[st.particle_q, st.particle_qd, self.model.particle_flags, self.regions, self.V_SLEEP], outputs=[self.col_n, self.col_fast, self.col_vmax, self.col_ztop])
        n, vmax, ztop = self.col_n.numpy(), self.col_vmax.numpy(), self.col_ztop.numpy()
        fast = self.col_fast.numpy()
        dormant_top = np.where(self.count_np > 0, self.col_floor + self.count_np * self.col_layer_np, -1.0e6)
        top = np.maximum(dormant_top, ztop)

        self.tick += 1
        rested = self.tick - self.woken_tick >= self.MIN_AWAKE
        near_any = np.zeros(self.n_cols, dtype=bool)
        for b in boxes:
            near_any |= self._near(b, self.SLEEP_MARGIN, self.SLEEP_DZ, top)
        still = (vmax < self.V_SLEEP) | ((fast <= self.FAST_FRAC * n) & (vmax < self.V_CREEP))
        sleep = (n > 0) & rested & still & ~near_any
        slept = int(sleep.sum())
        self.sleep.assign(sleep.astype(np.int32))
        self.freed_n.zero_()
        wp.launch(
            absorb_kernel, dim=self.pool,
            inputs=[st.particle_q, st.particle_qd, self.regions, self.sleep, self.LOST_Z, self.park, self.model.particle_flags, mm.particle_flags, mm.material_particle_flags, mm.material_particle_volume, self.count, self.freed, self.freed_n, self.lost],
        )
        nf = int(self.freed_n.numpy()[0])
        if nf:
            self.active_np[self.freed.numpy()[:nf]] = False
        if slept:
            self.count_np = self.count.numpy().astype(np.int64)
            self.counters["slept"] += slept
        self.counters["lost"] = int(self.lost.numpy()[0])

        # wake: each column wakes from its top down to a level; the soil below stays dormant as a base
        has = self.count_np > 0
        dormant_top = np.where(has, self.col_floor + self.count_np * self.col_layer_np, -1.0e6)
        level = np.full(self.n_cols, np.inf)
        # down to just below the tool's lowest point under it, rising 1:1 away from it (the soil that
        # fails ahead of and beside a cutting edge lies above a ~45 deg line from the edge)
        for lo, hi in boxes:
            near = has & self._near((lo, hi), self.WAKE_MARGIN, self.WAKE_DZ, dormant_top)
            dist = np.hypot(np.clip(lo[0] - self.col_x, 0, None) + np.clip(self.col_x - hi[0], 0, None), np.clip(lo[1] - self.col_y, 0, None) + np.clip(self.col_y - hi[1], 0, None))
            level[near] = np.minimum(level[near], (lo[2] - self.WAKE_DEPTH + dist)[near])
        # avalanche: dormant soil standing steeper than the angle of repose against a neighbour wakes
        # down to where it would rest, so cut faces slump instead of standing as rigid walls
        surface = np.maximum(np.where(has, dormant_top, self.col_floor), np.where(n > 0, ztop, -1.0e6))
        low = self._lowest_neighbour(surface)
        steep = has & (dormant_top - low > self.AVALANCHE_DROP)
        level[steep] = np.minimum(level[steep], low[steep] + CELL * self.REPOSE_SLOPE)
        keep = np.clip(np.floor((level - self.col_floor) / self.col_layer_np / PER_LAYER) * PER_LAYER, 0, None)
        keep = np.where(np.isfinite(keep), keep, self.count_np).astype(np.int64)
        keep = np.minimum(keep, self.count_np)
        wake = np.flatnonzero(keep < self.count_np)
        woken = 0
        if len(wake):
            # columns nearest a tool first, in case the pool runs short
            d = np.min([np.hypot(self.col_x[wake] - 0.5 * (lo[0] + hi[0]), self.col_y[wake] - 0.5 * (lo[1] + hi[1])) for lo, hi in boxes], axis=0)
            wake = wake[np.argsort(d)]
            free = np.flatnonzero(~self.active_np)
            fit = int(np.searchsorted(np.cumsum(self.count_np[wake] - keep[wake]), len(free), side="right"))
            if fit < len(wake):
                self.counters["pool_short"] += 1
            wake = wake[:fit]
            self.counters["avalanche"] += int(steep[wake].sum())
        if len(wake):
            pos = self._spawn_positions(wake, keep[wake])
            slots = free[: len(pos)]
            self.active_np[slots] = True
            ext = st.mpm
            slots_wp = wp.array(slots.astype(np.int32), dtype=int)
            wp.launch(
                spawn_kernel, dim=len(slots),
                inputs=[slots_wp, wp.array(pos.astype(np.float32), dtype=wp.vec3), self.particle_volume, self.model.particle_flags, mm.particle_flags, mm.material_particle_flags, mm.material_particle_volume, st.particle_q, st.particle_qd, ext.particle_qd_grad, ext.particle_elastic_strain, ext.particle_Jp, ext.particle_stress, ext.particle_transform],
            )
            self._mark(slots_wp, np.repeat(self.col_loose[wake], (self.count_np[wake] - keep[wake]).astype(np.int64)).astype(np.int32))
            if self.on_spawn is not None:
                self.on_spawn(slots_wp)
            self.count_np[wake] = keep[wake]
            self.woken_tick[wake] = self.tick
            self.count.assign(self.count_np.astype(np.int32))
            woken = len(wake)
            self.counters["woken"] += woken
        if slept or woken:
            self._refit()
        return woken, slept

    def merge_heightmap(self, hm, x0, y0, cell, nx, ny):
        wp.launch(merge_heightmap_kernel, dim=nx * ny, inputs=[hm, x0, y0, cell, nx, self.count, self.col_layer, self.regions])

    def stream_dormant(self, max_out, counter, out):
        wp.launch(dormant_grains_kernel, dim=self.n_cols, inputs=[self.count, self.col_xy, self.floor_z, self.col_layer, self.show, max_out, counter, out])
