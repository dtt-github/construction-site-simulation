# Phase 1 results: SimReady assets in a three.js + Jolt site

Date: 3 Nov 2026. Machine: DGX Spark (GB10). Browser: Chrome 156 ARM64 with `--enable-unsafe-webgpu --enable-features=Vulkan --use-angle=vulkan`.

## 1. Physics-data extractor (`extract_physics.py`)

Reads a SimReady USD root with `usd-core` 26.11 (native aarch64 wheel), selects the physics variant (PhysX by default), and writes a JSON sidecar per asset (`sidecars/<asset>.json`):

- up axis, metres per unit, kilograms per unit, semantic labels
- every rigid body: explicit mass/density, centre of mass, inertia, world pose, bounds
- every collider: owning body, approximation, physics material (friction, restitution, density), mesh volume and convex-hull volume
- every joint: type, bodies, local frames, axis, limits and drives
- effective mass per body, as an engine computes it: explicit mass, else mesh volume × density

Lessons from building it:
- Colliders live inside instanced prims; a plain `stage.Traverse()` misses all of them. Use `Usd.TraverseInstanceProxies`.
- Every collider in the 22 assets uses `approximation = "sdf"` in the PhysX variant, which browser engines can't use. Convex hulls must come from elsewhere (the GLB conversion provides them).
- Only 6 of 22 assets carry an explicit mass; the rest rely on density × volume.

## 2. Converted GLB check (`compare_glb.py`, `data/glb_vs_usd.md`, `data/glb_diff.json`)

**Coverage:** codepawl has 2,595 of NVIDIA's 2,677 assets. The 82 missing are mostly modular warehouse building pieces (52) and small tabletop items; **no construction props are missing**. 8 GLBs have no physics data (none construction-related).

**Physics fidelity, 22 construction assets:**

| Field | USD vs GLB |
|---|---|
| Rigid bodies | Match in all 22 |
| Joints | Match in all 22 |
| Friction values | Match in all 22 |
| Mass, where the USD has an explicit mass (beam, hammer, bucket, scissor lift) | Match within 1–5% |
| Mass, where it comes from density × volume | **Disagree, by 0.015× to 9×** |

**The masses can't be trusted from either source.** The NVIDIA data itself is the root problem:
- A plywood crate is assigned steel density (8,000 kg/m³). USD volume gives 6.3 t, the GLB gives 95 kg; a real 1 m plywood crate weighs ~40 kg.
- Hollow objects (barrier walls, tilt trucks, trolleys) get wildly different volumes depending on whether the mesh is treated as a closed solid, an open shell or a hull.
- The pallet stacker's total comes out around 275–320 kg; real walk-behind stackers weigh 600–900 kg.

**Decision:** keep NVIDIA's geometry, joints, friction and hulls, but set each asset's total mass from catalogue values (`web/site.json`, with a note per asset). Each part keeps its share of the GLB mass split.

## 3. Prop site (`web/`)

`web/index.html` + `web/site.js`, loaded from `web/site.json`. three.js 0.182 (WebGL), Jolt Physics 1.1.1 (WASM), both from jsDelivr; GLBs served locally.

- Each GLB body node → a Jolt body built from the GLB's stored convex hulls (compound shape when a part has several).
- Joints from node extras → Jolt hinge (revolute), slider (prismatic) or fixed constraints, with limits. Parts of the same asset don't collide with each other.
- Restitution capped at 0.3 (the GLBs give rubber wheels 0.8, which made them bounce).
- Click an object to push it; buttons drop 6 crates or a 865 kg steel beam; collider wireframes can be toggled.

**Measured** (`test_site.py`, Chrome on the GB10 via Vulkan):

| | Result |
|---|---|
| Content | 40 assets, 127 rigid bodies, 89 joints, 6.2 t |
| Load time | 2.3 s (local server) |
| After 8 s | All bodies at rest (max 0.2 m/s), none through the ground |
| Frame rate | 60 fps (vsync-capped) |
| Physics step | 0.51 ms while things move; bodies then sleep |
| After dropping 6 crates + steel beam | 134 bodies, all settled within 8 s, none through the ground, 60 fps |
| Console errors | None (one 404 for the missing tab icon, now fixed) |

Screenshots: `data/site_settled.png`, `data/site_after_drops.png`, `data/site_colliders.png`.

In the Claude browser pane (no GPU), the same page runs at about 1 fps but simulates correctly.

## Polish pass (3 Nov 2026)

### Joint motors
Configured per joint in `site.json` (`motors`, `controls`); created as Jolt hinge/slider motors.

| Machine | Joint | Motor | Control |
|---|---|---|---|
| Pallet stacker | fork slide (0–1.51 m) | position, 15 kN, ramps at 0.2 m/s | Fork height slider |
| Pallet stacker | steering base (±60°) | position, 400 N·m, 1.5 rad/s | Steering slider |
| Pallet stacker | 2 drive wheels | velocity, 150 N·m each, 12 rad/s (≈1 m/s) | Hold Forward / Reverse; zero speed = parking brake |
| Vehicle lift | 4 ramps (0–39/45°) | position, 6 kN·m, 0.5 rad/s | Ramp angle slider |

Measured (`test_motors.py`, `data/polish_test.json`):

| Check | Target | Result |
|---|---|---|
| Fork lift with pallet + 40 kg crate | 1.20 m | fork 1.19 m; pallet and crate rose 1.16 m and stayed on the forks |
| Drive forward, 4 s | ~1 m/s | 3.44 m travelled, straight |
| Steering | 40° | 40.0° |
| Ramps | 30° | 31.7° (small gravity sag) |
| Jolt mass vs intended, all 132 bodies | — | identical (0% error) |

Lessons:
- Jolt position motors are springs. At 1.5 Hz the fork settled 0.43 m short under load; 8 Hz brings the error to about 1 cm.
- Moving the target instantly made the stiff fork snap upward and knock the pallet off. Targets now move at a realistic actuator speed (`rate`), like hydraulics.
- The NVIDIA "scissor lift" asset is a vehicle lift whose platform is a single fixed body. Only its 4 ramps are hinged, so there is no platform height to motorise without re-rigging the asset.

### Better colliders (`decompose_colliders.py`)
CoACD convex decomposition of the USD render meshes, converted into each GLB body's frame and clamped to the real mesh bounds (raw CoACD hulls overshoot by about 1 cm). Listed in `site.json` → `hull_overrides`; files in `web/hulls/`.

| Asset part | Before | After |
|---|---|---|
| Metal crowd barrier | 1 hull (solid slab) | 24 hulls: open frame, rails, feet (`data/polish_fence_colliders.png`) |
| Safety gate | 1 hull | 13 hulls |
| Rolling scaffold frame | 1 hull | 16 hulls |
| Pallet stacker body, fork, tiller | 1 hull each | 16 hulls each; tines and carriage now separate, so a pallet rides on the tines |
| Vehicle lift body and ramps | 1 hull each | 5–16 hulls each |

The stacker chassis gets a 4 cm minimum ground clearance (`hull_min_y`). Its mesh sits only 9 mm above the ground, inside Jolt's contact margin, so the legs dragged and the machine couldn't drive.

### Terrain heightfield
128 × 128 m Jolt `HeightFieldShape` (256 × 256 samples, 0.5 m spacing) with a matching three.js mesh. Every placement is now relative to the ground under it.

- Flat working pad, 60 × 44 m
- Low rolling hills around it, up to about 6 m
- Haul road leaving through the gate at about 1:12
- Spoil stockpile, about 3.2 m high, flanks below sand's angle of repose
- Excavation pit, 6 × 5 m floor, 2.5 m deep, sides 1V:1.5H (`data/polish_pit.png`; the drum placed at its edge rolls to the floor)

After the polish: 44 assets, 132 bodies, 90 joints, 8 motors, still 60 fps with a 0.1–0.5 ms physics step, nothing under the terrain, no console errors.

## Known limitations

- **The terrain is rigid.** It can't be dug or deformed; that's Phase 2 (soil on the Spark).
- **Fence mesh infill** is a thin plate rather than wire, so small objects can't pass through it (realistic for most site use).
- **The stacker's steering doesn't turn its drive wheel**; the asset attaches that wheel to the chassis. It drives straight; steering only swings the tiller base.
- **Other assets' motors aren't wired up yet** (drill, extension ladder, etc.).
- **Masses are catalogue estimates**, not measured.

## Run it

```bash
python3 -m http.server 8731 --bind 127.0.0.1 --directory phase1
# then open http://127.0.0.1:8731/web/index.html
DISPLAY=:1 phase0/.venv-newton/bin/python phase1/test_site.py   # automated check (run from phase1/)
```

Re-extract physics for an asset:

```bash
phase1/.venv/bin/python -I phase1/extract_physics.py phase1/usd/<asset>/simready_usd/<root>.usd phase1/sidecars/<asset>.json
```
