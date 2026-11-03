# Phase 2 results: Newton soil on the Spark, streamed into the three.js site

Date: 3 Nov 2026. Server: DGX Spark (GB10), Newton 1.7 / Warp 1.19. Client: the Phase 1 site in Chrome 156 (GPU, Vulkan).

## What it is

```
phase2/soil_server.py  (GB10)                         phase1/web/soil.js  (browser)
  Newton implicit MPM sand bed  ── WebSocket, binary ──▶ sand surface mesh (10 cm)
  kinematic steel bucket            ~6–120 KB/frame        moving grains (points)
  heightmap / grains / force        20–29 frames/s         bucket mesh, live stats
  auto dig cycle or manual targets ◀── JSON commands ──   Jolt heightfield mirror
```

- **Soil:** 266,364 MPM particles (8 cm grid, 4 cm spacing), dry sand (friction 0.75, φ ≈ 37°), 1,600 kg/m³. Fills the pit floor of the Phase 1 site, 6 × 5 × 0.5 m.
- **Bucket:** 0.9 m wide, 0.7 m deep, 0.6 m tall, four steel plates (friction 0.4). Moved kinematically toward targets with arm-like limits (0.6 m/s, 50°/s). The soil pushes back through MPM collider impulses, giving digging force.
- **Per streamed frame:** bucket pose; soil force on the bucket; bucket load (sand inside it); cutting depth and the McKyes/Reece reference force for that depth; a 60 × 50 surface heightmap built only from resting sand (< 0.3 m/s); and every grain moving faster than 5 cm/s (up to 40,000, quantised to 16 bits).
- **Browser:** the surface becomes a vertex-coloured mesh; moving grains are drawn as points. Every 0.5 s the surface is copied into a Jolt heightfield, so site props rest on the dug ground.
- **Controls** (panel "Excavation · Newton soil"): auto dig cycle, reset soil, view pit, and manual sliders for bucket position, height and curl.

## Measured

Headless (`--bench 20`, auto dig, `logs/bench_auto.json`):

| | |
|---|---|
| MPM step | 16 ms (60 Hz step) |
| Real-time factor incl. outputs | 0.92× |

Streaming to Chrome (`test_stream.py`, `logs/stream_test.json`):

| | Result |
|---|---|
| Server step while digging | 16–19 ms → 0.89–1.07× real time (1.3× when the bucket is out of the sand) |
| Frames received | 20–29 /s (30 /s target) |
| Bandwidth | ~1 Mbit/s idle; up to 28 Mbit/s with ~19,000 grains in flight |
| Site frame rate | 60 fps throughout |
| Errors | none in browser console or server |

One auto dig cycle (18 s):

| Stage | Bucket load | Horizontal force | McKyes reference |
|---|---|---|---|
| Penetrate | rising | 0.3–6 kN, plus ~24–45 kN vertical | 1.6–8 kN |
| Drag, 0.33 m cut | 180 → 650 kg | 6–19 kN | ~8 kN |
| Curl and lift | 650 → 600 kg | small; ~5–8 kN vertical (the load's weight) | – |
| Swing | 600 → 450 kg (spillage) | – | – |
| Dump | → 0 kg | – | – |

Sanity checks:
- **Load:** the bucket's geometric volume is 0.38 m³ (≈ 600 kg at 1,600 kg/m³). A heaped fill of 650 kg is plausible.
- **Drag force:** measured drag runs above the static McKyes value (about 8 kN at a 33 cm cut). That's expected: the reference has no surcharge term, while the bucket pushes a growing heap ahead of it. Phase 0 showed the analytic range itself spans about 3× depending on soil–steel friction. This is a plausibility check, not validation.

Screenshots: `logs/stream_dig.png`, `logs/stream_lift.png`, `logs/stream_dump.png`, `logs/stream_after_cycle.png`.

## Bugs found and fixed on the way

- `Jolt._malloc` isn't exported in JoltPhysics.js 1.1.1; scratch memory for `SetHeights` must come from `Jolt._webidl_malloc`.
- After a soil reset, the server's send clock stayed at the old time and stopped streaming.
- numpy `float32` values crashed the JSON stats print once the bucket touched sand.
- A max-height surface pulled carried sand into a "curtain" up to the bucket. The surface now uses resting sand only, and moving sand is drawn as grains.

## Limits (next steps)

- **Fixed zone, not a moving active zone.** The whole pit floor is particles. The research design (particles only near the tool, a heightfield elsewhere, converting between them) isn't built yet.
- **Kinematic bucket.** It follows its targets regardless of resistance. A joint-driven excavator with hydraulics would slow or stall against hard digging.
- **Sand only.** No cohesion, moisture or compaction; no calibration against measured forces yet.
- **Coupling is one-way** for site props: they rest on the sand surface but don't push the sand.
- **Bandwidth peaks near 28 Mbit/s.** Fine on a local network; remote viewers would need delta-coding or fewer grains.
- **The browser pane** in the Claude app pauses animation when hidden; use real Chrome to watch.

## Run it

```bash
# 1. soil server on the Spark (Newton venv from phase0)
phase0/.venv-newton/bin/python phase2/soil_server.py            # ws://127.0.0.1:8765
# 2. site
python3 -m http.server 8731 --bind 127.0.0.1 --directory phase1
# 3. open http://127.0.0.1:8731/web/index.html  (Chrome on the Spark: --enable-unsafe-webgpu --enable-features=Vulkan)
#    another soil host: ...index.html?soil=HOST:8765
```

Headless benchmark: `phase0/.venv-newton/bin/python phase2/soil_server.py --bench 20 --out phase2/logs/bench_auto.json`

---

# Phase 2b: joint-driven excavator arm (`excavator_server.py`)

The kinematic bucket is replaced by the arm of a ~20 t backhoe excavator standing on the ground at the pit edge. `soil_server.py` stays as the kinematic baseline; the browser client (`phase1/web/soil.js`) recognises either server.

## Model

| Part | Value |
|---|---|
| Boom | 5.7 m, 1,800 kg, foot pin 4.1 m above the pit floor (1.8 m above ground) |
| Stick | 2.9 m, 900 kg |
| Bucket | 1.2 m wide, 0.9 m deep, 0.8 m mouth (~0.86 m³), 700 kg; 5 steel plates |
| Joints | 3 revolute (boom, stick, bucket), Newton articulation solved by MuJoCo (GPU), 4 substeps per 1/60 s frame, recorded as one CUDA graph |
| Sand coupling | Two-way: the bucket and stick are dynamic MPM colliders; sand impulses become forces/torques on the links (Newton's `mpm_twoway_coupling` pattern) |
| Hard ground | Bucket and stick collide with the pit floor under the sand (rigid contact) |

Hydraulic-style actuator per joint (`hydraulics_kernel`):
- **Flow:** the lever (−1…1) sets a joint speed up to the maximum (boom 0.35, stick 0.5, bucket 0.9 rad/s). A reference angle advances at that speed.
- **Stiff servo:** τ = Kp·(ref − q) + Kd·(v_cmd − q̇).
- **Relief valve:** τ is capped at the relief torque (boom 450, stick 160, bucket 120 kN·m, scalable 30–100 % from the UI). The reference may lead the joint by at most τ_max/Kp, so a blocked joint doesn't wind up: it slows, then stalls, and moves again when the resistance drops.

Holding with levers neutral: the boom sags 0.2° under full gravity load at 29 % of its relief pressure.

Auto dig cycle: approach → penetrate → drag → curl → lift → dump, defined as bucket-pin waypoints solved by inverse kinematics. The cycle only advances along a segment while the arm keeps up, so a stalled arm holds the cycle rather than skipping ahead.

## Measured

Headless, 40 s auto digging (`logs/exc_bench.json`):

| | |
|---|---|
| Frame time | 23–25 ms (MPM ~19.5 ms, arm ~3 ms) → **0.73× real time** |
| First-pass bucket load | ~820 kg (later passes ~400 kg: the fixed path digs an already-cut trench) |
| Peak relief-pressure use | boom 43 %, stick 55 % (during drag), bucket 20 % |

Stall tests (`--stall-test`, `logs/exc_stall.json`, and in the browser via `test_excavator.py`, `logs/exc_test.json`):

| Test | Result |
|---|---|
| Full relief, boom down + stick out into the sand | Pushes through 0.5 m of dry sand (a 20 t machine should). On reaching the hard floor, boom and stick sit at **100 % pressure, speed 0**. Browser shows "Boom, Stick at relief" |
| 35 % relief, hard cut | All three joints pinned at 35 %, arm stopped with ~1 t of sand in the bucket |
| 40 % relief, stick crowd through sand (browser, keyboard) | Stick stalls at its relief pressure (flagged). As the sand yields it creeps, then breaks free and returns to full speed (0.5 rad/s) |

Browser (Chrome on the GB10): 60 fps; stream 18–28 frames/s at 1–7 Mbit/s; no console errors. Screenshots: `logs/exc_drag.png`, `logs/exc_curl.png`, `logs/exc_lift.png`, `logs/exc_dump.png`, `logs/exc_floor_stall.png`, `logs/exc_weak_stall.png`.

Controls: Boom / Stick / Bucket levers (spring back on release) or keys W/S, A/D, Q/E; Relief pressure slider; per-cylinder pressure gauges (green, amber above 70 %, red at relief).

## Performance notes

- MPM with Newton's fixed grid (copied from the coupling example) cost 117 ms/step; the default sparse grid is 19.5 ms.
- GPU MuJoCo costs 1.7–3.5 ms per substep as separate kernel launches; recording the substeps as a CUDA graph cuts the whole rigid part to ~3 ms per frame.
- A second simulation sharing the GPU (e.g. a forgotten server) roughly doubles every timing. Run one server at a time.
- To reach real time: fewer particles (`--voxel 0.09`, ~30 % fewer) or a moving active zone.

## Limits

- **Simplified hydraulics.** No cylinder geometry, pump flow sharing between functions, or engine power limit. Each joint has its own flow and relief limit, and torques are applied at the joint rather than through cylinder linkages.
- **No swing or travel.** The upper structure doesn't rotate and the tracks don't move; the machine body is cosmetic.
- **Simplified arm contact.** Arm links collide only with the pit floor and the sand, not with the pit walls or each other.
- **0.73× real time** with the full 266k-particle bed.

---

# Phase 2c: swing and truck loading (full dig–swing–load cycle)

## What changed

| Part | Detail |
|---|---|
| Swing | 4th joint about the vertical axis between the undercarriage and a new house body (8 t: house, counterweight, cab). Hydraulic-style like the others: 0.8 rad/s flow limit, 65 kN·m relief torque, so the swing accelerates at a torque-limited rate |
| Dump truck | Rigid tipper parked on the machine's right at ground level. Bed 4.2 × 2.3 × 1.0 m, floor 1.75 m above ground, rated 15 t. The bed plates are MPM colliders (sand lands in it) and rigid colliders (the bucket can hit it) |
| Spill surfaces | Ground slabs behind the machine and beside the pit, plus sloped pit sides (1V:1.5H, matching the site terrain) on the machine and truck sides, so spilled sand lands where the ground actually is. The two vertical containment walls on those sides were removed |
| Bucket | Back fully closed (only the mouth is open). Plates 10 cm thick so the 8 cm MPM grid can't leak sand through them |
| Joint limits | Boom raise to 69° and bucket curl to −120°, about real machine ranges; needed to lift over the truck sides |
| Auto cycle | approach → penetrate → drag → curl → lift → swing to truck → dump → raise → swing back → lower. Each pass digs 15 cm deeper (up to 45 cm), like an operator working down |
| Streaming | `EXC2` frames: 4 link poses, 4 joint states, truck load and capacity, and a second heightmap (10 cm) for the sand in the bed. Grains are streamed when moving or lying outside the pit and bed (spill) |
| Browser | House swings with the arm; tracks and truck drawn from server geometry; truck-bed sand surface; Swing lever (←/→); "Truck load x t of 15 t" readout |

## Measured

Headless, 60 s auto (`logs/load_bench.json`), 0.67× real time:

| Pass | Bucket after curl | Delivered to truck | Truck total |
|---|---|---|---|
| 1 | 611 kg | 524 kg | 0.52 t |
| 2 (15 cm deeper) | 505 kg (stick at 65 % relief) | 512 kg | 1.04 t |

- **Cycle time:** about 27 s (a real 20 t excavator loading a truck: about 15–20 s).
- **Swing:** at its relief torque while accelerating and braking, as in a real swing circuit.
- **Payload:** 500–600 kg per pass against a bucket rated about 0.86 m³ (~1.4 t). The dig path is short, and sand spills during lift and swing (visible as falling grains). A fuller bucket needs a longer drag and a gentler swing profile.

Browser (`test_excavator.py`, `logs/load_test.json`):

| Check | Result |
|---|---|
| Auto cycle | Truck 0 → 0.52 t after the first pass; second pass under way |
| Manual swing (hold ←) | −37° → +13° in 3 s |
| Floor stall at full relief | Boom and stick flagged at relief |
| Speed | 0.53–0.93× real time; stream 11–26 frames/s; site 60 fps; no console errors |

Headless stall test with the swing joint (`logs/exc_stall.json`): at full relief the arm stops on the hard floor (boom and stick at 100 %, speed 0). At 35 % it stalls in the sand with 687 kg in the bucket.

Screenshots: `logs/load_drag.png`, `logs/load_lift.png`, `logs/load_swing.png`, `logs/load_dump.png`, `logs/load_after_two_loads.png`.

## Lessons

- A loaded bucket swung back while lowering jams against the truck side. The cycle needs separate raise → swing → lower steps.
- Interpolating the dig stages in joint space bends the drag path; interpolate the bucket-pin path (task space) instead.
- MPM collider cost grows with the number of large colliders near the sand. Two big slope boxes added ~6 ms; removing the two vertical walls they replaced brought it back to ~19.5 ms idle.
- A forgotten server on the GPU halves throughput. Stop servers between runs; the stale one here was stopped through its background task, since killing GPU processes directly was blocked by the sandbox.

## Limits

- **Static truck.** It doesn't drive away or tip. Reset soil empties it.
- **Spill on other sides.** Sand spilled on the −x/−y sides or beyond the slabs falls to the pit-floor level.
- **Speed.** 0.5–0.9× real time with the full bed and a browser attached.

---

# Phase 2d: payload tuning (toward the bucket's 1.38 t rating)

## Changes

| Change | Why |
|---|---|
| Bucket outer 1.0 × 0.85 × 1.3 m (inside 0.9 × 1.2 × 0.8 m = 0.86 m³ ≈ 1.38 t), 800 kg | The 10 cm anti-leak plates had shrunk the inside to 0.66 m³; restored to a 20 t machine's general-purpose bucket |
| **MPM solver iterations 30 → 60** (server default) | At 30, carried sand slowly *creeps* out of a stationary bucket (739 → 649 kg in 7 s, held still). At 60 the load holds (859 → 861 kg); at 100 the same (867 → 853). The under-converged frictional solve made dry sand behave like a slow fluid. This was the main loss |
| Cylinder end stops + reference clamped to the stroke | A joint back-driven by load could drag its reference past the joint limit (stick reached +88° against a −29° limit). Now a stiff end stop, not limited by relief, holds each joint inside its range |
| Floor-safe dig paths | Deeper passes are raised until the bucket's lowest corner stays ≥ 8 cm above the hard pit floor along the whole path into each dig waypoint (`_stage_z`) |
| Stage timeouts | The auto cycle skips a stage after 4 s of poor tracking, or when it takes > 3× its planned time (slow creep at relief), so a stall can't hang the cycle |
| Dig lanes | Each pass shifts the dig path to a fresh lane (swing offsets 0, +9°, −9°, +17°, −17°, about one bucket width at 6–8 m), and deepens only after every lane is dug |
| Per-stage speed field in `CYCLE` | Lets a stage run at a fraction of full lever (used for gentle-swing experiments) |
| `tune_payload.py` | One pass per cycle variant on a fresh bed, logging bucket load at every stage and the truck load after settling |

## Variant comparison (one pass, fresh bed; `logs/payload_*.json`)

At 30 solver iterations (rounds 1–2, before the convergence fix):

| Variant | To truck | % of 1.38 t |
|---|---|---|
| baseline | 760 kg | 55 % |
| long drag + curl while dragging | 744 kg | 54 % |
| long drag + curl, gentle lift/swing | 668 kg | 49 % |
| baseline, full curl, gentle lift/swing | 654 kg | 48 % |

At 60 solver iterations (rounds 3–5):

| Variant | To truck | % of 1.38 t | Peak in bucket | After curl | After lift | After swing | Cycle |
|---|---|---|---|---|---|---|---|
| **baseline (adopted)** | **893 kg** | **65 %** | 1,090 | 883 | 856 | 811 | **24.2 s** |
| long drag + curl while dragging | 897 kg | 65 % | 1,059 | 927 | 852 | 770 | 26.2 s |
| long drag + curl, gentle lift/swing | 869 kg | 63 % | 1,062 | 928 | 832 | 740 | 28.6 s |
| long drag, slow curl, gentle | 870 kg | 63 % | 1,063 | 932 | 835 | 743 | 28.7 s |
| baseline, full curl, gentle lift/swing | 832 kg | 60 % | 1,094 | 885 | 793 | 708 | 27.6 s |
| baseline from a longer reach | 868 kg | 63 % | 1,122 | 867 | 833 | 792 | 25.5 s |
| deep drag, early curl | 779 kg | 57 % | 829 | 805 | 742 | 678 | 25.3 s |
| deep drag, early slow curl | 776 kg | 56 % | 822 | 800 | 741 | 674 | 25.8 s |

"To truck" includes the heap above the bucket's mouth, which the bucket-load counter (inside the plates only) doesn't see. That's why it exceeds the "after swing" figure.

Findings:
- **Gentler lift and swing never helped.** Loss scaled with time in the air (creep), not with acceleration. Once creep was fixed, speed made little difference, so the fastest cycle wins on throughput.
- **Curling early cuts less.** It shortens the effective cut from 2.6 m to 1.7 m.
- **The baseline drag–curl path is the best throughput:** 893 kg in 24.2 s ≈ 133 t/h. It stays the default cycle.

## Multi-pass result (`python excavator_server.py --bench 130`, `logs/payload_bench_lanes.json`)

| Pass | Lane | Bucket after curl | Delivered | Truck total |
|---|---|---|---|---|
| 1 | 0° | 1,081 kg | 892 kg | 0.89 t |
| 2 | +9° | 961 kg | 862 kg | 1.75 t |
| 3 | −9° | 992 kg | 902 kg | 2.66 t |
| 4 | +17° | 916 kg | 841 kg | 3.50 t |

- **Payload:** mean 874 kg per pass (64 % of the 1.38 t rating), sustained. Before this round, passes delivered 524 and 512 kg, and without lanes passes 2–3 dropped to 429 and 257 kg because the arm re-cut its own trench.
- **Cycle time:** 27–30 s per cycle in this run. The truck reached 3.5 t after 108 s.
- **Speed:** 0.44× real time headless at 60 iterations, against 0.67× at 30. The extra solver work is the price of sand that stays in the bucket. Per frame while digging: 30 it 23.9 ms, 60 it 30.2 ms, 100 it 38.5 ms.

Stall test (`--stall-test`, `logs/exc_stall.json`): at full relief, the bucket on the hard floor puts boom and stick at 100 %, speed 0. At 35 % relief, boom and stick sit at 35 % with the stick held at its end stop (−29°). Before the end-stop fix it was being pushed past it.

Browser (`test_excavator.py`, `logs/payload_browser_test.json`): truck 0 → 893 kg on the first pass; site 60 fps; no console errors; floor stall flags boom and stick; 40 % relief flags the stick. In this run the manual-swing check hit swing relief (100 %) without moving, because the auto cycle had been stopped with the bucket buried mid-drag. A 65 kN·m swing can't drag a buried bucket sideways, so this is correct behaviour, not a key-handling fault.

## Why not 1.38 t

- **The bucket fills to only ~1.08 t (79 %) during the drag.** The mouth faces the machine almost horizontally while dragging, so the upper part of the bucket stays empty. Real buckets have a curved shell and teeth that pull material in; this one is a box.
- **About 200 kg spills during the curl.** The heap above the mouth slides off as the bucket rotates. Another 70–120 kg goes in the lift and swing.
- **Next steps:** a curved-shell bucket collider (several plates approximating the profile) and teeth; a drag that follows the bucket's own path ("heel–toe" kinematics) rather than a straight pin line; and calibration against field fill factors (typically 90–110 % of rated for loose sand).

# Phase 2e: the real Hitachi ZX200 in the physics server

The soil server now simulates the same machine the browser shows: the Hitachi ZX200 (20 t class) from the PWRI OPERA project (Apache-2.0), replacing the generic 4-joint arm.

![ZX200 lifting a load out of the pit](../assets/screenshots/zx200_digging.png)

## What changed

| Change | Detail |
|---|---|
| **URDF link frames** | Boom, stick and bucket joints use the OPERA URDF joint origins, fixed pitches and limits (boom −65°…+57°, stick 34°…154°, bucket −36°…136°). The physics links are the URDF frames (`body_link`, `boom_link`, `arm_link`, `bucket_link`), so joint angles are URDF angles. Pin to pin: boom 5.69 m, stick 2.90 m; the boom foot sits 0.055 m ahead of and 0.72 m above the swing joint, which is 1.12 m above the track bottom |
| **Real machine in the browser** | The hello message names `assets/machinery/opera_zx200.glb`. `soil.js` clones each link's meshes and places them at the streamed link poses (boxes remain as a fallback). The crawler is drawn parallel to the pit edge |
| **Curved bucket with teeth** | The inner shell profile is traced from the OPERA bucket mesh (17 points) and built from 16 shell plates, 22 side strips that stop at the mouth plane, and 5 teeth reaching 0.18 m past the lip. Inside width 1.06 m: 0.79 m³ struck, 0.91 m³ heaped (SAE J296) from the geometry. The bucket load counts grains inside the profile polygon (new `bucket_profile_fill_kernel`) |
| **Rating** | ZX200-6 standard bucket, 0.80 m³ ISO heaped → **1.28 t** of sand at 1,600 kg/m³ (was 1.38 t for the old box bucket) |
| **Masses** | Estimated from published 20 t class figures: upper structure 9,800 kg (with a 4.6 t counterweight), boom 1,450 kg, stick 680 kg, bucket 640 kg. Centres of mass and inertias come from the area-weighted mesh shells. **These are estimates; measured values are wanted** |
| **Levers** | Unchanged for the operator (raise / out / dump positive). They are mapped onto URDF signs with `LEVER_SIGN` |
| **Auto cycle** | Penetrate at 7.6 m instead of 7.9 m, because the real boom foot sits 0.45 m further back than the old one. New *drag-curl* stage: keep dragging while the bucket curls, so the 1.2 m mouth fills toward the top |
| Machine position | Swing axis moved from x = 7.0 to 7.4 m (zone frame) so the tracks overhang the pit slope less |

## Dig-path comparison (one pass, fresh bed; `python tune_payload.py`, `logs/payload_*.json`)

| Variant | To truck | % of 1.28 t | Peak in bucket | After curl | After lift | After swing | Cycle |
|---|---|---|---|---|---|---|---|
| baseline (old path) | 543 kg | 42 % | 623 | 603 | 554 | 489 | 23.8 s |
| long drag | 674 kg | 53 % | 742 | 705 | 685 | 607 | 24.3 s |
| **drag + curl while dragging (adopted)** | **869 kg** | **68 %** | 955 | 907 | 892 | 773 | **23.8 s** |
| raked drag + curl | 652 kg | 51 % | 726 | 693 | 669 | 593 | 29.1 s |

On the real bucket the teeth are the lowest point at every attitude. The 0.5 m sand bed over a hard floor therefore limits the cut to about 0.4 m. A straight drag fills only the lower part of a 1.2 m mouth; curling while dragging fills the rest.

## Multi-pass result (`python excavator_server.py --bench 130`, `logs/zx200_bench.json`)

| Pass | Peak in bucket | Delivered | Truck total |
|---|---|---|---|
| 1 | 949 kg | 864 kg | 0.86 t |
| 2 | 1,036 kg | 926 kg | 1.79 t |
| 3 | 1,137 kg | 1,060 kg | 2.85 t |
| 4 | 1,132 kg | 1,007 kg | 3.86 t |

- **Payload:** mean **964 kg per pass, 75 % of the 1.28 t rating**. Phase 2d's box bucket averaged 874 kg (64 % of 1.38 t). The truck holds 4.9 t at 130 s, against 3.5 t at 108 s before.
- **Cycle:** about 28 s. **Speed:** 0.43× real time headless (60 MPM iterations), unchanged.
- **Hold test:** the arm holds its pose with 2 cm sag; the boom sits at 26 % of relief.

Browser (`test_excavator.py`, `logs/zx200_browser_test.json`, screenshots `logs/load_*.png`, `logs/exc_*.png`):
- The ZX200 model loads.
- Site runs at 59–60 fps with no console errors.
- First pass: 864 kg in the truck.
- Manual swing moves.
- Bucket on the floor: boom and stick flagged at relief.
- At 40 % relief: the stick is flagged.

## Limits and next steps

- **Cylinders and linkage are not rigged.** The OPERA package has cylinder meshes, but no joints for them, so they aren't drawn. Relief torques are still per joint rather than computed from cylinder force through the linkage geometry.
- **Masses are estimates.** Hitachi or ZX200 owners with weighed components or pressure logs can turn this into a validated model (see CONTRIBUTING).
- **Shallow test bed:** a 0.5 m sand layer. A deeper bed (more particles, slower) would allow full-depth cuts and is needed to judge fill factors against field data (90–110 % for loose sand).
- **Swing is still the old 65 kN·m relief value.** The real ZX200 swing torque should be checked.

# Phase 2f: real-time soil (moving particle zone, heightfield elsewhere)

The ZX200 cycle ran at 0.43× real time with the full 266k-particle bed. It now runs at **1.12× real time** with the same digging behaviour: only the soil near the bucket is MPM particles. `soil_zone.py`, used by `excavator_server.py`.

## Where the time went

Profiled at the 8 s mark of the old bench (`SolverImplicitMPM` section timers):

| Part | ms per 60 Hz frame |
|---|---|
| MPM step (266k particles, 60 iterations) | 32.3 (strain solve 18.4, colliders 7) |
| Rigid arm (MuJoCo GPU, CUDA graph) | 0.95 |
| Measurements and stream (30 Hz) | 0.3 |

Two Newton details shaped the design:

- **Inactive particles still cost.** With the default sparse grid, voxels are allocated for every particle, active or not. Parking inactive particles at a single point made the step 13× slower (all of them binned into one cell). They are parked instead in a compact block, about 26 per voxel.
- **About 8 ms per step is fixed overhead.** Even one solver iteration costs about 8 ms (per-step grid build and the re-captured inner solver graph). Newton's capturable mode (rebuildable sparse grid) was slower here: 32–36 ms, because collider and solve kernels run over the reserved capacity. Halving the active particles therefore saves only the iteration part.

## How it works

- **Dormant soil.** Resting soil is stored as a particle count per 8 cm column, over two regions: the truck bed, and the pit with its slopes and the ground around it. That keeps mass exact. A column's top is floor + count × 1 cm.
- **Heightfield collider.** One Warp mesh holds a top quad per column, plus vertical walls between columns and at region borders. Wall winding makes each normal point to the lower side. Active sand rests on dormant columns and against them. Columns with no dormant soil sit at their floor, the lowest ground under them, so on slopes the mesh is buried and Newton's most-penetrated-collider rule ignores it.
- **Wake.** A dormant column wakes when it is within 0.5 m of the bucket's bounding box and its top is no more than 0.6 m below the bucket. Its count becomes fresh particles on a 4 cm lattice, stacked on the real ground under each sub-column.
- **Sleep.** A column sleeps when it is outside 0.8 m and all its particles are slower than 4 cm/s. Its particles are added back to the count and parked.
- **Pool.** The pool is 120k particles. Updates run at 15 Hz and take about 0.2 ms.
- **Display.** The pit and truck heightmaps include dormant columns. Dormant spill on slopes and ground is streamed as grains, one per particle, as before.
- **Leaks.** Particles that fall through the ground are removed and counted as lost (17 in 130 s, about 2 kg). The leak itself predates the zone: the full-bed server loses particles at the same static-collider seams (pit corner, slope toes), where they then fell forever.

## Result (`python bench_zone.py`, `logs/zone_bench.json`; 130 s auto cycle, same as the Phase 2e bench)

| | Full bed (2e) | Moving zone |
|---|---|---|
| Real-time factor | 0.43 | **1.12** |
| Frame cost, mean / dig stages / p95 | 47 ms / – / – | 14.7 / 16.3 / 18.9 ms |
| Active particles, dig mean / max | 266k | 41k / 60k |
| Passes in 130 s | 4 | 4 |
| Per pass to the truck | 864, 926, 1060, 1007 kg | 867, 898, 1023, 951 kg |
| Mean per pass | 964 kg (75 % of rated) | 935 kg (73 %) |

The first pass follows the full-bed run closely: same forces through the drag, and 885 kg against 864 kg delivered. Pass loads stay within 3 %.

The wake margin was compared at 0.8 m (72k active while digging) and 0.5 m (59k). The loads matched (956 vs 950 kg), so the smaller margin is the default. Those two runs were made before the ground region existed, while slope spill stayed active and slowed frames to about 21 ms.

Browser (`test_excavator.py`, `logs/zone_browser_test.json`):
- Two loads, 873 kg and 926 kg, in 56 s.
- Server 0.83–1.49× real time over the cycle, at least 1.0 outside the hardest cuts.
- Site at 60 fps.
- Stall flags work for the floor and for 40 % relief.
- No console errors.
- The panel shows the active particle count.

## Limits

- **Dormant soil is rigid.** It doesn't fail. The 0.5 m margin covers the failure wedge of the cuts made here, but deeper or wider cuts may need a larger margin; pass loads are the check.
- **Digging stages sit near the budget.** They average 16.3 ms against 16.7 ms, with p95 at 18.9 ms, so hard cuts briefly drop below real time. The fixed per-step MPM overhead (about 8 ms) is the next target, either a lighter Newton configuration or a capturable step that stays fast. (Done: the sand now steps at 30 Hz; see "Sand at 30 Hz, machine at 60 Hz".)
- **Stream size is about 35 Mbit/s,** mostly grains. Dormant spill could be sent as a heightmap instead.
- **The bed is still 0.5 m deep.** A deeper bed now costs little: only the zone's columns get taller.

# Phase 2g: real hydraulics (cylinders and bucket linkage)

Boom, stick and bucket are now driven by the ZX200's cylinders instead of joint-space actuators with fixed torque caps (`hydraulics.py`). Speed and force now depend on the pose and the direction, as on the machine. The cylinders and the bucket linkage are drawn in the browser.

## Data

From the Hitachi ZAXIS 200 brochure:

| | Value |
|---|---|
| Boom cylinders | 2 × bore 120 mm, rod 85 mm |
| Arm cylinder | bore 135 mm, rod 95 mm |
| Bucket cylinder | bore 115 mm, rod 80 mm |
| Implement relief | 34.3 MPa (36.3 MPa power boost) |
| Swing relief | 30.4 MPa |
| Main pumps | 2 × 194 L/min |

**Pin positions.** The OPERA package ships cylinder and linkage meshes that its URDF doesn't use. Its link meshes are in the URDF link frames, so pin positions come from a Hough vote on the pin bosses: faces whose normals are perpendicular to the pin axis vote along their normals, and bore centres stand out. Where several bosses were candidates, the one whose cylinder travel over the URDF joint limits matches Hitachi's stroke was kept:

| Cylinder | Travel in the model | Published stroke |
|---|---|---|
| Boom | 1.28 m | 1.26 m |
| Arm | 1.465 m | 1.475 m |
| Bucket (through the four-bar) | 1.04 m | 1.06 m |

The bucket's guide link (0.561 m) and side link (0.583 m) are measured from their own meshes.

## Model

- **Joint torque.** τ = F · dL/dq, where L(q) is the pin-to-pin cylinder length. For the bucket, L comes through the four-bar: guide link from the arm, side link from the bucket, both meeting the rod at one pin.
- **Relief.** The cap is p_relief × head area when pushing and p_relief × rod-side area when pulling, multiplied by the current moment arm. It is set per frame, so each direction has its own limit and both change with pose. The relief slider now acts at runtime; before, its value was frozen into the recorded CUDA graph.
- **Flow.** A lever opens its spool. Cylinder speed is spool flow divided by the area being filled; boom and arm get both pumps, the bucket one. Open spools share the 388 L/min. Joint speed is cylinder speed divided by dL/dq.
- **Gauges.** They show the loaded side's pressure as a fraction of 34.3 MPa. The panel also lists each cylinder's pressure in bar and which side is loaded.

## Check against published forces

Forces at the teeth at relief, with the pin to tooth tip at 1.50 m:

| | Model, 34.3 MPa | Model, power boost | Hitachi (dealer listings) |
|---|---|---|---|
| Bucket digging force | 149 kN | 158 kN | 151–158 kN |
| Arm crowd force (bucket in line) | 96 kN | 102 kN | 109–114 kN |

## Results

`bench_zone.py`, `logs/hyd_bench.json`; 130 s auto cycle:

| | Joint actuators (2f) | Cylinders |
|---|---|---|
| Real-time factor | 1.12 | 1.07 |
| Per pass to the truck | 867, 898, 1023, 951 kg | 976, 1022, 1146, 1044 kg |
| Mean per pass | 935 kg (73 % of rated) | **1047 kg (82 %)** |

The arm is stronger than the old 160 kN·m cap: up to 420 kN·m at mid-stroke. It cuts through more of the drag, so passes are fuller.

Typical pressures in a pass:
- Boom rod side at relief (−343 bar) while it presses the bucket down during the drag.
- Arm at 100–190 bar while dragging.
- Bucket cylinder at about 160 bar holding the curled load during the swing.
- Boom at 60–90 bar holding the arm up.

Hold test: 2 cm sag, boom at 64 bar.

Browser (`test_excavator.py`, `logs/hyd_browser_test.json`):
- Loads of 987 kg and 1056 kg in 50 s, at 60 fps with no errors.
- Bucket on the floor: boom and stick at −343 bar, flagged as stalled.
- At 40 % relief: the stick stalls at 137 bar, which is 40 % of 343.

Screenshots: `logs/hyd_lift.png`, `logs/hyd_linkage.png`, `logs/hyd_arm_cylinder.png`.

**Also fixed:** the auto cycle's swing hunted between lanes at the start of each pass (±25° at relief). When the pass count rose, the start of the lower→approach segment was re-evaluated for the next lane. This was in the old version too.

## Limits

- **The boom nears toggle at its lowest position.** The moment arm drops toward zero in the last 0.3 rad of boom-down travel; speed commands use a 5 cm floor there.
- **The bucket is weak near full curl.** The moment arm is about 5 cm while the curled bucket is carried, which is real four-bar behaviour. The load is held at about 160 bar.
- **No line or port relief.** The real valve has overload reliefs above 34.3 MPa that let an externally loaded cylinder yield. Here one relief value caps both pushing and holding.
- **Swing is still a joint motor** at 65 kN·m. Swing relief and motor displacement would make it physical. (Done: see "Hydraulic swing, port reliefs and an operator-style cycle".)
- **The rod meshes are slightly short.** Near full extension a few cm of gap shows between rod and barrel.

## Recorded replay for the static demo

GitHub Pages has no GPU server, so `record_replay.py` records one auto pass headless:
- The hello message plus the frames the WebSocket would have sent, at 15 Hz with grains thinned to 3000 per frame, gzip-compressed.
- The result is `assets/replay/zx200_cycle.bin.gz`: 27 s, 404 frames, 5.0 MB (10.2 MB uncompressed).

When no server is connected, `soil.js` plays it in a loop through the normal frame path, so the cylinders, pressures, stall flags, pit, truck and grains all show as live. The levers are disabled. `?replay` forces it locally. In a local static-site test, the replay loaded in 3.8 s and played at 60 fps with no errors. Re-record after physics changes, so the demo matches the code.

# Phase 2h: deep bed and trenching

The pit (2.35 m deep, 1:1.5 sides) is now filled with 2 m of sand, up from a 0.5 m layer on its floor: about 1.5 million particles' worth, nearly all of it dormant.

**Soil zone changes (`soil_zone.py`):**
- **Partial wake.** A column becomes particles only from its top down to 5 cm below the bucket's bounding box. The level rises 1:1 with horizontal distance from the bucket. The soil underneath stays a dormant base under the active sand. Without this, a 2 m column would wake whole and the zone would need about 3× more particles.
- **Avalanche wake.** Dormant soil more than 0.2 m above a neighbour wakes down to the 37° repose line, so trench faces slump instead of standing as rigid walls. Smaller steps are below MPM's 8 cm resolution, and waking them only churned.
- **Minimum awake time.** A woken column stays awake for at least 0.5 s before it can sleep, so its sand can flow first. Without this, fresh, motionless particles went back to sleep at once and woke again on every update.
- **Sleep margin** reduced from 0.8 m to 0.6 m. **Pool** raised to 150k (`--pool` sets it).

**Display.** The pit heightmap now covers the filled sides as well (9.6 × 8.6 m).

**Dig cycle.** The waypoints follow the bed top. Dig stages cut to 0.42 m below the top plus the deepening, which reproduces the first pass of the 0.5 m bed. `--trench` (`set_trench`) digs one lane and goes 0.25 m deeper every pass, down to the floor clearance.

## Results

**Normal cycle** (`bench_zone.py 130`, five lanes): 1.075× real time. Frames average 15.4 ms (17.0 ms while digging; p95 21.8 ms). Particles active while digging: 60k on average, 134k at most. Per pass: 1097, 1036, 1172, 979 kg, mean 1071 kg; the 0.5 m bed gave 1047 kg.

**Trench** (`bench_trench.py 8`, `logs/trench_bench.json`), one lane, 8 passes in 266 s:

| Pass | 1 | 2 | 3 | 4 | 5 | 6 | 7 | 8 |
|---|---|---|---|---|---|---|---|---|
| To the truck (kg) | 1099 | 1441 | 1712 | 1764 | 1885 | 1717 | 1412 | 1085 |
| Frame (ms) | 14.7 | 15.2 | 17.2 | 17.2 | 17.5 | 18.7 | 20.8 | 20.8 |

Cross-sections across the lane at the end:

| Position | Depth | Width at half depth | Side slopes |
|---|---|---|---|
| x = 0 | 1.78 m | 2.4 m | 42° and 37° |
| x = 1 m | 1.56 m | 2.3 m | 34° and 37° |
| x = 2 m (shallower end of the drag) | 1.0 m | 2.3 m | 12° and 20° |

Sand's angle of repose here is 37°, so the slumped trench walls come out at the angle they should.

**Limits found:**
- **Heaped loads.** Deep passes carry 1.7–1.9 t, 30–50 % over the rated 1.28 t, as cut faces slump into the bucket mouth during drag and curl.
- **Stalls at relief.** From pass 3, drag-curl, curl and lift time out (16 skipped stages by pass 8) with 0.12–0.37 rad tracking errors. That is the hydraulics at relief under heaped loads plus overburden: the auto operator takes too big a bite. An operator would take shallower cuts as the trench deepens. The next step is an adaptive cycle.
- **Speed in deep passes.** Deep passes wake up to 150k particles, and the run averages 0.92× real time. The 150k pool runs short at the deepest passes (469 times). A 220k pool removes that but makes no difference to loads or skips and is slower (0.87×; `logs/trench_bench_pool220k.json`), so 150k stays the default.

**Browser** (`logs/deep_browser_test.json`): two loads (1102 kg and 1085 kg), 60 fps, no errors. Floor stall: boom and stick at −343 bar. At 40 % relief: the stick at 137 bar. The demo replay is re-recorded on the 2 m bed: 24.8 s, 4.5 MB.

## Adaptive auto operator

The fixed cycle stalled on deep trench passes. The auto cycle now makes two operator calls:
- **Ease off a stalling cut.** When the arm or bucket cylinder is at 90 % of relief or more and the arm falls behind its path, the dig path rises at 0.4 m/s (up to 0.8 m per pass).
- **Curl when full.** The fill is checked every 6 frames while digging. At 110 % of the rated load, the cycle goes straight to curl, starting from the current pose.

Trench, 8 passes (`bench_trench.py 8`, `logs/trench_bench.json`):

| | Fixed cycle | Adaptive |
|---|---|---|
| Skipped (stalled) stages | 16 | **0** |
| Time per pass | 24–42 s | 24–31 s |
| 8 passes take | 266 s | 209 s |
| Real-time factor | 0.92 | 0.975 |
| Per pass to the truck | 1099–1885 kg | 1097, 1447, 1611, 1649, 1714, 1751, 1435, 1054 kg |
| Trench at x = 0 | 1.78 m deep, 37–42° sides | 1.73 m deep, 42–45° sides |

The early curl fires on passes 3 to 8. Loads still reach 1.6–1.75 t, about 130 % of rated, because sand from the cut face keeps pouring in while the bucket curls. Whether a real 0.8 m³ bucket heaps that much in loose sand is a question for field data.

The normal cycle is unchanged (`logs/deep_bench.json`): 1.095× real time, 1072 kg per pass, no skips.

## Sand at 30 Hz, machine at 60 Hz

Dig frames ran at 17–19 ms against the 16.7 ms budget. Profiling a dig frame (`nsys` with graph nodes, Newton's section timers, `cProfile`):
- **The MPM step is host-bound.** The GPU is busy about 12.6 ms of a 23 ms frame under the profiler. The rest is Python issuing about 1500 kernels, and 16 device-to-host readbacks per frame that make CPU and GPU wait on each other. The strain solve's graph is re-captured every step, because the sparse grid changes shape.
- **The solver never stops early.** All 60 Gauss-Seidel iterations run every step (3.9 ms of `gs_solve` in 445 launches).

What did not work:

| Tried | Dig frame | First-pass load | |
|---|---|---|---|
| Default (60 Hz, 60 iterations) | 17.8–19.2 ms | 1096 kg | |
| Rebuildable grid, whole step in one CUDA graph (64k cells, 512 leaves) | 56 ms (102 ms uncaptured) | 736 kg | slower and different physics |
| Keep the solver graph alive one step longer | 18.3 ms | 1095 kg | the wait moves to the next readback |
| `gs-soa` / `gs-batched` solver | 16.9 / 17.8 ms | 1099 / 1100 kg | within run-to-run noise |
| `jacobi` solver | 14.4 ms | 963 kg | −12 % load |
| 40 iterations | 16.0 ms | 892 kg | −19 % load |
| `Q1` collider basis | 14.9 ms | 71 kg | sand passes through the bucket |

**What works: one MPM step every two frames.** The server streams at 30 Hz anyway. The sand steps at 1/30 s; the machine keeps its 60 Hz rigid steps and feels the last sand impulses as a constant force over the two frames (`MPM_EVERY = 2`; the impulse-to-force and collider-velocity kernels use the MPM step). Per frame this halves the fixed host cost. The longer step needs more solver iterations to hold the load (first pass: 653 kg at 60, 928 kg at 100, 1056 kg at 150), so it runs 150 (`MPM_ITERATIONS`). The bucket tip moves about 5 cm per MPM step, under the 8 cm voxel. `--mpm-every 1` restores the old behaviour (with 60 iterations).

| | 60 Hz sand | 30 Hz sand |
|---|---|---|
| Normal cycle, real-time factor (`bench_zone.py 130`) | 1.095 | **1.325** |
| Normal cycle, dig frame mean | 16.8 ms | **13.9 ms** |
| Normal cycle, per pass | 1104, 1038, 1170, 977 kg (mean 1072) | 1057, 1005, 1184, 933 kg (mean 1045) |
| Trench, 8 passes, real-time factor (`bench_trench.py 8`) | 0.975 | **1.199** |
| Trench, frame mean per pass | 15.3–19.8 ms | 11.5–15.1 ms |
| Trench, per pass | 1097–1751 kg, 0 skips | 1061, 1413, 1607, 1735, 1774, 1832, 1500, 1176 kg, 0 skips |
| Trench at x = 0 | 1.73 m deep, 42–45° sides | 1.72 m deep, 43–45° sides |

Logs: `logs/zone_bench_mpm30.json`, `logs/trench_bench_mpm30.json`.

**Costs:**
- Loads are 2.5 % lower on the normal cycle (1045 kg against 1072 kg).
- Frames alternate between about 3 ms (rigid only) and 25–30 ms (with MPM), so p95 is 31 ms. The server paces on the average and streams at 30 Hz, so this does not show.
- More grains leak through collider seams and are culled below the floor: 555 in 130 s (was 28), 936 in the trench run (was 2). At 0.1 kg each that is 56 kg and 94 kg over the run.
- The demo replay was recorded at 60 Hz sand and is kept.

Load depends on the iteration count at both step sizes, so the solver is not fully converged in either case. Converged loads, and whether field data supports them, remain to be checked.

## Hydraulic swing, port reliefs and an operator-style cycle

**Swing drive** (`hydraulics.py`). It replaces the 0.8 rad/s, 65 kN·m joint motor with the ZX200's swing circuit:
- **Data:** 13.3 min⁻¹ swing speed and 68.9 kN·m swing torque (ZX200-3 and ZX210-6 spec listings), with the brochure's 30.4 MPa swing relief.
- **Displacement:** that torque at that relief means 2.27 L/rad at the house (motor plus reduction). At 13.3 min⁻¹ it needs 189 L/min, about one of the two 194 L/min pumps, so the published figures agree with each other.
- **Speed:** swing speed is flow over displacement, and the swing now shares the pumps with the cylinders.
- **Torque:** capped at the relief both ways. The cross-over reliefs also brake the house when the lever returns to neutral.

**Port reliefs.** The pump feeds the side the spool opens, up to the main relief (34.3 MPa). The other side, or both with the spool closed, holds an outside load up to the port relief, then gives way. The port relief is set to 39.2 MPa, a typical value for this class; it is not confirmed for the ZX200.

**Swing test** (lever full toward the truck with the boom raised, then released at 90°):

| | |
|---|---|
| 0 → 90° | 1.67 s, reaching 1.42 rad/s (peak 1.46; rated 1.39) |
| Braking on the cross-over relief after release | 68.9 kN·m, stops 45° later after about 1.1 s; settles back 2° |
| Inertia this implies | about 54,000 kg·m² (boom raised, bucket empty); 81,000 kg·m² measured in the cycle with a loaded bucket and the arm out |
| Swing alone / with full boom raise | 1.46 / 0.95 rad/s: the pumps are shared |

**Operator-style auto cycle.** At the faster swing, the old cycle moved its target at constant speed and paused whenever the house lagged in acceleration, so swings still took 3.3–3.6 s. The auto operator now:
- **Swings like an operator:** full lever until the house would coast onto the target on the reliefs, then neutral. The stopping distance comes from the house inertia, measured online while the swing runs at relief. The other joints follow the swing's progress.
- **Plans with direction-aware speeds:** a cylinder filling its rod side runs faster. The old plan used one speed per joint, so it overrated boom raise by 2× and underrated bucket dump.
- **Uses feed-forward outside the cut:** the lever opens to what the planned pace needs, and the error only corrects it. Feed-forward is only used while the plan advances; otherwise a joint pushed ahead of a held plan cancels its own correction. In the cut, the lever still follows the error only. With feed-forward there, penetrate, drag and drag-curl timed out.

| Stage (s) | Before | Now |
|---|---|---|
| Lift | 2.70 | 2.35 |
| Swing to truck | 3.63 | 2.18–2.38 |
| Dump | 3.32 | 2.00 |
| Raise | 3.88 | 2.67 |
| Swing back | 3.27 | 2.37 |
| Whole pass | 24.4–25.4 | 19.1–20.9 |

| | Before | Now |
|---|---|---|
| Normal cycle, passes in 130 s (`bench_zone.py 130`, `logs/zone_bench_swing.json`) | 4 (5281 kg) | **6 (6290 kg)** |
| Normal cycle, per pass | mean 1045 kg | 1001, 969, 1141, 897, 1044, 1242 kg (mean 1049) |
| Normal cycle, real-time factor | 1.325 | 1.286 |
| Trench, 8 passes (`bench_trench.py 8`, `logs/trench_bench_swing.json`) | 204.5 s | **166.4 s**, 0 skips |
| Trench, per pass | 24.4–27.4 s | 19.1–22.8 s; 1034–1758 kg (12.1 t in all, unchanged) |
| Trench at x = 0 | 1.72 m deep, 43–45° sides | 1.74 m, 42–46° |

Throughput on the normal cycle goes from 146 to 174 t/h. The cycle is still slower than a skilled operator's 15–18 s for a 90° truck load: lift and swing do not overlap yet, and dig stages wait on tracking. (Done: see "Lift and swing at the same time".)

`stats()` adds `swing_bar` and the measured `swing_inertia`.

**Limits:**
- The port relief value (39.2 MPa) and the house inertia need ZX200 data.
- The swing settles back 2° after braking: the servo pulls back to its reference. A blocked real motor rebounds less, about the oil's compressibility.
- No swing parking brake, and no swing priority or flow-summation logic between the two pumps.

## Lift and swing at the same time

An operator starts swinging as soon as the bucket breaks out of the cut and lifts on the way to the truck; on the way back, they lower the bucket once it is clear of the truck. The auto cycle now does the same. The separate lift, raise and lower stages are gone; the cycle is approach → penetrate → drag → drag-curl → curl → swing to truck → dump → swing back.

**Clearance.** The bucket's outline (shell and teeth, both side plates) has to stay above:
- the truck's sides plus 25 cm, within 30 cm of them;
- the heaped load plus 20 cm. The load height is estimated from the truck's load mass (1.5× its mean depth), because the truck heightmap also sees the sand in a bucket held over it;
- the soil, ground or pit walls plus 30 cm.

When the load heaps above the sides, the truck stages carry and dump the bucket that much higher.

**In a swing stage, the house and the arm each move at their own pace:**
- **The arm** follows the planned pin path on its own clock. It only lowers the bucket toward what the house would sweep before it could stop down to 15 cm above the clearance, and backs up along its path if it gets closer. Lifting is never held.
- **The swing** is feathered. The house swings no faster than it can stop before the first angle the bucket does not clear yet. It also arrives no sooner than the rising bucket will clear it, judged from the bucket's measured rate of rise. Above that speed, the lever goes to neutral and the reliefs brake the house.
- **The carry height** is 7.0 m instead of 6.6 m (pin height), so the boom is still rising at full speed as the bucket passes truck-side height. At 6.6 m, its slow final approach held the house about 1 s at the truck's edge.

The first version deadlocked on the swing back: the arm lowered faster than the gate checked and hooked the open bucket on the far side of the truck. That led to the arm margin and the back-up step above.

| Stage (s) | Before | Now |
|---|---|---|
| Lift + swing to truck | 2.35 + 2.18–2.38 | 3.9 |
| Dump | 2.00 | 2.1 |
| Raise + swing back + lower | 2.67 + 2.37 + about 1.5 | 2.9–3.2 |
| Whole pass | 19.1–20.9 | 15.4–18.1 |

| | Before | Now |
|---|---|---|
| Normal cycle, passes in 130 s (`bench_zone.py 130`, `logs/zone_bench_overlap.json`) | 6 (6290 kg) | **7 (6907 kg)**, 0 skips |
| Normal cycle, per pass | mean 1049 kg | 1043, 908, 1053, 914, 1083, 1124, 784 kg (mean 987) |
| Normal cycle, real-time factor | 1.286 | 1.166 |
| Trench, 8 passes (`bench_trench.py 8`, `logs/trench_bench_overlap.json`) | 166.4 s | **131.4 s**, 0 skips |
| Trench, delivered | 12.1 t | 11.8 t |
| Trench at x = 0 | 1.74 m, 42–46° | 1.79 m, 45° |

Throughput on the normal cycle goes from 174 to 191 t/h. Passes are now 15–18 s, the range of a skilled operator's 90° truck load.

**Costs:**
- **Load:** about 6 % less load per pass. The bucket accelerates sideways while it still lifts, and more sand slides off the heap.
- **Speed:** the real-time factor drops because more of each pass is spent digging.

`stats()` adds `swing_held_s` and `arm_held_s`: time the clearance has slowed the swing and held the arm.

**Limits:**
- The clearance treats the load as a flat top at 1.5× its mean depth, not its real shape.
- The house inertia used for the stopping distance is one value, but the true inertia changes with arm pose.
- The dump does not yet start before the swing has stopped, as it often does in practice.

## Soils: dry sand, moist sand, loam and clay

The pit can now be dug in four soils (`soils.py`), chosen with `--soil` or live from the browser's soil selector (the server rebuilds the pit in about 2 s). Until now everything was dry sand.

| Preset | Density | Friction (φ) | Cohesion c | Tensile | Remoulded | Stands for |
|---|---|---|---|---|---|---|
| Dry sand | 1600 kg/m³ | 0.75 (37°) | 0 | 0 | – | the calibrated sand of phases 0–2 |
| Moist sand | 1750 kg/m³ | 0.75 (37°) | 2 kPa (0.9 kPa in the solver, see below) | 0.5 kPa | 70 % | capillary ("apparent") cohesion of a few kPa |
| Loam | 1850 kg/m³ | 0.55 (29°) | 12 kPa | 2 kPa | 20 % | sandy clay loam, firm: c′ ≈ 10–15 kPa, φ′ ≈ 28–30° |
| Clay | 1950 kg/m³ | 0.05 (3°) | 45 kPa | 8 kPa | 30 % | firm clay, undrained as dug: cᵤ 40–75 kPa (ISO 14688-2, BS 5930) |

**How cohesion enters the solver.** Newton's implicit MPM yields on a Drucker–Prager cone, √J₂ ≤ `yield_stress` + μ (p + pₜ), with pₜ = `tensile_yield_ratio` × `yield_pressure`. I derived this from the solver's stress projection and its tensor-to-vector map. With μ taken as tan φ, as for the sand, `yield_stress` is the cohesion c. (The `hardening` option is for snow: fresh soil would start with zero strength, so it stays off.)

**Check 1: unsupported vertical cuts** (`soil_calibrate.py cut`, plane strain, 8 cm grid, as in the excavator). The critical height of a vertical cut is 2.67–3.83 · c/γ · tan(45° + φ/2) (Terzaghi with tension cracks, to the limit-analysis upper bound).

| Soil | Predicted critical height | Stands | Fails |
|---|---|---|---|
| Moist sand at the nominal 2 kPa | 0.62–0.89 m | 1.4 m | 2.0 m: **about 2× too strong** |
| Moist sand at 0.9 kPa in the solver | 0.62–0.89 m | 0.8 m | 1.0 m |
| Loam | 2.99–4.28 m | 2.0 m | starts at 3 m, slumps by 4–5 m |
| Clay at c/10 (heights scale with c/γ) | 0.66–0.95 m | 0.8 m | 1.0 m |
| Dry sand | 0 | – | 0.5 m face runs out 1 m |

Loam and clay match without tuning. At sand-like friction the Drucker–Prager cone is stronger than Mohr–Coulomb, so moist sand gets a calibrated solver cohesion (`mpm_cohesion`) next to its nominal value. Logs: `logs/soil_cut_*.json`.

**Check 2: cutting force** (`soil_calibrate.py blade`). A vertical blade, 0.96 m wide, is pushed at 0.5 m/s, and the force is compared with McKyes' fundamental earthmoving equation including its cohesion term (`soils.mckyes`). Log: `logs/soil_blade.json`.

| Soil | 0.2 m cut | 0.3 m | 0.4 m |
|---|---|---|---|
| Dry sand | 8.4 kN, 2.9× McKyes | 14.4 kN, 2.2× | 22.2 kN, 1.9× |
| Moist sand | 13.0 kN, 1.8× | 20.9 kN, 1.6× | 31.8 kN, 1.5× |
| Loam | 31.5 kN, 1.7× | 45.3 kN, 1.6× | 58.6 kN, 1.4× |
| Clay | 47.5 kN, 1.7× | 69.2 kN, 1.6× | 87.8 kN, 1.5× |

- **Cohesion term:** it scales right. The ratio to McKyes is the same 1.4–1.8 with and without cohesion, so the excess comes from the 8 cm grid and the soil–blade friction, as Phase 0 found for sand.
- **Test setup:** the bed is held by a rough end wall, because a cohesive bed otherwise slides along the floor as one block. Forces are averaged over the early cut, before the heap ahead adds surcharge.
- **Server reference:** the excavator's McKyes reference force (`mckyes_kN` in `stats()`) now includes the cohesion term, and the horizontal share of the tool force.

**Three things the first cohesive runs got wrong:**
- **Strands instead of cracks.** MPM's tension cap is perfectly plastic, so cohesive soil drew out into strands instead of cracking. The fix is `soil_bond_kernel`: each particle's cohesion and tensile strength fall toward the remoulded fraction (1/sensitivity) as it opens up plastically, at Jp − 1 up to 0.1. Newton only tracks per-particle Jp with particle stress warm-starting, so cohesive soils use `warmstart_mode = "particles"` (no measurable cost). The damage stays while a particle is awake. Soil woken from the undisturbed bed starts intact.
- **A clay lump stuck in the bucket.** Once healing was allowed, dug clay compacted into one 45 kPa lump wedged in the curved shell, and it rode back to the pit. With damage that stays, plus an operator move, the clay comes out. The move: if more than 25 % of a full bucket is still in after the dump, rap the bucket against its dump stop, up to 2.5 s (`_shake`, `shakes` in `stats()`).
- **Overfilled buckets.** Cohesive soil does not flow off, so a slab stood on the mouth while the operator, counting only the shell, kept dragging. In cohesive soils the operator now counts the heap on and ahead of the mouth too. That alone still left clay buckets at 140–165 % of rating; see "Bucket fill in cohesive soils" below.

The dormant zone wakes steep faces so they can fail. For a cohesive soil it now only wakes faces higher than 0.8 × the lower critical height, so trench walls in loam and clay stand.

### Bucket fill in cohesive soils

**Where the overfill came from.** A trace of the first clay pass showed it. The operator curled early at 1.3 t, counted with the heap, but the planned curl then dragged the bucket another 0.9 m along the cut while curling. That tore up a slab that rode on the bucket: +1.1 t during the curl, 2.64 t in the truck.

**Target: what a bucket typically carries.** Each preset now has a handbook bucket fill factor and a bank-to-loose swell (`fill_factor`, `swell`, `soils.payload_kg`). Fill factors for hydraulic excavators (Caterpillar Performance Handbook): sand and gravel 95–110 %, moist loam and sandy clay 100–110 %, hard tough clay 80–90 %. Typical swell: sand 10–15 %, loam 20–30 %, clay 30–40 %. For the 0.8 m³ (heaped) bucket:

| Soil | Fill factor | Swell | Typical payload |
|---|---|---|---|
| Dry sand | 100 % | 12 % | 1143 kg |
| Moist sand | 100 % | 15 % | 1217 kg |
| Loam | 105 % | 25 % | 1243 kg |
| Clay | 85 % | 35 % | 982 kg |

The old "of rated" figures divided by bank density, so they overstated the comparison. Against these targets, clay was at 226 % and loam at 123 %.

**Two operator changes:**
- **Curl out, not along the cut.** In loam and clay an early curl now lifts out where the bucket is: the pin moves 0.2 m in and 1.0 m up while curling (`CURL_OUT`), instead of going on to the planned curl point. Raising the lift from 0.7 to 1.0 to 1.4 m cut clay's gain while curling from 480 to 370 to 250 kg, with no change in cycle time. Loam loses part of its heap above 1.0 m. Only soils whose vertical cuts stand higher than the bucket (lower critical height > 1.2 m) hold a heap while lifting out. Moist sand (0.6 m) lost its heap this way, so it is curled along the cut as sand is.
- **Learn what the curl adds.** The operator curls when the load, counted with the heap, reaches the typical payload less what the curl added last pass (`curl_gain_kg` in `stats()`). The first guess is 30 % of the payload. The figure is negative when the heap falls off, as in loam. Dry sand is unchanged: it is still curled at 110 % of rated inside the shell.

**Result,** 130 s per soil, run one at a time (`SOIL=… bench_zone.py 130`, `logs/zone_bench_<soil>.json`):

| Soil | Passes | Per pass, mean (range) | Typical payload | Mean / typical | Before | Real-time factor | Skipped stages |
|---|---|---|---|---|---|---|---|
| Dry sand | 7 | 1015 kg (771–1209) | 1143 kg | 89 % | 1017 kg | 1.18 | 0 |
| Moist sand | 7 | 1214 kg (800–1469) | 1217 kg | 100 % | 1275 kg | 1.26 | 0 |
| Loam | 7 | 1242 kg (732–1605) | 1243 kg | 100 % | 1528 kg | 1.23 | 0 |
| Clay | **7** (was 5) | 1014 kg (690–1283) | 982 kg | 103 % | 2219 kg | 1.19 | 0 (was 2) |

Clay passes no longer need shaking: the smaller load comes out on the dump. That is how clay gets 7 passes instead of 5. The first clay pass now takes 14.9 s, against 17.2 s before.

**Limits:**
- **Pass-to-pass spread:** loads vary ±35 % from pass to pass. Lanes and depths differ, and the learned curl gain is a single number. Clay loads fall over the run, to 0.7 t in the deeper passes.
- **No bulking:** dug clods stay at bank density. A clay load on target therefore fills only about two-thirds of the bucket by eye, where real clay would fill it with voids.
- **Clay shaking:** it is still there for loads that stick, but did not trigger in these runs.
- **No adhesion:** soil–steel adhesion is not modelled, so sticky clay holds only by geometry and friction.
- **Healing:** soil does not regain strength over time, and the load in the truck becomes intact again only when it sleeps.
- **Unvalidated values:** the presets are typical published values, not one measured site. Moist sand's cohesion depends strongly on its water content.


## Validation 1: granular column collapse

The first check of the soil model against measurements, not just theory. Why these references and how they rank against other datasets is in [`reports/Soil excavation validation datasets.md`](../reports/Soil%20excavation%20validation%20datasets.md).

**The experiments:**
- **Aluminium rods (true 2D):** Nguyen, Bui & Fukagawa, "Failure Mechanism of True 2D Granular Flows" ([arXiv 1512.05180](https://arxiv.org/abs/1512.05180)).
  - Setup: 48 collapses of aluminium-rod columns (5 cm long rods), 50–200 mm high and 25–200 mm wide, each repeated at least twice. A true plane-strain material: unit weight 20.4 kN/m³, φ 21.9°, c = 0. The same material is the standard 2D benchmark that SPH and MPM codes validate against.
  - Fitted laws: final runout (d∞ − d0)/d0 = 3.25 a^0.96 for 0.65 ≤ a ≤ 1.5, and 3.80 a^0.73 above. Final height h∞ = h0 up to a = 0.65, then h0/h∞ = 1.42 a^(2/3).
- **Sand (quasi-2D):** Lube, Huppert, Sparks & Freundt 2005 (Phys. Rev. E 72, 041301), sands, sugar and rice in a channel. Runout (d∞ − d0)/d0 = 1.2 a below a = 1.8 and 1.9 a^(2/3) above 2.8, nearly independent of the material.

**The test** (`soil_calibrate.py column`, log `logs/soil_column.json`):
- **Geometry:** a column d0 = 2–4 m wide, held by a gate against a frictionless wall between frictionless side walls, on a rough floor.
- **Sequence:** the column settles for 1 s, then the gate is removed at once.
- **Scaling:** the soil model has no length scale, so the lab columns scale up to 2–12 m tall (25–150 cells), with times scaling as √length.
- **Runout is a range.** The front of the deposit is a sheet thinner than one particle, so runout is reported between d∞ (where the deposit stops being one particle thick) and d_far (its farthest connected particle).

**Results, aluminium rods** (φ 21.9° as solver friction tan φ = 0.40), at the excavator's MPM step (1/30 s) and at 1/120 s:

| a = h0/d0 | Runout, experiment | Runout, sim at 1/30 s (d∞ … d_far) | Runout, sim at 1/120 s | Final height h∞/h0, experiment | Sim at 1/120 s |
|---|---|---|---|---|---|
| 0.5 | 1.66 | 0.94 … 1.30 | 1.10 … 1.72 | 1.00 | 0.99 |
| 1 | 3.23 | 1.92 … 2.64 | 2.16 … 3.32 | 0.71 | 0.66 (−7 %) |
| 1.5 | 4.76 | 2.96 … 3.80 | 3.16 … 4.84 | 0.54 | 0.50 (−7 %) |
| 2 | 6.26 | 3.76 … 4.76 | 4.40 … 6.16 | 0.45 | 0.39 (−13 %) |
| 3 | 8.40 | 5.40 … 6.92 | 6.48 … 8.76 | 0.34 | 0.26 (−24 %) |
| 4 | 10.35 | 6.36 … 8.24 | 7.72 … 10.92 | 0.28 | 0.21 (−27 %) |
| 6 | 13.77 | – | 10.36 … 13.80 | 0.22 | 0.15 (−29 %) |

(The sim's a differs slightly from the nominal value, because a is taken from the settled height; references are evaluated at the sim's a.)

**Results, dry sand** (φ 37°, solver friction 0.75) against Lube et al.:

| a | Runout, experiment | Sim at 1/30 s | Sim at 1/120 s |
|---|---|---|---|
| 0.5 | 0.60 | 0.44 … 0.58 | 0.50 … 0.54 |
| 1 | 1.20 | 0.84 … 1.00 | 1.00 … 1.16 |
| 1.5 | 1.80 | 1.36 … 1.52 | 1.52 … 1.72 |
| 3 | 3.94 | 2.76 … 2.96 | 3.04 … 3.56 |
| 4 | 4.76 | 3.24 … 3.52 | 3.80 … 4.36 |
| 6 | 6.21 | – | 4.92 … 5.56 |

**What this says:**
- **Runout converges to the experiments as the time step shrinks.** At 1/120 s the measured runout for the rods lies within the sim's range at every aspect ratio except a = 2, where it is 1.6 % beyond. The farthest connected particle is within −2 to +6 % of it. For sand it is within −3 to −10 %. At the excavator's 1/30 s step, runout is 0–22 % short on the farthest particle and 20–41 % short on the solid front: the implicit time stepping takes too much energy out of a fast flow.
  - **Check:** the a = 2 rods column at 1/30, 1/60, 1/120 and 1/240 s gives d_far 4.76, 5.68, 6.16 and 6.84, against 6.26 measured.
- **Final height is too low for tall columns,** by 7 % at a ≈ 1 up to 29 % at a = 6, and the error does not shrink with the time step. Wall friction (0 to 1) changes it by under 4 %.
  - Raising the solver friction to 0.50 brings the height within ±8 %, but shortens runout by 20 %. So no single friction matches both.
  - That points at the material model: a rate-independent Drucker–Prager cone with no difference between friction at rest and in flow. Neither the solver nor the excavator has a rate-dependent μ(I) or static-versus-flowing friction.
- **Short columns are right.** Up to a ≈ 0.65 the top of the column stays put, as in the experiments: rods 0.99, sand 1.00.
- **Self-weight compaction is a solver artefact.** Held behind the gate, a column sinks under its own weight more than any real granular material would (about 1 %). The amount depends on the solver:

  | Column height | Iterations | Time step | Settlement |
  |---|---|---|---|
  | 2 m | 150 | 1/30 s | 6 % |
  | 8 m | 150 | 1/30 s | 24 % |
  | 8 m | 600 | 1/30 s | 11 % |
  | 8 m | 150 | 1/120 s | 2.5 % |

  The rheology solve does not converge in tall columns at the large step. The tests above use the settled height as h0. In the excavator only the top 1–2 m of soil is particles, so the error is about 5 %.

**What it means for the excavator:**
- **Sim settings:** the MPM step of 1/30 s, chosen for speed in Phase 2, costs accuracy in fast flows, such as soil spilling off the bucket or out of the truck, and in packing. A 1/60 s step would recover about half the runout error (d_far −8 % at a = 2) at roughly twice the MPM cost.
- **Next physics step:** final pile shapes need a material with friction that differs between rest and flow, or rate-dependent μ(I).
