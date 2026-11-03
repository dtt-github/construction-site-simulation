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
