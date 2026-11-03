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
