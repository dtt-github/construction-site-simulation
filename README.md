# Construction Site Simulation

An open-source, physically realistic construction-site simulator, built in the open by universities and industry.

![Open-licensed machines in the site: Komatsu D37PXi dozer, Hitachi IC120 carrier, Hitachi ZX200 excavator](assets/screenshots/showcase_machines.png)

**Goal:** a site you can trust numerically. Soil that digs like soil, machines whose hydraulics stall like real ones, and every model checked against measured data. Rigid-body site physics runs in the browser (three.js + Jolt). Soil and machine dynamics run on a GPU server (NVIDIA Newton: MPM sand, MuJoCo articulations) and stream into the same page.

## What works today

| Area | Status |
|---|---|
| Site | 128 m terrain with pad, haul road, stockpile and a 2.5 m excavation pit; 40+ physics props from NVIDIA SimReady (CC BY 4.0) with checked masses and colliders |
| Real machines (open-licensed) | Hitachi ZX200 excavator, Hitachi IC120 crawler carrier, Komatsu D37PXi dozer (articulated, from the PWRI OPERA project, Apache-2.0); 35 m tower crane, cones, barrels and jersey barriers (Gazebo, CC0) |
| Soil | 266k-particle MPM sand bed (Newton implicit MPM) on the GPU, two-way coupled to the machine |
| Excavator | 4-joint machine (swing, boom, stick, bucket) with flow-limited, relief-capped hydraulic actuators that slow and stall under load; full dig–swing–load cycle into a dump truck |
| Browser client | Levers and keyboard control, cylinder pressure gauges, stall warnings, live truck load; 60 fps |
| Validation | Bucket forces compared with the McKyes/Reece earthmoving equation; field data wanted (see below) |

| Folder | What's in it |
|---|---|
| `feasibility.html` | Feasibility page (browser vs Omniverse decision, research summary, build plan) |
| `reports/`, `research_notes/` | Cited research on soil, machine and crane simulation, and the open 3D-asset survey |
| `phase0/` | Platform tests on the DGX Spark: Newton vs Chrono soil benchmark, Chrome WebGPU, Isaac Sim. See [`phase0/RESULTS.md`](phase0/RESULTS.md) |
| `phase1/` | SimReady → web pipeline (USD physics extractor, GLB checks, CoACD colliders) and the three.js + Jolt site. See [`phase1/RESULTS.md`](phase1/RESULTS.md) |
| `phase2/` | Newton soil servers: `excavator_server.py` (excavator with hydraulic-style actuators loading a dump truck) and `soil_server.py` (kinematic-bucket baseline). See [`phase2/RESULTS.md`](phase2/RESULTS.md) |
| `assets/` | Open asset library: registry, licenses, converted machines and build scripts. See [`assets/README.md`](assets/README.md) |

## Quick start

Developed on an NVIDIA DGX Spark (GB10, Linux aarch64). Any recent NVIDIA GPU on Linux should work.

```bash
# Python environment with Newton/Warp (see phase0/RESULTS.md for details)
uv venv phase0/.venv-newton && uv pip install -p phase0/.venv-newton "newton[examples]" playwright websockets
# third-party assets that are not stored in git
phase1/fetch_assets.sh
python3 assets/fetch_assets.py
# soil + excavator server (ws://127.0.0.1:8765)
(cd phase2 && ../phase0/.venv-newton/bin/python excavator_server.py)
# site
python3 -m http.server 8731 --bind 127.0.0.1 --directory phase1
```

Open http://127.0.0.1:8731/web/index.html in Chrome. On Linux ARM, start Chrome with `--enable-unsafe-webgpu --enable-features=Vulkan`.

## Join

This project needs people who know the real thing. See [CONTRIBUTING.md](CONTRIBUTING.md). The highest-value contributions:

1. **Measured data**: bucket and blade forces with soil properties, excavator pressure logs, crane load tests. Open a *Validation data* issue.
2. **Open 3D models of real machines**: wheel loaders, articulated dump trucks, mobile cranes, concrete mixers and pumps, rollers, workers. Read the [asset policy](assets/README.md) first. Manufacturers: releasing models under an open license helps every university and company using this.
3. **Physics**: cohesive and wet soils, real hydraulic circuits, crane ropes, tyre–soil contact.
4. **Calibration and benchmarks**: reproducible scenes with reference results.

## Open asset library

Every 3D asset is listed in [`assets/registry.json`](assets/registry.json) with its source, license, author and changes. `python3 assets/fetch_assets.py --check` validates the registry; `--notice` regenerates the attribution list in [NOTICE](NOTICE).

- **In the repo:** converted machines in `assets/machinery/`, site items in `assets/site/`, built by `assets/build_machines.py` and `assets/build_static.py` from their upstream sources.
- **Downloaded on demand:** NVIDIA SimReady site props (CC BY 4.0).
- **Candidates:** vetted, redistributable models that still need a manual download or conversion (Rocketbox construction-worker avatars, Sketchfab CC-BY wheel loader, articulated dump truck, mobile crane, mixer, pump, skid steer, telehandler). Good first contributions.

Only licenses that allow redistribution, modification and commercial use are accepted (CC0, CC-BY, CC-BY-SA, MIT, BSD, Apache-2.0). Machine models depict trademarked products; the licenses cover the models, not the trademarks, and no endorsement by the manufacturers is implied.

## Roadmap

- Real-time soil: a moving particle zone around the tool with a heightfield elsewhere
- Fuller bucket loads and operator-realistic cycles (in progress)
- Real machine geometry in the physics server (ZX200 kinematics instead of the simplified arm)
- Excavator hydraulic cylinders and bucket linkage rigged (meshes are in the OPERA packages; joints are missing)
- Cohesive and wet soils; calibration against lab and field data
- Wheel loader, articulated dump truck, mobile and tower crane dynamics with ropes
- Multi-user sessions and operator training scenarios

## License

Code: [Apache License 2.0](LICENSE). Assets and data: their own licenses, listed in [NOTICE](NOTICE) and [`assets/registry.json`](assets/registry.json). Physics libraries used: NVIDIA Newton and Warp (Apache-2.0), Jolt Physics (MIT), three.js (MIT), Project Chrono (BSD-3), CoACD (MIT).
