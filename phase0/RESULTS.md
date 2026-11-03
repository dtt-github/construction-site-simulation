# Phase 0 results: soil engines on the DGX Spark

Date: 2 Nov 2026. Machine: DGX Spark (GB10, sm_121, 128 GB unified memory), Ubuntu 24.04, driver 580.178.04, CUDA 13.0.

## What got installed

| Engine | Version | How | GPU on GB10 |
|---|---|---|---|
| NVIDIA Warp | 1.19.0 | `uv pip install "newton[examples]"` into `.venv-newton` | Yes (sm_121 detected) |
| NVIDIA Newton | 1.7.0 | same | Yes; all MPM examples run, two-way coupling test passes |
| Project Chrono (PyChrono) | 10.0.0 | official conda package for linux-aarch64 (`projectchrono` channel) via micromamba into `.env-chrono` | Yes; CRM (GPU SPH soil) runs. Package bundles CUDA 12.8 runtime |

Not in the Chrono package: DEM-Engine, Chrono::GPU and the VSG viewer Python module. Those need a source build if we want DEM reference runs (Phase 4).

Both engines run on ARM with no workarounds (no `LD_PRELOAD`, no source builds).

## The test

Same scene in both engines (`newton_blade_bench.py`, `chrono_blade_bench.py`):

- Sand bed 4 × 2 × 0.5 m, bulk density 1600 kg/m³, internal friction 0.75 (φ ≈ 37°), no cohesion.
- Vertical blade 1 m wide, cutting 0.2 m deep, pushed at 0.5 m/s for 1 s (0.5 m of travel).
- Measured: wall-clock time, real-time factor, horizontal soil force on the blade (mean over the second half of the cut).
- Reference: McKyes/Reece earthmoving equation for a vertical blade. It depends strongly on blade-soil friction: **3.3 kN** with blade friction 0.4, **10.9 kN** with a fully rough blade (friction = soil friction). It ignores the pile that builds up in front of the blade, so real resistance after 0.5 m of travel should sit above the static value.

Timings are for a single blade with no rendering; the GPU was otherwise idle.

## Newton 1.7 implicit MPM

| Voxel | Particles | Solver iterations | ms per 1/60 s frame | Real-time factor | Blade force |
|---|---|---|---|---|---|
| 0.10 m | 36,531 | 50 | 11.6 | **1.44×** | 6.1 kN |
| 0.05 m | 273,861 | 20 | 15.5 | **1.07×** | 4.0 kN |
| 0.05 m | 273,861 | 50 | 21.2 | 0.79× | 4.4 kN |
| 0.05 m | 273,861 | 100 | 30.6 | 0.54× | 4.6 kN |
| 0.05 m | 903,991 | 50 | 28.6 | 0.58× | 5.0 kN |
| 0.03 m | 1,266,300 | 50 | 101.1 | 0.17× | 3.8 kN |

Peak throughput: about **32 million particle-steps per second** (900k particles).

## Chrono 10 CRM (GPU SPH)

| Spacing | Particles | Active domain | Wall s per sim s | Real-time factor | Blade force |
|---|---|---|---|---|---|
| 0.05 m | 36,531 | no | 8.5 | 0.12× | 13.1 kN |
| 0.03 m | 164,016 | no | 26.1 | 0.038× | 8.5 kN |
| 0.02 m | 527,826 | no | 78.9 | 0.013× | 8.7 kN |
| 0.02 m | 527,826 | yes | 21.1 | 0.048× | 8.2 kN |

CRM needs a 0.25 ms step (4,000 steps per simulated second) against Newton's one implicit step per 1/60 s frame. The active domain gives a 3.7× speedup with almost no change in force.

## What this means

1. **Speed: Newton wins clearly.** At similar particle counts Newton is 20–60× faster. A 270k-particle sand zone runs in real time on the GB10 with Newton; Chrono CRM at 160k particles is about 26× slower than real time.
2. **Force: neither can be called right yet.** Newton gives 3.8–5.0 kN at fine resolution; Chrono converges to 8.2–8.7 kN. Both sit inside the analytic range (3.3–10.9 kN). The spread comes mostly from how each engine treats friction between soil and steel: Chrono's SPH boundary behaves like a rough blade, Newton uses the blade friction we set (0.4). Deciding which is more realistic needs real or DEM reference data (Phase 4), not more particles.
3. **Resolution behaviour:** both engines' forces settle once resolution is about 0.03–0.05 m. That supports the research finding that a coarse, calibrated soil zone is good enough.
4. **Recommendation:** use Newton as the main soil engine for the live site, and keep Chrono CRM as the slower reference and for vehicle-on-soil work (it has Chrono::Vehicle and SCM built in).

## Chrome WebGPU on the Spark

Official Google Chrome 156 for Linux ARM64 (unpacked locally from Google's `.deb`, no system install), X11 session, driven by Playwright (`webgpu/run_chrome.py`, test page `webgpu/index.html`). The page moves 1,048,576 particles and scatters them into a 128³ grid with integer atomics, the same pattern browser MPM demos use.

| Chrome flags | Result | GPU used | Time per step | Particle updates per second |
|---|---|---|---|---|
| none | **No WebGPU** (no adapter) | none | n/a | n/a |
| `--enable-unsafe-webgpu` | Works, but software fallback | SwiftShader (CPU) | 9.9 ms | 105 million |
| `--enable-unsafe-webgpu --enable-features=Vulkan --use-angle=vulkan` | **Works on the GB10** | NVIDIA (Vulkan) | 0.44 ms | **2.4 billion** |
| same, headless | No adapter | none | n/a | n/a |

With Vulkan enabled, Chrome exposes a 2 GB storage-buffer limit, so large particle buffers fit. Viewers on the Spark itself need those flags; other client machines (Windows, macOS, Intel/NVIDIA-on-Wayland Linux) get WebGPU by default. Firefox 158 was not tested.

## Isaac Sim 6.2 container on the Spark

Image `nvcr.io/nvidia/isaac-sim:6.2.0` (public, has an arm64 build; 9.3 GB download, 26.8 GB on disk). Run headless with `--runtime=nvidia`, NVIDIA EULA accepted with the user's approval, usage-data sharing off (`PRIVACY_CONSENT=N`). Script: `isaac/isaac_smoke.py`; output in `isaac/out/`.

| Check | Result |
|---|---|
| Starts on driver 580.178.04 | **Yes**, no crash. Vulkan RTX renderer and PhysX GPU extension load |
| SimReady forklift, PhysX variant | **Loads and settles**: 15 rigid bodies, 14 joints, PhysX variant selected by default, stays on the ground over 5 s |
| RTX render (1280×720, one camera) | **Works**; first frames in 2.8 s. Image: `isaac/out/isaac_forklift.png` |
| 4,096 stacked boxes, GPU dynamics, writing poses back to USD | 259 ms per step (0.06× real time) |
| Same, without USD write-back | 87 ms per step (0.19× real time) |
| Same, CPU dynamics | 92 ms per step (0.18× real time) |

Notes:
- Writing every body pose back into the USD scene triples the cost; a real app must keep poses on the GPU (Fabric / tensor API) instead.
- GPU and CPU PhysX run at the same speed here, which is unexpected. Either the GPU pipeline isn't fully engaged on GB10 or the stepping call adds overhead. The log shows no fallback warning. This matters only if we lean on Isaac's PhysX; our plan uses Newton for live physics.
- The asset sample used earlier in research was incomplete (missing `geometries.usd`); the full forklift is now in `assets/`.
- Harmless noise: repeated "OmniHub failed to launch" warnings (no local Hub service in the container).

## Not done in Phase 0

- Isaac Sim web streaming viewer (one viewer per instance, no login; not needed until Phase 5).
- Firefox WebGPU.
- Chrono DEM-Engine (needs a source build).
- Two-way coupling of a dynamic, hydraulically driven bucket (the blade here is kinematic in both engines).
- Any rendering or visual check of the soil.

## Phase 0 verdict

| Component | Go / no-go on the DGX Spark |
|---|---|
| Newton / Warp (live soil + machines) | **Go.** Real-time sand zone of ~270k particles |
| Chrono 10 CRM (reference soil, vehicles) | **Go**, as an offline reference (20–60× slower) |
| Chrome WebGPU (browser client) | **Go, with flags** on the Spark; default on most other clients |
| Isaac Sim 6.2 (authoring checks, RTX, sensors) | **Go** for loading SimReady assets and RTX rendering; PhysX rigid speed needs a follow-up before relying on it |

## Reproduce

```bash
cd phase0
.venv-newton/bin/python newton_blade_bench.py --voxel 0.05 --ppc 2 --frames 60
.env-chrono/bin/python chrono_blade_bench.py --spacing 0.03 --duration 1.0
```

Raw results are in `logs/` (JSON per run; Newton runs also save final particle positions and per-frame forces as `.npz`).

```bash
DISPLAY=:1 .venv-newton/bin/python webgpu/run_chrome.py logs/chrome_webgpu.json
docker run --rm --runtime=nvidia --gpus all --network=host -e ACCEPT_EULA=Y -e PRIVACY_CONSENT=N \
  -v $PWD:/work -v $PWD/assets:/assets:ro -v $PWD/isaac/cache:/isaac-sim/kit/cache:rw \
  --entrypoint /isaac-sim/python.sh nvcr.io/nvidia/isaac-sim:6.2.0 /work/isaac/isaac_smoke.py
```
