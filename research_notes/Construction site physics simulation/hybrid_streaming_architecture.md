# Hybrid Server-Side Simulation + Browser Client Architectures on NVIDIA DGX Spark (as of 2 Nov 2026)

Scope note: research budget was ~15 tool calls. Where a claim comes from background knowledge rather than a source fetched in this session it is placed under Inferences or Gaps and marked UNVERIFIED.

## Pixel streaming options (Omniverse/ovstream, Isaac Sim livestream, Unreal Pixel Streaming 2, Unity Render Streaming)

### Takeaway
For a DGX Spark, the NVIDIA-native path is the most viable: Isaac Sim 6.x ships a Docker-Compose browser "web viewer" (WebRTC, built on the Omniverse Web SDK) that is documented as supported on DGX Spark, and the new pre-release `ovstream` library (pip, Linux aarch64 binaries) can stream NVENC-encoded frames, including Warp/PyTorch DLPack tensors, to a browser over WebRTC without Kit. Hard limit: Isaac Sim livestream is one client per instance. Unreal and Unity streaming have no documented/benchmarked Linux aarch64 support.

### Cited Findings
**Isaac Sim livestream / web viewer**
- Two clients: a native desktop "Isaac Sim WebRTC Streaming Client" (Win/macOS/Linux, recommended for local/same-network) and a browser "web viewer" deployed via Docker Compose, built from the Omniverse Web SDK (`@nvidia/create-ov-web-rtc-app`), runs in Chromium browsers, recommended for remote — [Isaac Sim Livestream Clients](https://docs.isaacsim.omniverse.nvidia.com/latest/installation/manual_livestream_clients.html)
- Web viewer deployment "supported on Ubuntu hosts and DGX Spark systems"; use `--aarch64` in build commands; Windows/WSL hosts not supported — [Isaac Sim Livestream Clients](https://docs.isaacsim.omniverse.nvidia.com/latest/installation/manual_livestream_clients.html)
- Ports: TCP 49100 signaling, UDP 47998 media, TCP 8210 web viewer; configurable via `ISAACSIM_SIGNAL_PORT`, `ISAACSIM_STREAM_PORT`, `WEB_VIEWER_PORT`; extension named is `omni.kit.livestream.app` (settings `primaryStream/publicIp`, `signalPort`, `streamPort`); example image `nvcr.io/nvidia/isaac-sim:6.2.0` — [Isaac Sim Livestream Clients](https://docs.isaacsim.omniverse.nvidia.com/latest/installation/manual_livestream_clients.html)
- Concurrency: "Only one client can access an Isaac Sim instance at a time"; one streaming method per instance — [Isaac Sim Livestream Clients](https://docs.isaacsim.omniverse.nvidia.com/latest/installation/manual_livestream_clients.html). Isaac Lab docs also: a second connection to an instance returns an error — [Isaac Lab isaaclab_physx.app](https://isaac-sim.github.io/IsaacLab/develop/source/api/lab_physx/isaaclab_physx.app.html)
- Requires NVENC-capable NVIDIA GPU (not supported on A100); streaming endpoints have NO authentication or encryption — use only on trusted networks with firewall rules — [Isaac Sim Livestream Clients](https://docs.isaacsim.omniverse.nvidia.com/latest/installation/manual_livestream_clients.html)
- No latency figures are given in the Isaac Sim docs — [Isaac Sim Livestream Clients](https://docs.isaacsim.omniverse.nvidia.com/latest/installation/manual_livestream_clients.html)
- Isaac Sim 6.2.0 release notes: new `isaac-sim-web-viewer` agent skill launches browser WebRTC streaming from a prebuilt NGC image; Newton WebRTC streaming launcher added (Docker Compose `ISAACSIM_PHYSICS_ENGINE=newton`, PhysX default); Kit SDK 110.4.0 — [Isaac Sim 6.2.0 release notes](https://docs.isaacsim.omniverse.nvidia.com/6.2.0/overview/release_notes.html)
- Isaac Sim 6.0 release notes list "full Docker container support and full livestreaming support for DGX Spark" (per search summary of the notes) — [Isaac Sim 6.0.0 release notes](https://docs.isaacsim.omniverse.nvidia.com/6.0.0/overview/release_notes.html)
- CONFLICT: Isaac Lab (develop) docs state "Livestreaming is not currently supported or validated on DGX Spark" (for the Isaac Lab `LIVESTREAM=1/2` path) — [Isaac Lab isaaclab_physx.app](https://isaac-sim.github.io/IsaacLab/develop/source/api/lab_physx/isaaclab_physx.app.html)

**Omniverse Kit App Streaming / Web SDK**
- Browser library: `@nvidia/omniverse-webrtc-streaming-library` (npm; AppStreamer class to connect/pause/terminate) — [Omniverse WebRTC Streaming Library](https://docs.omniverse.nvidia.com/ov-web-sdk/latest/web-streaming-library/overview.html)
- Kit App Streaming (OVAS) is Kubernetes/Helm-based; docs say "Omniverse Streaming on NVCF" replaces Omniverse Kit App Streaming (OKAS) as the reference architecture for streaming at scale — [OVAS docs](https://docs.omniverse.nvidia.com/ovas/latest/index.html); [Deploying Kit apps at scale](https://developer.nvidia.com/blog/deploying-your-omniverse-kit-apps-at-scale/)

**ovstream (new, 2026)**
- C/Python SDK for live streaming Omniverse content without Kit/Carbonite; pre-release; `pip install ovstream` self-contained wheel; binaries for Windows x86_64, Linux x86_64, Linux aarch64 (DGX Spark not named) — [NVIDIA-Omniverse/ovstream](https://github.com/NVIDIA-Omniverse/ovstream)
- Transports: WebRTC, RTSP, native (StreamSDK), SHM, CUDASHM (CUDA IPC); several can run simultaneously. Inputs: raw BGRA8 CUDA frames encoded by NVENC, DLPack tensors from Warp/PyTorch/CuPy/JAX, or pre-encoded H.264/H.265/AV1 passthrough; bidirectional input/messaging; per-client accept callback and resizable streams; example browser client at `examples/webrtc_client/index.html` (port 49100) — [ovstream](https://github.com/NVIDIA-Omniverse/ovstream)
- No quantified latency; no max-client count stated; license is NVIDIA Software License + Omniverse product-specific terms (not OSS) — [ovstream](https://github.com/NVIDIA-Omniverse/ovstream)
- ovrtx: C/Python library for RTX sensor simulation/visualization and headless SDG, pre-release; from v0.4 can attach to `ovstage` shared runtime stage; ovstream is described as its streaming companion; announced as part of NVIDIA Agent Toolkit Omniverse libraries at SIGGRAPH 2026 — [ovrtx docs](https://nvidia-omniverse.github.io/ovrtx/); [NVIDIA blog: Integrate Omniverse RTX sensor simulation into existing apps](https://developer.nvidia.com/blog/integrate-nvidia-omniverse-rtx-sensor-simulation-into-existing-apps/)

**Unreal Pixel Streaming 2**
- Epic docs list Pixel Streaming plugin support for Windows, Linux, Mac; many settings changed in Pixel Streaming 2; no ARM64 specifics — [Epic Pixel Streaming reference](https://dev.epicgames.com/documentation/unreal-engine/unreal-engine-pixel-streaming-reference)
- Epic only ships Linux x64 Docker images; community images exist to build Linux ARM64 targets — [wukakuki/unreal-engine Docker Hub](https://hub.docker.com/r/wukakuki/unreal-engine)
- Vendor (unverified, promotional) claim: Linux served ~50% more concurrent users and 12–15 streams per GPU — [Eagle 3D Streaming](https://eagle3dstreaming.com/maximizing-performance-linux-for-enterprise-unreal-engine-pixel-streaming); same vendor later reports higher Linux load times — [Eagle 3D blog](https://eagle3dstreaming.com/blog/linux-pixel-streaming-for-unreal-engine-what-it-actually-takes-and-where-nginx-fits-in)
- One app instance shared by many viewers means they compete for control; scaling = multiple instances — [Unreal forums](https://forums.unrealengine.com/t/pixel-streaming/2239966)

**Unity Render Streaming**
- Render Streaming 3.1 lists Linux (architecture unspecified), Windows x64 only, Android ARM64 only; depends on com.unity.webrtc — [Unity Render Streaming 3.1 manual](https://docs.unity3d.com/Packages/com.unity.renderstreaming@3.1/manual/index.html)

### Inferences
- On DGX Spark the low-risk pixel-streaming path is Isaac Sim 6.2 container + Docker-Compose web viewer (documented for DGX Spark), or a custom Python app that renders (ovrtx or Warp) and streams via ovstream WebRTC. Multi-user requires either one Isaac Sim instance per viewer (GPU memory/NVENC-session heavy) or fan-out via an SFU/relay — ovstream's per-client accept callback suggests multi-client is possible but is not documented.
- Typical WebRTC pixel-streaming glass-to-glass latency on LAN is in the tens of ms range (UNVERIFIED — no source in this session gave numbers for any of these products). Must be measured.
- GB10 NVENC session count and its effect on concurrency are undocumented in sources found; GeForce-class session caps vs. pro caps unknown for GB10 (UNVERIFIED).
- Unreal/Unity on Linux aarch64 with GPU WebRTC encoding would be a self-supported path; treat as high risk.

### Gaps
- No published latency or concurrency benchmarks for Isaac Sim web viewer, Kit App Streaming, or ovstream.
- No NVENC session limit / encoder count info for GB10 found.
- Whether OVAS/NVCF streaming stack (Kubernetes) runs on a single aarch64 DGX Spark was not confirmed.
- Unreal Pixel Streaming 2 and Unity Render Streaming on Linux aarch64: no official confirmation found.
- Licensing: Isaac Sim itself is free to use (Apache-2.0 source on GitHub per background knowledge, UNVERIFIED this session); Omniverse Enterprise licensing for production Kit App Streaming not checked.

## State streaming (server physics to three.js client) and existing frameworks

### Takeaway
Newton (Warp-based) already ships browser viewers via Viser and Rerun and runs on aarch64; Viser is the most direct "Python sim → browser three.js-like scene" framework (point clouds, meshes, batched instanced meshes, Gaussian splats). A custom three.js client over WebSocket/WebTransport with quantized buffers is the scalable option for 1M-particle point clouds; bandwidth is the main constraint (estimates below are computed, not sourced).

### Cited Findings
- Newton viewer backends: `gl`, `usd`, `rtx`, `rerun`, `viser`, `null`; `make_viewer()` can open a Viser web viewer on http://localhost:8080 — [NVIDIA Learning: Newton core concepts](https://docs.nvidia.com/learning/physical-ai/getting-started-with-newton/latest/newton-fundamentals/core-concepts.html) (via search summary)
- User report: Newton 1.2.0 (via Isaac Lab 3.0-beta) on DGX Spark GB10 aarch64 Ubuntu 24.04 ran Unitree Go2 locomotion, with Rerun used as a browser web viewer — [Classmethod blog](https://dev.classmethod.jp/articles/go2-simulation-newton/) (single user report)
- Newton requirements: Linux x86-64 and aarch64, Windows x86-64, macOS CPU-only; `usd-core` has no Linux aarch64 wheels so `usd-exchange` is used instead — [newton on PyPI](https://pypi.org/project/newton/) (via search summary)
- Warp publishes Linux AArch64 packages with CUDA support on Grace Blackwell when driver/runtime requirements are met — [Warp FAQ](https://nvidia.github.io/warp/latest/user_guide/faq.html)
- Isaac Sim 6.2 bundles Newton 1.6.1, MuJoCo / MuJoCo Warp 3.12.0, Warp 1.17.0 — [Isaac Sim 6.2.0 release notes](https://docs.isaacsim.omniverse.nvidia.com/6.2.0/overview/release_notes.html)
- Viser: Python 3D visualization library with a fully web-based client (good over SSH), Jupyter support; primitives include point clouds, meshes, line segments/splines, images, frames, a WebGL Gaussian splat renderer, and batched (instanced) meshes/GLBs — [Viser docs](https://viser.studio/main/)
- Large browser point-cloud precedent: Potree renders billions of LiDAR/photogrammetry points via octree LOD and frustum culling — [Potree (FOSSGIS slides)](https://www.fossgis.de/mediawiki/images/e/e4/Potree.pdf); WebGPU streaming viewers for point clouds exist — [IEEE BigData 2023 paper](https://computer.org/csdl/proceedings-article/dasc/2009/3929a339/12OmNASILUr)
- ovstream supports bidirectional messaging over WebRTC (usable as a side channel alongside video) — [ovstream](https://github.com/NVIDIA-Omniverse/ovstream)

### Inferences
- Bandwidth arithmetic (computed here, not sourced): 1M particles × 3 × float32 = 12 MB/frame → 360 MB/s (2.9 Gbit/s) at 30 Hz — infeasible over WAN, marginal on LAN. Quantized to 3 × uint16 relative to a site bounding box (e.g. 200 m / 65536 ≈ 3 mm resolution) = 6 MB/frame → ~1.4 Gbit/s @30 Hz. At 10 Hz with uint16 + client interpolation: ~480 Mbit/s. Further reductions: send only active/moving particles (most soil at rest), delta-encode vs. previous frame + zstd/meshopt-style compression, stream a heightmap/surface reconstruction instead of particles for the settled bulk (e.g. 512×512 uint16 heightmap = 0.5 MB full, deltas typically much smaller), and render only "flying" particles as points.
- Rigid bodies (excavators, trucks, cranes: maybe hundreds of transforms) are trivial: 100 bodies × (pos 3×f32 + quat 4×f16) ≈ 2 KB/frame → ~0.5 Mbit/s at 30 Hz; interpolate client-side (snapshot interpolation, ~100 ms buffer).
- meshopt/Draco are designed for static geometry; for per-frame particle streams simple quantization + delta + zstd/LZ4 is usually more practical (UNVERIFIED rule of thumb).
- Hybrid recommendation: state streaming (three.js + Spark splats for background) for many concurrent lightweight viewers; pixel streaming (ovstream/Isaac web viewer) for a few "RTX-quality" viewers.

### Gaps
- No Viser performance numbers (message format, max point count, float16 support) obtained — check Viser "Performance Tips" page and source.
- Foxglove, Meshcat, Rerun web viewer throughput limits not researched in this session.
- No source found for a "Newton web viewer" beyond Viser/Rerun backends.
- USD-over-websocket / Nucleus live-layer streaming to browsers: not verified; ovstage is a shared runtime stage but browser access was not documented in sources found.
- Chrono, AGX Dynamics, Genesis on aarch64 not checked.

## Support of Warp / Newton / Isaac Sim 5.x/6.x on DGX Spark aarch64 (RTX + streaming) and known issues

### Takeaway
Warp and Newton officially support Linux aarch64; Isaac Sim 6.x has DGX Spark containers (NGC lists DGX Spark / DGX OS 7 as a host) and the web viewer is documented for DGX Spark. Main known issues: driver-branch sensitivity (R590 595.71.05 crashes RTX init; R580 580.159.03 required for 6.0.1), the `LD_PRELOAD=libgomp.so.1` workaround for Python imports, missing usd-core aarch64 wheels, and conflicting statements about livestream validation.

### Cited Findings
- NGC Isaac Sim container lists DGX Spark (Linux aarch64, DGX OS 7) as a host requirement — [NGC isaac-sim 6.1.0](https://catalog.ngc.nvidia.com/orgs/nvidia/-/containers/isaac-sim/6.1.0) (via search summary)
- Direct Python commands importing Isaac Sim on aarch64 require `LD_PRELOAD=/lib/aarch64-linux-gnu/libgomp.so.1`; Isaac Lab says livestreaming not supported/validated on DGX Spark and points to an install-page list of other non-validated features — [Isaac Lab isaaclab_physx.app](https://isaac-sim.github.io/IsaacLab/develop/source/api/lab_physx/isaaclab_physx.app.html)
- Forum (July 2026): Isaac Sim 6.0.0/6.0.1 segfault at RTX init on DGX Spark (DGX OS 7.5.0, driver 595.71.05-open, CUDA 13.2); NVIDIA staff: 6.0.1 requires R580 driver 580.159.03 on DGX Spark; R590 incompatible with Omniverse RTX scene database init (also seen on RTX 4090 / RTX PRO 6000); generic Linux 580.65.06 driver is x86_64-only; update via DGX Spark Dashboard. Thread closed without confirmation of fix — [NVIDIA forum](https://forums.developer.nvidia.com/t/isaac-sim-6-0-1-gpu-crash-on-dgx-spark-gb10-arm64-driver-595-71-05/376418)
- DGX Spark release notes list driver 580.126.09 among components (at time indexed) and a separate known-issues page — [DGX Spark Release Notes](https://docs.nvidia.com/dgx/dgx-spark/release-notes.html)
- Isaac Sim 6.2.0 release notes do not mention DGX Spark/aarch64 issues; only RTX known issue listed is camera gizmos blocking RTX sensor rendering (workaround: hide cameras); Ubuntu 22.04 support slated for removal — [Isaac Sim 6.2.0 release notes](https://docs.isaacsim.omniverse.nvidia.com/6.2.0/overview/release_notes.html) (first 100k chars read)
- Warp: Linux AArch64 CUDA support incl. Grace Blackwell — [Warp FAQ](https://nvidia.github.io/warp/latest/user_guide/faq.html); Newton: aarch64 supported, usd-core workaround — [newton PyPI](https://pypi.org/project/newton/)

### Inferences
- Pin DGX OS driver to the version Isaac Sim's requirements page lists for your release (R580 branch at 6.0.1); avoid R590 until confirmed fixed.
- Use the NGC container path (not pip) on Spark to minimize aarch64 dependency issues.
- Isaac Sim 5.x on aarch64: no evidence found; DGX Spark support appears to begin with 6.0 (inferred from 6.0 notes).

### Gaps
- Isaac Sim 6.2 Known Issues page and DGX Spark known-issues page not read.
- No independent confirmation that RTX path tracing / DLSS works fully on GB10.

## GB10 performance characteristics for physics vs. RTX 4090/5090/RTX PRO 6000

### Takeaway
GB10 has 48 SMs (CC 12.1), 128 GB unified LPDDR5X at 273 GB/s spec (~205 GB/s measured STREAM triad). FP32 vector ~31 TFLOPS (unverified spec-database figure). MPM/DEM are memory-bandwidth-bound, so expect roughly 1/6–1/8 the throughput of an RTX 5090 / RTX PRO 6000 per step, but far larger problem sizes fit in memory.

### Cited Findings
- Measured: 48 SMs, compute capability 12.1, 119.7 GiB visible; tensor TF32 53.3 TFLOPS, FP16/BF16 (FP32 acc) 212.9 TFLOPS dense; NVFP4 427.3 (scaling questioned) — [NVIDIA forum metrics thread](https://forums.developer.nvidia.com/t/detailed-compute-performance-metrics-for-dgx-spark/351993)
- Memory: 128 GB 256-bit LPDDR5X, 273 GB/s spec — [The Register](https://theregister.com/2025/10/13/nvidia_gb10_spark); measured triad ~205–207 GB/s (forum user); Hot Chips slide ~301 GB/s raw — [Hot Chips 2025 slides](https://www.hc2025.hotchips.org/assets/program/conference/day2/21_nvidia_skende_final.pdf)
- FP32 31.0 TFLOPS listed for GB10 by a spec database flagged "no verified data" — [flopper.io](https://flopper.io/compare/nvidia-dgx-spark-vs-nvidia-gb10-grace-blackwell) (UNVERIFIED)
- StorageReview MAMF ~99.8 TFLOPS BF16, ~207.7 FP8 — [StorageReview](https://storagereview.com/review/nvidia-dgx-spark-review-the-ai-appliance-bringing-datacenter-capabilities-to-desktops)

### Inferences
- UNVERIFIED comparison figures (background knowledge, not fetched): RTX 4090 ~82.6 TFLOPS FP32 / 1.0 TB/s; RTX 5090 ~105 TFLOPS / 1.79 TB/s; RTX PRO 6000 Blackwell ~125 TFLOPS / 1.79 TB/s, 96 GB. Bandwidth ratio GB10:5090 ≈ 1:6.5 (spec) or ~1:8.7 (measured 205 GB/s).
- MPM: per-step traffic ~ hundreds of bytes per particle (P2G/G2P + grid). At ~200 GB/s effective and assuming ~500 B/particle/step, GB10 manages ~400M particle-updates/s → ~1M particles at ~100–400 substeps/s, i.e. real-time (30 FPS) only with few substeps per frame. Rough, needs measurement.
- Capacity advantage: 128 GB lets tens to hundreds of millions of particles fit (offline / slower-than-real-time), beyond a 32 GB 5090.
- Rendering + NVENC + physics share the same 48 SMs and 273 GB/s — RTX rendering will compete with physics; state streaming (client renders) frees the GPU for physics.

### Gaps
- No measured FP32 vector TFLOPS or RT-core count for GB10 found; no published Warp/Newton/MPM/DEM benchmark comparing GB10 to discrete GPUs.

## Gaussian splatting / photogrammetry backgrounds and NVIDIA real-to-sim tooling

### Takeaway
Same splat asset can serve both sides: train with 3DGRUT/3DGUT, export as OpenUSD ParticleField for Isaac Sim/Omniverse RTX (6.0.1+), and export PLY/SPZ for the browser via Spark (World Labs, MIT, three.js r180+, LOD streaming since 2.0). Splats need a separate mesh collider for physics.

### Cited Findings
- Isaac Sim 6.1/6.2: environments as USD stages using ParticleField geometry drawn by Omniverse RTX alongside meshes; training/export via open-source 3DGRUT; NuRec core algorithm is 3DGUT — [Isaac Sim Neural Volume Rendering 6.2](https://docs.isaacsim.omniverse.nvidia.com/6.2.0/assets/usd_assets_nurec.html)
- Native ParticleField support arrived in Isaac Sim 6.0.1; open question whether depth/normals/segmentation ground truth works on ParticleField prims — [IsaacSim discussion #798](https://github.com/isaac-sim/IsaacSim/discussions/798)
- Workflow: PLY splat → USDZ via 3DGRUT script; separate GLB mesh as collider placed under the Gaussian volume — [NVIDIA blog: Isaac Sim + World Labs Marble](https://developer.nvidia.com/blog/simulate-robotic-environments-faster-with-nvidia-isaac-sim-and-world-labs-marble)
- Spark: 3DGS renderer for three.js/WebGL2, desktop/mobile/WebXR, MIT, npm `@sparkjsdev/spark`, three.js r180+, formats PLY/SPZ/SPLAT/KSPLAT/SOG, renders alongside triangle meshes; Spark 2.0 (14 Apr 2026) added streamable LOD; latest version unclear (2.1–2.3 referenced) — [sparkjs.dev](https://sparkjs.dev/); [GitHub sparkjsdev/spark](https://github.com/sparkjsdev/spark)
- Viser also includes a WebGL Gaussian splat renderer — [Viser docs](https://viser.studio/main/)

### Inferences
- Drone photogrammetry of the real site → 3DGRUT splat (background) + mesh/heightmap (collision + deformable terrain init) is the practical pipeline. Deformable terrain must be simulated as mesh/heightmap/particles; the splat is static backdrop and will visibly diverge where soil is excavated (mask/fade the splat in the active work zone).

### Gaps
- Not researched this session: mkkellogg GaussianSplats3D status, gsplat, nvblox, PyCuSFM, Cosmos Transfer details. (Background: nvblox = GPU TSDF/ESDF mapping; PyCuSFM = CUDA SfM; Cosmos Transfer = controlled video-to-video photoreal augmentation — UNVERIFIED here.)
- Whether 3DGRUT training runs on aarch64/GB10 not confirmed.

## Construction digital-twin reference architectures (BIM/IFC → USD, connectors, IFC in browser)

### Takeaway
No first-party, shipped IFC→USD Omniverse connector was confirmed; practical path is IfcOpenShell-based conversion (IFC → USD/glTF) or vendor connectors (Revit, Civil 3D, Bentley iTwin). For the browser, IFC → glTF/fragments via That Open Engine is the usual route (not verified this session).

### Cited Findings
- NVIDIA discussed encoding the IFC schema in USD (AOUSD context); not confirmed shipped — [AEC Magazine](https://aecmag.com/news/nvidia-looking-to-encode-the-ifc-schema-in-usd)
- Community forum: Revit and Vectorworks talk to Omniverse via USD connectors, no full round-trip IFC/OpenBIM; IfcOpenShell can convert IFC to USD — [NVIDIA forum](https://forums.developer.nvidia.com/t/omniverse-open-usd-meets-open-bim-when/352210) (informal)
- Bentley iTwin–Omniverse integration announced 2021; iTwin supports USD, glTF, Datasmith interoperability; Bentley iModel bridge for IFC — [Reliabilityweb/Bentley](https://reliabilityweb.com/news/article/bentley-systems-brings-infrastructure-digital-twins-to-nvidia-omniverse); [Geo Week News](https://www.geoweeknews.com/articles/bentley-esri-announce-major-integrations-with-nvidia-omniverse/)
- AWS example of preparing BIM assets with Omniverse for IoT TwinMaker — [AWS IoT blog](https://aws.amazon.com/blogs/iot/using-nvidia-omniverse-to-prepare-3d-assets-from-building-information-models-for-aws-iot-twinmaker)

### Inferences
- Recommended stack: IFC (design) → IfcOpenShell → USD (sim stage on Spark) and → That Open Engine fragments/glTF (browser); site scan → 3DGRUT splat (USD ParticleField + Spark SPZ); Newton/Warp physics on Spark → (a) state stream to three.js client and (b) optional ovstream/Isaac web viewer RTX view.

### Gaps
- That Open Engine / web-ifc current version and aarch64 server-side use not checked; Autodesk/Trimble connector status in 2026 not verified; most Bentley sources are from 2021.
