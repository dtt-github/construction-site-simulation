# Browser-based physics engines and WebGPU/WebGL GPU compute for construction-site simulation (state as of 2 Nov 2026)

Research note: this pass used about 20 tool calls. Several primary pages (Codrops article, some forum threads) returned 403 or had no numbers. Package versions were checked live against npm and crates.io registries on 2026-11-02.

## Q1. Rigid/articulated physics engines in WASM: which support excavator booms (articulated + joint drives), tracked vehicles, wheel-terrain contact, rope/cable?

### Takeaway
Jolt Physics (JoltPhysics.js) is the strongest all-round browser choice for construction machinery. It exposes TrackedVehicleController, wheeled vehicles, motorised hinge/slider constraints, pulley and rack-and-pinion constraints, skeletons/ragdolls, soft bodies including rods, multithreaded and SIMD builds, and a cross-platform-deterministic option. PhysX 5 via physx-js-webidl now lists articulations and vehicles as covered, making it the best option for reduced-coordinate (Featherstone-style) excavator arms. MuJoCo has official Google DeepMind WASM bindings (`@mujoco/mujoco`) and is the most accurate for articulated dynamics with actuators, but it is not a vehicle or terrain engine. Rapier is a lightweight, deterministic engine with multibody joints, but it has no vehicle or track model.

### Cited Findings
**Jolt / JoltPhysics.js**
- The npm `jolt-physics` latest version is 1.1.1 (npm registry query, 2026-11-02) — [npm](https://npmjs.com/package/jolt-physics)
- "Almost the entire Jolt interface has been exposed". It ships 7 build flavours: wasm-compat, debug-wasm-compat, wasm, asm.js, plus 3 multithreaded variants (wasm-compat-multithread, debug-…-multithread, wasm-multithread). The build options are ENABLE_SIMD (on by default in multithread builds), DOUBLE_PRECISION (for worlds larger than "a couple of km") and CROSS_PLATFORM_DETERMINISTIC (results match the native library). Memory is managed manually (Jolt.destroy, AddRef/Release) — [JoltPhysics.js GitHub](https://github.com/jrouwe/JoltPhysics.js)
- JoltJS.idl (checked 2026-11-02) exposes these interfaces: `TrackedVehicleController`, `TrackedVehicleControllerSettings`, `VehicleTrack`, `VehicleTrackSettings`, `WheelSettingsTV`/`WheelTV`, `WheeledVehicleController`, `VehicleEngine`, `VehicleTransmission`, `VehicleDifferentialSettings`, `VehicleAntiRollBar`, and `VehicleCollisionTesterRay`/`CastSphere`/`CastCylinder`. It also exposes `PulleyConstraint`, `RackAndPinionConstraint`, `HingeConstraint`, `Skeleton`/`SkeletonPose`, and soft-body settings including `SoftBodySharedSettingsRodStretchShear`/`RodBendTwist` (Cosserat rods, usable for cables), LRA, Volume, Skinned and DihedralBend — [JoltJS.idl](https://github.com/jrouwe/JoltPhysics.js/blob/main/JoltJS.idl)
- The live demo page has: Wheeled Vehicle, Motorcycle, Tank Controller, Height Field Shape, Constraint Motor, Pulley, Path, Conveyor Belt (single- and multithreaded), Powered/Kinematic Rig, Soft Body (cloth, volume cubes, pressure, bend, LRA), Buoyancy, Stress Test (single- and multithreaded), and Snapshot. There is no dedicated tracked-vehicle demo page — [JoltPhysics.js demos](https://jrouwe.github.io/JoltPhysics.js/)
- Upstream Jolt lists tracked vehicles as an engine feature — [JoltPhysics GitHub](https://github.com/erincatto/JoltPhysics) (note: this URL is a fork/mirror; canonical is jrouwe/JoltPhysics)
- Anecdote from a rapier.js issue: a team hit an "unreachable executed" WASM error switching to Rapier multibody joints (rapier.js 0.11.2, old) and "ended up ditching Rapier altogether for JoltPhysicsJS" — [rapier.js issue #248](https://github.com/dimforge/rapier.js/issues/248)

**Rapier (rapier.js)**
- The npm `@dimforge/rapier3d-simd` latest version is 0.21.0 (registry query, 2026-11-02). The last changelog entry we read is 0.19.3 (05 Nov 2025). 0.19.1 updated to Rapier 0.30 with sparse voxel storage, enabling "orders of magnitudes larger maps without reaching the 4GB WASM memory limit". 0.19.0 brought Rapier 0.29 performance gains for contact-heavy scenes and removed the legacy PGS solver variants. Earlier releases (0.27 beta) reworked the broad-phase — [rapier.js CHANGELOG](https://github.com/dimforge/rapier.js/blob/master/CHANGELOG.md)
- Package split: `rapier3d-simd`, `-simd-compat`, `rapier3d-deterministic`, `-deterministic-compat`. Plain `rapier3d` is now built *without* the `enhanced-determinism` feature. A fix noted that the `*-simd-compat` builds earlier were not actually emitting SIMD — [rapier.js CHANGELOG](https://github.com/dimforge/rapier.js/blob/master/CHANGELOG.md)
- SIMD builds need browser simd128 support — [rapier.js docs index](https://docsearch.algolia.com/mcp/docs/repo/dimforge/rapier.js)
- Rapier multibody joints use reduced coordinates (Featherstone-style), with motor support in `unit_multibody_joint` — [rapier docs.rs source](https://docs.rs/crate/rapier_testbed3d-f64/latest/source/src/dynamics/joint/multibody_joint/mod.rs). rapier.js exposes a `MultibodyJointSet` — [rapier.js src](https://github.com/dimforge/rapier.js/blob/master/src.ts/dynamics/multibody_joint_set.ts)
- A Rapier WASM testbed claims it is fully deterministic on every browser — [apg-rpr](https://github.com/Pangeli70/apg-rpr)

**PhysX 5 (physx-js-webidl, by fabmax; community project, not NVIDIA-official)**
- The README (fetched 2026-11-02) says "Javascript/WASM bindings for Nvidia PhysX 5.12.0" (an earlier search snippet said 5.3.1, so the version string has moved). Checked coverage: all geometry incl. height field and triangle mesh; all joints (revolute, spherical, prismatic, fixed, distance, D6); **Articulations**; **Vehicles**; character controllers; serialization. CUDA/GPU features are excluded because they are impossible in WASM. The demos are in fabmax's Kotlin "kool" engine (vehicle, terrain character, ragdolls, a chain over two gears) — [physx-js-webidl README](https://github.com/fabmax/physx-js-webidl)
- PhysX `PxArticulationReducedCoordinate` is a tree of bodies "parametrized in reduced (joint) coordinates", with joint drives — [NVIDIA PhysX 5.4.1 docs](https://nvidia-omniverse.github.io/PhysX/physx/5.4.1/_api_build/classPxArticulationReducedCoordinate.html)
- I found no official NVIDIA web build of PhysX.

**MuJoCo**
- There are official JS/WASM bindings: npm `@mujoco/mujoco`, "developed and maintained by Google DeepMind and are always up to date". The npm latest is 3.17.0 (registry query, 2026-11-02); the README we read was for 3.9.0 — [@mujoco/mujoco README](https://cdn.jsdelivr.net/npm/@mujoco/mujoco@3.9.0/README.md)
- The MuJoCo main README lists zalo/mujoco_wasm as a third-party binding. zalo/mujoco_wasm now targets MuJoCo 3.3.8 and pulls in the official bindings. krishpop/mujoco_wasm is old (MuJoCo 2.3.1) — [google-deepmind/mujoco](https://github.com/google-deepmind/mujoco); [zalo/mujoco_wasm index](https://docsearch.algolia.com/mcp/docs/repo/zalo/mujoco_wasm); [krishpop/mujoco_wasm](https://github.com/krishpop/mujoco_wasm)

**Havok (Babylon.js), Ammo.js, cannon-es**
- Havok's WASM build is free under MIT but "close sourced with little to no documentation". The Havok team claims 50x the performance of ammo.js (a vendor claim). Forum users dispute how much of that comes from sleeping — [three.js forum Havok thread](https://discourse.threejs.org/t/anyone-used-the-havok-physics-engine-for-three/53205)
- In one informal forum comparison, Ammo and PhysX were "the only ones that calculate reasonably realistic gravity", at a performance cost. Rapier and Oimo felt faster but were seen as "cutting corners" — [three.js forum](https://discourse.threejs.org/t/anyone-used-the-havok-physics-engine-for-three/53205?page=2)
- An academic WebGL engine comparison (older) found Bullet (ammo.js) the best choice among cannon.js, bullet.js and jiglib.js — [BINUS journal](https://journal.binus.ac.id/index.php/jggag/article/download/7241/4134/38407)
- A three.js open-world engine uses an extended Cannon.js for tire-vs-static-terrain vehicles — [three.js forum](https://discourse.threejs.org/t/three-js-3d-browser-game-engine/87760)
- No rigorous head-to-head browser benchmark of Rapier, Jolt, Havok and PhysX was found. Tutorial sites give contradictory claims, e.g. calling Rapier "most performant" and also saying it "struggles with >100 bodies" — [cinevva tutorial](https://app.cinevva.com/tutorials/game-physics-libraries) (low-quality source)

### Inferences
- **Excavator boom/stick/bucket:** use PhysX articulations (reduced-coordinate, stiff drives, no joint drift), MuJoCo (actuators/tendons, very accurate but no vehicle or terrain-soil layer), Rapier multibody, or Jolt hinge+motor constraints / Powered Rig. Hydraulic cylinders are best modelled as position/velocity-driven joint motors with force limits in any of these. Jolt and PhysX also have prismatic/slider constraints for explicit cylinder geometry.
- **Tracked vehicles (dozers, excavator undercarriage):** only Jolt exposes a ready-made TrackedVehicleController in JS. PhysX vehicles (PxVehicle2) are wheeled-focused. Rapier offers only a DynamicRayCastVehicleController (from memory, not verified here) and no tracks.
- **Crane cables:** use Jolt Cosserat rod soft bodies or chains of capsule bodies with distance/pulley constraints, or PhysX articulation chains / distance joints. None of the sources show a dedicated crane-rope demo.
- **Determinism:** Rapier `-deterministic` packages and Jolt CROSS_PLATFORM_DETERMINISTIC are the routes to lockstep or replay.
- **Recommended stack:** Jolt multithreaded WASM for the rigid/vehicle layer. Add MuJoCo WASM only if research-grade arm dynamics or RL-policy playback is needed.

### Gaps
- No published, numeric, same-hardware browser benchmark (bodies vs FPS) across Jolt/Rapier/PhysX/Havok WASM was found. Users must benchmark the Jolt "Stress Test (Multi Threaded)" demo themselves.
- Whether Jolt TrackedVehicleController works in JS without bugs is unverified, since there is no demo. Not checked: whether rapier.js exposes multibody joint motors fully, and whether rapier.js now has a WASM-threads build (believed single-threaded).
- MJX (JAX) cannot run in the browser. No source was needed for this; it is a design fact.

## Q2. GPU particle simulation in the browser (MLS-MPM, PB-MPM, SPH, DEM, PBD sand): particle counts and fidelity

### Takeaway
WebGPU MLS-MPM is proven in the browser at about 100k particles on integrated GPUs and about 300k on "decent" discrete GPUs, in real time with 2 substeps per frame (fluids). EA SEED's PB-MPM has an official WebGPU reference implementation (SIGGRAPH 2024). Dimforge's "slosh" (GPU MPM coupled with Rapier rigid bodies, sand examples) is the most relevant engine-level project. I found no public WebGPU Drucker-Prager sand demo with published numbers.

### Cited Findings
- WebGPU-Ocean (matsuoka-601) reached MLS-MPM at about 100,000 particles on integrated graphics and about 300,000 on "decent GPUs", in real time with 2 sim steps per frame. The earlier SPH version managed only about 30,000 on iGPU. P2G scatter uses `atomicAdd` on **fixed-point i32** (floats scaled by a constant such as 1e-7), because WebGPU atomics are integer-only; the technique is credited to EA's pbmpm. Volume is recalculated each step to allow larger dt. Known issues: occasional explosions from the timestep, and the SPH mode crashes on Mac. Live demo: webgpu-ocean.netlify.app — [WebGPU-Ocean GitHub](https://github.com/matsuoka-601/WebGPU-Ocean)
- A Codrops write-up (Feb 2025) by the same author describes WebGPU-Ocean and WaterBall as MLS-MPM running 100k particles in real time on an AMD Ryzen 7 5825U iGPU. In MPM, material points interact only through a background grid — [Codrops](https://tympanus.net/codrops/2025/02/26/webgpu-fluid-simulations-high-performance-real-time-rendering/) (seen via search snippet; the page returned 403 on fetch)
- EA SEED `pbmpm` is a WebGPU implementation of Position Based MPM (Chris Lewin, SIGGRAPH 2024), BSD-3 licensed. The main branch is heavily optimized; the `siggraph2024` branch is readable. Demo scenes include blockCrusher, coiling, colliders, splashing. The README says "Firefox will not work". About 281 stars; last push around Oct 2024 per an aggregator. The README gives no particle counts — [electronicarts/pbmpm](https://github.com/electronicarts/pbmpm); [gittrend](https://gittrend.io/repo/electronicarts/pbmpm)
- Holtsetio "flow" is an MLS-MPM particle lab in three.js WebGPURenderer (TSL), derived from matsuoka-601's work — [reddit r/GraphicsProgramming mirror](https://redlib.hbubli.cc/r/GraphicsProgramming/duplicates/1kgsfyu); repo github.com/holtsetio/flow
- A three.js forum MLS-MPM project hit SPH limits at about 100k particles and switched to MPM. It has no rigid coupling yet ("Next up"). Demo: works.utsubo.com/labs/aoi-01 — [three.js forum](https://discourse.threejs.org/t/mls-mpm-simulation-with-webgpu-and-threejs/77686)
- Other WebGPU MPM work: chetan-parthiban/WebGPUMPM — [GitHub](https://github.com/chetan-parthiban/WebGPUMPM)
- Falling-sand cellular automata on WebGPU compute (2D, not physical granular mechanics) — [damiensgit/sand-simulator](https://github.com/damiensgit/sand-simulator). The SimoneDev Patreon sells three.js/WebGPU "sand" scenes (sand-cubes, sand-vortex); the solver is undisclosed — [Patreon](https://www.patreon.com/SimoneDev/posts/gpu-particles-164251113)
- **slosh (dimforge)** is described as "Cross-platform GPU 3D Material Point Method implementation". slosh3d is at 0.10.0 (crates.io, updated 2026-11-02). It has sand2/sand3 and heightfield3 examples that couple sand particles with Rapier rigid bodies (fixed platforms, kinematic rotating blades). The current README requires the Slang compiler (v2025.16) for shaders — [slosh README](https://github.com/dimforge/slosh); [slosh3d sand3 example](https://docs.rs/crate/slosh3d/latest/source/examples/sand3.rs); [slosh3d heightfield3](https://docs.rs/crate/slosh3d/0.5.0/source/examples/heightfield3.rs)
- I found no WebGPU Drucker-Prager sand demo with published numbers (search returned only fluid demos) — [search result e.g. indiana-dev WebGPU-Fluid-Simulation](https://github.com/indiana-dev/WebGPU-Fluid-Simulation)

### Inferences
- **Particle budgets:** about 300k MPM particles is a credible real-time budget on a desktop discrete GPU in a browser. A 3D sand pile at about 1–2 cm particle spacing covers only a few cubic metres, so MPM soil should be local (an "active digging zone" around the bucket). The rest of the site should be a heightfield, with conversion between heightfield and particles at the zone boundary.
- **Drucker-Prager:** this is a per-particle return-mapping step on the SVD of F, and is straightforward to add to an MLS-MPM WGSL shader. 3x3 SVD in WGSL must be hand-written. PB-MPM's formulation is designed for stable large timesteps, which suits game-rate sand.
- **Rendering:** screen-space fluid rendering suits water, not sand. Sand needs instanced point sprites or surface reconstruction / heightfield splatting.

### Gaps
- No measured FPS/particle-count figures for PB-MPM in WebGPU, slosh in browser, or any WebGPU DEM/PBD-sand (Macklin unified particles) implementation were found.
- Whether slosh's current Slang-based pipeline still targets WebGPU/browser (earlier versions were wgpu/WGSL) was not verified.
- Heightfield angle-of-repose cellular-automata browser demos were not found in this pass, although the technique is simple to implement in a compute shader.

## Q3. Two-way coupling of rigid bodies (excavator bucket) with GPU particles in the browser

### Takeaway
I found no finished browser excavator-bucket and GPU-soil two-way coupling demo. The closest building blocks are dimforge slosh (MPM sand plus Rapier rigid bodies, including kinematic moving colliders) and EA pbmpm's "colliders" scenes. Most WebGPU MPM demos list rigid coupling as future work.

### Cited Findings
- slosh sand examples couple MPM sand with Rapier rigid bodies, including kinematic rotating blades, the analog of a moving bucket — [slosh3d sand3](https://docs.rs/crate/slosh3d/latest/source/examples/sand3.rs); [slosh2d sand2](https://docs.rs/crate/slosh2d/latest/source/examples/sand2.rs)
- In the three.js-forum MLS-MPM project, collisions/interactions are "Next up", i.e. not implemented — [three.js forum](https://discourse.threejs.org/t/mls-mpm-simulation-with-webgpu-and-threejs/77686)
- The EA pbmpm repo shows a "colliders" demo scene (GIF name only) — [electronicarts/pbmpm](https://github.com/electronicarts/pbmpm)

### Inferences
- **Practical architecture:** the CPU WASM rigid solver (Jolt) drives the bucket kinematically onto the GPU grid as an SDF/collider. Grid momentum exchange gives the soil reaction force and torque; these are summed via integer atomics and read back asynchronously (1–2 frames latency via mapAsync) and applied to the bucket body as an external force. Readback latency makes this "loosely" two-way. That is acceptable for bucket load and resistance, but stiff contact is risky.
- A cheaper alternative used by many games: a heightfield terrain plus a bucket-volume intersection that removes soil height into a "bucket fill" particle set and redeposits it. This avoids full MPM.

### Gaps
- There are no published numbers on readback latency or force accuracy for browser GPU-to-WASM coupling.

## Q4. Platform limits: WebGPU compute limits, WASM threads/SIMD, SharedArrayBuffer, memory, and browser/OS support (incl. Linux aarch64, DGX Spark)

### Takeaway
WebGPU default limits are modest: 128 MiB storage binding, 256 MiB buffer, 16 KiB workgroup memory, and no float atomics (use fixed-point i32). Adapters can request higher limits. wasm32 caps at 4 GiB. Memory64 is shipped in Chrome 133+ and Firefox 134+ but not Safari. **Chrome on Linux ships WebGPU by default only for Intel Gen12+ (Chrome 144+) and NVIDIA on Wayland with driver ≥535.183.01 (Chrome 147+). Linux ARM/aarch64 (e.g. DGX Spark) is not listed, so it presumably needs `--enable-unsafe-webgpu`.** Firefox Linux WebGPU is Nightly-only.

### Cited Findings
- WebGPU defaults: `maxStorageBufferBindingSize` 134,217,728 B (128 MiB), `maxBufferSize` 256 MiB, `maxComputeWorkgroupStorageSize` 16,384 B. Browsers may report tiered values rather than exact hardware limits, to reduce fingerprinting — [MDN GPUSupportedLimits](https://developer.mozilla.org/en-US/docs/Web/API/GPUSupportedLimits); discussion in [gpuweb #1135](https://github.com/gpuweb/gpuweb/issues/1135)
- WebGPU atomics are 32-bit integer only, which is why MPM P2G uses fixed-point encoding — [WebGPU-Ocean](https://github.com/matsuoka-601/WebGPU-Ocean)
- Implementation status (wiki last edited 2 Oct 2026):
  - Chromium has shipped on Mac, Windows x86/x64 and ChromeOS since 113.
  - Chromium Linux: Intel Gen12+ since 144; NVIDIA on Wayland (driver 535.183.01+) since 147; other Linux hardware is behind `--enable-unsafe-webgpu` plus Vulkan/ANGLE flags. Linux ARM is not explicitly listed.
  - Chromium Windows ARM64 is behind a flag.
  - Chromium Android: ARM/Qualcomm/Intel since 121 (Android 12+), Imagination since 139.
  - Firefox: Windows since 141 (July 2025). macOS 145 for macOS 26+ and 147 for all macOS (ambiguous table). Linux is Nightly-only, with Mozilla expecting to ship on Linux in 2026. Android is behind a flag.
  - Safari 26: on by default on macOS/iOS/iPadOS/visionOS.
  
  — [gpuweb Implementation Status](https://github.com/gpuweb/gpuweb/wiki/Implementation-Status)
- In Nov 2025, web.dev said WebGPU was available in all major browsers, with Firefox Linux/Android/Intel-Mac support "in progress" — [web.dev](https://web.dev/blog/webgpu-supported-major-browsers)
- wasm32 linear memory maxes at 65,536 pages = 4 GiB. Memory64 (64-bit indexes) shipped in Chrome 133 (2025-02-04) and Firefox 134 (2025-01-07). Safari is marked unsupported, so availability is "Limited" — [web-features explorer](https://web-platform-dx.github.io/web-features-explorer/features/wasm-memory64); [memory64 overview](https://github.com/WebAssembly/memory64/blob/main/proposals/memory64/Overview.md); Safari status conflicts in [platform.uno State of Wasm](https://platform.uno/blog/state-of-webassembly-2024-2025/)
- WASM SIMD (simd128) has been supported in all major browsers since Safari 16.4 (March 2023) — [JoltPhysics.js README](https://github.com/jrouwe/JoltPhysics.js)
- Rapier explicitly cites the 4 GB WASM memory limit as a constraint for large voxel maps — [rapier.js CHANGELOG](https://github.com/dimforge/rapier.js/blob/master/CHANGELOG.md)

### Inferences
- Multithreaded Jolt WASM requires SharedArrayBuffer, which requires cross-origin isolation (COOP: same-origin + COEP: require-corp or credentialless headers). This is standard web-platform behaviour and was not re-sourced in this pass.
- For DGX Spark (aarch64 Linux, NVIDIA GB10): expect Chrome to need `--enable-unsafe-webgpu --enable-features=Vulkan` (possibly also Wayland). It is safest to test early, or to plan a native/Electron fallback (Electron can force-enable WebGPU flags).
- MPM grids larger than 128 MiB per binding must request a raised `maxStorageBufferBindingSize` at device creation (discrete GPUs commonly allow 1–4 GiB), or split buffers.

### Gaps
- No source specifically confirmed Chrome WebGPU on aarch64 Linux / DGX Spark.
- No WGSL float-atomics proposal status was found (WGSL still has only atomic<i32>/atomic<u32> as far as sources show).

## Q5. Terrain rendering in three.js (WebGPURenderer/TSL, clipmaps, virtual texturing, Gaussian splats) and deformable heightmaps

### Takeaway
three.js r186 merged a native WebGPU/TSL Gaussian splat renderer, and World Labs' Spark (v2.4.0, WebGL2) now has an experimental WebGPU sibling "Spark Radix". Splats are suited to photoreal static site capture (surroundings, existing structures). Deformable ground still needs a mesh/heightfield (clipmap or chunked LOD) updated from compute. I found no ready-made three.js WebGPU clipmap + deformation package in this pass.

### Cited Findings
- three.js r186 merged a native Gaussian splat renderer for WebGPU/TSL, with a GPU counting sort and PLY, SPZ, KSPLAT and glTF loaders — [radiancefields.com](https://radiancefields.com/three.js-merges-a-native-gaussian-splat-renderer-for-webgpu-in-r186)
- Spark Radix 0.0.1 is World Labs' experimental WebGPU splat renderer for three.js (MIT). Spark 2.4.0 targets WebGL2 and sorts on a worker — [radiancefields.com](https://radiancefields.com/spark-radix-brings-webgpu-splat-rendering-to-threejs); [sparkjsdev/spark](https://github.com/sparkjsdev/spark)
- Real-time splat deformation is an open feature request in Spark (#229) — [Spark issue 229](https://github.com/sparkjsdev/spark/issues/229)
- "Gaussian Splat Lite 1.0" for three.js offers WebGPU, streaming LOD and depth rendering — [three.js forum](https://discourse.threejs.org/t/gaussian-splat-lite-1-0-webgpu-streaming-lod-and-depth-rendering-for-three-js/94234)
- Jolt exposes HeightField shapes (there is a Height Field Shape demo), and PhysX bindings include height fields — [Jolt demos](https://jrouwe.github.io/JoltPhysics.js/); [physx-js-webidl](https://github.com/fabmax/physx-js-webidl). Rapier voxel shapes now scale far larger (sparse storage) — [rapier.js CHANGELOG](https://github.com/dimforge/rapier.js/blob/master/CHANGELOG.md)

### Inferences
- **Recommended setup:** a splat capture of the static site (Spark/three.js r186) composited with a GPU-displaced heightfield clipmap (TSL vertex displacement from a storage texture). Excavation edits the storage texture in a compute pass. A small dirty region is read back (or mirrored on the CPU) to update the Jolt HeightField collision, using sub-rectangle updates, not full rebuilds. Depth compositing between splats and meshes requires a splat renderer with depth output (Gaussian Splat Lite claims this).
- Splats cannot be "dug". Areas to be excavated should be mesh terrain, not splat.

### Gaps
- No specific three.js WebGPU clipmap or virtual-texturing library was identified. Whether Jolt-JS HeightField supports partial in-place updates (`SetHeights`) from JS was not verified.

## Q6. Complete browser excavator/construction simulators and their physics

### Takeaway
No complete, publicly documented browser-based excavator or construction simulator with real soil physics was found. Existing web vehicle demos (PlayCanvas, Cannon-based three.js engines, fabmax kool PhysX vehicle) cover vehicles on terrain but not digging.

### Cited Findings
- Searches for browser excavator simulators returned only generic vehicle-physics demos: a PlayCanvas vehicle sim on HF Spaces [HF Space](https://huggingface.co/spaces/madara-uchiha/PhysicsPlayCanvas/raw/main/README.md) and a three.js engine with Cannon vehicles [three.js forum](https://discourse.threejs.org/t/three-js-3d-browser-game-engine/87760)
- fabmax kool demos run PhysX-WASM vehicles and terrain characters in the browser — [physx-js-webidl README](https://github.com/fabmax/physx-js-webidl)
- slosh's kinematic-blade-in-sand examples are the closest public "digging" physics, but they are not a browser excavator product — [slosh3d sand3](https://docs.rs/crate/slosh3d/latest/source/examples/sand3.rs)

### Inferences
- A browser construction simulator in Nov 2026 would be novel. The feasible fidelity tier is:
  - Jolt (multithreaded, SIMD) for about a dozen articulated machines, including tracked dozers/excavators and cable cranes via rods/pulleys.
  - A deformable heightfield for the whole site.
  - A local WebGPU MPM/PB-MPM soil zone (about 100k–300k particles) around the active bucket, with loosely two-way coupled forces.
  - Splat-captured surroundings.

### Gaps
- Commercial web-delivered training sims (e.g. Pixel Streaming/cloud-rendered products) were not investigated; they may exist but would not be "physics in the browser".
