# Using NVIDIA SimReady OpenUSD assets in a browser three.js stack (state as of 2 Nov 2026)

Method note: besides web and registry lookups, one real asset (`vehicle_forklift_orange_a01`, package id `...2026.10.10_190706338705`) was downloaded from `nvidia/simready-assets` and inspected on the DGX Spark (aarch64) with `usd-core` 26.11 (the 19 MB `payloads/geometries.usd` was skipped). Findings marked "[inspected]" come from that sample. Their source is the dataset tree API, https://huggingface.co/api/datasets/nvidia/simready-assets/tree/main/vehicle_forklift_orange_a01?recursive=true, and the files under https://huggingface.co/datasets/nvidia/simready-assets/resolve/main/vehicle_forklift_orange_a01/. One asset is not proof that all 2,677 share every detail.

## Q1. Browser USD loading options and their current state

### Takeaway
Three routes can load USD in the browser today:
- **three.js r187 `USDLoader`:** now parses USDA and USDC, with a basic composer. It only resolves references and payloads that are inside the same USDZ or are pre-supplied, it ignores physics schemas, and it has no UDIM support.
- **Full OpenUSD-in-WASM runtimes:** `@needle-tools/usd` 1.1.2 (OpenUSD 26.05 + Hydra→three.js) gives true composition and MaterialX. It needs COOP/COEP headers. Its npm license is PolyForm-Noncommercial-1.0.0, which matters for a commercial product.
- **`@openusd-wasm/*` 0.0.17:** an MIT-licensed full pxr runtime, but very young.

NVIDIA's own web route (ovrtx + ovstream) streams server-rendered RTX video over WebRTC. It is not client-side three.js.

### Cited Findings
**three.js `USDLoader`**
- three.js latest release is **r187, published 2026-10-30** — [GitHub API releases/latest](https://api.github.com/repos/mrdoob/three.js/releases/latest)
- In r187, `examples/jsm/loaders/usd/` has `USDAParser.js` (~20 KB), `USDCParser.js` (~43 KB) and `USDComposer.js` (~113 KB, 4,660 lines) — [three.js repo contents](https://github.com/mrdoob/three.js/tree/r187/examples/jsm/loaders/usd)
- The r187 `USDLoader.js` doc comment reads: "A loader for the USD format (USD, USDA, USDC, USDZ). Supports both ASCII (USDA) and binary (USDC) USD files, as well as USDZ archives containing either format" — [USDLoader.js r187](https://github.com/mrdoob/three.js/blob/r187/examples/jsm/loaders/USDLoader.js); [threejs.org docs](https://threejs.org/docs/pages/USDLoader.html)
  - Conflict: older doc snapshots and a third-party guide say USDC inside USDZ was not supported and only USDA was. That reflects older releases — [Cinevva tech report](https://app.cinevva.com/guides/threejs-usdc-tech-report.html)
- What r187 `USDComposer.js` handles:
  - variantSets and variant selections, including externally supplied selections (`externalVariantSelections`)
  - `references` and `payload` arcs
  - UsdPreviewSurface, UsdUVTexture and UsdTransform2d
  - some MaterialX `ND_image_*` nodes
  - SkelRoot, Skeleton and SkelAnimation
  - Source: [USDComposer.js r187](https://github.com/mrdoob/three.js/blob/r187/examples/jsm/loaders/usd/USDComposer.js) (grep of source)
- What r187 `USDComposer.js` lacks: no handling of `subLayers`, `UDIM`, `mdl` or any `physics:` attributes was found in the source — [USDComposer.js r187](https://github.com/mrdoob/three.js/blob/r187/examples/jsm/loaders/usd/USDComposer.js)
- `_resolveReference()` resolves `@file@</prim>` only against `this.assets`, the pre-parsed files of a USDZ archive. If the file is not in that map it returns `null`. It does not fetch sibling files over HTTP — [USDComposer.js r187](https://github.com/mrdoob/three.js/blob/r187/examples/jsm/loaders/usd/USDComposer.js)
- Third-party pure-JS USD parser `@cinevva/usdjs` is self-described as a TS reimplementation with a Crate (USDC) parser. This is a vendor claim, and no npm package by that name was found — [Cinevva](https://app.cinevva.com/guides/threejs-usdc-tech-report.html)

**`@needle-tools/usd` (Needle)**
- Needle documents `@needle-tools/usd` as "generally available as 1.0". Details from the same page:
  - Runtime: OpenUSD **26.05** WASM with MaterialX **1.39.5** and a Needle-maintained three.js Hydra render delegate.
  - Supported features: references, payloads, variants, nested packages, native instancing and PointInstancer, purpose filtering (render/proxy/guide), UsdLux, UsdPreviewSurface and MaterialX, OpenSubdiv, usdDraco, and the Adobe usdGltf plugin.
  - Requirements: COOP `same-origin` and COEP `require-corp`, because it uses SharedArrayBuffer. For multi-file stages, `getFiles` must list the root first.
  - Page last updated 2026-07-14.
  - Source: [Needle docs: OpenUSD](https://engine.needle.tools/docs/cloud/openusd)
- npm `@needle-tools/usd` latest is **1.1.2 (2026-07-17)**, with **license field `PolyForm-Noncommercial-1.0.0`** — [npm registry](https://registry.npmjs.org/@needle-tools/usd)
- The open-source `needle-tools/usd-viewer`:
  - It is OpenUSD WASM bindings originally from Autodesk plus a three.js Hydra delegate, `ThreeJsRenderDelegate.js`.
  - The early Autodesk-based version lacked MaterialX and LightsAPI. Later Needle releases added MaterialX.
  - Source: [github.com/needle-tools/usd-viewer](https://www.github.com/needle-tools/usd-viewer)

**`@openusd-wasm/*`**
- Packages: `@openusd-wasm/core`, `/pxr`, `/three-loader` and `/utils`, version **0.0.17 (2026-10-26)**, **MIT**, first published 2026-06-08.
- They describe themselves as "Community OpenUSD pxr WebAssembly bindings" with a "Three.js loader for OpenUSD assets backed by @openusd-wasm/pxr".
- The core wasm package is ~13 MB unpacked, built with WASM pthreads, and needs COOP, COEP and CORP headers.
- Source: [npm @openusd-wasm/core](https://registry.npmjs.org/@openusd-wasm/core); [npm search](https://registry.npmjs.org/-/v1/search?text=openusd%20wasm)

**NVIDIA ovrtx + ovstream**
- ovrtx is a C/Python library for RTX rendering and sensor simulation. ovstream carries video, audio and input over WebRTC, RTSP or StreamSDK with NVENC.
- The NVIDIA tutorial pattern is: USD → ovrtx renders on the server → ovstream WebRTC → React client.
- NVIDIA's "Omniverse Realtime Viewer" agent skill says the browser should not render USD geometry and should not swap in three.js or Babylon.
- Sources: [NVIDIA-Omniverse GitHub](https://github.com/NVIDIA-Omniverse); [NVIDIA bootcamp lab](https://docs.nvidia.com/learning/physical-ai/physical-ai-agent-bootcamp/latest/lab-1-rtx-viewport/open-the-attic-portal.html); [skill page](https://skillselion.com/skills/nvidia/skills/omniverse-realtime-viewer). This comes from a search summary, and exact dates were not verified.

### Inferences
- A SimReady asset is a multi-file layer tree **[inspected]**:
  - The root `sm_*.usd` is binary crate ("PXR-USDC", crate version 0.8.0), Z-up, metersPerUnit=1, kilogramsPerUnit=1, kind=component.
  - It references `payloads/base.usda`.
  - It has three variantSets, `MuJoCo`, `Newton` and `PhysX`, each `Enabled`/`Disabled`. Their payloads are `payloads/<Engine>/{enabled,disabled}.usda`. The default selection is PhysX=Enabled and the others Disabled.
  - base.usda references `instances.usda` → `geometries.usd`. Materials live in `materials.usda`.
- three.js `USDLoader` therefore cannot load the loose folder over HTTP. You would first package each asset into a USDZ, for example with `usdzip` or `UsdUtils.CreateNewUsdzPackage` from usd-core, and then hope its simplified composer flattens the arcs correctly.
- A more robust option is to flatten offline (`usdcat --flatten`, or `Usd.Stage.Export` after choosing variants) to one USDC or USDA, then package it as USDZ. Physics would still be ignored by the loader, and UDIM textures (used by the forklift) would not resolve.
- For a commercial web app, the most practical render path is **offline conversion to GLB**, or using the existing `codepawl/simready-glb` conversion (see Q3). In-browser USD runtimes are better suited to a "view the original USD" feature.
  - Before using `@needle-tools/usd` commercially, check its license (PolyForm Noncommercial on npm) with Needle.
  - `@openusd-wasm` is MIT but pre-1.0.

### Gaps
- **Babylon.js and PlayCanvas:** no native USD loader for either was found in this pass. Babylon's ecosystem centres on glTF. Not verified for Nov 2026.
- **Autodesk's original usd-wasm repo:** current maintenance status was not checked.
- **WASM download size of `@needle-tools/usd`:** not documented on the page.
- **three.js r187 composer vs. the real forklift:** the composer was not tested on the forklift (that would need a USDZ package). Behaviour with `over` prims, instanceable prims and the `prepend apiSchemas` patterns is unverified.

## Q2. MDL materials in SimReady assets and conversion to UsdPreviewSurface / glTF PBR

### Takeaway
SimReady materials are **dual-authored** **[inspected]**. Each `Material` has an `outputs:mdl:surface` (OmniPBR.mdl or OmniGlass.mdl) and also a standard `outputs:surface` driven by a `UsdPreviewSurface` network with UsdUVTexture nodes. Browser loaders can therefore ignore MDL and use the PreviewSurface path, which maps almost 1:1 to glTF metallic-roughness PBR. The main gotcha is **UDIM** texture tiles.

### Cited Findings
These details come from the forklift's `payloads/materials.usda` **[inspected]** ([file](https://huggingface.co/datasets/nvidia/simready-assets/resolve/main/vehicle_forklift_orange_a01/simready_usd/payloads/materials.usda)):
- The MDL shader is `info:id = "mdl:OmniPBR"` with `info:mdl:sourceAsset = @../source_assets/OmniPBR.mdl@`. Glass uses `mdl:OmniGlass`.
- In the same Material, `outputs:surface.connect` points at a `UsdPreviewSurface` shader (named `Principled_BSDF`), with UsdUVTexture, UsdTransform2d and UsdPrimvarReader_float2 nodes.
- Textures are packed ORM: metallic ← ORM.b, roughness ← ORM.g. Texture paths use UDIM, for example `@../Textures/T_Forklift_A1_Albedo.<UDIM>.png@` with tiles 1001 to 1003. Color spaces are sRGB for albedo and raw for ORM and normal.
- `simready_usd/source_assets/` holds a second copy of the textures for the MDL shaders — [codepawl/simready-glb README](https://huggingface.co/datasets/codepawl/simready-glb)
- The three.js r187 composer reads UsdPreviewSurface and UsdUVTexture and no MDL — [USDComposer.js](https://github.com/mrdoob/three.js/blob/r187/examples/jsm/loaders/usd/USDComposer.js)
- `codepawl` notes that Blender's glTF exporter drops faces of some UDIM tiles. They split each UDIM tile into its own material and moved its UVs into 0–1 — [codepawl/simready-glb README](https://huggingface.co/datasets/codepawl/simready-glb)

### Inferences
- No MDL→PBR baking is needed for the main look. Map PreviewSurface diffuseColor→baseColor and ORM→glTF `metallicRoughnessTexture`, which uses G=roughness and B=metallic, so the packing already matches glTF. Normal maps carry over.
- OmniGlass transmission and IOR may look better than the PreviewSurface fallback. The glTF counterparts would be `KHR_materials_transmission` and `KHR_materials_ior`. This mapping was not verified per asset.

### Gaps
- No NVIDIA-official OmniPBR→glTF converter was confirmed in this pass.
- Some SimReady assets may be MDL-only, without a PreviewSurface fallback. Only one asset was inspected.

## Q3. USD → glTF/GLB conversion pipelines, and what is lost

### Takeaway
A complete, web-optimized conversion of this exact dataset already exists: **`codepawl/simready-glb`** (CC-BY-4.0). It covers 2,595 assets, 3.2 GB in total, with meshopt geometry and WebP textures.
- It uses Blender 5.2 USD import, then glTF Transform 4.5.
- It **re-injects physics read directly from the USD layers**, as `KHR_physics_rigid_bodies` plus node `extras`/`userData` (joints, mass, convex hulls).
- Generic converters (Blender, usd2gltf) drop all UsdPhysics data. That is the information that must be preserved by a sidecar or glTF physics extension.

### Cited Findings
- `codepawl/simready-glb` dataset metadata: last modified 2026-11-01, 12,995 downloads, 53 likes, `license:cc-by-4.0` — [HF API](https://huggingface.co/api/datasets/codepawl/simready-glb)
- Everything below comes from the dataset README — [README](https://huggingface.co/datasets/codepawl/simready-glb)

**Scale and file sizes**
- 2,595 assets.
- 228.1 GB of Blender-exported GLB reduced to 3.2 GB.
- 821M triangles reduced to 415M.
- Maximum texture edge 4096 px reduced to 1024 px.
- Example: the V8 engine went from 888 MB to 7.4 MB, and its first frame in Chrome from 9.4 s to 0.57 s.
- Median file is 0.416 MB.

**Loading in three.js**
- Files use `EXT_meshopt_compression` and `EXT_texture_webp`.
- three.js needs `GLTFLoader.setMeshoptDecoder(MeshoptDecoder)`.

**Pipeline**
1. Download `simready_usd/` and skip `source_assets/`.
2. Check that LICENSE.txt is CC BY 4.0.
3. Import into Blender 5.2, remove collision, proxy and grasp helper prims, and export GLB.
4. glTF Transform 4.5: dedup, prune and weld.
5. Convert textures to 1024 px WebP and apply meshopt at the high level.
6. Read rigid bodies, mass, physics materials and joints from the USD layers. Write them into the GLB JSON, rotating Z-up to Y-up.
7. Convex-decompose concave parts and append the hulls.

**Physics carried in the GLB**
- 2,587 files carry rigid bodies, mass and physics materials. 929 files carry joints, 5,550 in all.
- The data is in `KHR_physics_rigid_bodies`:
  - a `motion` with mass, centre of mass and inertia
  - a convex-hull `collider` with friction and restitution
  - joint limits
  - 2,145 joint `drives` in 333 files
- Node extras, which three.js exposes as `userData`, hold `joints: [{type,parent,pivot,axis,min,max,unit}]` plus `mass` (kg) and `density` (kg/m³). Revolute limits are in degrees and prismatic limits in metres.
- `extras.collisionHulls` holds 17,507 hulls:
  - 2,852 of 8,496 parts are concave.
  - A concave part gets up to 8 hulls of at most 32 points. Other hulls have at most 48 points.
  - Hull error is a median of 3.9 mm, 12 mm at p95 and 177 mm worst case.
  - Near-flat hull sides are snapped flat, which can move points outward by up to 2 cm.

**Validation and versions**
- A Newton 1.7 loader, `newton_loader/simready_newton` (`add_simready_glb`), is included.
- 43 files were simulated. Mass, CoM, joint type, pivot, axis, limits, friction and restitution "Match" the USD. 43 of 43 stayed finite when dropped.
- A tiny inertia floor is applied: 1e-4 kg·m² for jointed parts and 1e-6 kg·m² otherwise.
- The pre-physics files are kept at the `v1` git tag.

**Other converters**
- No generic converter preserves UsdPhysics, as the pipeline above shows. Blender's USD import was not relied on for physics: codepawl removed helper prims and re-read physics from the USD itself — [README](https://huggingface.co/datasets/codepawl/simready-glb)
- Needle Cloud converts USD to glTF with Draco, KTX2 and progressive loading — [Needle docs](https://engine.needle.tools/docs/cloud/openusd)

### Inferences
- A generic USD→glTF conversion loses all of the following unless it is re-injected:
  - RigidBodyAPI, MassAPI (mass, CoM, diagonalInertia, principalAxes) and CollisionAPI
  - MeshCollisionAPI `physics:approximation`
  - joints, with body0/body1, local frames, limits and drives
  - PhysicsMaterialAPI (friction, restitution, density)
  - semantics labels and the engine-specific `physx*`, `mjc:*` and `newton:*` attributes
- The codepawl set re-injects most of these. The README does not say it keeps per-engine approximations (for example, PhysX SDF), semantic labels, or variant choices.
- 2,595 converted vs. the "~2,677" in the request: either the source dataset has grown since the conversion, or some assets were skipped. Diff the asset lists before relying on full coverage.
- The codepawl set is third-party and unofficial. It is worth a spot check against the USD with a sidecar extractor (Q4) before relying on it.

### Gaps
- `usd2gltf` (PyPI) and NVIDIA's own Kit-based exporters were not evaluated: no version or status was gathered in this pass.
- Whether codepawl's `manifest.jsonl` or extras preserve SimReady semantic labels was not checked.

## Q4. Extracting physics data with usd-core into a JSON sidecar, and USD→MJCF/URDF paths

### Takeaway
`pip install usd-core` works on DGX Spark aarch64. Version 26.11 ships manylinux_2_28 aarch64 wheels for CPython 3.11 to 3.14, and was verified locally. The `pxr.UsdPhysics` API reads every needed field, and it also offers `UsdPhysicsLoadStageFromPrimRange`, a parsing utility that pre-digests a whole stage.

SimReady physics is authored as plain UsdPhysics in `payloads/Physics/physics.usda`, plus per-engine overlays. MuJoCo can import USD natively (experimental, source build) and could then save MJCF for MuJoCo WASM. Newton reads USD via `ModelBuilder.add_usd()`.

### Cited Findings
**Physics as authored in the sample [inspected]**
- `payloads/Physics/physics.usda`:
  - 15 prims with `PhysicsRigidBodyAPI`. Only the body prim has explicit `PhysicsMassAPI`: mass=158.818 kg, centerOfMass, diagonalInertia=(60.15, 55.66, 29.76), principalAxes. Other parts get mass from material density.
  - 7 `PhysicsMaterialAPI` materials, for example steel: density 7800, staticFriction 0.5, dynamicFriction 0.4, restitution 0.15. Rubber: 1200, 0.8, 0.8, 0.8.
  - Joints: 8 `PhysicsRevoluteJoint`, 5 `PhysicsPrismaticJoint` and 1 `PhysicsFixedJoint`, with `physics:axis`, `localPos0/1`, `lowerLimit/upperLimit` and `breakForce/Torque = inf`.
  - `PhysicsDriveAPI:angular` on 5 joints and `:linear` on 1, for example `drive:linear:physics:stiffness=20, damping=1, targetPosition=0.01`.
  - `SemanticsLabelsAPI`: `semantics:labels:material` (steel, plastic, rubber, clear_glass, paper) and a wikidata class/qcode on the root ("forklift").
  - No `PhysicsArticulationRootAPI` was found on this asset.
- Collision schemas differ per engine variant:
  - `instances_physx.usda`: `PhysicsCollisionAPI`, `PhysicsMeshCollisionAPI`, `PhysxCollisionAPI` and `PhysxSDFMeshCollisionAPI`, with `physics:approximation = "sdf"` on all 15 meshes and contactOffset/restOffset values.
  - `instances_mujoco.usda`: `MjcCollisionAPI` and `MjcMeshCollisionAPI`, with `physics:approximation = "convexHull"`, `mjc:inertia="convex"`, `mjc:maxhullvert=-1` and `mjc:group=2`.
  - `Newton/enabled.usda` adds `NewtonMaterialAPI`, with `newton:rollingFriction=0.0001` and `newton:torsionalFriction=0.005`.
  - `MuJoCo/enabled.usda` deletes `PhysicsDriveAPI` and `PhysicsJointStateAPI` from joints.
- A test extraction script using `UsdPhysics.RigidBodyAPI`, `MassAPI`, `Joint`, `RevoluteJoint`, `PrismaticJoint` and `MaterialAPI` ran successfully on aarch64. `UsdPhysics.LoadUsdPhysicsFromRange` exists but warns that it is "deprecated, use UsdPhysicsLoadStageFromPrimRange instead".

**Package versions**
- PyPI `usd-core` 26.11 has aarch64 wheels for cp311, cp312, cp313 and cp314 — [PyPI usd-core](https://pypi.org/project/usd-core/)
- Other PyPI versions as of 2 Nov 2026: `mujoco` 3.17.0, `newton` 1.7.0 and `mujoco-usd-converter` 0.8.0 — [PyPI mujoco](https://pypi.org/project/mujoco/); [PyPI newton](https://pypi.org/project/newton/); [PyPI mujoco-usd-converter](https://pypi.org/project/mujoco-usd-converter/)

**MuJoCo, Newton and converters**
- MuJoCo's OpenUSD support:
  - It is "experimental and subject to frequent change".
  - It can load USD layers via MJCF or drag-and-drop into `simulate`.
  - It primarily uses standard UsdPhysics schemas, extended by `mjcPhysics`.
  - It needs a source build with CMake `USD_DIR`.
  - Sources: [MuJoCo OpenUSD docs](https://mujoco.readthedocs.io/en/stable/OpenUSD/index.html); [mjcPhysics](https://mujoco.readthedocs.io/en/stable/OpenUSD/mjcPhysics.html)
- `mujoco-usd-converter` (newton-physics) converts **MJCF → USD**, not the reverse. It is alpha. Newton consumes USD via `newton.ModelBuilder.add_usd()` — [GitHub](https://github.com/newton-physics/mujoco-usd-converter)
- MuJoCo has official WASM bindings published to npm as `mujoco` ("MuJoCo WASM bindings", repo directory `wasm` in google-deepmind/mujoco), with releases throughout 2026. The registry's dist-tag `latest` shows 3.1.16, which looks inconsistent with the 3.4.0 entry — [npm mujoco](https://registry.npmjs.org/mujoco)

### Inferences
- Recommended sidecar extractor (offline, on Spark):
  1. Open the root `.usd` with usd-core and select the variant (`PhysX` or `MuJoCo` = Enabled) through `prim.GetVariantSet(...).SetVariantSelection`.
  2. Walk the prims and emit JSON per rigid body:
     - world or root-relative transform; convert Z-up to Y-up for three.js
     - mass/CoM/inertia when authored, otherwise density from the bound physics material (via `material:binding:physics`) × mesh volume
     - collider approximation; compute convex hulls yourself (for example with V-HACD or CoACD) when the approximation is `sdf`, because a browser engine cannot use PhysX SDF
     - friction and restitution
     - joints: type, body0/body1, localPos/Rot0/1, axis, limits (revolute limits are degrees in UsdPhysics), drive stiffness/damping/target
     - semantic labels
  3. Key the JSON by prim path or name so that it matches GLB node names.
- MuJoCo-WASM route: a USD-enabled native MuJoCo build on Spark could load the asset with the `MuJoCo=Enabled` variant and save MJCF. The MJCF and meshes would then be loaded into the npm `mujoco` WASM runtime.
  - This is unverified end-to-end, and the source build with USD on aarch64 is untested.
  - Alternative: use the codepawl GLB data (hulls, joints, mass) to generate MJCF or Rapier/Jolt bodies directly in JS.
- No maintained USD→URDF or USD→MJCF standalone converter was found in this pass, so it is a gap.

### Gaps
- Not verified: whether `mj_saveXML` after USD import produces clean MJCF for SimReady assets, and the stability of MuJoCo's USD build on aarch64.
- Whether all assets include the `MuJoCo` and `Newton` variants was not checked. Only one was inspected.

## Q5. glTF physics extensions status and engine support

### Takeaway
`KHR_physics_rigid_bodies` is still an **open draft PR (KhronosGroup/glTF #2424, opened 2024-07-10)**, and `KHR_implicit_shapes` is a separate open draft (#2370). Neither is in the Khronos registry's main branch yet, and both remain subject to change.

Importers exist for Blender, Babylon.js (community plugin with Havok) and Godot. fastgltf merged parsing support. **three.js has no built-in loader**: in three.js you read the extension JSON yourself (via a `GLTFLoader.register` plugin or `gltf.parser.json`) and feed Rapier, Jolt, Havok or MuJoCo.

### Cited Findings
- PR #2424 "KHR_physics_rigid_bodies Draft Proposal" is open and was created 2024-07-10 — [GitHub search API](https://api.github.com/search/issues?q=repo:KhronosGroup/glTF+KHR_physics_rigid_bodies); [PR #2424](https://github.com/KhronosGroup/glTF/pull/2424)
- Details from the PR page:
  - It is still a draft, not merged. In Sept 2025 the author said "the plan is to ratify this extension".
  - A label was added in Oct 2026.
  - Geometry refers to an implicit shape (`shape`), a mesh `node`, or `convexHull`. The PR also defines joints with `linearAxes` and `angularAxes` limits, triggers, and `collisionFilter`.
  - Implicit shapes moved to `KHR_implicit_shapes` (draft PR #2370).
  - Known implementations: the Blender importer/exporter by eoineoineoin, glTF_Physics_Babylon, and glTF_Physics_Godot_Importer. fastgltf merged support on 2025-04-29. three.js, Unity and Omniverse are not mentioned.
  - Source: [PR #2424](https://github.com/KhronosGroup/glTF/pull/2424)
- `extensions/2.0/Khronos/KHR_physics_rigid_bodies` returns 404 on the main branch, so it is not merged into the registry — [GitHub contents API](https://api.github.com/repos/KhronosGroup/glTF/contents/extensions/2.0/Khronos/KHR_physics_rigid_bodies)
- Babylon.js has no core loader. The community package `babylon-gltf-rigid-body-loader` provides `KHR_PhysicsRigidBodies_Plugin`:
  - It needs `scene.enablePhysics()` (Havok) first.
  - Its README warns that the KHR extension "may experience name changes which requires assets to be re-exported".
  - npm latest is 0.5.4, from 2025-07-16, which is older.
  - Sources: [Babylon forum](https://forum.babylonjs.com/t/gltf-physics-babylon/43923); [npm](https://registry.npmjs.org/babylon-gltf-rigid-body-loader)
- Recent ratification activity was elsewhere: KHR_gaussian_splatting was reportedly ratified in Sept 2026 (single source, which conflicts with another listing it as Release Candidate), and KHR_interactivity was submitted for ratification — [Khronos news](https://www.khronos.org/news/); [radiancefields.com](https://radiancefields.com/platforms/the-khronos-group)
- No `OMI_physics_*` status was gathered in this pass. The OMI extensions (OMI_physics_body, OMI_physics_shape, OMI_physics_joint) are older community drafts, mainly implemented in Godot. This comes from background knowledge, not verified this session.

### Inferences
- For three.js, treat `KHR_physics_rigid_bodies` as a convenient interchange container. Do not expect any engine to auto-instantiate it.
- Pin to the exact draft revision that codepawl wrote, because field names may still change before ratification.

### Gaps
- three.js community plugins for KHR_physics_rigid_bodies (for example with Rapier) were not searched.
- Current OMI_physics status and adoption was not verified.

## Q6. Licensing implications of CC-BY-4.0 for redistribution in a web app

### Takeaway
Each SimReady asset ships the full CC BY 4.0 legal text in `.metadata/LICENSE.txt` **[inspected]**, and the dataset card and package JSON declare `license: CC-BY-4.0`. CC BY 4.0 allows commercial use, modification (for example GLB conversion and compression) and redistribution, including serving files from a web app, provided attribution and license notice are given and changes are indicated. Separately, check the third-party runtime licenses: `@needle-tools/usd` is PolyForm-Noncommercial, while three.js, `@openusd-wasm` and usd-core are permissive.

### Cited Findings
- `vehicle_forklift_orange_a01/.metadata/LICENSE.txt` (18,657 bytes) begins "Attribution 4.0 International" and contains the "Creative Commons Attribution 4.0 International Public License" **[inspected]** — [LICENSE.txt](https://huggingface.co/datasets/nvidia/simready-assets/resolve/main/vehicle_forklift_orange_a01/.metadata/LICENSE.txt)
- `com.nvidia.simready.packaging.json` has `"license": "CC-BY-4.0"` and SHA-256 content and package hashes **[inspected]**. The dataset card front-matter has `license: cc-by-4.0` — [dataset README](https://huggingface.co/datasets/nvidia/simready-assets)
- codepawl asks users of its derivative to "credit NVIDIA as the original author and link the source dataset" — [codepawl README](https://huggingface.co/datasets/codepawl/simready-glb)
- `@needle-tools/usd` is under `PolyForm-Noncommercial-1.0.0` — [npm](https://registry.npmjs.org/@needle-tools/usd). `@openusd-wasm/*` is MIT — [npm](https://registry.npmjs.org/@openusd-wasm/core)

### Inferences
- In the app, include a credits page or in-viewer attribution listing:
  - "NVIDIA SimReady assets (nvidia/simready-assets), CC BY 4.0"
  - a link to the license
  - a statement that the models were converted or compressed
  - attribution to codepawl's conversion if it is used (also CC BY 4.0)
- CC BY 4.0 Section 2(a)(5)(C) forbids applying technological measures that restrict recipients' exercise of the licensed rights. Encrypting or DRM-locking the GLBs would conflict with this.
- CC BY covers copyright only, not trademarks. Avoid implying NVIDIA endorsement.

### Gaps
- No legal advice was obtained. The CC BY 4.0 clauses cited above are from the standard license text (bundled in LICENSE.txt), but the exact clause numbering was not re-verified line by line in this session.
- Whether any individual SimReady asset in the 2,677-asset set carries a different license was not audited. codepawl checked LICENSE.txt per asset during its conversion.
