# Open 3D assets for the construction-site simulation

Research date: 2026-11-03. I checked every license on the source page or repo metadata for the specific model. Nothing was assumed site-wide, except Kenney and Quaternius, where the site states CC0 for all assets. I tested the direct URLs with `curl -sI` or a small GET on 2026-11-03.

**Accepted licenses:** CC0, CC-BY 3.0/4.0, CC-BY-SA (flagged **[SA]**), MIT, BSD, Apache-2.0, public domain.
**Rejected:** NC, ND, editorial, store "standard" licenses, personal-use-only terms, and no license at all (all rights reserved by default).

Realism scale: **low** = stylised low-poly (<5k tris); **medium** = recognisable proportions, untextured or flat-coloured; **high** = real-machine geometry with textures.

---

## TL;DR: the excavator question

**A realistic, articulated, openly licensed excavator does exist.** The Japanese Public Works Research Institute (PWRI) **OPERA** project publishes ROS description packages under **Apache-2.0** for:

- the Hitachi **ZX120** and **ZX200** hydraulic excavators
- the Komatsu **D37PXI** bulldozer
- the Hitachi **IC120**, Morooka **MST-110CR** and Yanmar **C30R** crawler dump carriers

The meshes are textured Collada (.dae, Blender-authored, metres, Z-up). They come pre-split into body/house, crawler, boom, arm/stick, bucket and hydraulic cylinder/rod parts, with URDF/xacro joints (`swing_joint`, `boom_joint`, `arm_joint`, `bucket_joint`). Each part is mid-poly: about 15.6k tris for the ZX200 house, 11.9k for the crawler, about 1.4k each for the arm and bucket, with a baked texture. That is exactly the "split into parts + physics" shape we need.

Caveats:
- (a) The Apache-2.0 `LICENSE` file at the repo root covers the whole repo, meshes included. The READMEs say nothing separate about where the meshes came from.
- (b) The models depict trademarked products (Hitachi, Komatsu). Copyright is licensed, but strip or avoid manufacturer logos and don't imply endorsement.
- (c) Apache-2.0 needs the LICENSE text plus a NOTICE/attribution kept with redistributed copies, and modified files need a "changed" note.

Gaps that remain: no open, realistic wheel loader, mobile crane, concrete pump, concrete mixer truck, motor grader, telehandler, skid steer or backhoe in group A. Those exist only as low-poly CC0/CC-BY items (A) or as good CC-BY models on Sketchfab that need a manual logged-in download (B).

---

## (A) Directly downloadable without login + redistributable

### A1. PWRI OPERA construction machines (Apache-2.0): best realistic option

Author/attribution: "Public Works Research Institute (PWRI), OPERA project – https://github.com/pwri-opera", Apache-2.0.
License check: GitHub license API returns `Apache-2.0` for each repo listed. The LICENSE is the standard Apache-2.0 text with no copyright line filled in.

| Model | Category | Repo (page) | Direct download | Parts / articulation | Detail | Realism |
|---|---|---|---|---|---|---|
| Hitachi ZX200 | Hydraulic excavator (20 t class) | https://github.com/pwri-opera/zx200_ros2 | Repo zip: https://github.com/pwri-opera/zx200_ros2/archive/refs/heads/main.zip ; meshes e.g. https://raw.githubusercontent.com/pwri-opera/zx200_ros2/main/zx200_description/meshes/zx200-boom.dae (HTTP 200, 293 KB) | Separate `body`, `crawler`, `boom`, `arm`, `bucket`, `boom-cyl1/2`, `arm-cyl1/2`, `bucket-cyl1/2`, `part1/2` (linkage). Xacro with swing/boom/arm/bucket joints. MoveIt config and ros2_control xacros included | Body 15.6k tris, crawler 11.9k, arm 1.4k, bucket 1.35k. Baked texture `bake.png` (716 KB) | **High** (mid-poly, game-ready) |
| Hitachi ZX120 | Hydraulic excavator (12 t class) | https://github.com/pwri-opera/zx120_ros (ROS1). ROS2 variant `zx120_ros2` has no license file, so use the ROS1 repo | https://github.com/pwri-opera/zx120_ros/archive/refs/heads/main.zip ; e.g. https://raw.githubusercontent.com/pwri-opera/zx120_ros/main/zx120_description/meshes/zx120-boom.dae (200) | Same split as ZX200 plus `p1/p2`; `zx120.urdf` + `.xacro` | Body 15.3k tris; texture zx120.png is 9.2 MB | **High** |
| Komatsu D37PXI-24 | Bulldozer | https://github.com/pwri-opera/d37pxi_ros2 | https://github.com/pwri-opera/d37pxi_ros2/archive/refs/heads/main.zip ; e.g. https://raw.githubusercontent.com/pwri-opera/d37pxi_ros2/main/d37pxi_description/meshes/d37pxi24-blade.dae (200) | Separate `body`, `blade`, `blade-arm`, `blade-arm-cylinder/piston`, `blade-angle-piston-left/right`; urdf + xacro | Body 10.2k tris, textured | **High** |
| Hitachi IC120 | Crawler dump carrier (tracked dump truck) | https://github.com/pwri-opera/ic120_ros2 | https://github.com/pwri-opera/ic120_ros2/archive/refs/heads/main.zip | `body`, `vessel` (tipping bed), `cylinder1/2`; gazebo urdf | Body 23.9k tris, texture 10 MB | **High** |
| Morooka MST-110CR | Crawler carrier / dump | https://github.com/pwri-opera/mst110cr_ros2 | https://github.com/pwri-opera/mst110cr_ros2/archive/refs/heads/main.zip | `body`, `crawler`, `vessel`, `vessel_cylinder`, `vessel_rod`; urdf | Body 10.1k tris, textured | **High** |
| Yanmar C30R | Small crawler carrier | https://github.com/pwri-opera/OperaSim-PhysX (`Assets/Machines/c30r/`) | https://raw.githubusercontent.com/pwri-opera/OperaSim-PhysX/main/Assets/Machines/c30r/meshes/c30r-body.dae (200, 4.3 MB) | `body`, `vessel1/2`, `cylinder1/2` | Body 22.4k tris | **High** |
| Generic excavator + IC120 CAD (STL) | Excavator / carrier | OperaSim-PhysX `Assets/Machines/excavator_model/` and `ic120_model/` | raw.githubusercontent paths under the same repo | Multi-part STL with urdf. Origin isn't documented, so treat it as lower confidence | High-poly CAD STL (track_frame.stl 21 MB) | Medium |

Units and axes: metres, Z-up, per the DAE `<unit meter="1">` / `<up_axis>Z_UP`. These are Blender 2.9x exports. The OperaSim-PhysX/-AGX Unity projects (also Apache-2.0) contain the same meshes plus Unity ArticulationBody/AGX setups. They're useful as a reference for joint limits and masses. The AGX version needs a proprietary AGX license to *run*, but the meshes themselves are Apache.

Conversion note: DAE converts to GLB cleanly with Blender or `assimp`. Keep each `*-boom.dae` etc. as a separate node and use the URDF joint origins as pivots.

### A2. Other permissively licensed GitHub / robotics sources

| Name | Category | Page | Direct URL (tested) | License | Author | Format | Parts | Detail / realism |
|---|---|---|---|---|---|---|---|---|
| MathScavator 9000 (MathWorks autonomous excavator) | Excavator | https://github.com/mathworks-robotics/autonomous-excavator | e.g. https://raw.githubusercontent.com/mathworks-robotics/autonomous-excavator/main/ExcavatorCADFiles/URDF_MathScavator9000_flat/MathScavator9000_flat/meshes/boom_stick_link.STL (200) | **BSD-3-Clause** (repo `license.txt`) | The MathWorks, Inc. | SolidWorks SLDPRT, STL, URDF, Unreal Datasmith | Separate chassis, boom, stick, bucket, cylinders/pistons, links. URDF with joints | CAD-clean, untextured. Generic (non-branded) design, so **medium** realism. Chassis STL 11.7 MB |
| araobp Unity excavator | Excavator | https://github.com/araobp/unity-excavator | https://raw.githubusercontent.com/araobp/unity-excavator/master/Excavator/Assets/Excavator/Excavator.fbx (200, 71.7 MB) | **Apache-2.0** (repo) | araobp | FBX | Multi-axis articulated ("a lot of rotary axes"), with Unity controller scripts | Medium–high. The model's provenance isn't stated (it's probably the author's own work). Lower confidence than OPERA |
| Project Chrono bulldozer | Bulldozer | https://github.com/projectchrono/chrono/tree/main/data/models/bulldozer | https://raw.githubusercontent.com/projectchrono/chrono/main/data/models/bulldozer/bulldozerB10.obj | **BSD-3-Clause** (repo) | Project Chrono developers | OBJ (Blender 2.76) + track shoe/wheel meshes | Single body object; separate shoe and wheel | 19.3k faces, untextured, **medium** |
| Project Chrono forklift | Forklift (telehandler stand-in) | .../data/models/forklift | https://raw.githubusercontent.com/projectchrono/chrono/main/data/models/forklift/body.glb (200, 776 KB) | BSD-3-Clause | Project Chrono | GLB/OBJ | Separate body, arm (mast), forks, wheel | Medium |
| Project Chrono traffic cone, mixer drums, MAN Kat1 trucks (5t/7t/10t), "truck" | Props / trucks | .../data/models/traffic_cone, .../data/vehicle/MAN | raw.githubusercontent.com/projectchrono/chrono/main/data/... | BSD-3-Clause | Project Chrono | OBJ, .blend (cone) | Chassis/wheel split | Medium. MAN trucks are military 6x6 and could be reskinned as a dump-truck base |
| Microsoft Rocketbox avatars: **Construction_Male_01…08, Construction_Female_01** | Construction workers in PPE (hard hat, hi-vis, tools), **rigged** | https://github.com/microsoft/Microsoft-Rocketbox | e.g. https://raw.githubusercontent.com/microsoft/Microsoft-Rocketbox/master/Assets/Avatars/Professions/Construction_Male_01/Export/Construction_Male_01.fbx (200, 676 KB) plus Textures/*.tga | **MIT** (relicensed Nov 2020, per the README) | Microsoft / Rocketbox Studios | FBX (Biped skeleton, LODs), _facial.fbx variant, 3ds Max source, TGA textures | Full skeleton, facial blendshapes in `_facial` | **High** (realistic humans). Best open rigged worker set |

### A3. Gazebo Fuel (license field on each model's API record)

The download pattern `https://fuel.gazebosim.org/1.0/<owner>/models/<name>.zip` works for GET (HTTP 206 on a range request); HEAD returns 405.

| Name | Category | Page | License (Fuel field) | Owner | Format | Realism |
|---|---|---|---|---|---|---|
| Tower crane | Tower crane | https://app.gazebosim.org/OpenRobotics/fuel/models/Tower%20crane | **CC0 1.0** | OpenRobotics | DAE + diffuse/spec JPG, SDF (825 KB) | Medium (single static mesh) |
| Construction Cone | Traffic cone | .../OpenRobotics/fuel/models/Construction%20Cone | CC0 1.0 | OpenRobotics | DAE/SDF | Medium-high |
| Construction Barrel | Drum barrier | .../Construction%20Barrel | CC0 1.0 | OpenRobotics | DAE/SDF | Medium-high |
| Jersey Barrier; DRC Orange/White Jersey Barrier | Concrete barrier | .../Jersey%20Barrier | CC0 1.0 | OpenRobotics | DAE/SDF | Medium |
| Walking person / Standing person / Casual female / Walking actor | Humans (not in PPE); Walking actor is animated | .../Walking%20person etc. | CC0 1.0 | OpenRobotics, chapulina | DAE (scanned, about 26 MB) | High (but civilians) |
| actor (Mingfei / OpenRobotics "actor - relative paths") | Rigged animated human | | CC-BY 4.0 | Mingfei / OpenRobotics | DAE skinned | Medium |
| Forklift | Forklift | .../OpenRobotics/fuel/models/Forklift | CC-BY 4.0 | OpenRobotics | DAE | Medium-high |
| Euro pallet; Pallet_Standard; Dumpster; Dumpster_Large; Cinder Block | Site clutter | | CC0 / CC-BY 4.0 (see each) | OpenRobotics | DAE | Medium |
| Shipping Container (Nokoru); std_cargo_container_40ft (jfrascon) | Container / site cabin base | | CC-BY 4.0 | Nokoru, jfrascon | DAE | Medium |
| IBeam (TjalleDijkstra) | Steel beam | | CC-BY 4.0 | TjalleDijkstra | | Low-medium |
| MINI_EXCAVATOR, MINI_ROLLER, 30_/60_/ROAD_CONSTRUCTION_SET | **Toy** scans (Google Scanned Objects) | .../GoogleResearch/fuel/models/MINI_EXCAVATOR | CC-BY 4.0 | GoogleResearch | OBJ + texture | Photoreal *toys*: wrong scale, not articulated. Background props only |

Reject: MovAi `pallet`, `pallet_box_mobile` (CC-BY-NC-ND 4.0).

### A4. Poly Pizza (static GLB at static.poly.pizza; license from each model page's JSON `Licence` field)

Attribution format: "<Title> by <Creator> [CC-BY 3.0] via Poly Pizza (<page URL>)". Google Poly originals are "Poly by Google", CC-BY 3.0. The direct GLB pattern was tested: `https://static.poly.pizza/<ResourceID>.glb` returns 200 without login.

**No excavator exists on Poly Pizza.** A search for "excavator" returns bulldozers, dump trucks and cranes only.

| Name | Category | Page | Direct GLB | License | Author | Tris | Realism |
|---|---|---|---|---|---|---|---|
| Bulldozer | Bulldozer | https://poly.pizza/m/ddxtaegI3HQ | https://static.poly.pizza/4178fe17-d39d-49a1-9bad-7312216d28d5.glb (200, 160 KB) | CC-BY 3.0 | Poly by Google | 2,847 | Low |
| Bulldozer | Bulldozer | https://poly.pizza/m/eY1N7Rz9Drr | https://static.poly.pizza/4e401f2d-1000-472e-8167-0e3279b2d86d.glb | CC-BY 3.0 | Poly by Google | 2,518 | Low |
| Bulldozer | Bulldozer | https://poly.pizza/m/9-cyY0XbvKi | https://static.poly.pizza/174889a4-23e9-438c-b67f-167ea579cbdf.glb | CC-BY 3.0 | Poly by Google | 1,472 | Low |
| Dump truck | Dump truck | https://poly.pizza/m/1BpGYg14QGD | https://static.poly.pizza/1511f362-d635-4dc6-b9a2-a70fdb6370b2.glb | CC-BY 3.0 | jeremy | 2,916 | Low |
| Dump Truck | Dump truck | https://poly.pizza/m/gYWUCpb09B | https://static.poly.pizza/662368d6-79ce-47ba-82f2-073c85f785e5.glb | CC-BY 3.0 | Crachboss32 | 552 | Low |
| Crane | Mobile/truck crane | https://poly.pizza/m/gCcpjaxFdv | https://static.poly.pizza/0242ec01-e2ee-47f5-9403-4d65366f2307.glb | CC-BY 3.0 | J-Toastie | 9,408 | Low-medium |
| Crane | Crane | https://poly.pizza/m/17aBNzIsVg | https://static.poly.pizza/d3396b86-e17c-440e-98cd-8898d774a7b3.glb | **CC0 1.0** | cg_world | 5,982 | Low |
| Building construction crane | Tower crane | https://poly.pizza/m/cm5teXZ5Ctr | https://static.poly.pizza/b7a0797d-8cd6-495c-93eb-87fe7c44ed54.glb (200, 275 KB) | CC-BY 3.0 | Kieran Farr | 1,524 | Low |
| Crane | Crane | https://poly.pizza/m/aZFdGwasy60 | https://static.poly.pizza/a7ba3c60-1efe-4ca5-8e38-fed851d46a51.glb | CC-BY 3.0 | Max Hancock | 164 | Very low |
| Tractor | Tractor (loader base) | https://poly.pizza/m/eiXGnD1wN5q | https://static.poly.pizza/060b24b3-5f9d-4cf7-8a7b-a71cac5d77be.glb | CC-BY 3.0 | Poly by Google | 2,766 | Low |
| Forklift | Forklift | https://poly.pizza/m/DTQBuenKJY | https://static.poly.pizza/2923ce44-178b-49f3-9172-c3515b8b5ec8.glb | CC-BY 3.0 | KolosStudios | 1,590 | Low |
| Worker (animated) | Rigged worker | https://poly.pizza/m/Yg2bQZO6Hj | https://static.poly.pizza/3a5f3056-ffe6-42eb-bd52-122afcbd22b2.glb (200, 1.3 MB) | **CC0 1.0** | Quaternius | 3,350 | Low (stylised, animated) |
| Worker (animated) | Rigged worker | https://poly.pizza/m/E8079Ahx7k | https://static.poly.pizza/c0253218-85f2-4d67-b3f2-a4611a7901fe.glb | CC-BY 3.0 (as listed) | Quaternius | 3,422 | Low |
| Character Hazmat (animated) | Rigged worker in PPE suit | https://poly.pizza/m/z3TSQYx1Kn | https://static.poly.pizza/484450a4-b76c-4e76-95d2-352337bb41e8.glb | CC0 1.0 | Quaternius | 12,118 | Low-medium |
| Hard hat / Hardhat | PPE prop | https://poly.pizza/m/9AMKar2NlkX ; https://poly.pizza/m/cyjbHPC6QAM | https://static.poly.pizza/afc203f7-60ac-4374-87b9-60af1b722693.glb ; https://static.poly.pizza/535cb2d4-eb2a-441d-899b-d0a5ebd3e37d.glb | CC-BY 3.0 | Poly by Google | 740 / 17,564 | Low / medium |
| Scaffolding | Scaffolding | https://poly.pizza/m/AYQd5RQ8w4 | https://static.poly.pizza/79ea8731-ae53-44df-a52d-bba8a1f8d441.glb | CC0 1.0 | lakescorp | 384 | Low |
| metal scaffold | Scaffolding | https://poly.pizza/m/fETtk2OoFZe | https://static.poly.pizza/c4695591-6090-4252-b6a3-d96d68167986.glb | CC-BY 3.0 | Simon Hischier | 1,136 | Low |
| Scaffolding | Scaffolding | https://poly.pizza/m/1_PM9UWLgAb | https://static.poly.pizza/31fa3e92-7599-48cd-98d0-d92df64f681b.glb | CC-BY 3.0 | Marisha | 304 | Low |
| Shipping Container | Container / cabin | https://poly.pizza/m/dQXRtm5GbO | https://static.poly.pizza/e56c8efb-940f-4150-9bf8-d468c0165116.glb | CC0 1.0 | Quaternius | 1,336 | Low |
| Industrial Shipping Container | Container | https://poly.pizza/m/j5sQIqPke1 | https://static.poly.pizza/c668c1e0-5a22-459b-9c2d-216e08aeb941.glb | CC0 1.0 | Davidlamic000 | 1,280 | Low |
| Container Bathroom A/B/C | Site toilet/cabin | https://poly.pizza/m/urJYCyd6eL | https://static.poly.pizza/2105ba9c-e78a-4d48-b6be-ed5dabdb7683.glb | CC0 1.0 | Isa Lousberg | 296 | Low |
| Generator | Generator | https://poly.pizza/m/K58RQ63qR5 | https://static.poly.pizza/e9fc0901-7600-48f4-881d-b546f3df4cec.glb | CC-BY 3.0 | KolosStudios | 478 | Low |
| Large Electric Generator | Generator | https://poly.pizza/m/ZPlQHwiqTp | https://static.poly.pizza/e81145eb-091e-4803-9523-8611bd7e24e4.glb | CC-BY 3.0 | miro_art_studio | 15,786 | Medium |
| Construction Light | Light tower (small) | https://poly.pizza/m/22gr93Sp3c | https://static.poly.pizza/42beb3b8-2924-4507-9649-85917e5acada.glb | CC0 1.0 | Kenney | 84 | Low |
| Construction Barrier; Barricade (x2) | Barriers | https://poly.pizza/m/4mrO9ueiQr ; https://poly.pizza/m/JSTjgVIyX8 | https://static.poly.pizza/1bc2f056-361f-4074-8b8a-c1e12aa40de6.glb ; https://static.poly.pizza/aaa1966b-5c4b-4c11-86a5-66494a5ccb55.glb | CC0 1.0 | Kenney | 60 / 108 | Low |
| Plastic Barrier; Traffic Barrier; Barrier Fixed/Single | Barriers | https://poly.pizza/m/QAiXMsbWRc ; https://poly.pizza/m/nugx3heueH | https://static.poly.pizza/5285ff90-c136-4abe-9772-aa7b9244bf88.glb ; https://static.poly.pizza/17611df7-a65e-4c71-8e45-e36ecabc608b.glb | CC0 1.0 | Quaternius | 852 / 732 | Low |
| Concrete Barrier | Jersey barrier | https://poly.pizza/m/he10X5eGD0 | https://static.poly.pizza/cd3068d2-ebe7-4d2f-9da3-387fca14211e.glb | CC-BY 3.0 | J-Toastie | 164 | Low |
| Road Cone | Traffic cone | https://poly.pizza/m/ZPhinXAGtY | https://static.poly.pizza/4b1d77ea-f538-4d65-8259-763ecaa46ee7.glb | CC0 1.0 | smallbigsquare | 168 | Low |
| Traffic Cone | Traffic cone | https://poly.pizza/m/MRMsQXKz4L | https://static.poly.pizza/c80db56c-ea57-41d3-a2e3-d15853644378.glb | CC-BY 3.0 | J-Toastie | 284 | Low |
| steel beam | Steel beam | https://poly.pizza/m/3IUrN3EXi15 | https://static.poly.pizza/0c654a94-78b8-4b4a-b714-2ab4ca9a4257.glb | CC-BY 3.0 | Geoffrey Bell | 66 | Low |
| Pallet; Wooden Pallet | Pallet | https://poly.pizza/m/cUAsYHDqfD | https://static.poly.pizza/e2109848-da3e-41c0-a96f-4471e91f78d9.glb | CC0 1.0 | Quaternius | 176 | Low |
| Cinder block(s), Bricks | Materials | https://poly.pizza/m/lLkVKoJsKm ; https://poly.pizza/m/Tvlvh8AAbs | https://static.poly.pizza/8f213c50-1add-4a33-b14c-94b68cb862f1.glb ; https://static.poly.pizza/664edd5c-2799-4b04-adb4-1485950f3743.glb | CC0 1.0 | Quaternius | 368 / 484 | Low |
| Structure (x2) | Building frame under construction | https://poly.pizza/m/ilWoURnbZW ; https://poly.pizza/m/PB5Cd3dL24 | https://static.poly.pizza/52b9ad24-fe0a-4f6f-a44d-66f2bdff6f08.glb ; https://static.poly.pizza/2845deb2-647b-422c-90b7-7b4c0065b985.glb | CC0 1.0 | Quaternius | 5,123 / 3,364 | Low |
| Building frame | Building frame | https://poly.pizza/m/f6IpG9b27Eo | https://static.poly.pizza/57a61b31-4300-43c6-adb4-9b01358ebbe8.glb | CC-BY 3.0 | VR XRTIST | 1,224 | Low |

### A5. OpenGameArt (license from the "License(s)" field on each page; files download without login)

| Name | Category | Page | Direct URL (HTTP 200) | License | Author | Format | Notes / realism |
|---|---|---|---|---|---|---|---|
| Excavator | Excavator | https://opengameart.org/content/excavator | https://opengameart.org/sites/default/files/excavadora_0.blend (4.7 MB) | **CC0** | Iacox | .blend with about 1 min of animation, so likely separate parts | Low-poly university project. **Low–medium** |
| Earth mover | Wheel loader / earth mover | https://opengameart.org/content/earth-mover | https://opengameart.org/sites/default/files/EarthMover.3ds ; .../Earth%20Mover_0.7z | CC0 | Cuzco | 3DS (+7z with 3 formats) | Flat-coloured (9 hues). **Low** |
| Crane Machine | Mobile crane | https://opengameart.org/content/crane-machine | https://opengameart.org/sites/default/files/crane.blend (5.6 MB) | CC0 (re-uploaded by josepharaoh99; original by RedFrost on BlendSwap) | RedFrost / josepharaoh99 | .blend | Has a Blender logo texture. Because it's a re-upload, verify on BlendSwap if you can. **Low** |
| 3D House Construction Site (LowPoly) | **Whole site**: building, fence, 2 cranes (mounted/ground), dumper trucks, concrete trucks, small truck, containers (cargo/office), materials, ramp, road | https://opengameart.org/content/3d-house-construction-site-lowpoly-cc0 | https://opengameart.org/sites/default/files/lowpoly-house-construction-site-by-majadroid_2.zip (3.3 MB; contents verified with `unzip -l`) | CC0 | Majadroid | FBX per group + .blend | Good **low-poly** placeholder set covering mixer truck, dump truck, cranes and site cabins |
| Construction Site Kit | Scaffold 1–4, stairs, planks, platform, steel beam, warning cone, wood barrier, chemical toilet | https://opengameart.org/content/construction-site-kit | https://opengameart.org/sites/default/files/Contruction%20Site%20Kit.zip (633 KB) | CC0 | Silverknot | FBX | Low |
| Caution Cone | Cone | https://opengameart.org/content/caution-cone | https://opengameart.org/sites/default/files/cautioncone.blend | CC0 | altcoin | .blend | Medium |
| Construction Barrier 3D | Barrier | https://opengameart.org/content/construction-barrier-3d | https://opengameart.org/sites/default/files/construction%20Barrier_0.blend | CC0 | PagDev | .blend | Medium |
| Construction Light Set | Light tower / work lights | https://opengameart.org/content/construction-light-set | https://opengameart.org/sites/default/files/Construction_Light_Set.zip (25 MB) | CC-BY 4.0 / CC-BY 3.0 / CC0 (multi-licensed: pick CC0) | PagDev | zip | Medium |
| Concrete pipes | Materials | https://opengameart.org/content/concrete-pipes | https://opengameart.org/sites/default/files/concrete%20pipes.7z | CC0 | Yughues | 7z | Medium |
| Low poly Cement machines | Cement mixer (small drum) | https://opengameart.org/content/low-poly-cement-machines | https://opengameart.org/sites/default/files/mylandscaping.zip | **CC-BY-SA 3.0 [SA]** | NathanS21 | zip | Low |
| Bulldozer | Bulldozer | https://opengameart.org/content/bulldozer-0 | https://opengameart.org/sites/default/files/bulldozer-mikeeusa-3.blend | **CC-BY-SA 3.0 [SA]** / GPL 2.0 | MikeeUSA | .blend | Low-medium. Share-alike, so avoid it if you can |
| Forklift Jungheinrich DFG 425s | Forklift | https://opengameart.org/content/forklift-jungheinrich-dfg-425s | https://opengameart.org/sites/default/files/forklift_0.blend | CC-BY 4.0 / 3.0 | luke1985 | .blend | Medium |

### A6. Kenney.nl and Quaternius (site-wide CC0)

- **Kenney** (https://kenney.nl/assets): each asset page says "CC0". Useful kits are **Car Kit** (45 assets, CC0), **City Kit (Industrial)** (40 assets, CC0) and City Kit Roads (cones/barriers/dumpster). On kenney.nl the download is a button that calls a session-protected endpoint, so scripted download didn't work. No login is needed, though: click "Download" in a browser. Several Kenney construction props are mirrored on Poly Pizza with direct GLBs (see A4). Realism: low (stylised).
- **Quaternius** (https://quaternius.com): site states CC0. Packs are downloaded from the pack page (Google Drive/itch links, no login). Many individual models are mirrored on Poly Pizza with direct GLBs (A4: Worker, Character Hazmat, barriers, containers, pallets). Realism: low.

---

## (B) Redistributable license, but needs a manual logged-in download (Sketchfab)

Sketchfab "CC Attribution" means **CC-BY 4.0**. I verified the license via `api.sketchfab.com/v3/models/<uid>` (`license.label`, `isDownloadable: true`). Attribution format: "<title> by <author> (<URL>), licensed under CC-BY 4.0". **Needs manual download** (Sketchfab account). Caution: many of these depict branded machines (CAT, JCB, Komatsu, Liebherr, Hitachi). The CC-BY license covers the uploader's copyright only, not trademarks, so remove or replace logos. Photogrammetry scans are static single meshes.

| Name | Category | URL | Author | Faces | Notes | Realism |
|---|---|---|---|---|---|---|
| Hitachi excavator | Excavator | https://sketchfab.com/3d-models/3dde2c846cc54a4d9fabcc821c9b9caa | Lassi Kaukonen (thesidekick) | 300k | Photogrammetry scan, single mesh | High (static) |
| Hyundai Excavator | Excavator | https://sketchfab.com/3d-models/a805e66f565a412682ac8ce45ab9ac3e | Lassi Kaukonen | 283k | Scan | High (static) |
| Excavator (CAT) | Excavator | https://sketchfab.com/3d-models/5f195244108c46e495a1e78040f02f7e | sarvesh777 | 182k | Modelled in Maya, FBX. Likely separate parts | Medium-high |
| Excavator (stylised) | Excavator | https://sketchfab.com/3d-models/3c4d505819294b4982a67218fa8baaa2 | Anton Krupnov | 134k | Blender + Substance, 2K textures | Medium (stylised) |
| Construction Equipment LowPoly Free 3d models | Multi (excavator, loader, dozer, trucks…) | https://sketchfab.com/3d-models/dcd9c8edd2cd4ed3bee650a82e3f091d | GOGA (gigabate) | 1.6M total | Set | Medium |
| Lowpoly Construction Vehicles Set | Multi | https://sketchfab.com/3d-models/8164c456f9f8443bbe180d1914ccfae3 | kyuandzo | 138k | Set | Low-medium |
| Pete – Driving Articulated Dump Truck | ADT | https://sketchfab.com/3d-models/79edce3312424b5995d6a674f9bbd7b6 | TG13730 | 888k | | High |
| ADT Dump Truck | ADT | https://sketchfab.com/3d-models/270ee486dae54da49a45bace433af279 | Victor.Bonnke | 145k | | Medium-high |
| Komatsu HD785-7 Dump Truck | Rigid haul truck | https://sketchfab.com/3d-models/9f6d3aaf49f4460e81fa8b47f767bdd0 | Maxim Lukyanov | 53k | Game-ready, interior | High |
| Dump Truck | Dump truck | https://sketchfab.com/3d-models/781c616da8f44982a59cb3fc3fa67f98 | ElectroNick | 34k | Game-ready PBR 4K | Medium-high |
| Hyundai HL975A Wheel loader | Wheel loader | https://sketchfab.com/3d-models/3ae998672f994c6f9ce2f336c88c8cac | Lassi Kaukonen | 595k | Scan | High (static) |
| Cat 990K | Wheel loader | https://sketchfab.com/3d-models/ba3713b51f7d4097a4207d8fec059c8c | gelber | 398k | Possibly a game rip, judging by the junk description. **Provenance doubtful, so avoid** | High |
| D8 Tractor | Bulldozer (WW2-era) | https://sketchfab.com/3d-models/b26ce56081304c1988ec7490e439d882 | artfletch | 49k | Scan | High (static, vintage) |
| Bulldozer | Bulldozer | https://sketchfab.com/3d-models/b06a715d23a7450babac383b8bb7fb0a | semyon.kan | 111k | Modelled | Medium-high |
| K-51 / KS-4574 / KS-3577 mobile crane | Mobile crane | https://sketchfab.com/3d-models/f18f83e1f952408cbaabe844fc59b723 ; .../7df018a87e0a49e0b85cd8a8e281323b ; .../336aad4bce43485f86b42db02a702b83 | Randonavt | 184–291k | Soviet-era | Medium-high |
| Mobile crane | Mobile crane | https://sketchfab.com/3d-models/fb62f04101694627803d5a829b8893f0 | l.ft | 616k | Blender | High |
| Tower Crane | Tower crane | https://sketchfab.com/3d-models/49851dc7a51b43bda6aea06856c26a85 | Chamod1999 | 32k | Blender + Substance, most-liked | Medium-high |
| Liebherr tower crane | Tower crane | https://sketchfab.com/3d-models/0f8befc223734608a5d54aad4904951f | Cem Alagozlu | 179k | | High |
| Tower crane KB-674 | Tower crane | https://sketchfab.com/3d-models/0667cfae11e44969978ab28eea512d84 | Randonavt | 33k | | Medium |
| Concrete Mixer Truck | Mixer truck | https://sketchfab.com/3d-models/8d1d88a5955f4e7da13562bb7bec146f | doty_aecom | 45k | | Medium |
| Kraz 250 mixer / Magirus D mixer | Mixer truck | https://sketchfab.com/3d-models/87a687084ec146d682fe8ec6998fe4ff ; .../2816fbf6ca4d489592282b77dbf1c28e | Randonavt | 127–132k | | Medium-high |
| Concrete Pump Truck (Boom Pump) | Concrete pump | https://sketchfab.com/3d-models/4328809da1c94ffb95a8eeb60bb6ce9b | doty_aecom | 78k | | Medium |
| Concrete Pump (Turbosol Mini) | Trailer concrete pump | https://sketchfab.com/3d-models/8e883fffc1b14e089304c41c5d9b484a | RhymOne | 110k | | Medium-high |
| Road Roller ARP 35 / Road Roller (Bomag) | Roller | https://sketchfab.com/3d-models/6a9b1acd83dd4463a2e839ba28ae1ded ; .../02c2ffa7c88f40fca1718e568adebda6 | Lassi Kaukonen | 445k / 135k | Scans | High (static) |
| Komatsu SK820 Skid Steer Loader | Skid steer | https://sketchfab.com/3d-models/9d16f3d8b3b0456e86db243376b2a390 | orphanrtg | 69k | 2x8K textures | High |
| Caterpillar Skid Steer Loader | Skid steer | https://sketchfab.com/3d-models/859a64d242314ec3af5bfb449bdfa673 | 3DBuilder | 688k | | Medium-high |
| JCB Backhoe Loader 3CX | Backhoe | https://sketchfab.com/3d-models/5b2d02e1c1c14deba0f9c8d75fff54b6 | Kyle Burton | 153k | | Medium |
| JCB Backhoe loader | Backhoe | https://sketchfab.com/3d-models/2a61c987f8b048f592e084b4eefc1dd1 | sarvesh777 | 148k | FBX | Medium |
| Telehandler Merlo P.40 | Telehandler | https://sketchfab.com/3d-models/a482d286e1f443fe915c03a6c6dfe21d | Max (Max-7215) | 123k | Has 1 animation, 4K textures | High |
| Telehandler (Telescopic Forklift) | Telehandler | https://sketchfab.com/3d-models/bf47ad2e581d4850ba73b45aa0f88040 | doty_aecom | 139k | | Medium |
| Caterpillar 140H Motor Grader | Motor grader | https://sketchfab.com/3d-models/9ae93444540648c981dc4bc500c691e5 | Stanz | 421k | | High |
| Construction Site Asset Pack | Site props | https://sketchfab.com/3d-models/a6cccf5af6514f2f9289beeead38fe1b | vmatthew | 27k | | Low-medium |
| CONSTRUCTION 100 PROPS ASSET PACK | Barriers, fencing, props | https://sketchfab.com/3d-models/30a093c821e04af28fde96aec06bc257 | Oxygen3D | 86k | | Medium |
| Free Wooden Scaffolding Set / Scaffold Stairs Steel | Scaffolding | https://sketchfab.com/3d-models/de6d3cb59d0e4ef99103523ae22fbd88 ; .../d2230c5644234b3781cd7b2a6cec9638 | Just8 / dchant | 48k / 79k | | Medium |
| Formwork 1–4 AO | Formwork | https://sketchfab.com/3d-models/63ea7be8390a40938613da78ec736038 (and 00217f75…, c034d825…, 51b900f0…) | Zorg741 | about 200 each | | Low |
| REBAR reinforced concrete | Rebar | https://sketchfab.com/3d-models/465ad1e439304aa0a16e6895f7e36a51 | sinnervoncrawsz | 142k | | Medium |
| Diesel generator | Generator | https://sketchfab.com/3d-models/7a0b72c7d01944a8aafab59d44eb120e | epilogueronin | 10k | | Medium |
| Light Tower | Light tower | https://sketchfab.com/3d-models/9addea6a4129438b83a1fc3ed4a784ec | krambulini | 15k | | Medium |
| Jersey Barrier | Barrier | https://sketchfab.com/3d-models/b52246c9611a42a99a03b425535a0237 | emran.bayati | 1.7k | | Medium |
| Site Office using Container | Site cabin | https://sketchfab.com/3d-models/f1f8f02b8ad2440984b041a489a4ec0b | aseranempoeng | 468k | **CC-BY-SA [SA]** | Medium |

The Sketchfab search API (`/v3/search?type=models&downloadable=true&license=cc0|by|by-sa`) returned no notable CC0 construction machines.

---

## (C) Not redistributable, or not usable under our rules

| Source | Finding |
|---|---|
| **Manufacturer CAD/BIM (Caterpillar, Komatsu, Volvo CE, Liebherr, JCB via BIMobject)** | BIMobject's User Terms (https://business.bimobject.com/terms-of-service-eula, effective 27 Sep 2024) say downloaded BIM objects may be used only for your own personal use, not for commercial purposes. Manufacturers can add their own terms on top (e.g., Legrand and Saint-Gobain forbid re-uploading or sublicensing). Caterpillar's BIM models (e.g., via SpecSizer for gensets) need a Cat.com account and show no open license. Third-party CAD catalogs that host Liebherr/CAT blocks (cadforum.cz) explicitly prohibit redistribution. **Conclusion: you can't put any manufacturer CAD/BIM in a public repo.** You can use it privately as a dimension reference to build your own models. |
| Fuel MovAi `pallet`, `pallet_box_mobile` | CC-BY-NC-ND 4.0, so rejected. |
| GitHub repos with no LICENSE: `pwri-opera/zx120_ros2`, `haoyangli16/ROS-SLAM-MATLAB-Moveit2.0`, `WDWSD/ROS_Excavator`, `leggedrobotics/rsl_heap` (ETH HEAP), `orientpine/hr35_modeling` (ix35e excavator STL), `MoltenXircuit/Pico-Excavator`, `OlliKolli318/isaac_lerobot_excavator`, `markaren/kine_ros2` / `threepp_data` (Crane3R), `ruiming-wu/excavator`, `ishidur/pai_proj` | No license means all rights reserved, so these can't be redistributed. For ZX120, use `pwri-opera/zx120_ros` (Apache-2.0) instead. |
| `jinbeomjeong/excavator_simulation` | GPL-3.0. Copyleft and not on our accept list, though it is functionally share-alike. It has a split excavator in STL (boom, arm, bucket, links, cylinders). Flag it; avoid unless the project accepts GPL assets. |
| CARLA | README: "CARLA specific assets are distributed under the CC-BY License". It has cones/barriers/props but **no construction machinery**, and the assets are Unreal `.uasset`, so extracting them needs UE. Low value. |
| AirSim | Code is MIT. Environments come from the Unreal Marketplace and aren't redistributable. No construction machines. |
| Smithsonian Open Access (CC0) | API search for "excavator" returned only a dental excavator. No construction machinery. |
| NASA 3D Resources | Has the Crawler-Transporter and VAB, but no construction plant. |
| Wikimedia Commons 3D (STL) | Searches for excavator/bulldozer/crane/dump truck/cone/scaffold with `filemime:model/stl` returned **0 results**. |
| Thingiverse / Printables / MakerWorld | Mostly scale/print-in-place models, many NC or "Standard Digital File License". The Thingiverse API needs a token, so it wasn't checked model by model. Printable designs are not simulation-grade (wrong proportions, print tolerances). Only consider ones explicitly CC-BY/CC0, checked individually. Low priority. |
| Mixamo (Adobe) | Characters/animations may be used in projects but not redistributed as standalone assets. Don't commit raw Mixamo FBX to a public repo. |
| Unity Asset Store / Unreal Marketplace / CGTrader / TurboSquid "royalty free" | Standard/EULA licenses forbid redistributing the source files. Rejected. |

---

## Best picks per category

| Category | 1st | 2nd | 3rd |
|---|---|---|---|
| Hydraulic excavator | **OPERA ZX200** (A, Apache-2.0, split + URDF) | OPERA ZX120 (A) | MathWorks MathScavator9000 (A, BSD-3, CAD-split); Sketchfab CAT/Hitachi (B) |
| Dump truck / ADT | **OPERA IC120 / MST-110CR / C30R crawler carriers** (A, tipping vessel + cylinders) | Majadroid dumper trucks (A, CC0, low) / Poly "Dump truck" (A) | Sketchfab ADT "Pete" / Komatsu HD785 (B) |
| Wheel loader | OGA "Earth mover" (A, CC0, low) | Sketchfab Hyundai HL975A scan (B) | *Gap: no realistic group A option* |
| Bulldozer | **OPERA Komatsu D37PXI** (A, split blade linkage) | Chrono bulldozerB10 (A, BSD-3) | Poly "Bulldozer" (A, CC-BY 3.0, low) |
| Mobile crane | Poly "Crane" J-Toastie (A, CC-BY 3.0) / OGA Crane Machine (A, CC0) | Majadroid crane (A) | Sketchfab "Mobile crane" l.ft (B) |
| Tower crane | **Fuel "Tower crane"** (A, CC0) | Poly "Building construction crane" (A) | Sketchfab "Tower Crane" Chamod1999 (B) |
| Concrete mixer truck | Majadroid concrete truck (A, CC0, low) | Sketchfab Concrete Mixer Truck (B) | Chrono mixer drum meshes as a part source (A) |
| Concrete pump | *Gap in A* | Sketchfab Concrete Pump Truck (B) | Sketchfab Turbosol (B) |
| Roller / compactor | *Gap in A* | Sketchfab Bomag / ARP35 scans (B) | |
| Skid steer | *Gap in A* | Sketchfab Komatsu SK820 (B) | |
| Backhoe loader | *Gap in A* | Sketchfab JCB 3CX (B) | |
| Telehandler | Chrono forklift (A, BSD-3, mast/forks split) | Fuel Forklift (A, CC-BY 4.0) | Sketchfab Merlo P.40 (B) |
| Motor grader | *Gap in A* | Sketchfab CAT 140H (B) | |
| Workers in PPE (rigged) | **Microsoft Rocketbox Construction_Male_01–08 / Female_01** (A, MIT) | Quaternius Worker (A, CC0, stylised, animated) | Fuel Walking actor (A, CC0, civilian) |
| Site cabins / containers | Majadroid Containers-Office (A, CC0) | Quaternius Shipping Container (A, CC0) | Fuel Shipping Container (A, CC-BY 4.0) |
| Scaffolding | OGA Construction Site Kit (A, CC0) | Poly Scaffolding lakescorp (A, CC0) | Sketchfab Free Wooden Scaffolding (B) |
| Formwork | *Gap in A*: build procedurally (boxes/planks) | Sketchfab Formwork 1–4 (B) | |
| Rebar | *Gap in A*: procedural (cylinders along splines is trivial) | Sketchfab REBAR (B) | |
| Generators | Poly "Large Electric Generator" (A, CC-BY 3.0) | Poly "Generator" KolosStudios (A) | Sketchfab Diesel generator (B) |
| Light towers | OGA Construction Light Set (A, CC0 option) | Kenney Construction Light (A) | Sketchfab Light Tower (B) |
| Cones / barriers | Fuel Construction Cone / Barrel / Jersey Barrier (A, CC0) | Kenney / Quaternius barriers (A, CC0) | OGA cone/barrier .blend (A, CC0) |

## Fallback plan for the gaps (wheel loader, mixer, pump, roller, skid steer, backhoe, grader, telehandler)

1. **Kitbash from OPERA parts.** Crawler, cab, cylinders and baked-texture style are reusable. For example, OPERA ZX cab + boom cylinders make a backhoe arm, and the D37PXI blade linkage gives a grader blade. Everything stays Apache-2.0 as derivative work.
2. **Procedural/parametric modelling** in Blender (Geometry Nodes) or a CadQuery/OpenSCAD script for simple-geometry items: rollers (drum + frame), mixer drum (revolved profile), concrete pump boom (box sections + cylinders), formwork, rebar, scaffolding. These are our own work, so we can license them however we like.
3. **Ask Sketchfab authors** (B list) for permission to mirror. CC-BY already permits it legally, so the only real blocker is the logged-in download. A team member can download manually and commit the files with attribution. That is legal under CC-BY 4.0, as long as you avoid trademark logos.
4. **Commission CC-BY or CC0 models** (e.g., a freelancer on ArtStation/Fiverr with a contract assigning CC0/CC-BY) for the wheel loader and mixer truck, if fidelity matters.

## Attribution checklist for the repo

- Keep a `ASSETS_LICENSES.md` or `CREDITS.md` listing every asset: title, author, source URL, license and link, plus modifications made ("converted to GLB, split into parts, collision added").
- Apache-2.0 (OPERA, araobp): ship the LICENSE text and state changes. MIT (Rocketbox): include the MIT copyright notice. BSD-3 (Chrono, MathWorks): include the copyright notice and the disclaimer.
- CC-BY: credit the author, link the license, indicate changes. CC-BY-SA **[SA]**: derivatives must stay CC-BY-SA, so keep those assets in a separate folder.
- Remove manufacturer logos and decals from textures (Hitachi, Komatsu, CAT, JCB, Liebherr) to avoid trademark issues.
