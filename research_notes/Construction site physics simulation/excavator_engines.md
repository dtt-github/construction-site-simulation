# Heavy-Equipment / Excavator / Earthmoving Simulators and Physics Engines (as of 2 Nov 2026)

Scope: candidate foundations for a highly realistic construction-site simulation. Covers commercial and open-source engines, soil/terrain models, machine modeling, licensing, platform support (especially NVIDIA DGX Spark: GB10, Linux aarch64, 128 GB unified memory), integrations, validation, and NVIDIA Omniverse construction workflows.
Method note: budget-limited web research (about 18 tool calls). Many vendor-detail questions (exact prices, aarch64 builds) are not publicly documented and are listed under Gaps.

## Commercial engines and simulators (AGX, Vortex, Mevea, Simscape, OEM/training sims, DEM tools)

### Takeaway
AGX Dynamics (Algoryx) and Vortex Studio (CM Labs) are the two mature commercial real-time engines with built-in excavation soil models: agxTerrain is a hybrid continuum/particle model, and Vortex uses a DEM-mesh hybrid. AGX has the stronger open research and validation record and has Unity/Unreal plugins on Linux. Neither publicly documents Linux aarch64 support, so DGX Spark compatibility is unconfirmed and should be put to the vendors. Mevea and Simscape are strongest on hydraulics and drivetrain modeling, but their soil handling is weak or relies on co-simulation.

### Cited Findings
**Algoryx AGX Dynamics / agxTerrain**
- AGX Dynamics for Unreal runs on Windows and Linux. Its agxTerrain module "allows you to simulate realistic earth moving equipment and terramechanics in real time". The docs list UE 5.2–5.4 support, which may be outdated, and a paid AGX license is required to run simulations — [AGX for Unreal docs](https://us.download.algoryx.se/AGXUnreal/documentation/current/agxunreal.html); [Fab listing](https://fab.com/s/5f333e56971b)
- The AGX Dynamics for Unity docs list Ubuntu 24.04/22.04 and Unity 2023.x, 6.0 and 6.1 as supported. IL2CPP is not supported — [AGXUnity getting started](https://us.download.algoryx.se/AGXUnity/documentation/current/getting_started.html); earlier Linux announcement: [Algoryx news](https://www.algoryx.se/agx-dynamics-for-unity-adds-linux-support-with-a-view-to-expanding-machine-learning-capabilities/)
- Licensing is a yearly per-seat subscription. There are academic single-user and group (up to 10 users) non-commercial licenses, an Academic License Grant Programme that requires an acknowledgment, runtime licenses for distributed products, and OEM/academic discounts. No price is published, so buyers contact sales for evaluation — [Algoryx buy page](https://algoryx.se/buy)
- The agxTerrain tech report (Berglund & Servin) validates and calibrates the model against high-resolution DEM and multibody simulations, finding "good agreement" in soil dynamics and tool reaction forces. That validation is simulation-to-simulation, not field data — [agxTerrain tech report PDF](https://www.algoryx.se/download/agxTerrain_tech_report.pdf)
- Sim-to-real for wheel loaders: Aoshima & Servin (Multibody System Dynamics) compared a DEM terrain and a reduced multiscale terrain model against field tests of a real wheel loader filling its bucket. The work was supported by Komatsu Ltd and Algoryx — [arXiv 2310.05765](https://arxiv.org/html/2310.05765v3)
- A UC Davis AHMCT calibrated digital twin of an EVERUN ER12 wheel loader in AGX was validated against real-world operation data and used for data-efficient excavation-force estimation — [arXiv 2506.22579](https://arxiv.org/pdf/2506.22579)
- Algoryx write-up of a PhD thesis that combines real-world data with AGX synthetic data for autonomous wheel loading — [Algoryx](https://algoryx.se/doctoral-thesis-integrates-real-world-data-and-agx-dynamics-simulations-to-demonstrate-autonomous-wheel-loading/); excavator trajectory optimization using AGX — [Algoryx](https://www.algoryx.se/optimizing-autonomous-excavation-through-simulations/)
- PWRI (Japan) OPERA project: an open excavator and dump-truck simulator in Unity that uses the AGX Dynamics soil model and can be driven from ROS or game controllers (github.com/pwri-opera). It still needs a commercial AGX license — [search summary via ipros](https://mono.ipros.com/en/cg2/High-precision%20simulator)

**CM Labs Vortex Studio**
- Vortex Studio pairs multibody dynamics with a "DEM-Mesh hybrid soil simulation". It ships a ready-to-use excavator model, and wheel/track–soil interaction uses Bekker equations. Earthmoving runs at real time or faster on a standard desktop. These claims come from a US Army sole-source procurement notice, which calls Vortex the only such commercial product — [procurement notice](https://bidbanana.thebidlab.com/bid/0XLIbbG4Bj3BTOz7nfhZ)
- Release 2018c added a water-bridge cohesion model for more realistic cutting, pouring and piling — [CM Labs news](https://cm-labs.com/en/news/cm-labs-breaks-new-ground-with-massive-update-to-vortex-studio/)
- The Vortex Player runtime runs on Windows and Linux, and the docs include a "Vortex Studio on Linux" page. No evidence was found of a Linux Editor or an aarch64 build — [Vortex docs](https://vortexstudio.atlassian.net/wiki/spaces/VSD231/pages/3287827844); [Vortex Studio product page](https://cm-labs.com/en/vortex-studio/)
- ETH RSL's HEAP walking excavator: free-form trench dig cycles were developed in Vortex and transferred to the real machine "without any changes" thanks to force control. This is real-world validation of a Vortex-based workflow — [ETH Research Collection](https://www.research-collection.ethz.ch/entities/publication/85e9f204-6ca2-4073-ad80-ee4b891e7ee3)

**Mevea**
- Mevea sells excavator and large-excavator training simulators. It says its generic 24-t excavator uses hydraulic components and a powertrain "designed to be the same as in a real machine" (a design claim, not published validation) — [Mevea excavator](https://mevea.com/solutions/training-simulators/construction-equipment/excavator/); [Large excavator](https://mevea.com/solutions/training-simulators/construction-equipment/large-excavator/)
- At Bauma 2025 Mevea showed what it called the first real-time simulation of an excavator with electromechanical actuators, used for design validation — [Mevea news](https://mevea.com/news-events/news/mevea-exhibits-the-worlds-first-real-time-simulation-of-excavator-with-electromechanical-actuators-at-bauma-2025/)
- Mevea's 2025 seminar said it covered sim-vs-measurement comparisons, but no numbers are public. A LUT thesis built an excavator hydraulic circuit in Mevea — [Mevea seminar 2025](https://mevea.com/news-events/news/mevea-virtual-seminar-2025-real-time-simulation-in-development-of-autonomous-electrified-heavy-machinery/); [LUT thesis](https://lutpub.lut.fi/handle/10024/167509)

**MathWorks Simscape**
- The "Excavator Design with Simscape" example models swing-motor and boom/stick/bucket hydraulics, a valve block with regeneration valves, dig-cycle time and efficiency. Bucket soil loads come from co-simulation with ThreeParticle/CAE (DEM), and a later version adds a Mevea co-simulation that includes the tracks. These co-simulation examples run on Windows 64-bit only — [MathWorks File Exchange](https://www.mathworks.com/matlabcentral/fileexchange/119268-excavator-design-with-simscape)
- MathWorks has an off-road vehicle solutions page covering vehicle, powertrain and implement modeling — [MathWorks](https://au.mathworks.com/solutions/robotics/offroad-vehicles/dynamic-simulations.html)
- Load-sensing hydraulics for an excavator have been modeled in Simcenter Amesim (Politecnico di Torino thesis). No Simscape-specific load-sensing excavator example was confirmed — [PoliTo thesis](https://webthesis.biblio.polito.it/24382/1/tesi.pdf)
- Dyad (JuliaHub) offers a planar excavator example with a SoilContact component that models penetration and cutting resistance — [JuliaHub Dyad](https://help.juliahub.com/dyad/v3/stdlib/MultibodyComponents/examples/PlanarMechanics.examples.excavator.Excavator.html)

**OEMs and Omniverse (also see the last section)**
- Caterpillar + NVIDIA (CES, 7 Jan 2026): the Cat AI Assistant runs on Jetson Thor and is being piloted on the Cat 306 CR mini excavator. Factory digital twins are built on Omniverse and OpenUSD — [Caterpillar IR](https://investors.caterpillar.com/news/news-details/2026/Caterpillar-Teams-With-NVIDIA-to-Revolutionize-Heavy-Industry-with-Physical-AI-and-Robotics); [Equipment World](https://www.equipmentworld.com/business/article/15814787/caterpillar-nvidia-expand-ai-partnership)

### Inferences
- For maximum excavation realism with a licensed engine, AGX is the best-documented choice: Linux Unity/Unreal plugins, published wheel-loader sim-to-real work, and use by PWRI OPERA and TERA. Vortex is the main alternative and has the strongest operator-training and crane pedigree.
- Neither AGX nor Vortex documents aarch64. A DGX Spark plan should assume x86 is needed, or a vendor-built ARM port, until the vendors confirm otherwise. A possible split is to run physics on an x86 node and rendering or AI on the Spark.
- Mevea and Simscape (or Amesim) are the reference tools for valve, pump and cylinder fidelity. They are better used as co-simulation partners for a hydraulics model than as the site engine.

### Gaps
- No public prices for AGX, Vortex, Mevea, EDEM, Rocky or Duality Falcon.
- No evidence found that AGX or Vortex run on Linux aarch64 or GB10. Ask support@algoryx.com and CM Labs directly.
- Not researched because of the call budget: Altair EDEM + MotionSolve coupling, Ansys Rocky DEM, Tenstar, Oryx, Cognata, Duality Falcon, Bentley/Synchro, and the Komatsu/Volvo/Liebherr operator simulators. From background knowledge (unverified here), EDEM and Rocky are GPU DEM tools that are not real-time and are used offline for bucket and tool design.
- No published quantitative validation (error metrics) was found for Vortex or Mevea excavator hydraulics.

## Open-source engines (Chrono, Isaac Sim/Lab/Newton, Genesis, MuJoCo, others) and research simulators

### Takeaway
Project Chrono (BSD-3) is the most complete open-source option for terramechanics. It offers SCM (Bekker-Wong), CRM (SPH continuum soil) and GPU DEM-Engine, plus tracked-vehicle models, PyChrono and builds on AArch64. Isaac Lab and Newton (Apache-2.0, Linux Foundation) run on DGX Spark and offer MPM granular two-way coupling, but no published excavator workflow uses Newton MPM soil yet: Isaac Lab excavator work adds soil forces as an external analytical model. Genesis's MPM speed claims are self-reported.

### Cited Findings
**Project Chrono**
- Chrono 10.0.0 was announced on 27 Mar 2026 and tagged around 7 Apr 2026. It refactors Chrono::FSI (SPH and TDPF solvers), adds Jacobian update strategies for implicit integrators, VSG visualization plugins, and a modern CMake build — [Chrono news](https://projectchrono.org/news/); [GitHub releases](https://github.com/projectchrono/chrono/releases); [Zenodo](https://zenodo.org/records/6414450/latest)
- CRM terrain is an SPH continuum soil model with MU_OF_I or MCC rheology. It acts as both a ChTerrain and an SPH-FSI problem builder, co-advanced with the multibody system. The docs warn that rendering large particle counts is expensive — [CRM terrain docs](https://api.projectchrono.org/vehicle_terrain_crm_api_.html)
- Chrono 7.0.3 was a bugfix release "for Apple-M1 and other AArch64 CPUs". A platforms file lists AArch64 with GCC 7+ (from a fork, so possibly outdated) — [GitHub releases](https://github.com/projectchrono/chrono/releases); [PLATFORMS.md fork](https://www.github.com/dcagritan/chrono/blob/main/PLATFORMS.md)
- The M113 tracked vehicle offers single- and double-pin segmented tracks plus bushing-based and ANCF continuous band tracks. A demo runs the M113 on SCM deformable terrain — [Chrono M113 docs](https://api.projectchrono.org/group__vehicle__models__m113.html); [Chrono::Vehicle tutorials](https://api.projectchrono.org/tutorial_table_of_content_chrono_vehicle.html)
- DEM-Engine (DEME) is GPU DEM optimized for two GPUs, with scaling tests on two A100s — [arXiv 2311.04648](https://arxiv.org/abs/2311.04648v2)
- PyChrono provides SWIG-based Python bindings — [Wikipedia: Project Chrono](https://en.wikipedia.org/wiki/Project_Chrono)

**NVIDIA Isaac Sim / Isaac Lab / Newton**
- Newton is a GPU physics engine built on NVIDIA Warp. Disney Research, Google DeepMind and NVIDIA started it, and it is now a Linux Foundation project under Apache-2.0. Its examples include "MPM Granular", "MPM Anymal" and "MPM Two-Way Coupling". PyPI shows versions up to 1.6.x release candidates — [Newton GitHub](https://github.com/newton-physics/newton); [PyPI newton](https://pypi.org/project/newton/)
- Isaac Sim and Isaac Lab run on DGX Spark but must be built from source on aarch64. Missing features include livestream, the Hub workstation cache, and Mimic/visuomotor inference (no non-DLSS denoiser on aarch64). JAX runs on CPU by default — [Isaac Lab 2.3 install docs](https://docs.nvidia.com/isaaclab/2.3/isaac-lab/local-installation); [Arm learning path](https://learn.arm.com/learning-paths/laptops-and-desktops/dgx_spark_isaac_robotics/2_isaac_installation/)
- Isaac Lab 3.0 beta with the Newton backend on Isaac Sim 6.0 ran a Go2 policy on DGX Spark (community report, May 2026). Forum reports describe Isaac Sim 6.0.1 GPU crashes on GB10 (Jul 2026) and PhysX GPU issues on 5.1 — [Classmethod](https://dev.classmethod.jp/articles/go2-simulation-newton/); [NVIDIA forum](https://forums.developer.nvidia.com/t/will-issac-sim-support-arm64-on-the-dgx-spark/328238)
- In an Isaac Lab excavator RL study (boulder excavation, ETH, 2025), Isaac Lab simulates the rigid excavator and rocks while a separate soil-resistance model applies external forces on the shovel — [arXiv 2509.17683](https://arxiv.org/pdf/2509.17683)

**Genesis**
- Genesis claims to be 10–80x faster than Isaac Gym and MJX and reports 43M FPS for a Franka arm on an RTX 4090. These are self-reported figures. Its MPM solver uses FluidLab as a reference — [Genesis docs: why a new simulator](https://genesis-world.readthedocs.io/en/v0.3.3/user_guide/overview/why_a_new_simulator.html); [Genesis World docs](https://genesis-world.readthedocs.io/)
- No independent benchmark of Genesis MPM for sand excavation was found.

**Research simulators**
- TERA (arXiv 2501.01430, IEEE SIMPAR 2025) is built on Unity3D with AGX Dynamics. It supports multiple excavators in real time on desktop hardware, ROS and Unity Python APIs, soil/gravel/sand terrain parameters, a time-varying system-response model, and perception for end-to-end autonomy. The paper promised a code release on publication — [arXiv](https://arxiv.org/html/2501.01430v1); [UB record](https://researchconnect.buffalo.edu/en/publications/tera-a-simulation-environment-for-terrain-excavation-robot-autono/)
- ETH RSL HEAP: simulation work used the commercial Vortex engine, and no open-source HEAP soil model was found — [ETH](https://www.research-collection.ethz.ch/entities/publication/85e9f204-6ca2-4073-ad80-ee4b891e7ee3)
- MPM has been used for 2D soil-bucket simulation in earthwork planning, with Lamé parameters tuned to observed deformation — [KNU](https://research.knu.ac.kr/en/publications/2d-soil-bucket-simulation-using-material-point-method-for-efficie/)
- UDC (Spain) built a real-time multibody excavator simulator with its own soil model — [UDC portal](https://portalinvestigacion.udc.gal/documentos/609c25951aec1f036bb232b6)

### Inferences
- For DGX Spark, the most practical open stacks are Chrono (CPU, plus CUDA for CRM/FSI and DEME, though aarch64 CUDA builds still need local verification) and Newton/Isaac Lab (officially on Spark, with caveats).
- No open engine ships a validated excavator hydraulics and soil package. Expect to build valve, pump and cylinder models yourself, for example as a Modelica/FMU or a custom ODE coupled at a 1 kHz control step, while the engine provides soil and multibody dynamics.
- A credible hybrid design: Chrono SCM or CRM, or Newton MPM, for soil; a hydraulics ODE on top; and Omniverse or Unreal for rendering through OpenUSD.

### Gaps
- Not verified: MuJoCo/MJX/MuJoCo Warp soil capability, Gazebo, Brax, Taichi sims, ManiSkill. From background knowledge, none of these has a built-in excavation soil model; MuJoCo and MJX are rigid-body only.
- Not confirmed: whether Chrono's CUDA modules (FSI/CRM, DEME, Sensor/OptiX) build on GB10 aarch64, or the status of Chrono–Unreal integration.
- Newton's MPM solver formulation (implicit or explicit) and its excavation fidelity have not been validated anywhere I found.
- I did not check the license or AGX dependency of the TERA and PWRI OPERA code releases.

## Hydraulics, tracked vehicle and wheel-loader modeling, and operator-in-the-loop latency

### Takeaway
Hydraulic fidelity (valves, pumps, load sensing) is best in Mevea, Simscape and Amesim. AGX and Vortex provide hydraulic or actuator components, but the public docs describe them only lightly. Open engines usually command joint velocities or forces directly. Tracked vehicles are mature in Chrono (multiple track models on SCM) and in Vortex (Bekker-based). Wheel-loader sim-to-real evidence is strongest for AGX.

### Cited Findings
- Simscape example: the valve block, regeneration valves, swing motor and boom/stick/bucket cylinders produce dig-cycle time and efficiency — [MathWorks](https://www.mathworks.com/matlabcentral/fileexchange/119268-excavator-design-with-simscape)
- Load-sensing excavator circuit modeled in Amesim — [PoliTo thesis](https://webthesis.biblio.polito.it/24382/1/tesi.pdf)
- TERA commands joint velocities through AGX rather than simulating full hydraulics, and adds a time-varying response model — [arXiv 2501.01430](https://arxiv.org/html/2501.01430v1)
- Vortex: track/wheel–soil interaction via Bekker; earthmoving runs at real time or faster on a desktop — [procurement notice](https://bidbanana.thebidlab.com/bid/0XLIbbG4Bj3BTOz7nfhZ)
- Chrono M113: segmented and continuous band tracks on SCM — [Chrono docs](https://api.projectchrono.org/group__vehicle__models__m113.html)
- Wheel-loader bucket-filling sim-to-real gap quantified for DEM and reduced-order terrain — [arXiv 2310.05765](https://arxiv.org/html/2310.05765v3)

### Inferences
- For operator training, a realistic machine feel depends mostly on pump/valve dynamics and boom inertia under load. Plan a dedicated hydraulic model (FMU from Simscape/Amesim, or custom) if the base engine lacks one.

### Gaps
- I found no published operator-in-the-loop latency figures (motion platform or display latency) for any engine.
- AGX and Vortex hydraulic library specifics (load-sensing pumps, compressibility) were not documented in the sources retrieved.

## Validation against real machine data

### Takeaway
The best public sim-to-real evidence is AGX for wheel loaders (Aoshima & Servin field tests, UC Davis ER12 twin) and Vortex for excavators (ETH HEAP zero-shot dig-cycle transfer). The agxTerrain core validation is against DEM, not the field. Mevea claims real-world comparisons but publishes no metrics. No validation of Newton or Genesis soil against machines was found.

### Cited Findings
- AGX wheel loader vs. field tests — [arXiv 2310.05765](https://arxiv.org/html/2310.05765v3); AGX ER12 twin calibrated on real data — [arXiv 2506.22579](https://arxiv.org/pdf/2506.22579)
- agxTerrain validated against high-resolution DEM — [tech report](https://www.algoryx.se/download/agxTerrain_tech_report.pdf)
- HEAP dig cycles from Vortex transferred unchanged to the real machine — [ETH](https://www.research-collection.ethz.ch/entities/publication/85e9f204-6ca2-4073-ad80-ee4b891e7ee3)
- Mevea seminar referenced sim-vs-measurement comparisons (no figures) — [Mevea](https://mevea.com/news-events/news/mevea-virtual-seminar-2025-real-time-simulation-in-development-of-autonomous-electrified-heavy-machinery/)

### Inferences
- Chrono SCM, CRM and DEME have a terramechanics validation literature for wheels and rovers (background knowledge, not retrieved here). Excavator-specific validation of Chrono seems thin.

### Gaps
- No quantitative excavator force or energy validation papers were retrieved for Chrono, Vortex, Mevea or Isaac.

## NVIDIA Omniverse-connected construction workflows (2025–2026)

### Takeaway
Caterpillar's CES 2026 NVIDIA announcement is the main OEM signal: Jetson Thor in-cab AI and Omniverse/OpenUSD factory twins, plus one report of a construction-site digital twin pilot fed by machine telemetry. Komatsu's NVIDIA partnership dates from 2017 and focuses on Jetson perception. Its current jobsite twin is the EARTHBRAIN Smart Construction Dashboard, and no Omniverse link was found. Startups such as Bedrock Robotics list Isaac Sim/Omniverse as one option among several simulation stacks.

### Cited Findings
- Caterpillar + NVIDIA (CES, 7 Jan 2026): Cat AI Assistant on Jetson Thor, piloted on the Cat 306 CR. Omniverse/OpenUSD factory digital twins are in pilot — [Caterpillar IR](https://investors.caterpillar.com/news/news-details/2026/Caterpillar-Teams-With-NVIDIA-to-Revolutionize-Heavy-Industry-with-Physical-AI-and-Robotics); [Assembly Mag](https://www.assemblymag.com/articles/99752-caterpillar-teams-with-nvidia-to-deploy-ai-in-factories); [HeavyQuip](https://www.heavyquipmag.com/2026/01/09/caterpillar-expands-nvidia-partnership-to-integrate-ai-across-equipment-and-manufacturing/)
- Single-source claim: Cat machines send about 2,000 messages per second, feeding an Omniverse construction-site digital-twin pilot for planning and material estimation — [techstrong.ai](https://techstrong.ai/articles/caterpillar-unveils-ai-powered-equipment-and-autonomous-technology-at-ces/)
- Komatsu–NVIDIA (2017): Jetson cameras and GPU site visualization, with simulation of construction sites as a stated goal — [NVIDIA newsroom](https://nvidianews.nvidia.com/news/japans-komatsu-selects-nvidia-as-partner-for-deploying-ai-to-create-safer-more-efficient-construction-sites); [Komatsu](https://komatsu.jp/en/newsroom/2017/20171213)
- Komatsu Smart Construction Dashboard (2026) builds a 3D virtual twin from design files, drone data and machine as-built data — [Komatsu newsroom 2026](https://komatsu.com/en-us/newsroom/2026/komatsu-smart-construction-makes-stronger-connection-between-machines-office-work); [Komatsu digital twins blog 2026](https://www.komatsu.com/en-us/blog/2026/digital-twins)
- Bedrock Robotics job postings: the simulator must model terrain deformation for digging at a scale of thousands of runs. Isaac Sim/Omniverse and Unreal are preferred skills, and Gaussian splats and NeRFs are being explored — [Bedrock posting](https://careers.georgian.io/companies/bedrock-robotics/jobs/85945787-software-engineer-simulation); [Neural rendering intern](https://radiancefields.com/job-board/2027-internship-simulation-engineer-neural-rendering-bedrock-robotics-inc-8376485d)
- Omniverse Standard license is free for individuals; Enterprise is about $4,500 per GPU per year (from a search snippet) — [NVIDIA free trial](https://www.nvidia.com/en-us/omniverse/enterprise/free-trial/); [forum](https://forums.developer.nvidia.com/t/updated-pricing-plans-for-omniverse-enterprise/282857)

### Inferences
- No commercial soil engine (AGX, Vortex) has a documented Omniverse connector. Omniverse integration would likely go through OpenUSD export or co-simulation rather than a native plugin.

### Gaps
- Not found: Bentley iTwin + Omniverse, Trimble + Omniverse, and Volvo CE or Liebherr NVIDIA construction-twin details, all outside what this budget could cover.
- No AGX–Omniverse or OpenUSD integration was found.
