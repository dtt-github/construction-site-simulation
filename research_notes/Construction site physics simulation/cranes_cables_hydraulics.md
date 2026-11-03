# Cranes, Cables, Hydraulics, Heavy Vehicles, Structural Failure and Workers: Realistic, Real-Time Simulation of the Non-Soil Parts of a Construction Site

Scope note: the research budget was about 15 tool calls. Most sources are abstracts, vendor docs and API references; few full texts were read. Items marked "(abstract/snippet only)" were not checked against the full paper.

## 1. Crane and payload dynamics (tower/mobile/crawler), wind, anti-sway, operator-training simulators

### Takeaway
The standard real-time crane model has three parts. First, the crane's slew, trolley and hoist degrees of freedom are derived with Lagrange's method. Second, the payload is a spherical pendulum, or a double pendulum when the hook mass is comparable to the load or the load is large. Third, wind is applied as a drag force on the load, with an optional stochastic wind field on the structure. Vaughan, Kim and Singhose (IEEE TCST 2010) is the key reference for double-pendulum tower cranes and input shaping. Commercial training simulators such as CM Labs Vortex have vendor case studies but no published independent validation results.

### Cited Findings
- Tower crane equations of motion for slewing, trolley travel and hoisting can each be derived with Lagrangian modelling. Pure trolley motion behaves like a planar pendulum. Slewing produces radial and tangential swing components, so the payload moves as a spherical pendulum. — [arXiv 2404.05581, time-energy optimal anti-swing planner for autonomous tower cranes](https://arxiv.org/pdf/2404.05581)
- Double-pendulum tower-crane trajectory planning (trolley and slew): trolley motion excites planar double-pendulum swing. — [arXiv 2305.18745](https://arxiv.org/pdf/2305.18745)
- A 2026 jib-crane study models the slewing mechanism with a double-pendulum payload as a 4-DoF system, with second-order ODEs from Lagrange's equations. — [OMTC/KNUBA article](https://omtc.knuba.edu.ua/article/view/361803) (abstract/snippet only)
- Payload twisting of distributed-mass payloads during slewing is ignored by point-mass models and has received much less control attention than swing. — [Payload Twisting Dynamics and Oscillation Suppression of Tower Cranes during Slewing (Missouri S&T)](https://scholarsmine.mst.edu/ele_comeng_facwork/4135)
- Vaughan, Kim and Singhose, "Control of tower cranes with double-pendulum payload dynamics," IEEE TCST 2010, is the key input-shaping and anti-sway reference. Other approaches include 3-D models with friction and energy-coupling nonlinear anti-swing control, and RBF neural-network adaptive control. — cited in search results around [arXiv 2305.18745](https://arxiv.org/pdf/2305.18745) and [Frontiers in Robotics and AI 2021](https://www.frontiersin.org/journals/robotics-and-ai/articles/10.3389/frobt.2021.639734/pdf) (secondary citation)
- Recent overhead-crane control work includes output-constrained controllers with fuzzy-tuned parameters (2025). — [arXiv 2512.03680](https://arxiv.org/pdf/2512.03680)
- Wind on the crane structure: CFD-derived wind coefficients for a full-scale tower crane were combined with autoregressive simulated wind time histories on an FE model. The peak along-wind load direction shifted 30° to 60° from the expected direction, and the authors say fluctuating wind should be included in safety checks. — [Chen et al. 2020, J. Low Freq. Noise Vib. Active Control, DOI 10.1177/1461348419847306](https://journals.sagepub.com/doi/xml/10.1177/1461348419847306)
- Buffeting workflow: simulate a spatially correlated multivariate stochastic wind field, then compute buffeting forces from CFD lift, drag and moment coefficients. Surrounding structures can create unexpected aerodynamic effects. — [Safety and serviceability assessment for high-rise tower crane to turbulent winds, Front. Struct. Civ. Eng. 2009](https://journal.hep.com.cn/fsce/EN/Y2009/V3/I1/18)
- Cargo sway under wind is modelled as a load force f·u_x, f·u_y (wind resistance coefficient × wind speed components). Sway angle and speed peak when wind is along or perpendicular to the jib, and scale with wind speed, windward area and cargo mass. — [Atlantis Press tower crane sway paper](https://www.atlantis-press.com/article/25847238.pdf) (snippet only)
- Crane design rules apply static wind force coefficients only to upwind surfaces and can miss negative pressure and viscous effects (CFD study on gantry cranes). — [Atlantis Press gantry-crane CFD](https://www.atlantis-press.com/article/25847644.pdf) (snippet only)
- Operator-training simulators: Crane Industry Services, CM Labs and West Georgia Technical College announced a study (from 2016) to run 14 months or until 500 operators had taken part. It covered inexperienced to very experienced operators on a 40-ton rough-terrain crane module with a 102-ft boom. No final results were found. — [Equipment World](https://www.equipmentworld.com/crane-study-seeks-to-determine-benefit-of-simulators-in-operator-training/); [Crane Hot Line](https://www.cranehotline.com/articles/study_established_to_assess_crane_operation_skills_using_simulators)
- In a vendor case study, a CSI trainer reports that foundational crane training fell from 8–10 hours of one-on-one instruction to 4–6 hours with Vortex. — [CM Labs CSI case study PDF](https://www.cm-labs.com/wp-content/uploads/2023/01/CSI-Crane-Industry-Case-Study.pdf); [Crane Institute of America case study](https://www.cm-labs.com/en/case-study/crane-institute-of-america-improves-training-with-vortex-simulator/) (vendor, anecdotal)

### Inferences
- For a real-time construction-site twin, a practical crane model is: a rigid-multibody crane (slew, trolley, hoist joints with motor/brake models) plus a hoist rope as an adaptive lumped-mass wire (see Section 2), a hook block and a rigid payload with full 6-DoF, so that double pendulum and twisting emerge without extra modelling. Add quadratic drag (½ρC_dA|v_rel|v_rel) on the payload and a stochastic gust field. The analytic Lagrangian pendulum models are best kept as validation references and for anti-sway controller design (input shaping).
- Mobile and crawler cranes add boom compliance, outrigger/track ground-bearing contact and tipping. These couple to the vehicle and soil modules and are best handled by the same direct-solver multibody engine.

### Gaps
- No peer-reviewed, quantitative validation of a commercial crane training simulator (Vortex, Liebherr, Serious Labs) was found. The CIS/CM Labs study results appear unpublished.
- No specific drag-coefficient tables for lattice jib members or typical payloads were retrieved (they are in crane design codes such as EN 13001-2 / ISO 4302, which were not fetched).
- No mobile- or crawler-crane specific dynamics papers were retrieved in this pass.

## 2. Cable/rope/sling simulation (lumped-mass, DER, Cosserat, XPBD, AGX Wire, PhysX/Isaac, Chrono ANCF, MuJoCo)

### Takeaway
For crane hoist ropes and slings carrying heavy loads in real time, the best-proven option is the Servin & Lacoursière adaptive-resolution lumped-element wire (commercialised as AGX Wire). It is solved with a direct solver and stays stable at extreme tension and mass ratios. Graphics-style rods (DER, PBD/XPBD Cosserat) model bending and twist well, but they struggle with stiff, heavily loaded ropes unless they use a direct or Newton solve, as Deul et al. 2018 do. Chrono ANCF and ALE-ANCF suit variable-length hoist ropes in near-real-time digital twins. PhysX/Isaac rope is a capsule-joint workaround, and MuJoCo's cable plugin has known limitations on twist.

### Cited Findings
- Servin & Lacoursière line of work: "Massless Cable for Real-time Simulation" (Computer Graphics Forum 2007), "Rigid Body Cable for Virtual Environments" (IEEE TVCG 2008), "Adaptive resolution in physics based virtual environments" (SIGRAD 2008) and "Hybrid, Multiresolution Wires with Massless Frictional Contacts" (Servin, Lacoursière, Nordfelth, Bodin, IEEE TVCG 17(7):970–982, 2011). — [DBLP](https://dblp.dagstuhl.de/pid/53/4865.html); [Umeå publication list](https://people.cs.umu.se/bokg/ssf/publications-multibody-simulations.html)
- The method combines a quasistatic massless cable model with a dynamic lumped-element model. An adaptive multiresolution algorithm keeps the model numerically stable at high tension while allowing slack at low tension. — [UMIT wiki: Cable and Wire](https://umit.cs.umu.se/wiki/Cable_and_Wire)
- AGX Wire specifics:
  - It is a lumped-element chain of rigid bodies with distance, bend and friction constraints, with sphere nodes and kinematic cylinder segments for edge contact.
  - The maximum resolution is set per unit length (example: 2 elements/m). Nodes are removed under high tension to avoid vibrations from large tension-to-mass ratios.
  - It is "stable even under very high mass ratios/tension".
  - The winch controller is a prismatic-constraint motor with an independent brake (a motor cannot hold position at zero speed, so the brake is used to hold a load).
  - The default kinematic contact model (ShapeContactNodes) is "unconditionally stable". The dynamic contact model is more realistic but less stable at high tension.
  - The Pulley and Gypsy geometry properties keep the wire on sheaves.
  - Limitations: no torsion, links cannot slide through eye nodes, and wire–wire contact is off by default.
  - Example: tension falls from 504 kN to 7 kN across an eye node in a shark-jaw example.
  - Source: [AGX Dynamics User Manual – agxWire](https://www.algoryx.se/documentation/complete/agx/html/doc/UserManual/source/agxwire__simulating_wires.html)
- Discrete Elastic Rods (Bergou et al., ACM TOG 27(3), art. 63, SIGGRAPH 2008): the centerline is dynamic and the material frame quasistatic. Coupling to rigid bodies uses an inextensibility-preserving projection. It was validated with buckling, stability and coupled-mode experiments. — [Columbia DER page](https://www.cs.columbia.edu/cg/rods/index.html); [IP Paris record](https://researchportal.ip-paris.fr/en/publications/discrete-elastic-rods-2/)
- Discrete Viscous Threads (Bergou et al., SIGGRAPH 2010) reports an order-of-magnitude speedup in elastic-rod dynamics from implicit treatment. — [Discrete Viscous Threads PDF](https://ddg.math.uni-goettingen.de/pub/Discrete_Viscous_Threads.pdf)
- Position-based rods:
  - Umetani, Schmidt & Stam 2014 (ghost points define material frames) simulates hundreds of strands in real time. — [Autodesk Research](https://www.research.autodesk.com/publications/position-based-elastic-rods/)
  - Kugelstadt & Schömer SCA 2016 (quaternion orientation constraints, Cosserat) is more than an order of magnitude faster than earlier position-based rods. — [RWTH SCA 2016 PDF](https://animation.rwth-aachen.de/media/papers/2016-SCA-Cosserat-Rods.pdf)
  - Deul, Kugelstadt, Weiler & Bender, "Direct Position-Based Solver for Stiff Rods" (CGF 2018), replaces XPBD Gauss–Seidel with Newton iterations and a direct solve over the acyclic constraint structure. It handles rods of about 10,000 segments about 100× faster than original XPBD. — [Eurographics DL](https://diglib.eg.org/items/7e668ddc-ff48-4ad4-bdf6-167cc4a338ed/full)
- MuJoCo:
  - The cable is `composite type="cable"` with the `mujoco.elasticity.cable` plugin (config: twist, bend, vmax). The plugin library was renamed to "elasticity" in 2.3.1. — [MuJoCo cable.xml](https://github.com/google-deepmind/mujoco/blob/main/model/plugin/elasticity/cable.xml); [MuJoCo 2.3.1 changelog](https://mujoco.readthedocs.io/en/2.3.1/changelog.html)
  - One paper argues the native model's linear torque–deformation relation is not consistent with Kirchhoff rod theory and proposes a DER alternative. — [arXiv 2310.00911](https://arxiv.org/html/2310.00911v3)
  - A third-party benchmark reports that MuJoCo cannot run its twist experiment and that a doubly clamped cable diverges at about 5 rad of twist. — [cosseratbench (PyPI)](https://pypi.org/project/cosseratbench/0.1.0/)
  - A GitHub issue reports overly soft joints at transitions between composite sections. — [MuJoCo issue #1678](https://www.Github.com/google-deepmind/mujoco/issues/1678)
- PhysX / Isaac Sim:
  - NVIDIA staff recommend capsules plus spherical joints (the "Rigid-Body Ropes" example). — [NVIDIA forum: correct way to build a rope](https://forums.developer.nvidia.com/t/what-is-the-correct-way-to-build-a-simulated-rope/244881)
  - Users report instability with many articulation segments (cable carrier, Isaac Sim 4.5). — [NVIDIA forum](https://forums.developer.nvidia.com/t/cable-carrier-drag-chain-simulation-via-joints-possible-instability-or-over-segmentation-issue/350913)
  - Users report non-physical behavior when the rope is attached to robots. — [NVIDIA forum](https://forums.developer.nvidia.com/t/some-problems-about-the-rope-simulation/247985)
  - Volumetric deformables handle slender ropes poorly. — [Isaac Sim GitHub discussion #814](https://github.com/isaac-sim/IsaacSim/discussions/814)
  - As of Isaac Sim 6.0.1, the Newton engine's Cosserat-rod cable is not yet integrated into Isaac Sim; staff suggested standalone Newton. — [NVIDIA forum](https://forums.developer.nvidia.com/t/isaac-sim-6-0-1-newton-engine-cable-cosserat-rod-example-cable-twist-support-how-to-convert-physx-usd-to-newton/381163) (forum sources; version-dependent)
- Chrono ANCF:
  - Chrono's FEA cable demo uses ANCF gradient-deficient "cable" beams with an implicit linearized Euler timestepper. — [Chrono demo_FEA_cables](https://api.projectchrono.org/tutorial_demo__f_e_a_cables.html)
  - ANCF cables are reported as suitable for wire, rope and strand. — [Mech. Sci. 16, 75 (2025)](https://ms.copernicus.org/articles/16/75/2025/ms-16-75-2025.pdf)
  - ALE-ANCF models the variable-length steel rope in hoisting mechanisms. — [MDPI Machines 13(12):1103](https://www.mdpi.com/2075-1702/13/12/1103)
  - Fotland/Haugen (NTNU) target a real-time ALE-ANCF crane digital twin. The generalized-α integrator gives fast, accurate results by damping high frequencies, while RK4 needs a time-step criterion. Lagrange-multiplier, penalty and linear-coupling constraints were all viable. — [NTNU Open](https://ntnuopen.ntnu.no/ntnu-xmlui/handle/11250/2998113)
- SOFA has a Cosserat beam plugin (cable and needle). — [SOFA Cosserat](https://www.sofa-framework.org/?p=23056)

### Inferences
- Choosing by use case:
  - Hoist ropes and heavy-load slings: adaptive lumped-mass wire with a direct solver (AGX Wire), or ALE-ANCF in Chrono when length-varying rope fidelity matters more than speed.
  - Light slings, hoses and electrical cables where bending or twist is visible: XPBD/Cosserat (Deul-style direct solve) or DER.
  - PhysX capsule chains are acceptable only at modest mass ratios and with few segments.
- The stability problem is fundamental. A light rope node between a heavy load and a stiff winch gives a high stiffness/mass ratio. Iterative solvers produce "rubber band" stretch, so resolution reduction (fewer, heavier nodes under tension) combined with a direct or Newton solve is the proven approach.

### Gaps
- No head-to-head real-time benchmark comparing AGX Wire, XPBD, DER, ANCF and MuJoCo on a crane-hoist case was found.
- No experimental validation of the AGX wire against measured rope tension was found in this pass. Validation in the Servin papers was not read in full text.
- Sling-to-load attachment models (multi-leg slings, spreader bars, hook latch) are not covered by sources found.

## 3. Hydraulic actuator modeling (cylinder/valve/pump, load sensing, Simscape, Hopsan, Modelica, AGX, Chrono, FMI)

### Takeaway
The lumped-volume (pressure-node) approach is the practical real-time standard. It models chamber pressure dynamics through bulk modulus, uses orifice-equation valves, models the pump and tank as constant pressure or flow, and adds a load-sensing pump/compensator for excavators. It is implemented natively in Chrono (ChHydraulicActuator2/3) and AGX, or coupled to a multibody solver through FMI. Model Exchange FMUs are much faster than Co-Simulation FMUs in one reported Dymola–Simpack case. Validated load-sensing excavator models exist mainly from the Parma group (Casoli et al.) and run faster than real time.

### Cited Findings
- Chrono hydraulic actuators:
  - ChHydraulicActuator2 has two fluid volumes, a constant-pressure pump and tank, two hoses, one cylinder and a 4/3 directional valve. Its states are spool position and two chamber pressures. — [Chrono API ChHydraulicActuator2](https://api.projectchrono.org/classchrono_1_1_ch_hydraulic_actuator2.html)
  - ChHydraulicActuator3 adds a throttle valve and a third hose volume. — [Chrono API ChHydraulicActuator3](https://api.projectchrono.org/classchrono_1_1_ch_hydraulic_actuator3.html)
  - The base class can run between two bodies or stand-alone. — [ChHydraulicActuatorBase](https://api.projectchrono.org/classchrono_1_1_ch_hydraulic_actuator_base.html)
- Chrono FMI "hydraulic crane co-simulation" demo: a crane FMU (mast and payload) is coupled to an actuator FMU (piston, valve, hoses, pump, tank) through an explicit force–displacement exchange (demo_FMI_hydraulic_crane_cosim). — [Chrono FMI tutorials](https://api.projectchrono.org/tutorial_table_of_content_chrono_fmi.html)
- Dymola (Modelica) hydraulics coupled to a Simpack multibody model through FMI: a Co-Simulation FMU took about 46 min 42 s versus about 1 min 43 s for a Model Exchange FMU. — [Linköping Electronic Conf. Proc. 148 (Modelica 2018?), ecp18148024](https://ep.liu.se/ecp/148/024/ecp18148024.pdf)
- Load-sensing excavator: Casoli, Pompini, Riccò (Univ. Parma) built nonlinear models of an excavator with a load-sensing pump and flow-sharing valves. — [dlib.si full text](https://www.dlib.si/stream/URN:NBN:SI:DOC-5J62A1QS/73b59103-51b1-4f4d-b16d-fd7f8f696f17/TEXT); [SAE 2012-01-2042](https://saemobilus.sae.org/content/2012-01-2042/)
- Parma 2015 (Control Engineering Practice): a coupled diesel-engine plus excavator-hydraulics model ran faster than real time and was validated against experimental data from a real machine. — [air.unipr.it postprint](https://air.unipr.it/bitstream/11381/2789171/5/paper_CEP_revised_20150415_postprint.pdf)
- A load-sensing excavator circuit was modeled in Simcenter Amesim (PoliTo thesis). — [PoliTo thesis](https://webthesis.biblio.polito.it/24382/1/tesi.pdf)
- Excavator payload and dynamic parameter estimation (SICFP 2021). — [SICFP paper](https://ecp.ep.liu.se/index.php/sicfp/article/download/30/29)
- A patent covers identifying work-machine parameters (e.g., bucket coefficients) from operational data. — [US11200761B2](https://patents.google.com/patent/US11200761)
- MBD + DEM + hydraulics via FMI co-simulation (Gan, Zhou, Yu et al., Monash) captured the full working cycle including pressure, flow and power versus cylinder position. — [Monash research portal](https://research.monash.edu/en/publications/investigation-of-performance-of-hydraulic-excavators-by-co-simula/)
- Commercial workflows: Altair MotionSolve plus a Twin Activate hydraulic FMU plus EDEM. — [Altair community](https://community.altair.com/discussion/35588/excavator-response-and-load-estimation-for-fatigue-analysis-in-motionsolve-twin-activate-and-edem); [Siemens Simcenter excavator closed-loop](https://blogs.sw.siemens.com/simcenter/excavator-closed-loop-simulator/)
- A 2025 field-data-driven co-simulation framework was applied to a 20-t backhoe excavator. — [Scientific Reports s41598-025-29412-5](https://link.springer.com/10.1038/s41598-025-29412-5) (abstract only)
- Algoryx states that AGX covers hydraulics, terrain and machine subsystems in real time (vendor claim). — [algoryx.se](https://www.algoryx.se/)
- TERA uses Unity plus AGX for real-time excavator simulation and reports a similarity evaluation against a real excavator. — [arXiv 2501.01430](https://arxiv.org/html/2501.01430v1)

### Inferences
- For real-time construction machines, the recommendation is to model hydraulics natively inside the multibody step (Chrono ChHydraulicActuator, AGX hydraulics) or as a Model-Exchange FMU integrated by the host solver. Explicit co-simulation of stiff hydraulic states with a multibody solver at large macro steps is a common instability source and is much slower in the reported case.
- Hopsan (Linköping, open source, TLM-based) is the natural free tool for building and exporting such FMUs. However, no Hopsan-specific excavator identification paper was found in this pass.

### Gaps
- No Hopsan excavator or load-sensing validation paper was retrieved. A Simscape Fluids excavator validation paper was not retrieved either.
- No documentation of AGX's hydraulics module internals or its validation was found.
- Parameter-identification papers for bulk modulus, valve coefficients and friction (Stribeck) on real excavators need a targeted search (e.g., Mattila/Tampere, Aalto, Linköping groups).

## 4. Tracked vehicles, wheel loaders, articulated dump trucks, tire models on deformable ground

### Takeaway
Chrono::Vehicle is the most complete open-source framework. It provides templated tracked vehicles (single- and double-pin shoes, segmented bands), soft real-time with an RTF estimate, rigid and FEA (ANCF) tires, and SCM (Bekker–Wong/Janosi–Hanamoto), DEM and continuum CRM terrains. AGX Dynamics has the strongest published sim-to-real evidence for wheel-loader digging. No ADT-specific multibody papers or FTire-on-deformable-ground sources were found.

### Cited Findings
- Chrono::Vehicle TrackedVehicle supports soft real-time (off by default) with an RTF query, and track-shoe collision toggles. The M113 template takes a shoe type and a double-pin topology. — [Chrono API TrackedVehicle 9.0](https://api.projectchrono.org/9.0.0/classchrono_1_1vehicle_1_1_tracked_vehicle.html); [M113_Vehicle](https://api.projectchrono.org/8.0.0/classchrono_1_1vehicle_1_1m113_1_1_m113___vehicle.html)
- The sprocket type and wheel type must match the track-shoe type, and lateral guiding pins were not implemented (2019 slides). — [Chrono::Vehicle Tracked slides](https://www.projectchrono.org/assets/slides_3_0_0/5_Vehicle/3_ChronoVehicle_Tracked.pdf)
- Chrono terrain options:
  - Semi-empirical Bekker/SCM terrain. — [SBEL: Terramechanics methods for real-time off-road mobility](https://sbel.wisc.edu/2018/05/19/terramechanics-methods-for-real-time-off-road-vehicle-mobility-simulation-on-deformable-terrain/)
  - Chrono::CRM continuum terramechanics (2025). — [arXiv 2507.05643](https://web3.arxiv.org/pdf/2507.05643)
- Recuero, Serban et al.: ANCF nonlinear FE tires on granular material, run as a vehicle–tire–terrain co-simulation. — [ISTVS](https://istvs.org/publication-news/a-high-fidelity-approach-for-vehicle-mobility-simulation-nonlinear-finite-element-tires-operating-on-granular-material)
- Earlier SBEL work: an HMMWV in Chrono::Vehicle on 3-D deformable terrain. — [SAE 2014-01-0873](https://saemobilus.sae.org/content/2014-01-0873)
- A DEM vs SCM comparison in Chrono found SCM computationally efficient, while DEM was limited by particle count. — [PoliTo thesis](https://webthesis.biblio.polito.it/37442/1/tesi.pdf)
- Wheel loaders:
  - "Examining the simulation-to-reality gap of a wheel loader digging in deformable terrain" (AGX-supported). — [arXiv 2310.05765](https://arxiv.org/pdf/2310.05765)
  - Data-efficient excavation force estimation with AGX parameters calibrated and validated against real wheel-loader data. — [arXiv 2506.22579](https://arxiv.org/pdf/2506.22579)
- Multibody 2023 extended abstract on vehicle–terrain (ID 134). — [Multibody 2023](https://multibody2023.tecnico.ulisboa.pt/prog_MULTIBODY_WEB/MULTBODY2023_ABSTRACTS/ID_134_160_extendedAbstract.pdf) (not read)

### Inferences
- An articulated dump truck can be built in Chrono::Vehicle or AGX as two chassis bodies joined by an articulation (yaw) joint plus an oscillation (roll) joint, steered by two hydraulic cylinders (Section 3 models). Tires are best handled by TMeasy or Pacejka MF on rigid haul roads and by SCM-coupled rigid or FEA tires on soft ground.
- FTire (cosin) is high-fidelity but rarely real-time for multi-tire vehicles on deformable ground. Chrono ships an FTire-free stack.

### Gaps
- No ADT, rigid-haul-truck or wheel-loader articulated-steering validation papers were found.
- No FTire–deformable soil coupling sources were found, and no Pacejka-on-soft-soil sources were retrieved (the MF model is for hard surfaces; this is general knowledge, not sourced here).
- No quantitative Chrono tracked-vehicle validation (e.g., against M113 test-track data) was retrieved.

## 5. Multibody solvers with large mass ratios and stiff contacts

### Takeaway
Iterative PGS solvers degrade at mass ratios beyond about 1:1000 and on long heavy-light chains such as wires, where they produce rubber-banding. Direct (pivoting/LDL) or hybrid split solvers are the robust choice. AGX's split solver (direct for joints and contact normals, iterative for friction) reports stability to about 1:100,000. Roblox and Unity added direct/LDL options for the same reason, and Deul et al. show Newton/direct XPBD for stiff rods.

### Cited Findings
- AGX guidance:
  - Iterative solvers do well on large stacks of similar bodies but poorly on wires and lines with large mass and force ratios ("rubber band behavior").
  - Heavy vehicles require the direct solver.
  - The split solver gives residual errors at least two orders of magnitude smaller than purely iterative solving, and is stable up to about 1:100,000 mass ratio with certain settings.
  - For the iterative solver, keep interacting bodies within about one order of magnitude (0.1 vs 100 can be unstable).
  - The default step is 300 Hz (3.33 ms); the rule of thumb is that the smallest feature should move less than half its length per step.
  - Source: [AGX Momentum: Stable Simulations](https://www.algoryx.se/documentation/complete/Momentum/source/Stable%20Simulations.html); [AGX overview](https://www.algoryx.se/documentation/complete/agx/html/doc/UserManual/source/overall_structure.html)
- Roblox: high mass ratios destabilize iterative solvers, so it added an LDL–PGS hybrid for equality constraints. — [Roblox: improving simulation performance](https://corp.roblox.com/newsroom/2020/08/improving-simulation-performance-advanced-physics-solver)
- Unity Physics offers a direct solver type for high mass or stiffness ratios and an iterative type for speed. — [Unity Physics SolverType](https://docs.unity3d.com/Packages/com.unity.physics@6.5/api/Unity.Physics.SolverType.html)
- A comparative study of direct, iterative and hybrid MLCP solvers on benchmarks including large mass ratios. — [Eurographics VRIPHYS 2018](https://diglib7.eg.org/bitstream/handle/10.2312/vriphys20181063/011-020.pdf)
- Large-dimensional multibody dynamics using contact nodalization and diagonalization. — [arXiv 2201.09212](https://arxiv.org/pdf/2201.09212)
- A direct/Newton XPBD solve for stiff rods gives a ~100× speedup over Gauss–Seidel XPBD. — [Deul et al. 2018](https://diglib.eg.org/items/7e668ddc-ff48-4ad4-bdf6-167cc4a338ed/full)
- Chrono's FEA cables use implicit Euler linearized timestepping, and generalized-α damps high frequencies for ANCF ropes. — [Chrono demo](https://api.projectchrono.org/tutorial_demo__f_e_a_cables.html); [NTNU](https://ntnuopen.ntnu.no/ntnu-xmlui/handle/11250/2998113)

### Inferences
- Ranking for crane and heavy-machine scenes, most robust first:
  1. Direct or split constraint solvers (AGX; Vortex is also known to use a direct solver, though this was not sourced here).
  2. Reduced-coordinate articulations (Featherstone ABA in PhysX articulations, MuJoCo, Isaac), which handle mass ratios within a tree well. Closed loops (hydraulic cylinder linkages) and contact-coupled heavy/light bodies still go through iterative or soft constraints.
  3. MuJoCo's convex soft-contact Newton/CG solver, which is robust and deterministic but adds compliance by design.
  4. XPBD, which is fast but needs substeps and Newton/direct variants for stiff, heavy chains.
  5. Plain PGS, the weakest.

### Gaps
- No published benchmark directly compares PhysX TGS, MuJoCo Newton, AGX direct, Chrono NSC/SMC and XPBD on the same crane/excavator scene.
- MuJoCo convex-solver and PhysX articulation mass-ratio documentation were not fetched in this pass.

## 6. Collapse/falling materials, structural failure, scaffolding stability, safety-incident simulation

### Takeaway
Scaffolding failure literature is FEM-based (buckling, imperfections, wind-induced progressive collapse) and is used offline or to train monitoring models. Real-time sites would use rigid-body or breakable-joint proxies calibrated against such FEM results. No sources combining real-time falling-object physics with construction safety were found.

### Cited Findings
- Steel scaffolds: experiments and FE analysis of flaws and imperfections found lateral buckling to be the typical collapse mode. — [Politecnico di Torino, Engineering Structures](https://iris.polito.it/bitstream/11583/2690093/3/1704020_ENGSTRUCT.pdf)
- JNIOSH simulated wind-induced scaffold collapse and notes that such collapses are often progressive. — [JNIOSH SRR No. 21](https://www.jniosh.johas.go.jp/en/publication/srr/srr-no21.html)
- Cho et al.: FEM analysis of a scaffold over 1,200 random load cases generated strain data for a data-driven collapse-prevention monitoring system. — [ASCE JCEM 2018](https://ascelibrary.com/doi/10.1061/%28ASCE%29CO.1943-7862.0001535); [ISARC 2018](https://www.iaarc.org/publications/fulltext/ISARC2018-Paper017.pdf)
- BIM-based analysis of workspace–scaffold conflicts and falling objects from scaffolds onto other activities. — [KoreaScience](https://koreascience.or.kr/article/CFKO201534159173278.pdf)
- Scaffold erection optimisation with mathematical models and virtual simulation. — [SBEnrc](https://sbenrc.com.au/wp-content/uploads/2013/10/Publication-3-27-1_A-Framework-Design-for-Optimizing-Scaffolding-Erection-by-Applying-Mathematical-Models-and-Virtual-Simulation.pdf)
- Scaffold structural analysis (Oxford Brookes, Beale 2014). — [Beale 2014](https://radar.brookes.ac.uk/radar/file/0e0e3673-87ad-4038-a8dd-1aa675941319/1/beale2014scaffold.pdf) (not read)

### Inferences
- A practical real-time approach is to represent scaffolds and temporary structures as rigid members with breakable joints, where failure thresholds come from offline FEM buckling and connection-capacity analysis. Collapse and falling debris then come from the multibody or contact engine. Full FEM collapse is for offline incident reconstruction.

### Gaps
- No Automation in Construction paper on real-time falling-object or collapse simulation was retrieved.
- No sources were found on crane collapse or tipping incident reconstruction, formwork collapse, or dropped-load impact simulation.

## 7. Human workers: animation, biomechanics, crowd navigation on site

### Takeaway
Construction-specific work is mostly agent-based simulation in game engines driven by 4D BIM to estimate worker–equipment collision probability, plus VR serious games for struck-by and overhead-load hazards. Dedicated pedestrian-dynamics or biomechanics models for site workers were not found.

### Cited Findings
- Sorbi et al. (CONVR 2022): agent-based simulation in Unity, driven by 4D BIM, predicts worker–heavy-equipment collision events at the site-design stage. — [Sorbi et al. CONVR22 PDF](https://flore.unifi.it/retrieve/205992cc-4463-4404-b367-c8e604d86209/2022%20-%20Sorbi%20et%20al.%20-%20CONVR22%20-%20A%20Framework%20Proposal%20for%20Construction%20Site%20Agent-Based%20Simulation.pdf)
- ISARC 2021 (Aarhus): an asymmetric multiplayer VR game with vibrotactile haptics (two HMD workers and one gantry-crane operator) raised awareness by up to 35% and reduced time spent under crane loads. — [ISARC 2021 Paper 160](https://www.iaarc.org/publications/fulltext/081%20ISARC%202021%20Paper160.pdf)
- Struck-by hazards caused 15% of US construction fatalities in 2019. A VR virtual-accident simulation was used to counter risk desensitization (Texas A&M). — [ROSA P report](https://rosap.ntl.bts.gov/view/dot/61777); [TAMU 2026 story](https://stories.tamu.edu/news/2026/06/23/can-virtual-reality-and-ai-help-construction-workers-avoid-accidents/)
- Monash crane-lift safety work notes many crane incidents stem from misjudged clearance to workers, power lines and structures. — [Monash news](https://monash.edu/news/articles/innovations-raise-safety-of-crane-lift-operation)
- Ruhr-Uni Bochum: a BIM-based VR CAVE lets workers experience being run over by trucks or hit by crane booms. — [Construction Management](https://constructionmanagement.co.uk/workers-ha2ve-acci7dents-they-ar7rive-site/)

### Inferences
- For a site twin, the recommendation is to combine navmesh plus RVO/ORCA or social-force crowd steering (Unity, Unreal, Isaac) with a 4D-BIM task schedule, as Sorbi et al. do. Workers should be kinematic animated characters with collision proxies. Biomechanical (musculoskeletal) models are only needed for ergonomics studies and are not part of real-time crowd simulation.

### Gaps
- No construction-specific pedestrian-dynamics validation, worker biomechanics in simulation (e.g., OpenSim lifting tasks), or crowd model calibration on construction sites was found in this pass.
