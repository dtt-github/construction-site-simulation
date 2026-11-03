# Numerical Methods for Soil / Terrain and Soil-Tool Interaction in Excavation and Earthmoving: Survey Notes

Notes compiled 2026-11-02. Legend: **[PR]** = peer-reviewed; **[PP]** = preprint / not yet peer-reviewed; **[DOC]** = vendor or project documentation. Items marked **(TK)** come from my training knowledge with a canonical DOI or URL; I did not re-fetch them this session. The report writer should treat their exact numbers with more caution than items fetched live.

---

## Q1. Classical terramechanics and analytical bucket/blade force models (Bekker-Wong, Janosi-Hanamoto, SCM, Reece/McKyes FEE, Balovnev, Zelenin)

### Takeaway
Semi-empirical models are the cheapest option, running at real time or much faster on a CPU. They give instantaneous tool forces but do not track where the soil goes; the swell, heap, spill and pile shape have to be added on top, for example with a heightfield. Chrono's SCM is 2-3 orders of magnitude faster than DEM and matches DEM drawbar-pull well for wheels. The Fundamental Earthmoving Equation (FEE) is the standard real-time bucket-force model in RL excavation work (ETH RSL, UC Davis), but it only holds for quasi-static, steady cutting.

### Cited Findings
- Chrono::CRM authors frame Bekker-Wong and Janosi-Hanamoto as the semi-empirical baseline that physics-based methods aim to move beyond **[PP]** — [Unjhawala et al. 2025, arXiv 2507.05643](https://arxiv.org/abs/2507.05643)
- SCM vs DEM in Chrono: single-wheel and full-rover runs on SCM "match well" with DEM results, and SCM is 2-3 orders of magnitude faster than DEM. SCM parameters were calibrated with a *virtual bevameter* (DEM-simulated bevameter rig plus Bayesian inference) **[PP→PR]** — [Zhang, Dai, Hu, Negrut, arXiv 2407.18903](https://arxiv.org/abs/2407.18903); [SBEL summary](https://sbel.wisc.edu/publications/journal-papers/calibration-of-an-expeditious-terramechanics-model/)
- SCM calibrated against CRM (SPH continuum) is a "good proxy" for CRM, at roughly 10x speedup. Example calibrated parameters: cohesion 2495 Pa, friction angle 24°. Drawbar-pull vs slip agreed well for a single wheel and the full VIPER rover. This comparison is SCM vs CRM, not vs physical tests **[PR conference abstract]** — [Hu et al., ECCOMAS Multibody 2023 extended abstract](https://multibody2023.tecnico.ulisboa.pt/prog_MULTIBODY_WEB/MULTBODY2023_ABSTRACTS/ID_106_121_extendedAbstract.pdf); [Authorea preprint 2023](https://www.authorea.com/doi/full/10.22541/au.167355891.12085319)
- Chrono SCM extends DLR's SCM to arbitrary 3D shapes and contact patches, with multi-core parallelism and multi-vehicle scalability. It achieves "real-time or close" — [IDETC session listing](https://idetc.secure-platform.com/a/solicitations/180/sessiongallery/11308/application/89470); SCM origin: Krenn & Hirzinger (DLR), "SCM – A soil contact model for multi-body system simulations", 11th European ISTVS Conf. 2009 (TK)
- FEE (Reece 1964) is used in current wheel-loader trajectory optimisation, combined with nonlinear parameter estimation and optimal control (UC Davis) **[PP]** — [arXiv 2509.10642](https://arxiv.org/pdf/2509.10642). Original source: Reece, "The fundamental equation of earth-moving mechanics", Proc. IMechE 179(3F), 1964 (TK) — [DOI 10.1243/PIME_CONF_1964_179_134_02](https://doi.org/10.1243/PIME_CONF_1964_179_134_02)
- TERA (Univ. at Buffalo + MOOG) describes FEE-based analytical models (e.g., "Park's model") as fast and real-time but "limited to stationary conditions" and unable to describe soil motion. FEE plus particle dynamics captures soil motion but is too heavy for multi-robot RL **[PP]** — [TERA, arXiv 2501.01430](https://arxiv.org/html/2501.01430v1)
- ETH RSL "Soil-adaptive excavation using reinforcement learning" (Egli, Gaschen, Kerscher, Jud, Hutter, RA-L 2022) trained in Isaac Gym with an FEE-type soil-force model and a heightfield (TK) — [DOI 10.1109/LRA.2022.3189834](https://doi.org/10.1109/LRA.2022.3189834)
- Bekker pressure-sinkage plus Janosi-Hanamoto shear: Wong, *Theory of Ground Vehicles* (4th ed., Wiley 2008) and *Terramechanics and Off-Road Vehicle Engineering* (2010) (TK). Janosi & Hanamoto, "The analytical determination of drawbar pull as a function of slip for tracked vehicles in deformable soils", 1st ISTVS Conf., Turin, 1961 (TK)
- McKyes, *Soil Cutting and Tillage* (Elsevier, 1985) generalises the Reece wedge to 3D tools with side-failure terms (TK). Balovnev (*New Methods for Calculating Resistance to Cutting of Soil*, 1963/1983) and Zelenin (*Machines for Earthmoving Work*, 1975/1986) give Russian empirical bucket-resistance formulas still used in Eastern European and Chinese bucket studies (TK). I found no online primary sources for these in this session.

### Inferences
- For a construction-site twin, semi-empirical force models (FEE, McKyes) are viable as the force channel and calibration target inside a hybrid (heightfield + particles) scheme. They are not adequate on their own for realistic soil flow, spill or pile formation.
- SCM is wheel- and track-focused. It handles sinkage and slip well, but it is not designed for bucket filling or material transport.

### Gaps
- I found no published quantitative error of FEE vs measured bucket forces on full-scale excavators in this session. Literature (TK) suggests FEE often underestimates dynamic and penetration forces substantially (factor ~1.5-3). This is unverified.
- No digital source for Balovnev or Zelenin accuracy comparisons.

---

## Q2. DEM (Discrete Element Method)

### Takeaway
DEM is the most widely used high-fidelity method for bucket and blade studies and the reference "ground truth" in most hybrid/real-time papers. GPU DEM (Chrono DEM-Engine, Chrono::GPU, Altair EDEM GPU, Rocky, Fraunhofer Demify) reaches 10^7-10^8 particles but is roughly 3-4 orders of magnitude slower than real time at realistic resolutions. Field validations of DEM bucket filling reach ~10% agreement. Calibration (triaxial or standard lab tests, JKR cohesion, rolling resistance) is the main accuracy driver.

### Cited Findings
- **Chrono DEM-Engine (DEME)**, Zhang, Negrut et al.: a dual-GPU DEM with customizable contact force models (user-injected CUDA) and non-spherical clump elements. Linear scaling to **150 million elements on two NVIDIA A100 GPUs**. Co-simulation of **11 million elements over 15 s** with an external multibody system (rover on granular terrain) "can be efficiently executed within a day" **[PR, Computer Physics Communications 2024; arXiv 2311.04648]** — [arXiv](https://arxiv.org/abs/2311.04648v2); [NSF PAR copy](https://par.nsf.gov/servlets/purl/10518863)
- **Chrono::GPU** (Fang et al. 2021): handles monodisperse spheres only, up to **123 million elements on one GPU** **[PR]** — [NSF PAR](https://par.nsf.gov/servlets/purl/10349475)
- **Wheel-loader sim-to-real (Aoshima & Servin)**: DEM soil at resolutions 50-400 mm and time steps 2-500 ms, with and without multiscale acceleration. Speed ranged from **1/10,000 to 5x real time**. Measured against field bucket-filling time series (motion, actuator forces, loaded mass, work), the **sim-to-real gap was ~10% and depended only weakly on model fidelity**. Controller transfer saw a ~15% domain gap and a ~5% performance drop **[PR, Multibody System Dynamics 2024; arXiv 2310.05765]** — [arXiv](https://arxiv.org/abs/2310.05765); [Algoryx publication list](https://www.algoryx.se/publications/?tgid=3)
- **Wheeled-excavator DEM validation (Fraunhofer, GPU solver "Demify")**: calibration/validation routine built on a standardized soil lab test, with parameters optimized by matching triaxial simulation to real triaxial data **[PR, Multibody System Dynamics 2025]** — [Springer 10.1007/s11044-025-10061-5](https://link.springer.com/10.1007/s11044-025-10061-5); [EasyChair preprint](https://10wvvw.easychair.org/publications/preprint/jsX2/open)
- **EDEM + ADAMS co-simulation** of excavator bucket digging, giving digging resistance per bucket part **[PR, Applied Sciences 2023]** — [DOAJ](https://doaj.org/article/9b6c39f5ea254a629e91df18aabf1424). Adams-EDEM coupled model with soil parameters calibrated against theoretical (analytical) resistance **[PR, Jixie Qiangdu 2023]** — [DOAJ](https://doaj.org/article/31aff726754443ef89b295314aebb482)
- Scale-model clamshell DEM study: DEM under-predicts mid-range digging forces, which the authors attribute to particle interlocking — via search summary of [Wits thesis](https://wiredspace.wits.ac.za/items/03a1301c-f4e0-434f-8f5e-41da919881ae). A 2025 Russian mining paper validated DEM digging force and scooped volume against a physical model — [Mining-Media](https://mining-media.ru/en/articles/original-paper/19596-numerical-simulation-of-the-digging-process-with-an-excavator-bucket-using-the-discrete-element-method)
- **Wiberg, Servin, Nordfjell (2021)**: DEM of large soil deformations under heavy (forestry) vehicles **[PR, J. Terramechanics 93:11-21]** — [Algoryx list](https://www.algoryx.se/publications/?tgid=3)
- **Lindmark & Servin (2018)**: computational exploration of robotic rock loading with DEM **[PR, Robotics and Autonomous Systems 106:117-129]** — [Algoryx list](https://www.algoryx.se/publications/?tgid=3)
- **DEM as reference for SCM calibration** (virtual bevameter) — [arXiv 2407.18903](https://arxiv.org/abs/2407.18903)
- Other open-source DEM codes (TK): LIGGGHTS (CFDEM, Kloss et al. 2012, CPU/MPI; LIGGGHTS-PUBLIC development has largely stalled); Yade (CPU, Python-scriptable); MFiX-DEM (NETL, CFD-DEM) — [MFiX](https://mfix.netl.doe.gov/?p=15599); Blaze-DEM (Govender, Wilke et al., GPU polyhedral DEM, Computer Physics Communications 2016) (TK). Commercial: Altair EDEM (GPU solver; used by OEMs including Caterpillar and Volvo CE per vendor case studies (TK; not verified this session)); Ansys Rocky (a case reported ~98% match vs an analytical excavation force — via search summary, original not retrieved).
- Cohesion and moisture in DEM (TK): JKR cohesion (Johnson-Kendall-Roberts) and linear cohesion models in EDEM; liquid-bridge capillary models; Edinburgh Elasto-Plastic Adhesion (EEPA, Morrissey/Thakur 2013-2014) for cohesive, compactable bulk solids. Rolling resistance (type C / EPSD) stands in for particle shape. Calibration usually uses angle of repose, shear-box or triaxial tests, and penetration tests.

### Inferences
- DEM is the right *offline* reference for a construction-site twin: calibrating faster models, generating training data for learned surrogates, and checking bucket fill factors. It is not viable for real-time full-site simulation on a single GPU. At 10^6-10^7 particles, a single-GPU DEM typically runs 10-1000x slower than real time for realistic grain-scale-coarsened soils.
- The Aoshima & Servin result (gap ~10%, weakly dependent on fidelity) is the strongest evidence that coarse, real-time-capable particle models can be "good enough" for bucket-filling KPIs such as loaded mass and work.

### Gaps
- Could not retrieve publicly documented EDEM excavator studies by Caterpillar, Komatsu or Volvo CE with quantitative accuracy. OEM use is known from Altair marketing, not peer-reviewed papers.
- Exact per-step timings of DEME for an excavator-scale bucket problem not found.

---

## Q3. MPM (Material Point Method)

### Takeaway
MPM handles large deformation, separation and re-merging of continuum soil naturally. It supports Drucker-Prager sand, Mohr-Coulomb, (modified) Cam-Clay for clays and compaction, and mu(I) rheology. GPU MLS-MPM codes reach millions of particles at interactive rates. Engineering validation of MPM for bucket excavation exists but is thin. ETH RSL used GPU MPM for excavator RL with full-scale (11.5 t excavator) sim-to-real transfer in 2026, and NVIDIA's Newton (Isaac Lab 3.x) now ships an experimental implicit MPM granular solver.

### Cited Findings
- **Klár et al. 2016**, "Drucker-Prager elastoplasticity for sand animation", ACM TOG 35(4) (SIGGRAPH) **[PR]** (TK) — [DOI 10.1145/2897824.2925906](https://doi.org/10.1145/2897824.2925906). This is the standard Drucker-Prager return-mapping for MPM sand used by most graphics and robotics MPM codes.
- **Hu et al. 2018, MLS-MPM**, "A moving least squares material point method with displacement discontinuity and two-way rigid body coupling", ACM TOG 37(4) **[PR]** (TK) — [DOI 10.1145/3197517.3201293](https://doi.org/10.1145/3197517.3201293). Basis of Taichi `mpm88`/`mpm128`, Genesis and PhysGaussian-style simulators.
- GPU MPM performance (TK): Gao et al. 2018 "GPU optimization of material point methods" (TOG 37(6)) reported millions of particles at a few seconds per frame or better on a single GPU — [DOI 10.1145/3272127.3275044](https://doi.org/10.1145/3272127.3275044). Wang et al. 2020 "A massively parallel and scalable multi-GPU MPM" (TOG 39(4)) scaled to ~10^8 particles.
- **Implicit MPM in NVIDIA Newton / Isaac Lab**: Newton's `SolverImplicitMPM` models sand, snow and other granular media. It solves rheology implicitly on a background voxel grid and resolves collider contact inside the implicit solve, bypassing Newton's collision pipeline. Rigid-MPM coupling is **experimental**. A shared grid prevents per-world reset of solver history **[DOC]** — [Isaac Lab Implicit MPM docs](https://isaac-sim.github.io/IsaacLab/main/source/concepts/using_mpm.html); [NVIDIA docs](https://docs.nvidia.com/isaaclab/concepts/implicit-mpm)
- **ETH RSL 2026**: "Size Doesn't Matter: Material-State RL for Excavator Transferable Soil Manipulation" (Werner, Eyschen, Costello, Micarelli, Cramariuc, Hutter). RL in a **GPU-parallelized MPM** simulation, with policies conditioned on material state (shape, compactness). Deployed on an **11.5 t hydraulic excavator**: built a 42 m x 2.1 m embankment in 45 min with 201 strokes and no failures, matching an expert operator's progression speed **[PP, arXiv 2609.12677, Sept 2026]** — [arXiv](https://arxiv.org/abs/2609.12677). The simulator name and soil constitutive model are not stated in the abstract.
- **GeoTaichi** (Shi, Guo, Yang): Taichi-based DEM, MPM and coupled MPDEM for geophysical large-deformation problems, running on CPU and GPU, GPL-3.0, `pip install geotaichi`. Authors claim notable efficiency and memory gains over similar tools **[PR, Computer Physics Communications 301 (2024) 109219]** — [LAMMPS abstract page](https://lammps.org/abstracts/abstract.35453.html); [PyPI](https://pypi.org/project/geotaichi); [Mendeley data](https://data.mendeley.com/datasets/858bmcf7j6/1)
- **CB-Geo MPM** (Kumar et al.): "A scalable and modular MPM for large-scale simulations" (C++/MPI, CPU) **[PP]** — [engrXiv e24rb](https://engrxiv.org/e24rb/). **Anura3D** (Deltares/TU Delft/UPC consortium; CPU; geotechnical, multiphase, saturated soils) has an excavation feature validated on the Selborne slope experiment — [UPC thesis](https://upcommons.upc.edu/bitstreams/694ea70e-c7d1-4b72-bf7a-b2d37d7f707f/download). **Itasca MPoint** is a commercial MPM — [Itasca](https://itascasoftware.com/resources/modeling-very-large-deformation-with-the-material-point-method/)
- **Excavation-specific MPM validation**: Haeri et al., ISARC 2020, "Efficient numerical methods for accurate modeling of soil cutting operations". MPM and a hybrid method were compared to soil deformation and tool forces measured by robotic test equipment **[PR conference]** — [IAARC](https://www.iaarc.org/publications/2020_proceedings_of_the_37th_isarc/efficient_numerical_methods_for_accurate_modeling_of_soil_cutting_operations.html); [PDF](https://www.iaarc.org/publications/fulltext/ISARC_2020_Paper_60.pdf)
- 2D soil-bucket MPM for automated-equipment task planning (Kyungpook National Univ., Korea) — [KNU](https://research.knu.ac.kr/en/publications/2d-soil-bucket-simulation-using-material-point-method-for-efficie/)
- MPM (u-εv formulation) coupled with DDA (Discontinuous Deformation Analysis) for hydraulic excavator bucket simulation; the DDA part enables rocks and blocks (Okagawa & Hashimoto, Kyoto, PARTICLES 2025) **[PR conference abstract]** — [PARTICLES 2025](https://particles2025.cimne.com/event/contribution/f6b9199b-0d96-11f0-9835-000c29ddfc0c)
- **Wheel-terrain MPM validation (TK)**: Agarwal, Senatore, Zhang, Kingsbury, Iagnemma, Goldman, Kamrin, "Modeling of the interaction of rigid wheels with dry granular media", J. Terramechanics 85 (2019). Continuum MPM with a plasticity model; wheel-soil sinkage and drawbar-pull compared with experiments, with good agreement reported. The same group's Resistive Force Theory (Li, Zhang, Goldman, Science 2013, [DOI 10.1126/science.1229163](https://doi.org/10.1126/science.1229163)) offers a very fast intermediate model for intruders.

### Inferences
- MPM is the most realistic *continuum* option that can still approach real time on one GPU at moderate resolution (~10^5-10^6 particles; cell size ~5-10 cm for a bucket workspace). It handles cohesive soils and mud (Cam-Clay / Herschel-Bulkley / viscoplastic) better than DEM because constitutive laws are explicit, not emergent.
- Rigid-MPM two-way coupling (bucket, tracks) is the weakest link in current GPU frameworks. Newton labels it experimental; MLS-MPM's CPIC handles thin boundaries.
- Validation against excavation force data is sparse compared with DEM. Expect to calibrate yourself, e.g., against DEM or against bucket-force rig data.

### Gaps
- Found no published quantitative excavation-force error (%) for GPU MPM vs full-scale experiments.
- Throughput numbers for Newton implicit MPM (particles/s, real-time factor) not published in the docs fetched.

---

## Q4. SPH for soil and Chrono::CRM (Continuum Representation Model)

### Takeaway
SPH with elasto-plastic soil (Bui et al. 2008) underlies Chrono::CRM, the strongest open-source (BSD-3) GPU continuum terramechanics package. It is validated against physical tests (including NASA's MGRU3 rover) and a DEM RASSOR digging benchmark. Its authors claim efficiency "comparable" to semi-empirical methods using active domains, and scale to 100 M SPH particles on 10 km terrain at "near interactive rates". In a 2025 soil-cutting head-to-head, SPH was less accurate than CEL FEM (13.8% vs 8.8% error).

### Cited Findings
- **Chrono::CRM** — Unjhawala, Bakke, Zhang, Taylor, Arivoli, Serban, Negrut, "A Physics-Based Continuum Model for Versatile, Scalable, and Fast Terramechanics Simulation". Built on Chrono's SPH framework, GPU-accelerated. Supports digging, grading, deformable wheels and grousers, coupled to rigid and flexible multibody implements. "Active domains" support terrain up to **10 km long with 100 million SPH particles at near interactive rates**. Validated against **three physical tests, including NASA MGRU3 rover**, and benchmarked vs high-fidelity DEM of **RASSOR digging**. BSD-3 open source **[PP, arXiv 2507.05643 v2 Aug 2025; submitted to J. Terramechanics]** — [arXiv](https://arxiv.org/abs/2507.05643). The abstract gives no error percentages.
- CRM used as the high-fidelity reference for SCM calibration with VIPER rover single-wheel and full-vehicle tests; the CRM approach reportedly runs at "real-time factors of 30 and above" (i.e., 30x *slower* than real time) — [search summary of Hu et al. ECCOMAS 2023](https://multibody2023.tecnico.ulisboa.pt/prog_MULTIBODY_WEB/MULTBODY2023_ABSTRACTS/ID_106_121_extendedAbstract.pdf); NASA context — [NTRS 20230017332](https://ntrs.nasa.gov/citations/20230017332)
- **Bui, Fukagawa, Sako, Ohno 2008**, "Lagrangian meshfree particles method (SPH) for large deformation and failure flows of geomaterial using elastic-plastic soil constitutive model", Int. J. Numer. Anal. Methods Geomech. 32(12) **[PR]** (TK) — [DOI 10.1002/nag.688](https://doi.org/10.1002/nag.688). This is the foundational Drucker-Prager SPH soil paper.
- **SPH vs CEL for soil cutting** (biomimetic digging shovel, soil-bin validated): **CEL error 8.81%, SPH 13.76%**; runtime CEL 6 h 35 min, SPH 7 h 18 min (CPU, Abaqus) **[PR, PLOS ONE 20(5) 2025]** — [Dong et al. 2025](https://ideas.repec.org/a/plo/pone00/0322861.html)

### Inferences
- Chrono::CRM is the best-documented open-source candidate for *physics-based continuum soil coupled to a full multibody excavator* (Chrono has hydraulics and vehicle modules). However, its own group's numbers ("RTF 30+", "near interactive") suggest it is not real time on one GPU for bucket-scale problems with fine resolution.
- The NASA lineage (VIPER, MGRU3, RASSOR) means validation is skewed to lunar regolith and rover mobility, not construction soils (cohesive, moist).

### Gaps
- Specific CRM error numbers vs the MGRU3 test, and exact RTF per particle count, are in the paper body and were not extracted.
- No evidence found of CRM validation on wet, cohesive construction soils.

---

## Q5. FEM / CEL (Abaqus Coupled Eulerian-Lagrangian, ALE)

### Takeaway
CEL/ALE FEM achieves the best single-tool cutting-force accuracy in the comparisons found (~9% error). It costs hours per case on a CPU, does not capture soil flow or spill well beyond the cutting zone, and is not viable for real time.

### Cited Findings
- CEL 8.81% vs soil bin, 6.6 h runtime; better than SPH on both accuracy and cost — [Dong et al., PLOS ONE 2025](https://ideas.repec.org/a/plo/pone00/0322861.html)
- Zhang et al. 2018 (Biosystems Engineering): CEL FE study of soil-tool interaction; rake angle and depth effects on reaction force (cited in a 2025 Abaqus excavator-tooth study) — via search summary of [KoreaScience 2026](https://www.koreascience.or.kr/article/JAKO202612672121560.page?&lang=en)
- Ibrahmi et al. 2014 Abaqus Explicit blade study: draught force is the largest component at all cutting angles — via search summary, [Dialnet](https://dialnet.unirioja.es/descarga/articulo/4951264.pdf)
- Abaqus CEL allows portions of a model to be Eulerian or Lagrangian with automatic contact between them (spudcan penetration tech brief) — [SIMULIA tech brief](https://imechanica.org/sites/default/files/Architecture-SIMULIA-Tech-Brief-10-Installation-Spudcans-Full.pdf)

### Inferences
- Use FEM/CEL only for offline component design (teeth, bucket structural loads; often DEM→FEM load transfer), not for site simulation.

### Gaps
- No GPU-accelerated CEL soil-cutting work found.

---

## Q6. Hybrid real-time methods (AGX Terrain, Holz et al., heightfield + active particle zones)

### Takeaway
The hybrid "heightfield + active-zone particles + aggregate body" approach (Servin, Berglund et al., AGX Terrain) is the most mature *real-time* earthmoving soil model with published accuracy. It reaches **10-25% agreement with full DEM at >1000x speed**, and field sim-to-real of ~10% for wheel-loader bucket filling. It is commercial (Algoryx AGX Dynamics). Open alternatives must be built from the published papers.

### Cited Findings
- **Servin, Berglund, Nystedt (2020/2021)**, "A multiscale model of terrain dynamics for real-time earthmoving simulation" **[PR, Advanced Modeling and Simulation in Engineering Sciences 8:11, 2021; arXiv 2011.00459]**. Resting terrain is a **heightmap**. Inside an **active zone** near the tool, soil is particles that may grow and shrink in size and number. Bucket reaction forces are mediated by an **aggregate body** inheriting the shape, inertia and momentum of active particles. The model couples continuum solid, particles and rigid multibody. **Digging resistance and soil displacement agree with a high-resolution DEM reference within 10-25%, running >3 orders of magnitude faster** — [arXiv](https://arxiv.org/abs/2011.00459v1); [journal](https://link.springer.com/doi/10.1186/s40323-021-00196-3); [Algoryx page](https://www.algoryx.se/papers/terrain/)
- **Holz et al. 2009** (prior art cited by Servin): a heightfield grid for soil at equilibrium plus DEM particles for soil in motion; a 2013 follow-up replaced the heightfield with a 3D level-set volume — [Servin et al. background](https://arxiv.org/pdf/2011.00459); original: Holz, Beer, Kuhlen, "Soil deformation models for real-time simulation: a hybrid approach", VRIPHYS 2009 (TK) — [DOI 10.2312/PE/vriphys/vriphys09/021-030](https://doi.org/10.2312/PE/vriphys/vriphys09/021-030)
- AGX Terrain internals as summarized by TERA: a 3D grid of cells storing mass, **compaction** and soil type. "Shovel" elements have top, bottom and cutting edges, and deformation depends on cutting angle and shear properties. Excavated soil becomes dynamic particles. Parameters are Young's modulus, friction angle and cohesion (e.g., E = 6.5 MPa dirt, 4.6 gravel, 4 sand). It uses an iterative solver for soil and a direct solver for equipment, and runs real time with multiple excavators in Unity. **Limits:** the interaction block uses 2 CPU threads and does not scale with excavator count; performance drops with larger terrain, more particles and large excavation volumes **[PP]** — [TERA arXiv 2501.01430](https://arxiv.org/html/2501.01430v1). TERA real-vs-sim track length 57.4 m vs 50.5 m; path RMSE 1.38 m.
- **Aoshima & Servin 2024**: ~10% sim-to-real gap for wheel-loader bucket filling (forces, mass, work), weakly dependent on fidelity, with models from 1/10,000x to 5x real time — [arXiv 2310.05765](https://arxiv.org/abs/2310.05765)
- **Aoshima, Servin, Wadbro (ISARC 2021)**: wheel-loader simulator with 3D multibody and a hybrid continuum-particle terrain, used for loading optimization — [arXiv 2107.14615](https://export.arxiv.org/abs/2107.14615)
- **Data-driven / learned surrogates (Umeå/Algoryx)**: Servin & Wallin, "Reduced order modeling for realtime simulation with granular materials", PARTICLES 2019 (blade cutting a granular bed). **Wallin & Servin, "Data-driven model order reduction for granular media", Computational Particle Mechanics (2021/2022)**, tested on pile discharge and bulldozing. **Aoshima, Fälldin, Wadbro, Servin, "World modeling for autonomous wheel loaders", Automation 5(3):259-281 (2024)** predicts loaded mass, time, work and resulting pile shape from simulator data — [Algoryx publication list](https://www.algoryx.se/publications/?tgid=3); [arXiv 2309.12016](https://arxiv.org/html/2309.12016v4)
- Rock/boulder excavation RL with progressive-resolution (coarse→fine) simulation: 7x learning-time efficiency vs finest-resolution training **[PP]** — [arXiv 2412.07477](https://arxiv.org/html/2412.07477v2)
- ETH "ExT" (2025): large-scale multi-task pretraining for excavation in GPU-parallel Isaac Gym (not MPM). The RL expert reaches ~98% success with domain randomization over soil types **[PP]** — [arXiv 2509.14992](https://arxiv.org/pdf/2509.14992)

### Inferences
- For "most realistic *feasible* on a single GPU", the evidence favours a **hybrid**: a heightfield (or voxel/level-set) for resting soil, plus a GPU particle (DEM or MPM) active zone near tools, plus FEE or aggregate-body force mediation. Calibrate it against offline DEM or field data. This architecture is validated (10-25% vs DEM; ~10% vs field) and real-time capable.
- The literature now has two converging alternatives: (a) GPU MPM directly (ETH 2026 full-scale transfer); (b) GPU SPH/CRM with active domains (Chrono). Neither has published validated real-time bucket-force accuracy matching the AGX papers.
- The "user asked" item "Wallin & Servin 2024 data-driven" likely corresponds to Wallin & Servin, *Computational Particle Mechanics* 2021/2022 (data-driven MOR). I found no 2024 Wallin & Servin paper.

### Gaps
- No public, peer-reviewed open-source implementation of the Servin multiscale model. AGX is commercial.
- "Berglund & Servin real-time deformable terrain model with particles and heightfield" exact title/year not confirmed beyond the 2021 AMSES paper.

---

## Q7. Game/graphics granular methods (PBD/Flex, PB-MPM)

### Takeaway
Position-based methods (Macklin 2014 Flex; EA PB-MPM 2024) are unconditionally stable and fast on GPU, but stiffness depends on iteration count and parameters are non-physical. They give plausible visuals, not force-accurate soil.

### Cited Findings
- **PB-MPM** (Chris Lewin, EA SEED, SIGGRAPH 2024 Talk): a semi-implicit compliant-constraint MPM, stable at any time step, about as simple as explicit integration. Open source (`electronicarts/pbmpm`) with a WebGPU demo — [EA SEED](https://www.ea.com/seed/news/siggraph2024-pbmpm); [paper PDF](https://media.contentapi.ea.com/content/dam/ea/seed/presentations/seed-siggraph2024-pbmpm-paper.pdf); [SIGGRAPH archive](https://history.siggraph.org/?p=160656)
- PB-MPM inherits PBD drawbacks: non-physical parameters and iteration-dependent stiffness — [CRESSim-MPM, arXiv 2502.18437](https://arxiv.org/pdf/2502.18437)
- **Macklin, Müller, Chentanez, Kim 2014**, "Unified particle physics for real-time applications" (NVIDIA Flex; granular via PBD friction constraints), ACM TOG 33(4) **[PR]** (TK) — [DOI 10.1145/2601097.2601152](https://doi.org/10.1145/2601097.2601152)

### Inferences
- Good for visual "spill" particles in a hybrid scheme; unsuitable as the force source for excavator loads without calibration.

### Gaps
- No validation of PBD/PB-MPM soil forces vs experiments found.

---

## Q8. Moisture, compaction, cohesion, rocks/boulders, multi-layer soils, mud — which methods handle them

### Takeaway
Cohesion and moisture: DEM (JKR, EEPA, liquid bridges) and MPM (Cam-Clay, cohesive Drucker-Prager, viscoplastic mud) are best. Compaction: AGX Terrain (per-cell compaction), MPM (Cam-Clay hardening) and DEM (EEPA). Rocks/boulders: DEM clumps/polyhedra, AGX rigid multibodies, and MPM-DDA or MPM-DEM coupling. Multi-layer soils: easiest in grid-based (heightfield-layer, voxel) and MPM/SPH via per-particle material IDs.

### Cited Findings
- AGX Terrain tracks per-cell compaction and soil type; cohesion and friction angle parameters — [TERA](https://arxiv.org/html/2501.01430v1)
- Servin multiscale: soil as continuum + particles + **rigid multibodies** (rocks) — [arXiv 2011.00459](https://arxiv.org/abs/2011.00459v1); rock loading DEM — Lindmark & Servin 2018, [Algoryx list](https://www.algoryx.se/publications/?tgid=3)
- MPM + DDA for blocky material in excavator bucket simulation — [PARTICLES 2025](https://particles2025.cimne.com/event/contribution/f6b9199b-0d96-11f0-9835-000c29ddfc0c)
- GeoTaichi coupled MPDEM (continuum soil + discrete rocks); "soil-gravel-structure" simulation listed as under development — [PyPI](https://pypi.org/project/geotaichi)
- ETH 2026 MPM conditions on soil "compactness"; it reports qualitative compaction and backfilling experiments — [arXiv 2609.12677](https://arxiv.org/abs/2609.12677)
- DEM ground-deformation and compaction under heavy vehicles — [Wiberg et al. 2021](https://www.algoryx.se/publications/?tgid=3)
- Anura3D supports multiphase (saturated) soil — [UPC thesis](https://upcommons.upc.edu/bitstreams/694ea70e-c7d1-4b72-bf7a-b2d37d7f707f/download)

### Inferences
- Mud and wet cohesive soils are the least-validated area across all methods for excavation. MPM with a rate-dependent (Bingham / Herschel-Bulkley) or critical-state model is the most physically principled route. DEM requires expensive capillary calibration.

### Gaps
- No excavation-force validation in wet clay or mud found for any GPU method.

---

## Q9. Comparative / benchmark studies across methods

### Takeaway
Direct cross-method benchmarks exist but are pairwise: multiscale-hybrid vs DEM (10-25%, >1000x faster), SCM vs DEM (2-3 orders faster) and SCM vs CRM (~10x faster), CEL vs SPH (8.8% vs 13.8% error), DEM resolutions vs field (~10%), CRM vs DEM (RASSOR). There is no single community benchmark for excavator buckets comparable to the single-wheel test in terramechanics.

### Cited Findings
- Multiscale hybrid vs DEM: 10-25%, >10^3x speedup — [Servin et al.](https://arxiv.org/abs/2011.00459v1)
- DEM at multiple fidelities vs field wheel-loader: ~10% gap — [Aoshima & Servin](https://arxiv.org/abs/2310.05765)
- SCM vs DEM: good match, 100-1000x faster — [arXiv 2407.18903](https://arxiv.org/abs/2407.18903)
- SCM vs CRM: good proxy, ~10x faster — [ECCOMAS 2023](https://multibody2023.tecnico.ulisboa.pt/prog_MULTIBODY_WEB/MULTBODY2023_ABSTRACTS/ID_106_121_extendedAbstract.pdf)
- CRM vs DEM (RASSOR digging) and vs 3 physical tests — [arXiv 2507.05643](https://arxiv.org/abs/2507.05643)
- CEL vs SPH vs soil bin: 8.81% vs 13.76% — [PLOS ONE 2025](https://ideas.repec.org/a/plo/pone00/0322861.html)
- MPM and hybrid vs robotic-rig soil cutting — [Haeri et al. ISARC 2020](https://www.iaarc.org/publications/fulltext/ISARC_2020_Paper_60.pdf)

### Inferences — summary decision table (my synthesis from the above)
| Method | Fidelity | Validated accuracy (best found) | Speed on 1 GPU | Real-time? | Best open source |
|---|---|---|---|---|---|
| Bekker/J-H, SCM | Low (no material flow) | Matches DEM drawbar-pull "well" | ≥ real-time (CPU) | Yes | Chrono SCM (BSD-3) |
| FEE/McKyes force + heightfield | Low-med | Quasi-static only | ≫ real-time | Yes | (custom; ETH/UC Davis code) |
| Hybrid heightfield + active particles | Medium | 10-25% vs DEM; ~10% vs field | Real-time (CPU, AGX) | Yes | None open (AGX commercial) |
| GPU MPM (MLS / implicit) | Med-high, continuum | Sparse; ETH full-scale RL transfer 2026 | ~10^5-10^6 particles interactive | Near (coarse) | Taichi, GeoTaichi, Newton (experimental), PB-MPM, Genesis |
| GPU SPH / Chrono::CRM | Med-high | 3 physical tests + DEM (numbers not extracted) | "near interactive" at scale; RTF 30+ reported | No (bucket-scale fine) | Chrono::CRM (BSD-3) |
| GPU DEM | High (granular) | ~10% vs field (bucket filling) | 10^7-10^8 elements, 10^2-10^4x slower than RT | No | Chrono DEME, Chrono::GPU, LIGGGHTS, Yade |
| FEM/CEL | High (local cutting) | 8.8% vs soil bin | Hours/case (CPU) | No | (Abaqus commercial) |
| PBD / PB-MPM | Visual | None | ≫ real-time | Yes | NVIDIA Flex, EA pbmpm |

- Most realistic *feasible* on a single GPU: a GPU-MPM (or SPH-CRM) active domain embedded in a heightfield/voxel site terrain, with rigid-body coupling to the bucket. Calibrate offline against Chrono DEME DEM and/or field force data, and use FEE as a sanity check. This mirrors the validated AGX architecture while using open-source continuum solvers.

### Gaps
- No standard excavator-bucket benchmark dataset spanning methods.
- Komatsu, Caterpillar and Volvo CE internal validation data are unpublished.
- China, Japan and Korea coverage: found Korean MPM (KNU), Japanese MPM-DDA (Kyoto) and Chinese EDEM-Adams papers. Many Chinese-language EDEM bucket papers exist but none with full accuracy metrics were retrieved.
