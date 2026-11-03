# Learned / Neural Surrogate Models for Granular Media, Soil & Terrain Deformation, and Validation Datasets for Excavation / Terramechanics

Research date: 2026-11-02. Preprints are marked [preprint]. Claims come from search snippets and abstract pages; full PDFs were mostly not read, so per-paper numbers should be checked against the paper before they go into a final design doc.

## Q1. ML surrogate families for granular / soil physics (GNS, MeshGraphNets, NeuralMPM, learned terrain/heightmap models, neural operators, PINNs, world models)

### Takeaway
Two families of learned soil models are mature enough to use. The first is particle-based: GNS, NeuralMPM and localized graph dynamics, trained on MPM or DEM data. The second is heightmap or "world model" predictors that map (local terrain heightmap + action) to (new heightmap + load/force). The heightmap models are the better fit for a real-time construction-site sim. Umeå/Algoryx (wheel-loader world models trained on AGX Dynamics) and the 2025-2026 pile-excavation work have already deployed them in planning loops. Neural operators (FNO/DeepONet) and PINNs for soil are mostly geotechnical, quasi-static or flow-runout studies. They are not yet tool-soil interaction models.

### Cited Findings
**Graph Network Simulators (particle-based)**
- Sanchez-Gonzalez et al., "Learning to Simulate Complex Physics with Graph Networks" (ICML 2020). This is the original GNS: an encoder-processor-decoder GNN over particles. DeepMind released 15 datasets, including Sand, Sand-3D, SandRamps, Goop, Water/WaterDrop, MultiMaterial and RandomFloor — [arXiv 2002.09405](https://arxiv.org/abs/2002.09405); [DeepMind repo](https://github.com/google-deepmind/deepmind-research/tree/master/learning_to_simulate)
- The reference GNS was trained at batch size 2 on one V100. The XL and 3D datasets ran out of memory in that setup, so GNS memory cost is a real limit at large particle counts — [DeepMind repo](https://github.com/google-deepmind/deepmind-research/tree/master/learning_to_simulate)
- Kumar & Vantassel, "GNS: A generalizable Graph Neural Network-based simulator for particulate and fluid modeling" (JOSS 2023), is an open-source PyTorch GNS with multi-GPU training. It reports predictions within ~5% error of the associated MPM simulation. A reported ~5,000x speedup (2.5 h MPM vs 20 s GNS) comes from summaries of this paper — [arXiv 2211.10228](https://ar5iv.labs.arxiv.org/html/2211.10228); [JOSS PDF](https://theoj.org/joss-papers/joss.05025/10.21105.joss.05025.pdf)
- Kumar & Choi, differentiable GNS for forward and inverse problems [preprint 2023]. It reports >165x speedup for granular flow prediction vs parallel CPU numerical simulation. Its hybrid GNS/MPM interleaves MPM steps into GNS rollouts to restore conservation laws and gives 24x speedup — [arXiv 2309.13348](https://arxiv.org/html/2309.13348v1)
- Choi, Macedo, Liu (2025) used GNS on multi-layered slope runout and report up to 145x speedup over MPM — [arXiv 2504.15938](https://export.arxiv.org/pdf/2504.15938)
- An alphaXiv overview cites up to 2,867x vs MPM. This figure could not be verified against the primary paper — [alphaXiv](https://alphaxiv.org/overview/2305.05218v2)
- UT Austin (Kumar) Grand Challenge 2024-25 report: a GNS trained on MPM data serves as a fast surrogate. Its differentiable version recovered soil friction angle from video alone with 1.4 degrees of error. This is a calibration route for a site sim — [ICES report](https://www.ices.utexas.edu/media/research/grand-challenge-awardees/final-reports/Grand_Challenge_Final_Reports_24_25_Kumar.pdf)
- MeshGraphNets (Pfaff et al., ICLR 2021) is the mesh-based sibling of GNS, built for Lagrangian/Eulerian mesh simulation (cloth, CFD, structures). It is not specific to granular media — [arXiv 2010.03409](https://arxiv.org/abs/2010.03409)

**NeuralMPM (grid-based neural emulator)**
- Rochman Sharabi, Lewin, Louppe (Univ. Liège), "A Neural Material Point Method for Particle-based Emulation" (TMLR 2025; arXiv 2408.15753). It interpolates particles onto a fixed grid and uses image-to-image networks for grid updates. Against GNS and DMCF it cuts training time from 10 days to 15 hours, uses 10-100x less memory and runs inference 5-10x faster, with comparable or better long-term accuracy. It was evaluated on 6 datasets, including MPM and SPH fluid/solid interaction — [arXiv 2408.15753](https://arxiv.org/abs/2408.15753v3); [TMLR](https://mlanthology.org/tmlr/2025/rochmansharabi2025tmlr-neural)

**Localized / scalable graph dynamics for terrain manipulation**
- Liu, Li, Hauser (UIUC), "Localized Graph-Based Neural Dynamics Models for Terrain Manipulation" [preprint 2025, v2 Sep 2025]. Terrain is a large particle graph, but the model predicts only a small, learned "active" subgraph. A region-of-interest proposer selects the particles likely to move, which lets it handle terrains too large for GPU memory. It is reported as much faster and more accurate than naive graph dynamics, and was tested on excavation and shaping tasks — [arXiv 2503.23270](https://arxiv.org/abs/2503.23270)
- Hauser's group received a $963k NSF grant (2024) for GNN-based terrain prediction and a learning-based excavation simulator — [Illinois news](https://siebelschool.illinois.edu/news/ai-excavation)

**Heightmap / world-model surrogates (excavation and loading)**
- Aoshima, Fälldin, Wadbro, Servin (Umeå/Algoryx), "World Modeling for Autonomous Wheel Loaders" (Automation 5(3):259-281, 2024). Network Φ predicts the new pile heightmap after a loading action. Network Ψ predicts loaded mass, time and work. Training used >10,000 random loading actions in gravel piles simulated with 3D multibody dynamics (AGX Dynamics). Ψ uses a 3.6 m, 36x36 heightmap; Φ needs a larger window because of avalanching. Repeated inference enables long-horizon planning — [arXiv 2309.12016](https://arxiv.org/html/2309.12016v4); [DOAJ](https://doaj.org/article/5e3680c390db491187b0535583f350fc)
- "Optimizing wheel loader performance — an end-to-end approach" [preprint 2025] reuses the world model on a Komatsu wheel loader. It predicts a local heightmap cutout around the dig point and pastes the prediction back into the global heightmap. This local-patch-update pattern maps directly onto a browser terrain heightfield — [arXiv 2501.06583](https://arxiv.org/pdf/2501.06583)
- Aoshima's PhD thesis ("High-Performance Autonomous Wheel Loading: A Computational Approach", Umeå, defended 24 Jan 2025) used AGX Dynamics throughout to generate synthetic deep-learning data — [Algoryx](https://algoryx.se/doctoral-thesis-integrates-real-world-data-and-agx-dynamics-simulations-to-demonstrate-autonomous-wheel-loading/)
- Wallin & Servin, "Data-driven model order reduction for granular media" (2020). It predicts the DEM velocity field from mass distribution and control signals, and is applied in solid and liquid regions while gaseous regions keep resolved DEM. The regressor appears to be linear (sklearn Ridge), not deep learning. Test systems were AGX Dynamics DEM, not specifically excavation — [arXiv 2004.03349](https://arxiv.org/pdf/2004.03349)
- Zhang, Gao, Zhang, Leong, Wu, Wang, "From Prediction to Decision: World-Model-Guided Action Selection for Continuous Pile Excavation" [preprint, arXiv 2609.15382, submitted 14 Sep 2026], about wheel-loader excavation. It proposes candidate scoops (matched diffusion proposals), rejects inadmissible ones, and predicts signed terrain change and loaded volume with a "physics-structured predictor" chosen from five compared architectures. On 32 MinSlope test episodes, world-model ranking cut mean scoop count from 651.8 to 540.6 (-17.1%) with 32/32 completion, vs 29/32 for a SAC baseline. ROS2/TensorRT runs 5 candidates in 72.4 ms on a Jetson AGX Orin. The abstract page lists no affiliations, so the brief's "ETH" attribution is unconfirmed — [arXiv 2609.15382](https://arxiv.org/abs/2609.15382)
- "PileBelief: Persistent Physical State for Interaction-Driven World Modeling" [preprint 2026] is related work on world models with persistent pile state under local sensing. Only the title was seen — [arXiv 2609.22858](https://arxiv.org/pdf/2609.22858)
- ETH "Deep Sandscapes" (2022): an image-to-image GAN predicts sand heightmaps after planned robotic excavation trajectories — [ETH Research Collection](https://www.research-collection.ethz.ch/entities/publication/77d7aa94-12b0-4ae2-a1ad-54aabce01a87)
- SCM_Unet (GitHub, PabloVD): a U-Net on heightmaps replaces Soil Contact Model (SCM) computations in Project Chrono, trained on Chrono-generated data. No paper, date or accuracy numbers were found — [GitHub](https://github.com/PabloVD/SCM_Unet)

**Learned bucket-soil force / dynamics models**
- Koopman-DFL lifting linearization learns bucket-soil dynamics for MPC contouring control of excavators. It relies on a static dataset from a single soil type; varying soils are future work — [arXiv 2107.04314](https://arxiv.org/pdf/2107.04314)
- Azure et al. (2021) compared six ML methods (including an ANN) to predict hydraulic-shovel bucket resistive forces, trained on particle-flow-code (DEM) simulated digging — [Scholars' Mine](https://scholarsmine.mst.edu/min_nuceng_facwork/1632)
- In-situ soil-property estimation with a simulated compact track loader [preprint 2025]: a physics-informed NN trained on simulation data estimates soil properties, with Bayesian mapping — [arXiv 2507.22356](https://arxiv.org/html/2507.22356v1)
- Wheel-loader soil calibration from bucket forces using the fundamental equation of earthmoving (FEE) [preprint 2025]. It notes that purely data-driven approaches such as RBF NNs need large labelled datasets — [arXiv 2506.22579](https://arxiv.org/pdf/2506.22579.pdf)
- A Chalmers 2026 MSc thesis built excavator plant models with LSTMs and TCNs. Models trained only on simulated data struggled with real-world noise and unpredictable soil-tool interaction — [Chalmers ODR](https://odr.chalmers.se/bitstreams/82bf3148-bf47-4cc7-a941-b4f1d34eb996/download)
- Hybrid wheel-soil model: an NN is trained on the residual between a semi-empirical terramechanics model and DEM ground truth, in a single-wheel testbed context — [PARTICLES 2023](https://particles2023.cimne.com/event/contribution/356b30ff-e371-11ed-9a1c-000c29ddfc0c)

**Neural operators / PINNs / ROMs for soil**
- A review of ML for granular materials [preprint 2024] names FNOs and PINNs as remedies for weak extrapolation. It notes PINNs and neural operators have been used mainly for soil consolidation — [arXiv 2410.14767](https://arxiv.org/pdf/2410.14767)
- Secchi, Pasculli, Sciarra (Land 15:759): a 3D FNO trained on synthetic shallow-flow solver data acts as a site-specific debris-flow surrogate over real terrain, with mean R² ~0.94-0.97 for thickness and velocity — [MDPI Land](https://mdpi-res.com/d_attachment/land/land-15-00759/article_deploy/land-15-00759.pdf)
- SIMAI 2025: a kernel operator-learning surrogate maps density, friction angle and slope to granular flume flow outputs, as an alternative to FNO and DeepONet — [SIMAI 2025](https://simai2025.cimne.com/event/contribution/504dc8ff-349e-11f0-a5f6-000c29ddfc0c)
- EGU25-13428 (abstract): early FNO/PINO work for compaction-driven flow and tunnelling stress/deformation; still at the dataset-generation stage — [EGU25](https://meetingorganizer.copernicus.org/EGU25/EGU25-13428.html)
- Dundee ROM framework: autoencoder/PCA plus DeepONet in latent space for geotechnical problems — [Dundee](https://discovery.dundee.ac.uk/en/publications/a-reduced-order-model-framework-suitable-for-geotechnical-problem/)
- LSTM/autoencoder nonlinear ROM for soil-structure interaction — [arXiv 2203.01842](https://arxiv.org/pdf/2203.01842)
- Neural-operator surrogate of a granular micromechanics failure envelope [preprint 2026] — [arXiv 2604.19027](https://arxiv.org/pdf/2604.19027)

### Inferences
- For a browser construction-site sim, the most deployable pattern is the Umeå/Komatsu local-heightmap world model. A small CNN takes a fixed-size heightmap patch plus action parameters and outputs a delta-height patch plus scalar load or force, and the prediction is pasted back into the global heightfield. It has fixed tensor shapes, a small input (e.g. 36x36 to ~64x64) and plain conv ops, all friendly to ONNX/WebGPU. Training data can come from AGX, Chrono SCM, Newton/Isaac MPM or a GNS.
- Particle GNS/NeuralMPM is better suited to offline data generation, calibration (differentiable GNS for inverse friction angle) and close-up "hero" shots of bucket-soil flow than to whole-site real-time simulation. NeuralMPM's grid formulation (image-to-image CNN) maps to WebGPU much more naturally than GNS's dynamic radius-graph construction.
- L-GBND's "simulate only the active region" idea is the key scalability trick for any particle-based learned model on a large site.
- I found no published neural operator or PINN that models tool-soil interaction for excavation. That remains a research gap, not an off-the-shelf option.

### Gaps
- No Cosmos (NVIDIA) or other video/diffusion world-model application to terrain or excavation physics was found in searches. The only diffusion use found was as an action proposer in arXiv 2609.15382.
- Affiliations and architecture details of arXiv 2609.15382 were not available from the abstract page. Its "physics-structured predictor" design and data source (simulated vs real) are unknown.
- No accuracy or frame-rate numbers were found for L-GBND or SCM_Unet.
- Aoshima world-model prediction errors (e.g., mass and heightmap RMSE) were not extracted.

## Q2. Accuracy vs ground truth, speedups, generalization limits, GPU inference cost, browser inference feasibility

### Takeaway
Reported speedups of learned granular surrogates over CPU/GPU MPM range from ~24x (hybrid GNS+MPM, conservation-preserving) to ~145-165x (GNS) and up to ~5,000x (secondary-source figure), with ~5% error vs MPM in reported cases. Speedup figures use different baselines and are not comparable. The main weaknesses are rollout drift, loss of conservation, and degradation outside the training distribution (new soils, geometries, real noise). onnxruntime-web's WebGPU EP exists but is still flagged experimental. I found no published GNN or physics surrogate running in a browser, so browser deployment is unproven but plausible for CNN/heightmap models.

### Cited Findings
- GNS within ~5% of MPM; ~2.5 h MPM vs ~20 s GNS (≈5,000x; secondary summary) — [arXiv 2211.10228](https://ar5iv.labs.arxiv.org/html/2211.10228)
- GNS >165x vs parallel CPU MPM. Hybrid GNS/MPM gives 24x while enforcing conservation by interleaving MPM steps — [arXiv 2309.13348](https://arxiv.org/html/2309.13348v1)
- GNS up to 145x vs MPM for multi-layer slope runout — [arXiv 2504.15938](https://export.arxiv.org/pdf/2504.15938)
- NeuralMPM: 5-10x faster inference and 10-100x less memory than GNS/DMCF, and training in 15 h vs 10 days — [arXiv 2408.15753](https://arxiv.org/abs/2408.15753v3)
- GNS memory: DeepMind's reference implementation runs out of memory on a V100 for the XL and 3D datasets — [DeepMind repo](https://github.com/google-deepmind/deepmind-research/tree/master/learning_to_simulate)
- Edge inference example: the pile-excavation world model evaluates 5 action candidates in 72.4 ms on a Jetson AGX Orin (TensorRT) — [arXiv 2609.15382](https://arxiv.org/abs/2609.15382)
- Generalization limit, soil type: the Koopman bucket-soil model was trained on a single soil type — [arXiv 2107.04314](https://arxiv.org/pdf/2107.04314)
- Generalization limit, sim-to-real: LSTM/TCN plant models trained on simulation struggled with real noise and soil-tool unpredictability — [Chalmers thesis](https://odr.chalmers.se/bitstreams/82bf3148-bf47-4cc7-a941-b4f1d34eb996/download)
- Generalization limit, extrapolation: the granular-ML review identifies weak extrapolation as the key issue and proposes FNO/PINN as partial remedies — [arXiv 2410.14767](https://arxiv.org/pdf/2410.14767)
- onnxruntime-web WebGPU EP: imported from `onnxruntime-web/webgpu` and flagged experimental. It needs Chromium ≥113 (Windows), and fp16 needs Chrome ≥121 / Edge ≥122. It supports `Tensor.fromGpuBuffer` (GPU-resident I/O, avoiding CPU readback) and graph capture for static-shape models running fully on WebGPU. WebGL EP is in maintenance mode — [ORT WebGPU EP docs](https://onnxruntime.ai/docs/execution-providers/WebGPU-ExecutionProvider.html); [npm onnxruntime-web](https://npmjs.com/package/onnxruntime-web)

### Inferences
- Graph capture and GPU-buffer I/O in ORT-Web fit a per-frame surrogate loop well, if the model has static shapes. That favours fixed-size heightmap-patch CNNs or NeuralMPM-style grid CNNs over GNS, whose neighbor graph changes size every step. Scatter/gather/segment-sum ops needed by GNNs may be unsupported or slow on the WebGPU EP; the operator list should be checked.
- A rough budget extrapolated from the 72.4 ms / 5 candidates on Orin: a single small patch-CNN step should be a few to tens of ms on a desktop-class browser GPU. This is an inference, not a measured figure.
- Hybrid schemes (learned steps interleaved with occasional physics corrections, as in GNS+MPM) are a robust way to limit drift in long interactive sessions. A browser equivalent would be a learned heightmap update followed by a cheap analytic angle-of-repose relaxation pass.

### Gaps
- No benchmark of any GNN or physics surrogate on onnxruntime-web WebGPU or TensorFlow.js was found.
- No ORT-Web WebGPU operator-coverage check for GNN ops (ScatterND, ScatterElements, GatherND) was performed.
- No apples-to-apples accuracy table across GNS, NeuralMPM and heightmap models on the same soil task was found.

## Q3. Datasets and benchmarks for calibrating / validating a construction-site soil simulation

### Takeaway
No standard public benchmark covers excavation forces. Usable data is fragmented:
- **Simulation particle datasets:** DeepMind Sand/SandRamps/Sand-3D/Goop, and the Kumar GNS/MPM datasets.
- **Real excavator logs:** new Hugging Face sets from 2025-2026 (Flywheel 100 h Volvo EC380, fuxi-robot excavator-motion, WIKA CAN-bus MCAP), plus an older IEEE DataPort set with bucket pressure/pose and point clouds.
- **Single-wheel terramechanics:** Tohoku LiDAR rut dataset, Pavlov & Johnson open slip data, the DROVE mobility database.
- **Soil-parameter compilations:** Italian lithotype dataset, Mendeley shear-strength sets, a lunar regolith DB.
- **Construction-site perception:** ConSLAM, GOOSE-Ex.

### Cited Findings
**Simulation datasets (granular particle trajectories)**
- DeepMind "Learning to Simulate": Sand, SandRamps, Sand-3D, Goop, Goop-3D, Water, WaterDrop, WaterDrop-XL, WaterRamps, MultiMaterial, RandomFloor, FluidShake, FluidShakeBox, Continuous, Water-3D. TFRecords are on GCS at `storage.googleapis.com/learning-to-simulate-complex-physics/Datasets/{NAME}/{train|valid|test}.tfrecord` plus `metadata.json` (with normalization stats and connectivity radius) — [DeepMind repo](https://github.com/google-deepmind/deepmind-research/tree/master/learning_to_simulate)
- Kumar & Vantassel GNS (PyTorch) ships with MPM-generated granular flow data (from the CB-Geo MPM ecosystem). The repo is the access point — [JOSS PDF](https://theoj.org/joss-papers/joss.05025/10.21105.joss.05025.pdf)
- NeuralMPM was evaluated on 6 datasets (MPM and SPH, fluid and fluid-solid) — [arXiv 2408.15753](https://arxiv.org/abs/2408.15753v3)
- Newton (NVIDIA Warp-based) implicit MPM solver, exposed in Isaac Lab via `MPMSolverCfg` in `NewtonCfg`. It handles sand, snow and granular media with colliders treated inside the implicit grid solve, and ships an example `isaaclab example mpm-granular`. Rigid-MPM coupling is experimental. It is a generator for new training data, not a released dataset — [Isaac Lab implicit MPM docs](https://isaac-sim.github.io/IsaacLab/main/source/concepts/using_mpm.html); [NVIDIA docs](https://docs.nvidia.com/isaaclab/concepts/implicit-mpm)

**Real excavator / earthmoving logs**
- Flywheel (YC S25) with Frodobots.ai: an announced open-source ~100 h excavator dataset from real construction sites. It covers a Volvo EC380 with 4 cameras and operator expert actions for imitation learning of digging and dumping. The source is a Hacker News launch post, and the release date and contents were not verified — [HN](https://hn.svelte.dev/item/45362914)
- `hanfeng2006/excavator-motion` (Hugging Face, fuxi-robot): dig-to-truck-load trajectories for 3 excavator models with RGB, elevation maps and joint angles. Samples are dated Apr 2025 — [HF](https://huggingface.co/datasets/hanfeng2006/excavator-motion)
- `aiwika/Excavator-Embodied-Intelligence-Dataset` (HF, ~2026): MCAP recordings by job type (e.g., site levelling) from 4 CAN buses, labelled Apache-2.0. The DBC/EDS decode files were not published, so CAN signals may be undecodable — [HF](https://huggingface.co/datasets/aiwika/Excavator-Embodied-Intelligence-Dataset); [HF discussion](https://huggingface.co/datasets/aiwika/Excavator-Embodied-Intelligence-Dataset/discussions/5)
- IEEE DataPort, "Training Data & Original Data for Autonomous Excavation" (Huh & Hong, 2022): CSVs of bucket-tip pressure, position and angle from real excavation, plus a point cloud captured before each dig. This is directly useful for force and volume validation — [IEEE DataPort](https://ieee-dataport.org/documents/training-data-original-data-autonomous-excavation)
- IEEE DataPort, deep-learning bucket trajectory planning dataset — [IEEE DataPort](https://ieee-dataport.org/documents/deep-learning-based-autonomous-excavation-bucket-trajectory-planning-algorithm)

**Single-wheel / terramechanics**
- Tohoku Univ. Space Robotics Lab (Aerospace 12(1):71, 2025): 3D LiDAR scans of grouser-wheel ruts in Toyoura sand from a single-wheel testbed, released to the community — [MDPI Aerospace](https://www.mdpi.com/2226-4310/12/1/71); [Preprints.org](https://www.preprints.org/manuscript/202411.0728)
- Pavlov & Johnson: a terramechanics model for high slip angle and skid. They describe their data as the only open dataset covering both high slip angles and slip ratios — [ISTVS](https://istvs.org/publication-news/a-terramechanics-model-for-high-slip-angle-and-skid-with-prediction-of-wheel-soil-interaction-geometry)
- DROVE (Database Records for Off-road Vehicle Environments) 2.0 has 8,179 records including fine-grained soil tests. A 3.0 release was announced as pending — [DTIC](https://apps.dtic.mil/sti/pdfs/ADA566231.pdf)
- DLR TROLL single-wheel testbed data is available only on request — [arXiv 2404.11965](https://arxiv.org/pdf/2404.11965); [DLR TROLL](https://www.dlr.de/en/rm/about-us/institute/infrastructure/troll)

**Soil property databases (cohesion, friction angle, density)**
- Italian lithotype geotechnical dataset (Scientific Data 2024): >2,300 records of effective cohesion, friction angle and porosity from international literature. Bulk density is not a focus — [PMC](https://www.ncbi.nlm.nih.gov/pmc/articles/PMC11655648/)
- Mendeley (Central South Univ.): cohesion, friction angle and plastic limit across test methods — [Mendeley](https://data.mendeley.com/datasets/y94jyms83b)
- Mendeley JS/SCPTU/372: soft-soil CPTU parameters — [Mendeley](https://data.mendeley.com/datasets/s7j4935fwm/1)
- Anderson & Schjetne (2013): friction-angle database from >500 triaxial tests on sands, silts and clays. It is a paper, not a downloadable dataset — [ISSMGE](https://xyzhtc.issmge.org/uploads/contributions/Anderson_Schjetne2013.pdf)
- Open lunar regolith database (friction angle, cohesion, bulk density, bearing capacity, with test type), code and data on GitHub [preprint 2026] — [arXiv 2602.03829](https://arxiv.org/abs/2602.03829v1)

**Construction-site perception / point clouds**
- ConSLAM (ECCV-W 2022): 4 periodically collected handheld sequences of a construction site with LiDAR, 9-axis IMU, RGB and NIR. The GitHub repo is mac137/ConSLAM, and the sequence-1 bag is flagged faulty — [ConSLAM](https://mlanthology.org/eccvw/2022/trzeciak2022eccvw-conslam); [HAL](https://hal.univ-cotedazur.fr/hal-04039392v1)
- SLAM2REF built a ~0.5 cm-accurate BIM model from ConSLAM's TLS scan of sequence 2 — [arXiv 2408.15948](https://arxiv.org/pdf/2408.15948)
- GOOSE-Ex (2024): an excavator-mounted dataset for semantic/panoptic segmentation in off-road and excavation scenes, usable as an odometry/SLAM test set with GNSS ground truth — [arXiv 2409.18788](https://arxiv.org/pdf/2409.18788)

### Inferences
- A practical validation stack for a site soil sim has three tiers. (1) Unit-level: compare tool force against FEE/analytic predictions and the Huh & Hong bucket pressure logs. (2) Deformation-level: compare against heightmap before/after pairs (Huh & Hong point clouds, Tohoku rut LiDAR, generated AGX/Newton/Chrono data). (3) Task-level: compare cycle times and fill volumes against real operation logs (Flywheel, fuxi-robot, WIKA).
- Soil-parameter compilations suit domain-randomization ranges (friction angle, cohesion) more than per-site calibration.

### Gaps
- No public datasets were found from Built Robotics, Komatsu Smart Construction, Caterpillar, or ETH "HEAP" machine logs. These appear proprietary; nothing citable was found.
- "MOCS" (Moving Objects in Construction Sites) was not surfaced by search, so its details are unverified.
- No public drone-photogrammetry earthworks volume-change dataset was found in this pass.
- No dataset of excavation force measured together with soil property tests (lab-scale bucket testbed with known c and φ) was found openly released.

## Q4. Sim-to-real studies for autonomous excavation and the soil model each used

### Takeaway
Most RL excavation work that reached real machines uses cheap analytic soil models. The typical choice is the Fundamental Equation of Earthmoving (FEE) in Isaac Gym/Isaac Lab with heavy soil-parameter randomization (ETH Egli/Hutter lineage, boulder excavation). Higher-fidelity alternatives are AGX Dynamics (TERA, Umeå wheel loaders) and particle simulation (progressive-resolution distillation, obstacle removal). Learned soil surrogates appear mainly as planning world models, not as RL training environments.

### Cited Findings
- Egli, Gaschen, Kerscher, Jud, Hutter, "Soil-Adaptive Excavation Using Reinforcement Learning" (RA-L 2022, DOI 10.1109/lra.2022.3189834). It is trained fully in simulation with a simple analytical FEE-based soil model, extensive randomization of soil parameters, online adaptation without explicit soil knowledge, terrain elevation input and a max-depth constraint. Its deployment status is reported inconsistently. One snippet says it was tested on a 12-ton excavator — [ETH Research Collection](https://www.research-collection.ethz.ch/entities/publication/6e3c879a-dc54-4600-aacb-29997e768bef). A later paper's snippet says the soil-adaptive controller was deployed in simulation only — [search result citing arXiv 2507.22356](https://arxiv.org/pdf/2507.22356v1). Verify against the RA-L paper.
- Egli PhD thesis "Learning-Based Excavator Automation" (ETH 2024): RL for hydraulic excavators with learned (NN) machine dynamics and randomized soil parameters, reporting direct transfer to a physical excavator for position tracking — [ETH Research Collection](https://research-collection.ethz.ch/handle/20.500.11850/679546)
- Zhai, Terenzi, Frey, Garcia Soto, Egli, Hutter, "ExT: Towards Scalable Autonomous Excavation via Large-Scale Multi-Task Pretraining and Fine-Tuning" [preprint, Sep 2025, ETH RSL]. Full excavation cycles with centimeter-level accuracy in sim and on the real machine, with performance comparable to single-task controllers. The simulator and soil model were not stated in the abstract — [arXiv 2509.14992](https://arxiv.org/abs/2509.14992)
- "Towards Learning Boulder Excavation with Hydraulic Excavators" [preprint 2025]. The policy is trained in NVIDIA Isaac Lab, with rigid-body dynamics in Isaac and bucket-soil forces from a quasi-static 2D FEE-based model. Soil cohesion, friction angle and adhesion are randomized — [arXiv 2509.17683](https://arxiv.org/pdf/2509.17683)
- "Towards Autonomous Excavation Planning" (ETH, 2023) — [arXiv 2308.11478](https://arxiv.org/pdf/2308.11478)
- TERA: "A Simulation Environment for Terrain Excavation Robot Autonomy" [preprint, Jan 2025]. It uses the Algoryx AGX terrain plugin, described as a computationally efficient model with a fast iterative solver, with ROS and Unity Python API integration. The authors argue particle-based simulators are too heavy for multi-robot RL. No Isaac Lab integration was found — [arXiv 2501.01430](https://arxiv.org/pdf/2501.01430)
- Korean work: "Adaptive Excavation Automation in Complex Soil Environments Using Reinforcement Learning" (Korea Univ.; IEEE RAS news Sep 2025). FEE combined with PPO for trajectory planning, with soil properties assumed static within a cycle. It reports 18.1% lower total excavation resistance than baselines and 0.065 m MAE, in simulation only; real-world soil estimation is future work — [IEEE RAS](https://www.ieee-ras.org/news/adaptive-excavation-automation-in-complex-soil-environments-using-reinforcement-learning/); [Korea Univ.](https://pure.korea.ac.kr/en/publications/adaptive-excavation-automation-in-complex-soil-environments-using/)
- Progressive-Resolution Policy Distillation [preprint, Dec 2024, v4]: policies are trained in coarse, fast particle simulation and distilled to finer resolution. It reports a 7x reduction in learning time and was validated in a rock-excavation simulator and 9 real rock environments, but only on simple excavation robots, not full excavators — [arXiv 2412.07477](https://arxiv.org/html/2412.07477v4)
- "Autonomous Obstacle Removal for Excavators through Policy Learning with Particle Simulation" [preprint 2026]: particle-sim RL deployed on a real excavator via a shared observation-action interface — [arXiv 2606.09183](https://arxiv.org/pdf/2606.09183)
- "Learning to Capture Rocks using an Excavator" [preprint 2025]. It notes many prior approaches ignore soil reaction forces, which leads to stalls, and that sim-to-real transfer for conventional construction machinery remains rare — [arXiv 2510.04168](https://arxiv.org/pdf/2510.04168)
- Umeå/Algoryx wheel loaders: AGX Dynamics multibody and terrain simulation to train world models, which were then used on a real Komatsu wheel loader — [arXiv 2309.12016](https://arxiv.org/html/2309.12016v4); [arXiv 2501.06583](https://arxiv.org/pdf/2501.06583)
- Loughborough PhD thesis "Reinforcement learning for trench excavation" — [Loughborough repo](https://repository.lboro.ac.uk/articles/thesis/Reinforcement_learning_for_trench_excavation/22714543)

### Inferences
- The field has converged on three tiers of soil model: (a) FEE/analytic with domain randomization for RL at scale, (b) AGX-style hybrid terrain or MPM/particle for fidelity and data generation, and (c) learned heightmap world models distilled from (b) for planning. A browser site sim can follow the same split: analytic FEE forces plus a learned heightmap-delta model, with occasional analytic slope relaxation.
- Newton/Isaac Lab implicit MPM (experimental) is a plausible new GPU data generator for training such surrogates, alongside AGX and Chrono.

### Gaps
- No Japanese or Chinese excavation RL sim-to-real papers with a specified soil model were surfaced in this pass, beyond the Chinese-origin datasets (fuxi-robot). Coverage of those regions is incomplete.
- ExT's simulator and soil model were not confirmed from the abstract.
- The Egli 2022 real-machine deployment claim conflicts across sources (see above).
