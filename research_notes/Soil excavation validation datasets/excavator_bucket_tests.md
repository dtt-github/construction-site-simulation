# Excavator bucket digging measurements for validating a real-time excavator + MPM soil simulator (ZX200-class, 0.8 m3 bucket)

Scope note: about 18 search/fetch calls. Several key full texts (Springer, Nature/PMC, Mechanics & Industry, ISARC PDF) were paywalled, CAPTCHA-blocked or too large to fetch, so some numbers below come from search-result snippets. Where that is the case it is flagged. Classic references I could not verify in this session are listed under Gaps as leads only.

## 1. Instrumented full-size excavator / earthmoving digging tests

### Takeaway
I found no public raw dataset of a full-size excavator digging with cylinder pressures plus a characterised soil. The best full-scale validation package with soil data and a published sim-vs-real error budget is for a wheel loader: Aoshima & Servin's Komatsu WA320-7 tests in AGX. The Chinese full-scale excavator studies (36.5 t; two 20 t machines) do measure cylinder pressures and back-calculate digging resistance, but their data is proprietary or "on request".

### Cited Findings
- **Komatsu WA320-7 wheel loader (Aoshima & Servin, Multibody Syst. Dyn. 2024, arXiv 2310.05765).**
  - Machine: 15.175 t, flat-bottom 3.0 m3 bucket, 2.685 m wide.
  - Material: crushed sedimentary gravel, 30–40 mm with fines and moisture, bulk density 1727 kg/m3, angle of repose 32°.
  - Measured at 100 Hz: lift and tilt cylinder forces (from pressures), tractive force, drive speed, cylinder extensions, boom and bucket angles, chassis pitch, and vehicle and bucket position.
  - Also measured: pre-dig pile laser scan, loaded mass and net work.
  - Only 3 loading operations (FB35, HD27, RD21).
  - Simulators compared: AGX DEM at several resolutions, and an AGX multiscale model (friction 32°, dilatancy 8°, E = 4.6 MPa, bucket–soil friction 0.2).
  - Overall sim-to-real error was about 10% (std 3%) and depended only weakly on fidelity. Lift and tilt force error was about 11%, tractive force error 8–19%.
  - Loaded mass: DEM came in 15% low and multiscale 12% high. Work error was 17% (DEM) and 21% (multiscale).
  - Supplementary material is at umit.cs.umu.se/wl-sim-to-real/. Its contents and licence were not verified.
  - [arXiv HTML](https://arxiv.org/html/2310.05765v3); [Springer](https://link.springer.com/article/10.1007/s11044-024-10005-5)
- **36.5 t backhoe excavator, Huzhou, Zhejiang, China (PMC9519879, "matching characteristics of the theoretical digging force").**
  - Soil: "tertiary soil mixed with stones". No numeric soil parameters are given.
  - Sensors: 3 angular displacement sensors and 6 pressure sensors on the rod and rodless chambers of the boom, stick and bucket cylinders.
  - Two working conditions: piling onto the stopping surface, and free continuous trenching.
  - Resistance is reported only as 40 kN histogram bins. It stays below 80 kN during initial cutting and full-load lift, and exceeds 200 kN near maximum depth.
  - The limit digging force exceeded measured resistance in 84.40% of samples.
  - Bucket capacity is not reported. Data is "not publicly available" (company prototype) but can be requested from the corresponding author.
  - [PMC](https://pmc.ncbi.nlm.nih.gov/articles/PMC9519879/)
- **Two 20 t backhoe excavators (Mechanics & Industry 2024, mi230039, "Comparative analysis study of resistance characteristics").**
  - This is the closest class match to a ZX200.
  - Soil internal friction angle was measured by triaxial tests.
  - Boom, arm and bucket cylinder pressures and joint angles were sampled synchronously at a test site (Liuzhou per snippet) to back-calculate real-time excavation resistance.
  - The resistance moment rises, then plateaus once the bucket is full.
  - The model is reported within 12% error against excavation tests (snippet).
  - Full text returned 403, so bucket size, cohesion and data availability are unverified. The journal is open access.
  - [M&I full html](https://www.mechanics-industry.org/articles/meca/full_html/2024/01/mi230039/mi230039.html); [PDF](https://www.mechanics-industry.org/articles/meca/pdf/2024/01/mi230039.pdf)
- **Electromechanical-hydraulic co-simulation of an excavator working device (Sci. Rep. 2025, s41598-025-29412-5).**
  - Field test with pressure sensors on both chambers of the boom, arm and bucket cylinders, plus relative joint angles, at three digging depths.
  - Simulated pressures were within about 5% of working pressure (search snippet).
  - Machine model, bucket size and data statement could not be fetched (redirect/CAPTCHA).
  - [Nature](https://www.nature.com/articles/s41598-025-29412-5); [PMC mirror](https://pmc.ncbi.nlm.nih.gov/articles/PMC12749923/)
- **ETH Zurich HEAP (Menzi Muck M545, 12 t walking excavator).**
  - Cylinder forces come from pressure sensors integrated in the servo-valve modules.
  - Field tests on varied rocks (0.4–0.7 m) and soils: 70% autonomous success vs 83% for human operators.
  - A related 2D force/payload estimator on a 25 t excavator reports 13° direction accuracy, 383 N magnitude accuracy and about 1% payload accuracy. Snippet; machine attribution unclear.
  - [arXiv 2106.05059](https://arxiv.org/pdf/2106.05059); [ETH Research Collection](https://www.research-collection.ethz.ch/server/api/core/bitstreams/62e1de57-8939-4701-8672-ec2bb55e1c5d/content)
- **ETH "Soil-Adaptive Excavation Using Reinforcement Learning" (M545).**
  - Adapts digging-force references online to the soil.
  - Found the most efficient path in very soft soil is deep and short.
  - [ETH Research Collection](https://www.research-collection.ethz.ch/server/api/core/bitstreams/95ef5691-11e8-4a86-b02d-6f0e2501de9b/content)
- **Oulu University "smart excavator" (Adaptive Utilization of Pressure Sensor Data in Excavation).**
  - Bucket cylinder pressure correlated with static load from penetration tests and soil sampling.
  - Pressure can saturate at the relief valve, and site-specific calibration is needed.
  - [Scienpress PDF](https://www.scienpress.com/Upload/GEO/Vol%2014_1_1.pdf)
- **Large-sample real-world study of 260 excavators (Energy 2026).**
  - Records pilot secondary pressure, pump outlet pressure and engine status.
  - No joint angles or public data were evident. Useful for duty-cycle statistics, not soil forces.
  - [ScienceDirect](https://www.sciencedirect.com/science/article/abs/pii/S0360544226010303)
- **Volvo/Umeå wheel loader force estimation (Abdolmohammadi et al., IEEE Access 2025, arXiv 2506.22579).**
  - Calibrates soil-wedge parameters from the previous bucket cycle.
  - Validated only in Algoryx simulations, with 10–15% RMSE. No field data.
  - [arXiv](https://arxiv.org/pdf/2506.22579)
- **NASA "Field Excavator with Embedded Force Measurement".**
  - Strain gauges on the arm links, an alternative to pressure-based force estimation.
  - [Tech Briefs](https://www.techbriefs.com/component/content/article/22532-lew19237-1)

### Inferences
- For a ZX200-class simulator, the most usable full-scale targets are:
  - Envelope values: resistance below 80 kN in the cutting phase and above 200 kN near maximum depth on a 36.5 t machine. Expect roughly proportionally lower values on a 20 t machine.
  - The shape of resistance moment vs time: a rise, then a plateau once the bucket is full.
  - The Aoshima/Servin error budget (about 10% force, ±12–15% payload) as a realistic "good" sim-to-real benchmark for real-time terrain models.
- Acquiring raw data likely means emailing the Chinese authors, or running your own instrumented ZX200 tests. Pressure transducers on 6 cylinder chambers, joint encoders or IMUs, and a pre/post-dig terrain scan are the de facto standard sensor set.

### Gaps
- No verified instrumented-test publication from Hitachi, Komatsu (excavator), Caterpillar, Volvo excavators, Doosan, Built Robotics, KAIST, Purdue, Tampere, Luleå or Japanese PWRI surfaced in my searches. They may exist (e.g. PWRI unmanned-construction work, KAIST/Doosan autonomous excavator papers) but were not found or confirmed.
- The University of Iowa / Caterpillar "virtual excavator" validation data was not found.
- Exact bucket volume, soil cohesion and density for the two 20 t Chinese machines were not retrievable (paywall/403).

## 2. Lab-scale bucket / soil-bin tests with force time series

### Takeaway
The canonical lab bucket-filling validation is still Coetzee & Els (J. Terramech. 2006 and 2009). Those tests used a 2D-ish bucket dragged through corn grains, with measured drag force and fill. The best modern open alternatives are the NASA bucket soil-bin tests and the ISARC 2020 sandbox MPM comparison. Small-scale excavators estimating force from pump pressure give extra force magnitudes in sand.

### Cited Findings
- **Coetzee, Basson & Vermeer, "Discrete and continuum modelling of excavator bucket filling" (J. Terramech. 2006/2007).**
  - Measured bucket drag force in cohesionless corn grains, compared against DEM, polar MPM and nonpolar MPM.
  - Nonpolar MPM was the most accurate on force: it matched measurement up to 600 mm displacement, then under-predicted, reaching 12% error at 800 mm.
  - DEM and polar MPM under-predicted force. DEM captured the force trend but not the material flow, which the continuum methods predicted better.
  - [ScienceDirect](https://www.sciencedirect.com/science/article/abs/pii/S0022489806000565); [ResearchGate](https://www.researchgate.net/publication/222674515_Discrete_and_continuum_modelling_of_excavator_bucket_filling)
- **Coetzee & Els, "The numerical modelling of excavator bucket filling using DEM" (J. Terramech. 46 (2009) 217–227).**
  - DEM parameters calibrated, then checked on silo discharge and bucket filling.
  - Drag force error was 20% at end of fill. Bucket volume was predicted within ±6%. DEM under-predicted drag but captured the trend.
  - A snippet indicates measured maximum drag of about 250 N, with DEM 15–50 N low. Fragmentary; verify in the paper.
  - [ScienceDirect](https://www.sciencedirect.com/science/article/abs/pii/S0022489809000809); [ResearchGate](https://www.researchgate.net/publication/256801904_The_numerical_modelling_of_excavator_bucket_filling_using_DEM)
- **Maciejewski et al., "Study on the efficiency of the digging process using the model of excavator bucket" (J. Terramech. 2003).**
  - Scaled bucket model in a soil bin, studying digging efficiency vs trajectory.
  - Only the abstract page was reached; details unverified.
  - [ScienceDirect](https://www.sciencedirect.com/science/article/abs/pii/S0022489803000600)
- **Obermayr et al. (2011).**
  - DEM draft force on a straight blade in cohesionless material: steel balls or round washed gravel, small-scale lab tests with measured horizontal force.
  - Requires calibration of contact parameters. Blade, not bucket.
  - [Univ. Manitoba thesis review](https://mspace.lib.umanitoba.ca/server/api/core/bitstreams/6fd41ae9-095e-4ee4-99ef-cb637567276a/content); [arXiv 1903.04821 review](https://arxiv.org/pdf/1903.04821)
- **NASA, "Force Measurements to Excavate Lightly Compacted ..." (NTRS 20200003063).**
  - Bucket in a soil bin, with a force sensor carried on rails over the bin.
  - Linear trajectories were designed specifically to produce excavation-model validation data.
  - Horizontal force rises with displacement while vertical force falls as the bucket fills.
  - US government work, likely public domain.
  - [NTRS PDF](https://ntrs.nasa.gov/api/citations/20200003063/downloads/20200003063.pdf)
- **ISARC 2020 Paper 60, "Efficient Numerical Methods for Accurate Modeling of Soil ...".**
  - Excavator experiment in a sandbox, comparing measured forward, vertical and torque forces with MPM and a hybrid method.
  - Reported as "good agreement", with simulated forces somewhat more oscillatory. Snippet; full PDF exceeded the fetch size.
  - [IAARC PDF](https://www.iaarc.org/publications/fulltext/ISARC_2020_Paper_60.pdf)
- **Laboratory-scale excavator, about 4 L bucket ("Analysis of Digging Forces on Laboratory Scale Excavator").**
  - Digging force from pump pressure, calibrated with a load cell.
  - In loose sand: 1.13 kN at a 60° attack angle, 1.53 kN at 50° and 1.60 kN at 40°.
  - [ResearchGate](https://www.researchgate.net/publication/366029615_Analysis_of_Digging_Forces_on_Laboratory_Scale_Excavator)
- **Lunar/regolith excavation literature.**
  - MPM with nonlocal granular fluidity, validated on a blade excavation in GRC-1 simulant (velocity field and forces). [arXiv 2111.01523](https://arxiv.org/pdf/2111.01523)
  - Bucket-wheel scale effect: excavation force scales approximately with the cube of excavation dimensions. [ResearchGate](https://www.researchgate.net/publication/271892582_Effect_of_bucket-wheel_scale_on_excavation_forces_and_soil_motion)
- **Bench-scale tool cutting with online Mohr-Coulomb parameter fitting** ("Method and apparatus for on-line estimation of soil parameters during excavation", J. Terramech. 2012). [ScienceDirect](https://www.sciencedirect.com/science/article/abs/pii/S0022489812000262)
- **Blouin, Hemami & Lipsett review of resistive force models (2001).**
  - Found that cutting and excavation models lack a common validation basis, and proposed a normalised experimental comparison.
  - [ISC listing](https://search.isc.ac/Inventory/10/9840.htm); cited in [arXiv 2506.22579](https://arxiv.org/html/2506.22579v1)

### Inferences
- The Coetzee corn-grain bucket test is the de facto benchmark for MPM-vs-DEM bucket filling. It is cohesionless, 2D-ish and well documented, so it is a sensible first MPM regression test before any full-scale comparison.
- The "force scales with the cube of dimension" result supports using scaled tests with appropriate scaling laws. Still, a 0.8 m3 bucket validation should not rely on scaling alone.

### Gaps
- I could not verify in this session the classic resistance formulas (Zelenin, Balovnev) or the force-measurement studies of Tafazoli/Salcudean/Lawrence (UBC, instrumented mini-excavator), Luengo/Singh/Cannon (CMU), Jaganathan, or Hemami's bucket-loading experiments. Treat them as leads to check, not cited facts.
- No 2022–2026 scaled excavator bucket soil-bin study with an open force time series was found.

## 3. Open datasets (licences)

### Takeaway
No open dataset combining real excavator cylinder pressures, joint angles and soil characterisation was found. The open data that exists is kinematics-only (Hugging Face fuxi-robot), trajectory-only (IEEE DataPort), or wheel-loader supplementary material (Umeå).

### Cited Findings
- **fuxi-robot/excavator-motion (Hugging Face).**
  - 3 excavator models; one HDF5 file per full dig-to-truck-load cycle.
  - Channels at 10 fps: boom, arm, bucket and swing joint angles; camera images (384×480); lidar elevation map (200×200).
  - No pressures or soil data. 74.1 GB, licence CC-BY-NC-SA-4.0 (non-commercial).
  - [Hugging Face](https://huggingface.co/datasets/fuxi-robot/excavator-motion)
- **IEEE DataPort dataset (about 2.71 MB) for "Deep Learning-Based Autonomous Excavation: A Bucket-Trajectory Planning Algorithm".**
  - Point clouds and bucket motion from expert excavation.
  - Licence and authors not confirmed.
  - [IEEE DataPort](https://ieee-dataport.org/documents/deep-learning-based-autonomous-excavation-bucket-trajectory-planning-algorithm); [paper](https://www.researchgate.net/publication/370035082_Deep_Learning-Based_Autonomous_Excavation_A_Bucket-Trajectory_Planning_Algorithm)
- **ExcaScene (GitHub nvh2520).**
  - Simulated excavator perception benchmark. Zenodo DOI reserved, possibly not yet downloadable.
  - [GitHub](https://github.com/nvh2520/excascene)
- **Umeå wheel loader sim-to-real supplementary page:** umit.cs.umu.se/wl-sim-to-real/, cited in [arXiv 2310.05765](https://arxiv.org/html/2310.05765v3). Contents and licence unverified.
- **Zhao & Hu, RL-based bucket filling.**
  - Imitation-learned digging gave a mean payload of 6.52 kg per cycle vs 2.68 kg for a fixed-dig baseline. A small-scale machine is implied.
  - No dataset release confirmed.
  - [ResearchGate](https://www.researchgate.net/publication/382747612_Reinforcement_Learning-Based_Bucket_Filling_for_Autonomous_Excavation)
- **IMU-based excavator work-cycle recognition.**
  - Joint angles from IMU quaternions; covers loading, trenching, grading and idling.
  - No pressures seen.
  - [ResearchGate](https://www.researchgate.net/publication/382522200_Automatic_recognition_of_excavator_working_cycles_using_supervised_learning_and_motion_data_obtained_from_inertial_measurement_units_IMUs)
- **Zenodo searches.** "Excavator cylinder pressure" returned only unrelated pressure sets, e.g. drilling pressure (CC BY 4.0) and ZeMA electromechanical cylinder testbeds.
  - [Zenodo 8396258](https://zenodo.org/records/8396258); [Zenodo 3364431](https://zenodo.org/records/3364431)

### Inferences
- fuxi-robot is useful for validating trajectory and cycle-time realism of a simulated operator, but not soil forces. Its CC-BY-NC-SA licence restricts commercial use.
- A clean, openly licensed full-scale excavator digging-force dataset appears to be a genuine gap in the field. Publishing one could itself be a contribution.

### Gaps
- I did not directly search Kaggle or Mendeley Data. A search on "Lyu" or "Zhao" excavator datasets found no "Zhao Lyu" dataset. TERA and Bobcat datasets were not found.

## 4. Which studies validated DEM/MPM/SPH/AGX/Chrono excavation sims, and how close they got

### Takeaway
Typical reported agreement is 10–20% on peak or mean digging force and 6–15% on fill mass or volume. MPM (nonpolar) matched bucket drag best in Coetzee's lab test until large displacement. Real-time AGX multiscale models reach about 10% force error at full scale.

### Cited Findings
- **Nonpolar MPM vs corn-grain bucket test:** accurate to 600 mm displacement, 12% under-prediction at 800 mm. DEM and polar MPM were low. [ScienceDirect](https://www.sciencedirect.com/science/article/abs/pii/S0022489806000565)
- **DEM (Coetzee & Els 2009):** 20% drag error at end of fill, ±6% fill volume. [ScienceDirect](https://www.sciencedirect.com/science/article/abs/pii/S0022489809000809)
- **AGX DEM and multiscale vs the WA320-7:**
  - About 10% overall gap; 11% lift/tilt force; 8–19% traction.
  - Payload −15% (DEM) and +12% (multiscale); work 17–21%.
  - Real-time-compatible fidelity performed comparably to high-fidelity models. [arXiv](https://arxiv.org/html/2310.05765v3)
- **AGX multiscale real-time terrain model (Servin, Berglund, Nystedt):**
  - Penetration force scaling calibrated to 2 for most materials.
  - 11 ms compute per 16.7 ms step (real-time factor 1.5). [arXiv 2011.00459](https://arxiv.org/pdf/2011.00459); [Springer](https://link.springer.com/article/10.1186/s40323-021-00196-3)
- **agxTerrain tech report (2019):** compared against NDEM reference simulation, not physical measurement. Penetration force grows faster in NDEM. [Algoryx PDF](https://www.algoryx.se/download/agxTerrain_tech_report.pdf)
- **AGX Terrain manual:** recommends calibrating penetration-resistance scaling to measured initial-breach force. [Algoryx docs](https://www.algoryx.se/documentation/complete/agx/tags/latest/doc/UserManual/source/agxTerrain.html)
- **Rope shovel FE-DEM full-scale validation:** validated on bucket mass and full-scale tests. Dig variables significantly affect average dig force and fill factor. [Springer](https://link.springer.com/article/10.1007/s40571-020-00372-z)
- **ISARC 2020 MPM / hybrid vs sandbox excavator:** "good agreement", slightly more oscillatory. [IAARC](https://www.iaarc.org/publications/fulltext/ISARC_2020_Paper_60.pdf)
- **Wiberg, Servin & Nordfjell:** DEM with experimental validation of draft forces in cohesive soil (title only seen). [Algoryx publications](https://www.algoryx.se/publications/)
- **36.5 t excavator:** analytical limit digging force vs measured resistance, exceeding it in 84.4% of samples. [PMC](https://pmc.ncbi.nlm.nih.gov/articles/PMC9519879/)

### Inferences
- A defensible acceptance target for the MPM simulator is about ±15% on mean and peak bucket force, ±15% on payload per pass, and correct qualitative force-time shape. These figures are drawn from the AGX and Coetzee results above.
- Calibrating penetration (breakout) resistance separately, as AGX does, is likely necessary for any reduced or real-time model.

### Gaps
- No Chrono::GPU / Chrono-SPH (CRM) excavator bucket validation against physical measurements was found in these searches.
- No SPH bucket validation was found.
- No recent (2023–2026) MPM full-scale excavator validation was found.

## 5. Per-study parameters summary (bucket, trajectory, soil, quantity, reproducibility)

### Takeaway
Only two sources give enough to attempt reproduction in simulation: Aoshima & Servin (full scale, gravel, published soil and DEM parameters, pile scans, 3 runs) and Coetzee (lab, corn grains, calibrated DEM). Full-scale excavator studies omit soil numbers or bucket geometry in accessible text.

### Cited Findings
| Study | Bucket | Trajectory | Soil params | Measured | Reproducibility / data |
|---|---|---|---|---|---|
| Aoshima & Servin 2024 ([arXiv](https://arxiv.org/html/2310.05765v3)) | 3.0 m3, 2.685 m wide (WA320-7) | Operator loading, 3 runs; bucket tip path recorded | Gravel 30–40 mm, 1727 kg/m3, repose 32°, φ 32°, dilatancy 8° | Lift/tilt/traction forces, kinematics at 100 Hz, mass, work, pile scan | High; supplementary page, licence unverified |
| Coetzee & Els 2009 ([SD](https://www.sciencedirect.com/science/article/abs/pii/S0022489809000809)) | Lab bucket | Prescribed drag | Corn grains, calibrated DEM | Drag force, fill volume | Medium (paper only) |
| Coetzee et al. 2006 ([SD](https://www.sciencedirect.com/science/article/abs/pii/S0022489806000565)) | Lab bucket | Drag to 800 mm | Cohesionless corn | Drag force, flow pattern | Medium |
| 36.5 t backhoe ([PMC](https://pmc.ncbi.nlm.nih.gov/articles/PMC9519879/)) | Not reported | Piling and trenching | "Tertiary soil with stones", no numbers | 6 cylinder-chamber pressures, 3 angles, resistance | Low; data on request |
| Two 20 t backhoes, M&I 2024 ([M&I](https://www.mechanics-industry.org/articles/meca/full_html/2024/01/mi230039/mi230039.html)) | Unverified | Field digging | φ by triaxial (values unverified) | Cylinder pressures, angles, resistance | Unknown; OA paper |
| Lab excavator ([RG](https://www.researchgate.net/publication/366029615_Analysis_of_Digging_Forces_on_Laboratory_Scale_Excavator)) | About 4 L | Attack angles 40/50/60° | Loose sand (no params seen) | Force from pump pressure, 1.13–1.60 kN | Low |
| NASA soil bin ([NTRS](https://ntrs.nasa.gov/api/citations/20200003063/downloads/20200003063.pdf)) | Small bucket | Linear, designed for validation | Lightly compacted regolith simulant | Horizontal and vertical force | Medium; public-domain report |
| fuxi-robot ([HF](https://huggingface.co/datasets/fuxi-robot/excavator-motion)) | Not given | Real dig-to-truck cycles | None | Joint angles, images, elevation | Open, CC-BY-NC-SA-4.0 |

### Inferences
- For a ZX200 / 0.8 m3 target, a practical validation ladder is:
  1. Coetzee bucket test (lab, MPM vs DEM).
  2. NASA linear-trajectory bucket test.
  3. Aoshima/Servin gravel loading (full scale, published error budget).
  4. In-house instrumented ZX200 trials, using the sensor set from the Chinese studies (6 chamber pressures, joint angles) plus soil tests (density, φ, c, water content) and pre/post terrain scans.

### Gaps
- No source in this session gave a measured bucket fill factor vs trajectory for a 20 t excavator with soil parameters.
- ZX200-specific published digging-force data was not found.
