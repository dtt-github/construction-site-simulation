# Terramechanics experiments and soil-machine simulator validation suites

Context: picking validation cases for a real-time Newton implicit MPM soil + excavator simulator (GPU, 8 cm grid, sand/loam/clay presets). Research date 2026-11-03. About 18 tool calls; several primary PDFs could not be read in full (noted in Gaps).

## Plate sinkage, bevameter and wheel/track datasets (Wong tables, GRC-1, NASA, single-wheel testbeds)

### Takeaway
The classic Bekker-Wong parameter sets (dry sand, sandy loam, clay) are widely reproduced in secondary sources and can drive analytic reference curves. Openly downloadable raw plate-sinkage or bevameter data is scarce. The best open single-wheel set found is a small MIT-licensed digitised CSV (27 rows). NASA GRC-1 / SLOPE-lab wheel tests are the most-used experimental reference, but GRC-1 bevameter values were not found in open text.

### Cited Findings
**Wong-style Bekker-Wong terrain tables (as reproduced in secondary papers; units kN/m^(n+1) for k_c, kN/m^(n+2) for k_phi):**
- Dry sand: n = 1.1, c = 1.0 kPa, phi = 30 deg, k_c = 0.9, k_phi = 1523.4, shear deformation modulus K = 0.025 m. Sandy loam: n = 0.7, c = 1.7 kPa, phi = 29 deg, k_c = 5.3, k_phi = 1515.0, K = 0.025 m. Clayey soil: n = 0.5, c = 4.14 kPa, phi = 13 deg, k_c = 13.2, k_phi = 692.2, K = 0.01 m. The table also lists snow. — [Birzeit repository paper on equivalent soil mechanics for rigid wheels](https://fada.birzeit.edu/bitstream/20.500.11889/7935/1/An%20equivalent%20soil%20mechanics%20formulation%20for%20rigid%20wheels%20in%20deformable%20terrain%2c%20with%20application%20to%20planetary%20exploration%20rovers.pdf)
- A 2026 arXiv paper gives the same sandy loam values (k_c 5.27, k_phi 1515.04, c 1.72 kPa, phi 0.5061 rad = 29 deg, n 0.7) and clay values (k_c 13.19, k_phi 692.15, c 4.14 kPa, phi 0.2269 rad = 13 deg, n 0.5). This independently confirms the loam and clay rows. — [arXiv 2603.28965 (Koopman off-road vehicle)](https://arxiv.org/pdf/2603.28965)
- Dry-sand values differ widely across papers. One 2026 rover paper hand-tuned k_c 10.3, k_phi 5309.4, c 1200 Pa, phi 33.3 deg, n 1.9 to match its own drawbar-pull tests. — [arXiv 2606.06790](https://arxiv.org/pdf/2606.06790). A 2025 snake-robot paper uses n 0.6, c 25 Pa, phi 28 deg for "loose dry sand". — [arXiv 2512.05008](https://arxiv.org/pdf/2512.05008)
- A Chinese lunar-simulant pressure-sinkage study reports n = 0.698-1.123, k_c = 0.524-5.928, k_phi = 0.124-1.926 and stresses their sensitivity to bulk density. — [Scientific.net AMR 508:241](https://www.scientific.net/AMR.508.241)

**GRC-1 and NASA:**
- GRC-1 (Oravec, Zeng, Asnani 2010, J. Terramechanics) was designed for lunar terramechanics testing in Earth-ambient conditions. Its index and geotechnical properties are compared to lunar soil in the paper; the full text is behind a paywall. — [Stevens repository record](https://researchwith.stevens.edu/en/publications/design-and-characterization-of-grc-1-a-soil-for-lunar-terramechan/)
- Edwards et al. (2017, J. Terramechanics) fitted Bekker, Bekker-Wong and N2M parameters for Fillite and compared them with GRC-1. — [BL record](https://eld.bl.uk/catalog/100041663169_0x000001)
- A NASA SRR 2025 report (Mueller) runs Bekker-form pressure-sinkage tests on LHS-1E and BP-1 simulants. It is open on NTRS, but its full analysis was pending. — [NTRS 20250003024](https://ntrs.nasa.gov/api/citations/20250003024/downloads/SRR%202025%20Pressure%20Sinkage%20Testing%20FINAL%20Mueller.pdf)
- On GRC-1, a 1-g test produced 44% more drawbar pull and 13.5% less sinkage than a 1/6-g test, so gravity matters. — [ISTVS publication news](https://istvs.org/publication-news/experimental-evaluation-of-cone-index-gradient-as-a-metric-for-the-prediction-of-wheel-performance-in-reduced-gravity)
- Chrono::CRM's MGRU3 single-wheel case uses NASA SLOPE-lab tests on GRC-1. Its setup: bulk density 1760 kg/m3 (60% relative density), phi 38.4 deg; wheel radius 0.25 m, width 0.2 m, 24 grousers 2.5 cm high; soil bin 5 x 0.8 x 0.25 m; slopes 0-30 deg imposed by rotating gravity; omega 0.8 rad/s. The measured quantity is slip vs slope. — [arXiv 2507.05643](https://arxiv.org/html/2507.05643)

**Open single-wheel data:**
- Hugging Face "PINNeAPPle-Labs/single-wheel-tests" (MIT licence) has 27 rows of drawbar pull, sinkage and torque vs slip, plus wheel radius, width, grousers and load. The rows were digitised from figures in Ding et al. 2011 (J. Terramech. 48(1):27-45, TYII simulant), Wang & Han 2016 (KLS-1) and Hurrell et al. 2025 (Space Sci. Rev., FJS-1). The maintainers flag the values as "third-party readings of the figures" for validation only, and digitisation uncertainty is not quantified. The source is RoverDevKit (Duke CAS Lab) `data/validation/single_wheel_experiments.csv`. — [HF dataset](https://huggingface.co/datasets/PINNeAPPle-Labs/single-wheel-tests); [RoverDevKit arXiv 2606.21755](https://arxiv.org/pdf/2606.21755)
- Other testbeds with published curves but no open raw data found: the RUAG/ESA ExoMars single-wheel facility ([ESA ASTRA 2019](https://roboshare.esa.int/ASTRA/Astra2019/Monday%2027%20May/4b.%20Planetary%20Robots%20testing/S.4b_16.35_Oettershagen.pdf)) and TRI-1 lunar simulant wheel sinkage studies ([ResearchGate](https://www.researchgate.net/publication/332121467_Studies_on_the_sinkages_of_rigid_plain_wheels_and_lugged_wheels_on_TRI-1_lunar_soil_simulant)).

### Inferences
- With an 8 cm grid, plate-sinkage tests need plates of about 0.5-1 m diameter (6-12 cells across), not the 5-20 cm bevameter plates. The practical check is therefore to compare simulated pressure-sinkage against analytic Bekker-Wong curves from the Wong sand/loam/clay rows, rather than against raw small-plate data.
- Single-wheel rover tests (r = 0.25 m, sinkage of a few cm) fall below 8 cm resolution. They are useful only qualitatively, or as a local refinement test. Vehicle-scale wheels and tracks (r of 0.5 m or more, sinkage of 10 cm or more) suit the grid better.
- From memory, and not verified against the book in this session: Wong's Theory of Ground Vehicles lists LETE/LLL dry sand as n 1.1, k_c 0.99, k_phi 1528.43, c 1.04 kPa, phi 28 deg. The secondary source above rounds these to 0.9, 1523.4, 1.0 kPa and 30 deg. Verify against the book before adopting them.

### Gaps
- GRC-1 k_c, k_phi, n values were not found in open text (they are in the paywalled Oravec 2010 paper and NASA Glenn reports).
- Iagnemma (MIT) and JPL single-wheel raw data releases were not found. No Zenodo deposit of raw bevameter or plate-sinkage data was found.
- Track (rather than wheel) soil-bin data: nothing open found.

## Simulator validation suites (Chrono CRM, AGX, Vortex, ETH, MPM, learned simulators, Newton)

### Takeaway
Chrono::CRM (BSD-3, open scripts) has the most reproducible validation suite: sphere cratering, cone penetration, the MGRU3 single wheel, and a RASSOR drum checked against DEM. AGX Terrain is validated mostly against high-resolution DEM (10-25% agreement), plus one full-scale wheel-loader field study with about 10% average force error. Newton's implicit MPM has no published validation against physical soil. Most other works validate against DEM or qualitatively.

### Cited Findings
**Project Chrono CRM/SPH (Wisconsin SBEL, arXiv 2507.05643, submitted to J. Terramechanics 2025):**
- Sphere cratering vs Uehara et al. / Ambroso et al. Setup: density 1510 kg/m3, mu_s 0.3; sphere r 12.5 mm, densities 700 and 2200 kg/m3; drops of 5-20 cm; spacing d0 2.5 mm. Fitted slope 0.1336 (R2 0.9714) vs empirical 0.14; MSE 1e-7 m2. — [arXiv 2507.05643](https://arxiv.org/html/2507.05643)
- Cone penetration (free-fall cone) in glass beads (1500 kg/m3) and Ottawa 20-30 sand (1780 kg/m3), using mu(I) rheology. Cones: 60 deg / 19.8 mm and 30 deg / 9.2 mm, in a 0.1 m container. Mean relative error 3.5% (beads) and 7.4% (sand), all cases under 10%. — [arXiv 2507.05643](https://arxiv.org/html/2507.05643)
- MGRU3 single wheel on GRC-1 with d0 = 1 cm: the slip-vs-slope curve "closely matches" both experiment and DEM, but no numerical metric is given. Real-time factor (RTF) 20.25 with active domains (59.36 without). — [arXiv 2507.05643](https://arxiv.org/html/2507.05643)
- RASSOR excavation drum on GRC-1 (1700 kg/m3, phi 35 deg), d0 5 mm, compared only with Chrono DEM. Torque matches qualitatively. RTF 70.49. — [arXiv 2507.05643](https://arxiv.org/html/2507.05643); a related SBEL 2024 tech report is "RASSOR Excavation on Deformable Terrain Modeled via CRM and DEM" ([SBEL tech reports](https://sbel.wisc.edu/publications/technicalreports))
- Licence BSD-3. The paper says all models and simulations are in a public GitHub repo. — [arXiv 2507.05643](https://arxiv.org/html/2507.05643)
- SBEL also calibrated a faster soil contact model against CRM "ground truth" using single-wheel and full-rover sims. — [SBEL calibration paper](https://sbel.wisc.edu/publications/journal-papers/calibration-of-an-expeditious-terramechanics-model/)

**AGX Dynamics Terrain (Algoryx / Umeå, Servin et al.):**
- Multiscale real-time model (Servin, Berglund, Nystedt 2021, AMSES): validated against coupled DEM-multibody reference simulations of excavation and bulldozing. Digging resistance and soil displacement agree within 10-25%, and the model runs more than 1000x faster. — [AMSES 2021 PDF](https://amses-journal.springeropen.com/track/pdf/10.1186/s40323-021-00196-3.pdf); [Algoryx paper page](https://www.algoryx.se/papers/terrain/)
- A 2025 Umeå paper reports AGX digging forces and soil displacement at 75-90% of resolved DEM. — [arXiv 2505.22091](https://arxiv.org/pdf/2505.22091v2)
- The agxTerrain tech report covers calibration via direct shear and triaxial tests, a terrain library, digging-tool validation, and excavation and bulldozing simulations; only its outline could be read. — [agxTerrain tech report](https://www.algoryx.se/download/agxTerrain_tech_report.pdf)
- Field sim-to-real study: Komatsu WA320-7 wheel loader in crushed-rock gravel (30-40 mm, 1727 kg/m3), 100 Hz logging of lift, tilt and traction forces, laser-scanned pile, and loaded mass. Multiscale soil settings: phi 32 deg, dilatancy 8 deg, E 4.6 MPa, grid 50-400 mm. Results: average sim-to-real error about 10%; lift and tilt force errors about 11%; traction 8-19%; loaded mass +12% (multiscale) and -15% (DEM); work error 17-21%. The 200 mm grid runs in real time and 400 mm about 5x faster. Only videos are public; no field data release was found, and AGX is commercial. — [arXiv 2310.05765](https://arxiv.org/html/2310.05765v3)
- Holz, Beer & Kuhlen 2009 (VRIPHYS): hybrid particle/mesh real-time soil with compaction. No quantitative validation was found in accessible text. — [Eurographics DL](https://diglib.eg.org/items/af385bbf-0595-45cb-b286-44a7dd72903a)

**Vortex Studio earthmoving:**
- Wagner (arXiv 2505.19330, CC BY 4.0) derives the fundamental equation of earthmoving (FEE) for a flat blade in sloped soil and documents advanced configuration of Vortex soil-tool simulation. It is supplementary to ISTVS 2023 and 2025 soil-property-estimation papers. It contains no validation experiments or datasets. — [arXiv 2505.19330](https://arxiv.org/abs/2505.19330)
- ETH HEAP (Menzi Muck M545) dig cycles were developed in Vortex and transferred to hardware unchanged. This shows controller transfer, not soil-force validation. — [ETH research collection](https://www.research-collection.ethz.ch/bitstream/20.500.11850/403425.1/1/19-0276_04_MS.pdf)
- ETH/Isaac Lab boulder excavation uses a quasi-static 2D soil-force model with randomised parameters, not a validated continuum. — [arXiv 2509.17683](https://arxiv.org/pdf/2509.17683)

**MPM research codes:**
- Haeri & Skonieczny (CMAME 2022, doi 10.1016/j.cma.2022.114904; arXiv 2111.01523): 3D MPM with hyperelastic nonlocal granular fluidity. Validated against their own robotic sandbox excavation rig (3-DOF tool) on force and flow geometry, with additional checks on wheel-soil, silo and Taylor-Couette cases. The force trend and the drops and rises in force are captured. No numeric error and no code link appear on the abstract page. — [arXiv 2111.01523](https://arxiv.org/abs/2111.01523); [PDF](https://arxiv.org/pdf/2111.01523)
- An older 2D DEM vs MPM bucket-filling comparison against experiments (corn grains) found the nonpolar continuum predicted bucket force best and DEM under-predicted. — [zhenzhi record](https://data.zhenzhi.com/details/1/894574629)

**NVIDIA Newton / Isaac Lab:**
- Newton `SolverImplicitMPM` implements implicit granular rheology on a background voxel grid with a GPU-friendly solver. The Isaac Lab demo is `scripts/demos/mpm/newton_mpm_granular.py` (granular drop). The docs describe MPM and rigid-MPM coupling as experimental. No physical validation data was found. — [Newton API docs](https://newton-physics.github.io/newton/api/_generated/newton.solvers.SolverImplicitMPM.html); [Isaac Lab implicit MPM](https://isaac-sim.github.io/IsaacLab/main/source/concepts/using_mpm.html)

**Learned simulators:**
- GNS (Choi & Kumar, Computers and Geotechnics 2023; open source at github.com/geoelements/gns) is trained on MPM granular column collapse and 3D barrier flows. It reproduces the collapse stages, but the tall 3D column mobilises too much mass and ends with a lower final height than MPM. Reported speedups range up to about 2867x in one summary and 145x in a slope paper. No excavation validation exists. — [arXiv 2311.07416](https://arxiv.org/pdf/2311.07416); [Choi group page](https://sites.google.com/view/geo-yjchoi/research-list/gns-column)
- TRACE (2026) reduces long-rollout position error by 31-62% vs GNS on granular benchmarks. — [arXiv 2609.02991](https://arxiv.org/pdf/2609.02991)

### Inferences
- An apples-to-apples external target is the AGX wheel-loader study: about 10% force error at a 200 mm grid in real time. An 8 cm Newton MPM grid sits between the study's G50 and G100 settings, so 10-20% force error on bulk digging is a realistic bar.
- Chrono::CRM is the best open reference for code-to-code comparison (BSD-3). It can be run on the same excavation geometry as a slower "truth" model, mirroring how Algoryx validates against DEM.

### Gaps
- Caterpillar/Iowa "real-time excavation validated" work, Bobcat/Doosan simulators and Vortex soil-bin validation: no primary sources found in this pass.
- A search snippet suggested arXiv 2609.15382 compares Isaac Sim, Chrono and Newton MPM against a real wheel loader. The abstract page did not confirm this (it covers world-model action selection), so treat the claim as unverified.
- The Chrono CRM GitHub path for validation scripts was not captured (the paper states the scripts are public).
- Haeri & Skonieczny rig dimensions, soil and numeric errors are in the paywalled CMAME paper.

## Cone penetration test (CPT) as a validation check

### Takeaway
CPT is well studied in MPM geotechnics, typically matching chamber tests within about 25%. Chrono CRM matches a free-fall cone within 10%. Both use mm-scale resolution, so CPT and cone index are not directly resolvable on an 8 cm grid.

### Cited Findings
- Purdue MPM CPT thesis (Bisht): cone resistance within 25% of calibration-chamber measurements, with displacement fields similar to DIC. — [Purdue thesis](https://hammer.purdue.edu/articles/thesis/Cone_penetration_analysis_using_the_Material_Point_Method/15055686); [abstract](https://engineering.purdue.edu/CE/Academics/Graduate/Current/Abstracts/PhD_Defenses/2020/Bisht-Vibhav-Abstract.pdf)
- TU Delft dry-sand MPM CPT was validated against chamber data for cone resistance and displacement field. Accuracy is best in moderately dense to dense sand and depends on the critical-state constitutive model. — [TU Delft record](https://repository.tudelft.nl/record/uuid:bcd39900-9918-435f-8fb6-7ffdf6abdaa6); [TU Delft record 2](https://repository.tudelft.nl/record/uuid:d5fd431b-3484-41e1-84f8-1303c6ca82ed)
- Chrono CRM free-fall cone: 3.5-7.4% mean error (see above), with particle spacing far below 1 cm. — [arXiv 2507.05643](https://arxiv.org/html/2507.05643)
- Cone-index gradient has been evaluated as a predictor of wheel performance on GRC-1 in reduced gravity. — [ISTVS publication news](https://istvs.org/publication-news/experimental-evaluation-of-cone-index-gradient-as-a-metric-for-the-prediction-of-wheel-performance-in-reduced-gravity)

### Inferences
- For an 8 cm grid, the realistic options are an up-scaled "penetrometer" (a rod 0.3-0.5 m across, effectively a plate test) or a convergence study on a locally refined grid. A standard 35.7 mm CPT is sub-cell.

### Gaps
- No open CPT dataset with a licence was identified in this pass.

## Open benchmark suites for terrain/soil simulation

### Takeaway
No open "TerraBench" or "SoilBench"-style standard suite was found for 2023-2026. The closest options are the NATO AVT-308 mobility benchmark (vehicle-level), the ISTVS resource wiki, Chrono's open validation scripts, and small digitised CSVs.

### Cited Findings
- NATO STO AVT-308 ran a multi-vendor mobility benchmark with simple and complex terramechanics. Its vehicle database was made public. — [NATO TM-AVT-308 Annex D](https://www.sto.nato.int/publications/STO%20Technical%20Reports/STO-TM-AVT-308/TM-AVT-308-ANN-D.pdf); [Annex F](https://www.sto.nato.int/publications/STO%20Technical%20Reports/STO-TM-AVT-308/TM-AVT-308-ANN-F.pdf)
- ISTVS has a wiki-based resource initiative for exchanging models and data (2019 update; current state unknown). An ISTVS project started in 2016 to update standards for soil-parameter measurement and model validation. — [ISTVS newswire](https://www.istvs.org/newswire/istvs-resource-initiative-update-20190327); [Review of terramechanics models, U. Pretoria](https://repository.up.ac.za/handle/2263/65242)

### Inferences
- A defensible in-house suite for the project could combine six cases:
  1. Wong-table analytic plate-sinkage curves for the sand, loam and clay presets.
  2. Angle of repose and column collapse.
  3. A flat-blade cutting force vs the fundamental equation of earthmoving (Wagner's derivation).
  4. Code-to-code bucket digging vs Chrono CRM (open).
  5. A full-scale wheel-loader or excavator force and mass comparison in the style of AGX's about 10% result.
  6. Qualitative single-wheel slip-sinkage vs the MIT-licensed digitised CSV.

### Gaps
- No 2023-2026 open benchmark with downloadable data for excavation forces was found.
