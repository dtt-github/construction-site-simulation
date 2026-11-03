# Soil-bin blade / tine / flat-tool force experiments for validating an 8 cm-grid Newton implicit MPM soil simulator

Scope note: research was capped at about 15 tool calls, and several publisher pages (ScienceDirect, PMC verification walls) returned 403 or bot checks. Items marked **[unverified-training]** come from the researcher's background knowledge and were NOT confirmed by a source fetched in this session. The report writer should treat them as leads, not facts. Resolvability uses the simulator constraint: about 0.3 m or more tool width and 0.1-0.5 m cut depth means "resolvable"; smaller means "needs scaling" or "too small".

## Q1. Which soil-bin studies report horizontal/vertical draught vs depth, rake angle and speed, with full soil characterisation?

### Takeaway
Few studies give a full soil characterisation (density, phi, c, adhesion, soil-metal friction, water content, grain size) together with tabulated forces. The best open example confirmed here is Makange et al. 2021 (Sci. Rep., CC BY 4.0, clay, 260 mm subsoiler, 0.15-0.30 m depth, 3 speeds, tabulated Fx/Fz). The scaled Caterpillar D3K2 bulldozer-blade soil-bin series (J. Terramechanics 2020, 2024; Powder Technology 2025) is the best blade-shaped series, but it is paywalled.

### Cited Findings
- **Makange et al. 2021, "Prediction of precise subsoiling based on analytical method, DEM and soil-bin data", Sci. Rep. 11:11082.** Rectangular-blade subsoiler: effective blade width 260 mm, rake angle 16°, blade at 23° to the ground, 400 mm-radius arc shank, 30 mm thick. Depths 0.15/0.20/0.30 m; speeds 1, 2, 2.5 km/h. Clay soil: bulk density 1667.7 kg/m³, moisture 10.8-11.2%, cone index 325-3031 kPa with depth. DEM inputs: c = 10 kPa, soil-soil friction 0.45, soil-steel friction 0.55. Friction angle, adhesion and grain size are NOT reported. Open access, CC BY 4.0 — [PMC8155074](https://pmc.ncbi.nlm.nih.gov/articles/PMC8155074/)
  - Table 4 of the paper (measured soil-bin forces):

    | Speed km/h | Depth m | Horizontal N | Vertical N |
    |---|---|---|---|
    | 1 | 0.15 | 1235.24 | 490.22 |
    | 1 | 0.20 | 1905.15 | 740.51 |
    | 1 | 0.30 | 3381.81 | 1390.34 |
    | 2 | 0.15 | 1296.55 | 500.23 |
    | 2 | 0.20 | 1934.67 | 750.91 |
    | 2 | 0.30 | 3389.01 | 1401.90 |
    | 2.5 | 0.15 | 1312.11 | 510.50 |
    | 2.5 | 0.20 | 1941.32 | 760.45 |
    | 2.5 | 0.30 | 3417.52 | 1410.11 |
    — [PMC8155074](https://pmc.ncbi.nlm.nih.gov/articles/PMC8155074/)
  - The best DEM fit to horizontal force had R² = 0.998 and normalised RMSE 0.452 — [Sci. Rep. via nature.com](https://www.nature.com/articles/s41598-021-90682-w)
- **Scaled Caterpillar D3K2 XL bulldozer-blade soil-bin series.** The full-scale blade (n = 1) is 3.2 m wide and 1.07 m high. Soil-bin tests used 1/8 and 1/13 geometric scales, sandy loam, and a cutting depth of 20% of blade height. DEM predictions fit the soil-bin data linearly with R² = 0.9965 — [J. Terramechanics 2020, "Modeling soil-bulldozer blade interaction using DEM"](https://www.sciencedirect.com/science/article/abs/pii/S0022489819301466); [ResearchGate](https://www.researchgate.net/publication/340355669_Modeling_soil-bulldozer_blade_interaction_using_the_discrete_element_method_DEM)
  - A 2024 follow-up measured blade cutting forces with a load-cell dynamometer carriage in an artificial cohesive-frictional soil, at 1/8 and 1/13 scale. It notes that full-scale blade tests have been run only in large soil bins (1968-2011 references) — [J. Terramechanics 116 (2024), "Scaling law modeling artificial soil-to-bulldozer blade interaction"](https://www.sciencedirect.com/science/article/abs/pii/S0022489824000430)
  - A 2025 multiresponse DEM elasto-plastic calibration against the same tests reported, for the 1/8 blade, 4.5% error in horizontal reaction force and 12.9% error in soil mass — [Powder Technology 2025](https://www.sciencedirect.com/science/article/pii/S0032591025004516) (full text returned 403; only the abstract was seen)
- **"The cutting of soil by narrow blades", J. Terramechanics vol. 14 (1977).** Draught and vertical forces were measured, and force factors depending on width/depth ratio, rake angle and soil friction angle were derived. The search summary associated it with coarse sand, loamy sand and sandy loam, but that may be conflated with Chen et al. 2013 (see below) — [ScienceDirect](https://www.sciencedirect.com/science/article/abs/pii/0022489877900015)
- **Dynamic plane-tool cutting, Soil & Tillage Research (1998).** "Draught and vertical forces obtained from dynamic soil cutting by plane tillage tools" — [ScienceDirect](https://www.sciencedirect.com/science/article/abs/pii/S0167198798001275) (abstract only; parameters not retrieved)
- **Chen et al. 2013, sweep in three soils.** Coarse sand, loamy sand and sandy loam. Measured draught and vertical forces were used to calibrate DEM particle stiffness — [search summary via Scribd copy of chen2013](https://www.scribd.com/document/491682449/chen2013)
- **Subsurface tool, Soil & Tillage Research 2021.** "Dual sideway-share subsurface tillage implement, Part 1": soil-bin cutting resistance and disturbance measured at 0.25 m/s and 150 mm depth, then modelled with DEM — [ScienceDirect](https://www.sciencedirect.com/science/article/abs/pii/S0167198721002749)
- **Wide cutting blade in a sand-filled box (DEM vs experiment).** "Interaction between soil and a wide cutting blade using DEM", Soil & Tillage Research (2007); very good correlation with the experiments reported — [ScienceDirect](https://www.sciencedirect.com/science/article/abs/pii/S0167198707001249)

### Inferences
- Makange et al. 2021 is the single most usable open case for the "clay" or "loam" preset. The tool (260 mm) is just below the 0.3 m threshold, so it is "borderline / needs scaling": at 8 cm cells, the blade spans about 3 cells. The 0.15-0.30 m depths are resolvable. The near-insensitivity of force to speed (about 1-6% change from 1 to 2.5 km/h) makes it a quasi-static test, which suits implicit MPM.
- Full-scale D3K2 dimensions (3.2 m x 1.07 m, 0.21 m cut) would be ideal for 8 cm cells, but the measured data are at 1/8 scale (about 0.40 m wide x 0.13 m high, cut about 27 mm). That is "too small" in depth. A full-scale virtual replica would rely on scaling laws, not direct data.
- At 8 cm / 4 cm resolution, narrow tines (<0.1 m, common in Godwin & Spoor-type studies) are "too small". Only wide blades, bulldozer blades and large sweeps are directly comparable.

### Gaps
- Full soil characterisation (phi, c, adhesion, soil-metal friction) for the D3K2 soil-bin series could not be retrieved because of paywalls.
- **Classic datasets, unverified-training:** Hettiaratchi & Reece (1967/1974) wide-blade passive-pressure tests; Godwin & Spoor (1977) narrow tines; McKyes & Ali (1977) narrow blades; McKyes (1985) "Soil Cutting and Tillage" (book tables); Osman (1964) wide blades; Wong "Terramechanics" textbook examples; Fielke (1988/1999) sweeps in sand; Kobayashi bulldozer DEM; Shmulevich (2007, 2010) wide blade DEM vs soil bin in sand; Karmakar (CFD tillage). These were not verified in this session; their parameter values must come from the papers themselves.

## Q2. Are force-time series or tabulated data available (paper tables, Zenodo, Mendeley, GitHub)? Licence?

### Takeaway
Raw force-time series are rare. Most studies give only figure curves or a mean-force table. One recent open dataset article (soil-sweep draft force, cone index and surface profiles in field clay loam, with metadata) exists on PMC but could not be fully read. Makange 2021 provides a CC BY 4.0 table. No Zenodo dataset for soil-bin blade forces was found.

### Cited Findings
- Dataset article: "A dataset of soil-sweep tool interaction measurements on an agricultural field including cone penetration resistance, draft force, and surface profiles under varying soil moisture conditions". Field (not bin), clay loam; "Tillage" folders hold draft-force measurements, with a metadata file recording tool, nominal velocity and conditions — [PMC13584141](https://pmc.ncbi.nlm.nih.gov/articles/PMC13584141) (the fetch hit a bot-verification page; repository DOI, licence, sweep width and depth were NOT confirmed)
- Makange et al. 2021 force table, licensed CC BY 4.0 — [PMC8155074](https://pmc.ncbi.nlm.nih.gov/articles/PMC8155074/)
- A search for Zenodo soil-bin blade force datasets returned only papers, no archived data — [search result set incl. Powder Technology 2025](https://www.sciencedirect.com/science/article/pii/S0032591025004516)
- University of Manitoba theses (Murray, DEM soil-tool; Zeng, soil-tool-residue measurements and modelling) are open PDFs that likely hold tabulated soil-bin data — [Murray thesis](https://mspace.lib.umanitoba.ca/bitstream/handle/1993/31735/Murray_Steven.pdf?sequence=1); [Zeng thesis](https://mspace.lib.umanitoba.ca/server/api/core/bitstreams/bfae708b-3ac6-42cd-898d-853aa0a2f13a/content)
- Iowa State thesis on DEM soil-to-tool interaction (likely the bulldozer-blade group) is open — [ISU DR](https://dr.lib.iastate.edu/server/api/core/bitstreams/2cb16439-b0e0-4efe-ac79-13a48fe4aa16/content)
- Ucgul's Univ. of South Australia / Adelaide thesis on 3D DEM of tillage with cohesion and adhesion is open — [Adelaide digital library](https://digital.library.adelaide.edu.au/items/e53f026b-fb55-4682-99cf-e4c0dbc7f1fa/full); [CORE PDF](https://core.ac.uk/download/42947617.pdf)

### Inferences
- Theses are the most likely source of tabulated per-run values and soil-test data, which the journal versions omit. Licences are usually "all rights reserved, open to read". Using extracted numbers as validation targets with citation is normal practice, but redistribution of the numbers in a repo should be checked case by case.
- The PMC sweep dataset is the strongest lead for real time series, but it is field clay loam with probably agricultural sweep widths. Confirm its width (sweeps of 0.3-0.5 m would be resolvable) and licence before use.

### Gaps
- Repository DOI and licence of the PMC sweep dataset.
- No GitHub or Mendeley soil-bin blade datasets were confirmed.

## Q3. Which experiments have been used to validate DEM/MPM/SPH/FEM simulations?

### Takeaway
DEM validation against soil-bin tests is common (Ucgul/Fielke sweeps, Coetzee & Els blade, Obermayr straight blade, D3K2 bulldozer blade, Makange subsoiler). MPM-vs-experiment blade validation is scarce: Coetzee 2014 (2D MPM vs flat blade in corn grains) and Haeri & Skonieczny 2022 (3D MPM nonlocal granular fluidity vs robotic blade in GRC-1 lunar simulant). Chrono::CRM (SPH) is validated on rover and DEM-digging benchmarks, not on physical blade force tests.

### Cited Findings
- **Ucgul, Fielke & Saunders 2014 (Soil & Tillage Research):** a 3D DEM hysteretic spring contact model was validated against Fielke's (1988) sweep experiments in cohesionless soil. Draft and vertical forces gave R² = 0.95-0.99 using 10 mm particles — [Altair community abstract](https://community.altair.com/discussion/40774/3d-dem-tillage-simulation-part-2-validation-of-a-hysteretic-spring-plastic-contact-model-for-a-sweep-tooloperating-in-a-cohesionless-soil)
- **Ucgul et al. 2015:** added cohesion and adhesion to the hysteretic spring contact model and validated against direct shear and soil-tool tests across speed, depth, moisture and compaction — [Adelaide](https://digital.library.adelaide.edu.au/items/e53f026b-fb55-4682-99cf-e4c0dbc7f1fa); [SCU research portal](https://researchportal.scu.edu.au/esploro/outputs/journalArticle/Three-dimensional-discrete-element-modelling-DEM-of/991013001597902368)
- **Coetzee & Els 2009 (J. Terramechanics 46(1):15-26):** DEM calibrated from shear and compression tests, then validated on a blade moving through granular material. Maximum blade-force error was 26% — [search summary; Springer CPM paper](https://link.springer.com/content/pdf/10.1007/s40571-014-0014-7.pdf)
- **Coetzee 2014 (Comp. Part. Mech. 1:409-423):** compared DEM, nonpolar MPM and polar (Cosserat) MPM with flat-blade experiments in cohesionless corn grains. Nonpolar MPM underestimated draft at the lower blade depth. The material model affected flow more than draft force. Dilatancy angle affects draft — [Springer PDF](https://link.springer.com/content/pdf/10.1007/s40571-014-0014-7.pdf); [ResearchGate](https://www.researchgate.net/publication/271949928_Discrete_and_continuum_modelling_of_soil_cutting)
- **Obermayr et al. 2011:** DEM draft-force prediction validated on a straight blade in cohesionless granular material (small-scale lab; horizontal force). Obermayr & Vrettos 2013 calibrated shear strength using simulated triaxial tests — [Fraunhofer Publica](https://publica.fraunhofer.de/handle/publica/226634); [Structurae](https://structurae.net/literature/journal-article/anwendung-der-diskrete-elemente-methode-zur-vorhersage-von-kraften-bei-der-bodenbearbeitung)
- **Haeri & Skonieczny (arXiv 2111.01523; CMA, DOI 10.1016/j.cma.2022.114904):** 3D MPM with hyperelastic nonlocal granular fluidity, compared with their own robotic excavation data and a modified Drucker-Prager model. Interaction forces were compared at 2 cm and 5 cm depths in GRC-1 lunar simulant — [arXiv](https://arxiv.org/abs/2111.01523); [arXiv PDF](https://arxiv.org/pdf/2111.01523)
- **Chrono::CRM (Unjhawala, ..., Negrut 2025, arXiv 2507.05643):** an SPH continuum terramechanics model, validated on three physical tests (including NASA's MGRU3 rover) and benchmarked against high-fidelity DEM of RASSOR digging. A blade-leveling case (Gator) is a control demo. It is open source, BSD-3 — [arXiv](https://arxiv.org/abs/2507.05643v1); [arXiv HTML v2](https://arxiv.org/html/2507.05643v2)
- **Wagner (arXiv 2505.19330):** derives the Fundamental Equation of Earthmoving (FEE) for a flat blade in sloped Mohr-Coulomb soil and documents a Vortex Studio configuration. This is supplementary material to ISTVS 2023/2025 papers; validation is simulation-only, with no physical soil-bin data — [arXiv PDF](https://arxiv.org/pdf/2505.19330); [ISTVS 2025 paper 4558](https://2025.istvs.org/submissions/papers/4558); [arXiv 2507.22356](https://arxiv.org/html/2507.22356v1)
- **Multiscale real-time earthmoving model (arXiv 2011.00459):** relevant prior art for real-time terrain plus a blade-force model — [arXiv](https://arxiv.org/pdf/2011.00459)
- **FEM:** advanced 3D FE of soil-blade interaction in sandy soil found that cutting width, lateral boundary width and soil swelling significantly affect cutting force — [ResearchGate](https://www.researchgate.net/publication/229351785_Simulation_of_soil-blade_interaction_for_sandy_soil_using_advanced_3D_finite_element_analysis). See also "On the FE modeling of soil-blade interaction in tillage" — [ScienceDirect](https://www.sciencedirect.com/science/article/abs/pii/S0168874X14001371)
- **Cohesive soil-pushing by a bulldozer blade (3D DEM):** [J. Terramechanics 2011](https://www.sciencedirect.com/science/article/abs/pii/S0022489811001017)
- **Aikins, Ucgul et al. 2021:** DEM calibration for a Black Vertosol (cohesive) with a narrow-point opener. Ridge height was over-predicted because DEM particles are larger than real ones — [search summary of Ucgul group results](https://researchportal.scu.edu.au/esploro/outputs/journalArticle/3D-DEM-tillage-simulation-Validation-of/991013001597702368)
- **High-speed disc DEM draft prediction (Biosystems Eng. 2020):** [ScienceDirect](https://www.sciencedirect.com/science/article/pii/S1537511020303391)

### Inferences
- The Coetzee 2014 result matters directly for Newton MPM: draft in shallow cuts is sensitive to dilatancy, and nonpolar MPM underpredicts it. Expect the 8 cm grid to underpredict even more for cuts of 2 cells or fewer. Validate the dry-sand preset mainly with 0.2-0.5 m cuts.
- The Aikins result (oversized particles over-predict ridge height) suggests the same for 4 cm MPM particles: compare forces before surface profiles.
- Haeri & Skonieczny depths (2-5 cm) are "too small" for an 8 cm grid, but their approach (MPM vs blade, granular simulant) is the closest methodological template.
- The Wagner FEE (a McKyes-style analytical model) is best used as an analytical reference curve to sweep phi, c, delta and depth against the simulator's presets. This is not experimental validation.

### Gaps
- **Unverified-training:** Zhao/Sulsky MPM soil-cutting papers, Bui SPH soil-cutting, Tsuji DEM, Holz soil-tool, and Kobayashi bulldozer DEM were not confirmed in this session.

## Q4. Large-scale or field-scale blade/bulldozer tests

### Takeaway
Confirmed sources show that full-scale bulldozer blade soil-bin tests exist (cited as 1968-2011 in the 2024 J. Terramechanics scaling paper), and the NASA Glenn lunar soil bin has excavation force/torque data. Specific CRREL/ERDC, Caterpillar and PWRI datasets were not retrieved.

### Cited Findings
- Full-scale bulldozer-blade tests have been conducted only in large soil bins; references span 1968-2011 (the search summary named Garrity 1968, Reaves 1969, Rubinstein 2011) — [J. Terramechanics 2024](https://www.sciencedirect.com/science/article/abs/pii/S0022489824000430)
- The NASA Glenn Research Center lunar soil bin has excavation and Surveyor-type bearing tests with force/torque measurement on an excavator model — [NTRS 20150010197](https://ntrs.nasa.gov/api/citations/20150010197/downloads/20150010197.pdf)
- A Phoenix Mars lander robotic-arm "pit test duricrust" test-data label in PDS shows NASA archived excavation test data publicly — [PDS geosciences](https://pds-geosciences.wustl.edu/phx/phx-m-ra-4-rdr-sci-v1/phxra_1001/testdata/duricrust/pit_test_duricrust.lbl)
- A tractor-mouldboard field study reported 5.6-17.7 kN draft depending on field environment — [PMC9003410](https://www.ncbi.nlm.nih.gov/pmc/articles/PMC9003410/)
- J-STAGE (Japanese) study on the effect of soil strength characteristics, from a bulldozer/blade search — [J-STAGE](https://www.jstage.jst.go.jp/article/jjsem/14/Special_Issue/14_s320/_pdf) (contents not read)
- Saturated-sand cutting: "The Bulldozer Effect When Cutting Water Saturated Sand" (Miedema, dredging) — [ResearchGate](https://www.researchgate.net/publication/267606666_The_Bulldozer_Effect_When_Cutting_Water_Saturated_Sand)
- Analytical bulldozer-blade soil-tool model, J. Terramechanics 31 (1994) — [ScienceDirect](https://www.sciencedirect.com/science/article/abs/pii/0022489894900078)

### Inferences
- The full-scale D3K2 geometry (3.2 m x 1.07 m) with 0.2-0.3 m cuts maps to about 40 x 13 cells, which is ideal for the simulator. A practical validation plan: (a) match the 1/8-scale soil-bin forces through a scaling law, (b) compare full-scale MPM against Garrity/Reaves/Rubinstein full-scale data if obtainable.

### Gaps
- No US Army CRREL/ERDC, Caterpillar or PWRI blade-force datasets were located within budget. ERDC/GSL reports (for example, the dozer-blade and track-vehicle mobility reports) are likely on erdc-library.erdc.dren.mil and should be searched.
- The cited references for Garrity 1968, Reaves 1969 and Rubinstein 2011 were not fetched.

## Q5. Per-case resolvability summary (8 cm grid / 4 cm particles)

### Takeaway
Directly resolvable and confirmed: Makange 2021 (borderline width), full-scale D3K2 geometry (data only at small scale), and the field sweep dataset (width unconfirmed). Most classic narrow-tine and small-blade lab tests are "too small" or "needs scaling".

### Cited Findings
| Case | Tool | Depth | Speed | Soil / params | Measured | Data | 8 cm-grid verdict |
|---|---|---|---|---|---|---|---|
| Makange 2021 [Sci Rep](https://pmc.ncbi.nlm.nih.gov/articles/PMC8155074/) | 260 mm subsoiler blade, 16° rake | 0.15/0.20/0.30 m | 1-2.5 km/h | clay, 1668 kg/m³, w 11%, DEM c 10 kPa, soil-steel friction 0.55 | Fx, Fz means | Table 4, CC BY 4.0 | borderline width; depths OK; maps to "clay" or "loam" preset |
| D3K2 1/8 & 1/13 blades [JT 2020](https://www.sciencedirect.com/science/article/abs/pii/S0022489819301466), [JT 2024](https://www.sciencedirect.com/science/article/abs/pii/S0022489824000430) | full scale 3.2 x 1.07 m; 1/8 about 0.4 m wide | 20% of H (about 27 mm at 1/8) | n/a | sandy loam; artificial cohesive-frictional soil (2024) | horizontal force, soil mass | paywalled figures | width OK, depth too small; full-scale replica needs scaling-law transfer |
| Field sweep dataset [PMC13584141](https://pmc.ncbi.nlm.nih.gov/articles/PMC13584141) | sweep (width unknown) | unknown | several velocities | clay loam, varying moisture, cone index | draft force + surface profiles | open dataset (licence unconfirmed) | check width |
| Ucgul/Fielke sweeps [Altair abstract](https://community.altair.com/discussion/40774/3d-dem-tillage-simulation-part-2-validation-of-a-hysteretic-spring-plastic-contact-model-for-a-sweep-tooloperating-in-a-cohesionless-soil) | sweep (Fielke 1988) | not retrieved | varied | cohesionless sand | draft, vertical | thesis PDF open | likely needs scaling |
| Coetzee 2014 [CPM](https://link.springer.com/content/pdf/10.1007/s40571-014-0014-7.pdf) | flat blade | two depths | n/a | corn grains, cohesionless | draft + flow | paper figures | likely too small; good MPM methodology |
| Haeri & Skonieczny [arXiv](https://arxiv.org/abs/2111.01523) | robotic blade | 2 cm, 5 cm | n/a | GRC-1 lunar simulant | forces + velocity field | not stated | too small |
| Subsurface dual share [STR 2021](https://www.sciencedirect.com/science/article/abs/pii/S0167198721002749) | sideway shares | 150 mm | 0.25 m/s | soil-bin soil | cutting resistance, disturbance | paywalled | depth OK; width unknown |

### Inferences
- For the dry-sand preset (phi 37°): there is no confirmed open wide-blade dataset. The best route is to digitise figures from Shmulevich 2007 (wide blade in sand) or Fielke/Ucgul, supplemented by McKyes/Hettiaratchi-Reece analytical curves. Treat this as "needs scaling".
- For the moist-sand preset (about 2 kPa apparent cohesion): Ucgul 2015 (moisture and compaction variation) is the closest lead.
- For the loam and clay presets: Makange 2021 (clay, c about 10 kPa) and the clay-loam field sweep dataset.

### Gaps
- Exact Fielke 1988 sweep widths and depths, Shmulevich wide-blade dimensions, and Hettiaratchi-Reece blade sizes were not verified.
