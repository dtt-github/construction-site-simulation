# Field Measurements and Validation Studies of Excavator Digging/Loading Cycles (15-25 t focus)

Scope note: Few openly accessible primary sources give per-phase, field-measured cycle times for a 20 t class machine. The best primary numbers I could verify in full text come from a 6.5 t / 49 t autonomous-excavator study (Baidu AES), a 13.4 t IMU-instrumented machine (KIT/Tampere, Molaei), and a Korean tunnel muck-loading CCTV study. Machine size is stated for every figure. Research date 2026-11-04. Paywalled items (Science Robotics full text, Automation in Construction articles on ScienceDirect, ResearchGate) could not be opened. Their content is listed under Gaps and not used as data.

## Q1. Measured cycle times (total and per phase) and the effect of operator skill, swing angle and soil

### Takeaway
The only verified four-phase field breakdown is from Baidu AES on a truck-loading task. A human expert took 23.6 s per loop (scoop 5.8 / swing-to-truck 6.2 / dump 5.8 / swing-back 5.7 s) and the autonomous system took 25.0 s, with 86% vs 105% bucket fill. The machine was a test excavator; the paper's platforms are 6.5 t and 49 t. IMU field data on a 13.4 t Komatsu PC138US puts loading cycles at roughly 12-24 s at 60-120 deg swing, with experienced operators measurably faster. The industry BML model gives swing-angle sensitivity of about ±10% over 45-180 deg.

### Cited Findings
**Baidu/UMD Autonomous Excavator System (AES).** arXiv 2011.04848 (2020), the preprint of Zhang et al., "An autonomous excavator system for material loading tasks", Science Robotics 6(55), 30 June 2021, DOI 10.1126/scirobotics.abc3164.
- Table 2, Scenario 1 (material loading and dumping into a dump truck, material free of impurities). Average time per loading loop and bucket fill rate:
  - AES: scoop 5.5 s, swing to truck 8.5 s, dump 5.4 s, swing back 5.7 s, total 25.0 s, bucket fill rate 105%.
  - Human operator: scoop 5.8 s, swing to truck 6.2 s, dump 5.8 s, swing back 5.7 s, total 23.6 s, bucket fill rate 86%.
  - Per the authors, AES takes about 6% more time per loop but handles about the same material per hour as an experienced human operator. — [arXiv 2011.04848](https://arxiv.org/pdf/2011.04848)
  - Table 2 does not name the machine. The hardware section lists "a 6.5-ton compact excavator and ... a 49-ton standard excavator". — [arXiv 2011.04848](https://arxiv.org/pdf/2011.04848)
- Waste-disposal/recycling application. 145 excavation-and-dump operations per hour and "as much as 36.25 m3 per hour for the 6.5-ton excavator", run continuously for 24 h. — [arXiv 2011.04848](https://arxiv.org/pdf/2011.04848)
- The Science Robotics press coverage gives 67.1 m3/h for a compact excavator, "in line with" a human operator. It lists test machines of 6.5 t and 7.5 t compact, 33.5 t standard and 49 t large. — [Techxplore](https://techxplore.com/news/2021-06-autonomous-excavators-ready-clock-real-world.pdf); [Construction Equipment](https://www.constructionequipment.com/baidu); [KHL](https://www.khl.com/news/Robot-excavator-operates-alone-for-24-hours/8013551.article)
  - Conflict: the preprint says 36.25 m3/h and the published-paper coverage says 67.1 m3/h. The two may refer to different machines or material. I could not open the final paper to reconcile them.

**Molaei (KIT Dr.-Ing. dissertation, 2025), "Work performance evaluation of heavy-duty mobile machines (HDMMs)".** This dissertation contains the Automation in Construction 156 (2023) 105080 paper, "Automatic estimation of excavator actual and relative cycle times in loading operations".
- Machine: Komatsu PC138US, 13.4 t, mono boom, Novatron Xsite machine control, tiltrotator unused. Heaped bucket 0.37 m3 (SAE J296). Four IMUs on bucket (side link), arm, boom and swing body, sampled at 200 Hz over CAN. — [Molaei 2025, Sec. 4.2](https://d-nb.info/1374860573/34)
- Conditions: sand, gravel, clay and mixed material; digging depth up to 2 m; swing 60-120 deg; recorded across seasons over 18 months by two operators (one inexperienced, who did 47% of the experiments). — [Molaei 2025, Sec. 4.2](https://d-nb.info/1374860573/34)
- Loading case studies (Table 5.5), with material on the ground surface (depth about 0):
  - Inexp. 1: 120 deg, sand, 5.3 min
  - Exp. 1: 120 deg, sand, 5.2 min
  - Inexp. 2: 60 deg, gravel, 5.4 min
  - Exp. 2: 60 deg, gravel, 4.4 min
  - 66 cycles in total, with ground truth taken from video. — [Molaei 2025, Table 5.5](https://d-nb.info/1374860573/34)
- Results:
  - Plotted per-cycle times in Figs. 5.6-5.7 fall on a 12-24 s axis.
  - The experienced operator's cycle time is lower in both case studies.
  - IMU-estimated cycle-time accuracy (Table 5.6): 0.981, 0.978, 0.964 and 0.972. Error is under 1.5 s, and only 5 of 66 cycles exceed 1 s error.
  - Relative cycle time (theoretical BML ÷ actual): the experienced operator stays above 0.8 in nearly all cycles; the inexperienced operator is mostly 0.6-0.8. — [Molaei 2025, Sec. 5.3.3-5.3.6](https://d-nb.info/1374860573/34)
- The thesis also cites the Komatsu handbook: excavator cycle time "mostly within the range of [10, 20] s". — [Molaei 2025, Sec. 4.2.2](https://d-nb.info/1374860573/34)

**BML model (German construction-industry guideline), as used by Molaei, Eqs. 5.2-5.7.** The model gives a theoretical loading cycle time.
- Core formula: t = t_initial / (f_swing × f_depth).
- t_initial for high-excavability material (sand, gravel): -0.50·V² + 4.19·V + 13.13 [s], where V is the CECE heaped capacity in m3.
- t_initial for medium/low excavability: -0.07·V² + 3.30·V + 15.52 [s].
- Swing factor: f_swing ≈ 1.754·θ^-0.1258 for θ between 45 and 180 deg. Over that range, swing angle changes cycle time by about ±10%.
- Depth factor: f_depth ≈ h^-0.1164 for high/medium excavability and h^-0.1039 for low/very-low excavability, for h ≥ 1 m.
- The model gives no estimate for very-low-excavability material such as blasted rock.
- Molaei judges the Komatsu handbook model "more optimistic" than BML.
- Bucket-capacity conversion: V_SAE ≈ 1.10-1.20 × V_CECE. — [Molaei 2025, Sec. 5.2.3, Table 5.1](https://d-nb.info/1374860573/34)

**Planning and spec figures.** These are not field-measured; use them only as context.
- Komatsu PC200: cycle time 15 s, about 26 m3/h, from a Nepal municipal production-rate appendix (a planning calculation). — [indrasarowarmun.gov.np appendix](https://www.indrasarowarmun.gov.np/sites/indrasarowarmun.gov.np/files/appendix-1.pdf)
- A John Deere representative (trade press): "with a 20-ton machine ... a 9- to 10-second cycle time if you are doing things right". — [ForConstructionPros](https://www.forconstructionpros.com/equipment/earthmoving-compact/article/10212146/dig-into-excavator-productivity)
- Trade guides, not peer-reviewed: a 180 deg swing gives cycles 40-50% longer than a 90 deg swing; cutting the swing from 180 to 90 deg cuts the cycle about 20%. — [ForConstructionPros](https://www.forconstructionpros.com/equipment/earthmoving-compact/article/10212146/dig-into-excavator-productivity); [Almarwan](https://almarwan.com/news/4872/maximize-excavator-cycle-times)
  - These conflict with the BML ±10% figure above.
- Phase shares quoted from the mining literature (hydraulic excavator loading trucks): bucket fill 41-42%, swing loaded 25%, swing empty 24%, dump 9%. Loading a truck takes 3-6 passes.
  - Source: J. Phys. Conf. Ser. 2054 012086 (2021), Balaraaman & Chandrasekaran, quoting an open-pit study. This paper was RETRACTED by IOP on 27 Aug 2026 for "tortured phrases"; use only as a pointer to the original open-pit study. — [IOP (retracted)](https://iopscience.iop.org/article/10.1088/1742-6596/2054/1/012086/pdf)
  - The original appears to be "Investigation of Excavator Performance Factors in an Open-Pit Mine Using Loading Cycle Time" (Kobelco excavator, Toyota trucks). Its snippet gives a mean loading-tool cycle of 0.54 min (32.4 s) and a mean haul cycle of 14.58 min. — [ResearchGate](https://www.researchgate.net/publication/318497677_Investigation_of_Excavator_Performance_Factors_in_an_Open-Pit_Mine_Using_Loading_Cycle_Time) (403, abstract snippet only)
- A forum anecdote gives 0.23 min (13.8 s) for a Cat 320C L. Unsourced. — [HeavyEquipmentForums](https://www.heavyequipmentforums.com/threads/deere-200c-lc-cycle-time.2391/)

### Inferences
- AES human-operator phase shares, from 23.6 s total: scoop 24.6%, swing loaded 26.3%, dump 24.6%, swing back 24.2%.
  - Dump is far larger here than the 9% mining share. This suggests careful truck-bed placement and/or a small machine.
  - A simulator should not hard-code a 9% dump share for construction truck loading.
- AES waste application: 3600/145 = 24.8 s per cycle, and 36.25/145 ≈ 0.25 m3 per cycle on the 6.5 t machine. This matches the 25.0 s autonomous loop in Table 2.
- BML applied to a typical 20 t machine with a CECE heaped bucket of about 1.0 m3 (my calculation from Molaei's equations):
  - Sand/gravel at 90 deg and 1 m depth: about 16.8 s.
  - At 45 deg: about 15.5 s. At 180 deg: about 18.4 s.
  - At 2 m depth: about 8% longer.
  - Medium/low excavability, 1 m3: about 18.8 s.
  - These are conservative "theoretical" times. Experienced operators reached relative times above 0.8 against BML for the 13.4 t machine, i.e. actual cycles roughly equal to or within ~25% above BML.
- Molaei case studies: about 16-18 cycles in 4.4-5.4 min implies roughly 16-20 s per cycle for a 13.4 t, 0.37 m3 machine at 60-120 deg. This is approximate, read from figure axes and durations, and assumes continuous cycling.
- A reasonable 20 t truck-loading target band for simulator validation is about 14-22 s per cycle (the 15 s planning figure, the 16-18 s BML values, and the 13.8-24 s observations above). The 9-10 s vendor claim is likely an ideal 90 deg trenching-to-pile cycle.

### Gaps
- I found no openly accessible peer-reviewed field time study that gives per-phase times for a 20-23 t machine (PC200/210, CAT 320, EC220, DX225) loading trucks. Likely sources, not verified:
  - Korean CCTV/vision studies (Kim H. et al.; Kim J., Chi S., "Action recognition of earthmoving excavators based on sequential pattern analysis", AiC 2019)
  - Japanese MLIT/i-Construction productivity surveys
  - Purdue/Georgia Tech time-motion theses
- Koiwai et al. (J. Robotics Networking and Artificial Life, 2017) on novice vs skilled digging, and University of Tokyo (Asama lab) studies of skilled-operator trajectories, were found but not opened. I have no cycle numbers from them.
- I could not open the Science Robotics 2021 final paper or its supplementary tables. Per-machine-size data (33.5 t and 49 t) and the reason for the 36.25 vs 67.1 m3/h difference are unverified.
- ETH/HEAP and Built Robotics have no published cycle-time vs human benchmarks in m3/h that I could find. Built Robotics markets a trenching rate of up to 180 ft/h vs 130-180 ft/h for manual operation (vendor claim, linear feet). — [Built Robotics](https://www.builtrobotics.com/solutions/trenching)
- The Kobelco open-pit study's machine size was not readable (403).

## Q2. Measured bucket fill and payload per pass in different soils

### Takeaway
Measured bucket-fill data at 20 t scale is very scarce in open sources. The only verified human-vs-machine measurement is AES: 86% fill for the human operator vs 105% for the autonomous system. Onboard weighing systems (Cat Payload, Trimble Loadrite X2350) exist, but I found no published per-pass field datasets from them.

### Cited Findings
- AES Table 2: bucket fill rate (percentage of bucket loaded) was 86% for the human operator and 105% for AES in clean-material truck loading. — [arXiv 2011.04848](https://arxiv.org/pdf/2011.04848)
- AES waste application: 36.25 m3/h at 145 cycles/h on a 6.5 t excavator, which implies about 0.25 m3 per pass (derived). — [arXiv 2011.04848](https://arxiv.org/pdf/2011.04848)
- Bucket-capacity definitions: the SAE (1:1 heap) vs CECE (1:2 heap) conversion is V_SAE ≈ 1.10-1.20 × V_CECE. Molaei's 13.4 t machine has a 0.37 m3 SAE heaped bucket. — [Molaei 2025, Sec. 4.2.1](https://d-nb.info/1374860573/34)
- Onboard weighing products:
  - Cat Payload shows per-bucket payload and the truck's cumulative payload, with overload alerts. — [Cat](https://www.cat.com/en_ID/products/new/technology/payload/payload/15969808.html)
  - Trimble Loadrite X2350 weighs each bucket, tracks load-out tonnage and monitors cycle times. — [Equipment World](https://www.equipmentworld.com/construction-equipment/article/14952371/trimbles-loadrite-x2350-onboard-weighing-system-boosts-excavator-efficiency)
  - Neither publishes field accuracy or fill distributions.
- Gawlik & Kucybała (2017) built a dynamic weighing system for a Caterpillar 301.5 (about 1.5 t mini excavator), tested on a test stand rather than in the field. They report good agreement with their model; no payload statistics. — [Gawlik, Dynamic weighing system used in excavator](https://yadda.icm.edu.pl/baztech/element/bwmeta1.element.baztech-b9f3a653-52fc-44c2-b46d-c8fa3011f84e/c/gawlik_Dynamic_Weighing_System_Used.pdf)
- Passes per truck: 3-6 passes is quoted as the optimal match range (retracted IOP paper, citing the mining literature). — [IOP (retracted)](https://iopscience.iop.org/article/10.1088/1742-6596/2054/1/012086/pdf)
- Korean tunnel site: dump-truck payload 17 m3 (Kim et al., ISARC 2018). The excavator bucket size was not given. — [ISARC 2018 Paper 160](https://www.iaarc.org/publications/fulltext/ISARC2018-Paper160.pdf)
- Korean multi-site study: truck capacity 12.55 CY (about 9.6 m3). — [Kim Y. et al., ICCEPM 2022](https://koreascience.or.kr/article/CFKO202221543224741.pdf)

### Inferences
- A human fill factor of about 0.86 in clean, free-flowing material sits at the low end of handbook fill factors. Handbook values for loose sand and gravel are often around 0.95-1.1; that range is not verified in this note. A simulator calibrated to handbook fill factors may overpredict human payloads by 10-20%.
- A 20 t excavator with a 1.0-1.2 m3 SAE bucket at 0.86-1.0 fill loads about 0.9-1.2 m3 per pass (derived). That means about 8-11 passes for a 9.6 m3 (12.55 CY) truck and about 14-19 passes for a 17 m3 truck. This is far above the 3-6 pass mining guidance, which assumes matched large loaders.

### Gaps
- I found no open per-pass weighed payload distributions (mean/SD in t) for 15-25 t excavators in clay, sand, gravel or blasted rock. Vendor case studies (Trimble/Loadrite, Cat Payload, RMT) give no numeric fill statistics.
- I found no source on how fill factor depends on soil type at 20 t scale.

## Q3. Measured utilisation, waiting for trucks and productivity losses on real sites

### Takeaway
The best verified field number comes from a Korean NATM tunnel. One excavator served by 7 dump trucks was idle 43.97% of the time; simulation showed 10 trucks would cut idle to 21.12% and raise output from 100.88 to 141.78 m3/h. Telematics-based "idle ~38-40% of engine hours" figures come from vendors and are only indicative.

### Cited Findings
**Kim H., Bang, Jeong, Ham, Kim H. (2018), "Integration of Imaging and Simulation for Earthmoving Productivity Analysis", ISARC 2018 (Yonsei / Texas A&M).**
- Site: NATM tunnel, 680 m3/day of muck, temporary storage 1,500 m3. One excavator loads 7 dump trucks of 17 m3 each.
- Data: 8 h 11 min of CCTV video, with R-FCN detection at 99.09% mAP.
- Context-reasoning errors: 0% for the number of cycles, 1.6% for average loading duration, 0.12% for average hauling duration.
- The WebCYCLONE simulation was within 2.23% of the actual process over 44 task cycles.
- Current productivity was 100.88 m3/h at 5.59 $/m3, with excavator idle time of 43.97%.
- With 10 trucks: 141.78 m3/h at 5.45 $/m3, excavator idle reduced to 21.12%, saving $310,858 (12.25%) over 384 working days. — [ISARC 2018 Paper 160](https://www.iaarc.org/publications/fulltext/ISARC2018-Paper160.pdf)

**Molaei dataset (13.4 t PC138US), Table 4.1.** This is a 3 h labelled dataset (not a production site): loading 68.43 min, trenching 41.14 min, grading 35.26 min, idling 37.27 min, so about 20% idling. — [Molaei 2025, Table 4.1](https://d-nb.info/1374860573/34)

**Telematics and industry figures (vendor sources, unverified methods):**
- Average construction-equipment idle is about 38-40% of engine hours, from "manufacturer telematics ... ~75,000 machines" in North America. Individual machines range from under 20% to over 50%. — [FleetRabbit](https://fleetrabbit.com/industry/construction-management-system/construction-fleet-reducing-equipment-idle-job-sites)
- 10-30% of fuel is burned in non-productive idle (unsourced vendor claim). — [OxMaint](https://oxmaint.com/blog/post/cement-construction-jobsite-equipment-utilization)
- A researcher quoted by iSTART found about a 15% difference between manually recorded and machine-generated productivity data, and equipment idle "up to 50%" of the time. — [iSTART](https://istart.com.au/news-items/idle-data-idle-machines-the-utilisation-tech-blind-spot/)
- In a mining shovel-truck DES study in FlexSim, the shovel's mean cycle was 3.44 min (large mining shovel, not 20 t). The shovel sat idle because of a truck shortage. — [search result, Noppadon thesis ch.7](https://cuir.car.chula.ac.th/bitstream/123456789/66013/8/Noppadon_jo_ch7_p.pdf) (not opened in full)

### Inferences
- In single-excavator, multi-truck construction loading, the measured excavator idle share of 21-44% is driven mostly by truck availability. A simulator's queueing model (truck count, haul time) matters at least as much as dig physics for hourly production.
- 100.88 m3/h at 44% idle implies about 180 m3/h of loading capacity while actually loading (derived). The real tunnel excavator size is unknown.

### Gaps
- No peer-reviewed telematics study with machine-class breakdown (e.g. Komtrax/VisionLink fleet statistics for 20 t excavators) was accessible.
- Classic accelerometer studies of working vs idle time (e.g. Ahn, Lee, Peña-Mora, J. Comput. Civ. Eng. 2015) were not retrieved, so I have no numbers from them.

## Q4. Validation studies comparing a simulator or digital twin against real productivity

### Takeaway
Published validations are mostly discrete-event (CYCLONE-type) operation models checked against field cycle data, with errors of about 2-8%. I found no openly accessible study validating a physics-based (soil-tool) excavator simulator's cycle time or m3/h against a field machine.

### Cited Findings
- Kim H. et al., ISARC 2018: WebCYCLONE driven by CCTV-derived inputs matched the actual tunnel earthmoving process within 2.23% over 44 cycles. — [ISARC 2018 Paper 160](https://www.iaarc.org/publications/fulltext/ISARC2018-Paper160.pdf)
- Kim Y., Noh, Ko, Lee, Han (Inha Univ.), ICCEPM 2022, "Construction performance assessment framework by means of construction simulation for earthwork operations".
  - 15 conditions from 5 Korean sites; event times taken from site data.
  - WebCYCLONE vs RSMeans: average error 7.4% (range 2.1-20.82%), with 4 of 15 cases at errors up to 20.8%.
  - Productivity converged at 15 trucks.
  - Note: the comparison is against RSMeans deterministic rates, not measured output. — [ICCEPM 2022](https://koreascience.or.kr/article/CFKO202221543224741.pdf)
- Molaei 2023/2025: the BML theoretical model was used as a "digital reference" against IMU-measured actual cycles. Relative cycle time was above 0.8 for the experienced operator and 0.6-0.8 for the inexperienced one (13.4 t). — [Molaei 2025](https://d-nb.info/1374860573/34)
- A 2026 discrete-event model (Arena) of a hydraulic excavator was validated against case-study data for four operators. Production ranged 55.4-78.2 t/min (search-result summary only; not opened, machine size unknown). — [search result: eijest](https://eijest.journals.ekb.eg/article_96711_4684d17f855c5ef063da3d81991ba3b0.pdf)
- A fuzzy cycle-time model was built from 70 observed cycle times across four manufacturers (Univ. of Johannesburg). — [UJ Pure](https://pure.uj.ac.za/en/publications/a-computational-intelligent-fuzzy-model-approach-for-excavator-cy/)

### Inferences
- For a physics-based digital twin, the practical validation targets available openly are:
  1. AES Table 2 per-phase times and fill.
  2. BML/Komatsu theoretical cycle formulas.
  3. Molaei's 13.4 t IMU cycles.
  4. Tunnel-site m3/h and idle fractions for the queueing layer.

### Gaps
- No open study compares a terramechanics or physics excavator simulator (Chrono, Vortex, AGX, Newton) with measured field cycle times or m3/h for a 15-25 t machine.
- I did not find Vortex/CM Labs or Komatsu/Kobelco digital-twin validation papers with numbers.

## Q5. Open datasets of excavator cycles

### Takeaway
I found no fully open, cycle-annotated 20 t truck-loading dataset on Zenodo or GitHub. The nearest options are video datasets from AiC 2019 and the AES vision dataset, plus Molaei's IMU dataset from a 13.4 t machine. None was confirmed as publicly downloadable.

### Cited Findings
- UIUC (Roberts & Golparvar-Fard, Automation in Construction 2019): 10 ground-level videos of interacting excavator and dump-truck pairs. Per-frame bounding boxes, identities and activity labels. — [search summary via catalyzex/AiC](https://www.catalyzex.com/paper/vision-based-excavator-activity-analysis-and)
- AES vision dataset (arXiv 2110.03083, Baidu): 102 excavator video clips with 3 actions (digging, dumping, swinging), used for activity and productivity analysis. — [arXiv 2110.03083](https://arxiv.org/pdf/2110.03083)
- A related video study reports 87.6% activity-recognition accuracy and 83% productivity-calculation accuracy (AiC 2019, Chen et al., "Automated excavators activity recognition and productivity analysis from construction site surveillance videos"). — [ScienceDirect](https://www.sciencedirect.com/science/article/abs/pii/S0926580519303905)
- Molaei IMU dataset: Komatsu PC138US (13.4 t), 4 IMUs at 200 Hz, about 3 h covering loading, trenching, grading and idling (Table 4.1). Sand, gravel, clay and mixed material; two operators; 18 months. A second loading set is about 75 min, with the inexperienced operator doing about 35%. Prior literature cited in the thesis gives cycle-time estimation accuracy of 75.96% and 91.83%. — [Molaei 2025](https://d-nb.info/1374860573/34); related Construction Robotics 2024 paper on working-cycle classification — [Springer s41693-024-00130-0](https://link.springer.com/article/10.1007/s41693-024-00130-0)
- Tampere/Trepo theses list on classifying excavator motions in long-term activities. — [Trepo 10024/138710](https://trepo.tuni.fi/handle/10024/138710)

### Inferences
- For cycle-level benchmarking, the most useful openly described data is Molaei's IMU set. It has joint angles plus swing at 200 Hz and is directly comparable with simulator joint trajectories, but it covers a 13.4 t machine and would need to be requested from the authors.

### Gaps
- No public download link was confirmed for the UIUC, AES vision or Molaei datasets.
- I found no open telematics dataset (Komtrax/ISO 15143-3 AEMP feeds) with cycle annotations.
