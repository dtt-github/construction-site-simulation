# Loading and hauling with an excavator and small crawler dump carriers/trucks: reference figures

Compiled 2026-11-04. Primary sources are marked [P], secondary or vendor sources [S]. Units are as published, with SI conversions added in brackets.
Abbreviation: PHB29 = Caterpillar Performance Handbook, Edition 29 (1998). Copy used: http://courses.washington.edu/esrm468/468%20Class%20material/PHB29.pdf. Page numbers are the handbook's own section-page numbers (for example "4-147"). Later editions (Ed. 50, 2022: https://wheelercat.com/wp-content/uploads/2025/03/Cat-Performance-Handbook-Edition-50-June-2022.pdf) were found but not text-searched, so the values below may have been revised since Ed. 29.

## Q1. Passes per truck (pass matching)

### Takeaway
Caterpillar's current guidance is an "ideal" 3-5 passes for wheel loaders. For excavators, 3-5 passes is typical with large excavators; smaller excavators loading articulated trucks take 4-8 passes. A small 10-11 t crawler carrier loaded by a 0.8 m3 bucket needs about 7-9 passes, which is above the Cat ideal band. That figure is derived below, not published.

### Cited Findings
- Cat 2025 "Pass Match" sheet [P], articulated trucks loaded by excavators (passes per truck):
  - Cat 336: 725 = 5, 730/730EJ = 6, 735 = 7, 740EJ/740GC = 8.
  - Cat 350: 725 = 4, 730/730EJ = 5, 735 = 6, 740EJ/GC = 7-8.
  - Cat 352: 4-5 passes (730 class) and 5-6 passes (740 class).
  - Cat 374: 735 = 4, 740EJ/GC = 4-5, 745 = 5.
  - Cat 395: 3-4 passes into 740/745.
  - Source: [Cat Pass Match sheet CM20250318](https://s7d2.scene7.com/is/content/Caterpillar/CM20250318-49a43-6e751)
- Same sheet, wheel loaders into articulated trucks: "Ideal wheel loader pass match: 3-5 passes"; "Ideal wheel loader cycle time: 30-40 seconds". Examples: 950 into 725 = 4-5 passes; 966GC into 725 = 3 passes; 982 into 745 = 4 passes. — [Cat Pass Match sheet](https://s7d2.scene7.com/is/content/Caterpillar/CM20250318-49a43-6e751)
- Same sheet, wheel loaders into on-highway trucks: a 950GC loads 12 t in 2 passes, 20 t in 3-4 passes and 32 t in 5-6 passes. The sheet gives "Target Loader Cycle Time = 30 seconds" and "Target Truck Load Time = 2.5 minutes". — [Cat Pass Match sheet](https://s7d2.scene7.com/is/content/Caterpillar/CM20250318-49a43-6e751)
- Cat article: target 3-5 passes for articulated trucks. A machine that is too small, needing about 10 passes, "wastes time and fuel". — [Cat: Find the Correct Truck Pass Match](https://www.cat.com/en_US/articles/for-owners/find-the-correct-truck-pass-match.html). Only the search snippet was available; the page fetch timed out.
- PHB29 p. 9-7, "Typical Fixed Times for Hauling Units", gives truck load time against number of passes. It assumes the loader spots the hauler with a full bucket, so the first pass costs only the dump time. Columns are 988F wheel loader (0.60 min cycle) and 5130B shovel (0.45 min cycle):

  | Passes | 988F load time (min) | 5130B load time (min) |
  |---|---|---|
  | 1 (first pass, dump only) | 0.10 | 0.05 |
  | 2 | 0.70 | 0.50 |
  | 3 | 1.30 | 0.95 |
  | 4 | 1.90 | 1.40 |
  | 5 | 2.50 | 1.85 |
  | 6 | 3.10 | 2.30 |
  | 7 | 3.70 | 2.75 |
  | 8 | 4.30 | 3.20 |
  | 9 | 4.90 | 3.65 |
  | 10 | 5.40 | 4.10 |

  Implied rule: load time = first-pass dump time + (n - 1) x loader cycle time. — [PHB29 p. 9-7](http://courses.washington.edu/esrm468/468%20Class%20material/PHB29.pdf)
- PHB29 p. 4-147, excavator cycle-time chart. Conditions: above-average job, average operator, 60-90 deg swing, no obstructions. Times are load / swing loaded / dump / swing empty / total, all in minutes:
  - Cat 307, 280 L bucket, 1.5 m depth: 0.08 / 0.05 / 0.03 / 0.06 / **0.22** (13.2 s).
  - Cat 311B-318B, 450-520 L bucket, 1.5-3.0 m depth: **0.21** total (12.6 s).
  - Cat 320B, 800 L bucket, 2.3 m depth: 0.09 / 0.06 / 0.03 / 0.05 / **0.23** (13.8 s).
  - Cat 322B-325B, 1.0-1.1 m3 bucket: **0.25** (15 s).
  - Cat 330B, 1.4 m3 bucket: **0.27** (16.2 s).
  - Cat 350, 1.9 m3 bucket: **0.30** (18 s).
  - Cat 375, 2.8 m3 bucket: **0.34** (20.4 s).
  - The text adds that with the truck "on the floor of the excavation beside material being moved, 10 to 17 second cycles are practical". Condition class D (below average) includes "Swing angle to 90°. Loading trucks with truck spotted close to excavator". — [PHB29 pp. 4-146/4-147](http://courses.washington.edu/esrm468/468%20Class%20material/PHB29.pdf)
- PHB29 excavator loading layout: for maximum production, limit the work zone to 15 deg either side of the machine centreline and position trucks as close to the centreline as possible. — [PHB29, Excavators section](http://courses.washington.edu/esrm468/468%20Class%20material/PHB29.pdf)
- Komatsu case study [S, vendor]: with a 30 s truck exchange, cutting from 4 to 3 passes raised trucks loaded per hour from 18.92 to 23.72 (+25%). — [Komatsu 4800XPC case study](https://komatsu.com/-/media/case-studies/4800xpc_eliminatingoneloadingpass_casestudy_english_en-4800xpcaccs01-0623-v1.ashx). Snippet only, not fully read.
- A field-guide secondary source [S] recommends sizing trucks for 3-4 passes. — [planningengineer.net](https://planningengineer.net/earth-moving-equipment-function-and-standard-productivity/)

### Inferences
- Small carrier with a 0.8 m3 bucket (derived, not published):
  - Assume fill factor 0.9 and loose density about 1.6-1.8 t/m3. Each pass then carries about 0.72 m3, or about 1.15-1.3 t.
  - An 11 t payload (IC120, MST-2200VD, CD110R-3) is reached in about 8.5-9.5 passes by mass.
  - By volume, the CD110R-3's 6.2 m3 heaped body fills in about 8.6 passes.
  - So expect **8-9 passes**, which takes about 1.9-2.2 min at 0.23 min (14 s) per cycle. That load time is close to the Cat "2.5 min target truck load time".
  - With a 1.2 m3 bucket the count falls to about 6 passes.
- A 3.5-3.7 t carrier (Takeuchi TCR50-2, 2.06 m3 body) loaded by a 0.8 m3 bucket takes about 3 passes, which is inside the Cat ideal band.
- For the simulation, a pass count far above about 6 should be modelled as legitimate for small carriers, not as an error. The cost shows up as longer loading time per truck.

### Gaps
- No published Caterpillar or Komatsu pass-match table covers crawler carriers specifically.
- The Komatsu Specifications & Application Handbook was not retrieved.
- Peurifoy/Schexnayder pass-matching text was not retrieved; the book is not openly available online.

## Q2. Truck spotting / exchange times and loader idle

### Takeaway
Caterpillar's handbook figure for truck exchange is 0.6-0.8 min (typically 0.7 min = 42 s), added to the loader cycle. Cat's current "ideal" is an exchange no longer than one loader cycle: 18-24 s for excavators, 30-42 s for wheel loaders. Double-side loading can remove most of the exchange from the excavator's cycle, but I found no clean published "seconds saved" figure.

### Cited Findings
- PHB29 p. 9-7: fixed time for "Truck maneuver in load area (Truck exchange)" is typically 0.6-0.8 min. "Maneuver and dump time at dump point" is typically 1.0-1.2 min. The page adds that minimizing truck exchange time "can have a significant effect on productivity". — [PHB29 p. 9-7](http://courses.washington.edu/esrm468/468%20Class%20material/PHB29.pdf)
- PHB29 excavator production table note: "For estimating truck loading production include approximately 0.7 minutes for truck exchange time". The text says truck exchange "extends the average cycle time and reduces production potential". The same section gives a job-efficiency estimator: 50 min/h = 83%, 45 min/h = 75%. — [PHB29, Excavators production estimating](http://courses.washington.edu/esrm468/468%20Class%20material/PHB29.pdf)
- Cat 2025 Pass Match sheet: "Ideal truck exchange time for excavators and articulated trucks = that of the loader cycle time of 18-24 seconds". For wheel loaders, "Ideal truck exchange time = that of the wheel loader cycle times of 30-42 seconds". — [Cat Pass Match sheet](https://s7d2.scene7.com/is/content/Caterpillar/CM20250318-49a43-6e751)
- Caterpillar Field Guide [S, via secondary copy]: exchange time is measured from a truck's last pass to the next truck's first pass. Good = 0.7 min (42 s), acceptable = 0.9 min (54 s). The loader operator, not the truck driver, should set the truck's position. — [Caterpillar Field Guide (pdfcoffee copy)](https://pdfcoffee.com/caterpillar-field-guide-pdf-free.html). Search-snippet level only.
- Cat spotting advice [S]: place trucks as close as possible to the loader at a 30-45 deg angle to the face. — [Construction Business Owner: Manual to Effective Truck Loading](https://www.constructionbusinessowner.com/equipment/your-manual-effective-truck-loading)
- Measured mining data [S, peer-reviewed but site-specific]: dumping spot time min / mean / max = 10 / 26.6 / 56 s; mean dumping time 52.5 s. — [Cycle Time Analysis of Open Pit Mining Dump Trucks, IJG 2023](https://www.scirp.org/pdf/ijg_2023081610254581.pdf)
- Double-side loading:
  - Single-side loading makes the following truck wait for the preceding truck to finish. — [UTHM IJIE paper](https://publisher.uthm.edu.my/ojs/index.php/ijie/article/download/5730/4550/41150)
  - An Indonesian case study simulated double-side loading. Excavator productivity rose to 507 bcm/h by removing "hang time" from a reference cycle of 27.64 s. The post-change cycle could not be confirmed. — [Trisakti IMEJ paper](https://e-journal.trisakti.ac.id/index.php/imej/article/download/13766/8173/42579)
  - Double-sided loading was found most cost-effective and lowest in emissions among loading policies. — [Emerald, Construction Innovation, CI-05-2017-0047](https://www.emeraldinsight.com/doi/abs/10.1108/CI-05-2017-0047?af=R)
- Track-loader handbook values, relevant to "dump into truck" sub-times: typical dump into highway trucks takes 0.04-0.07 min. Dump-time adjustments are "small target" up to +0.04 min, "fragile target" up to +0.05 min, "independently owned trucks" up to +0.04 min, and "common ownership of trucks and loaders" up to -0.04 min. — [PHB29, Track Loaders cycle time factors](http://courses.washington.edu/esrm468/468%20Class%20material/PHB29.pdf)

### Inferences
- Single-side loading with one excavator (derived):
  - Effective cycle per truck = n x t_c + t_exchange.
  - Example: 8 passes x 14 s plus a 42 s exchange gives 154 s, so the excavator idles about 27% of the loading time even with zero queueing.
  - With a 20 s exchange (Cat ideal for excavators), idle falls to about 15%.
- Double-side loading, where the next truck is spotted during the current loading: the exchange is limited to roughly one swing, about 0.05-0.06 min per PHB29 swing components.
- Crawler carriers: a slewing upper body (CD110R-3, MST-VD rotating models, TCR50-2 180 deg bed) avoids three-point turns, so spotting time should sit at the low end. No published number was found.

### Gaps
- No published excavator waiting/idle fraction specific to construction truck loading was found. Mining papers report site-specific match factors instead (see Q3).
- No authoritative before/after seconds figure for single- vs double-side loading was found.

## Q3. Fleet match factor

### Takeaway
Match factor (MF), coined by Morgan & Peterson (1968), is the truck arrival rate divided by the loader service rate. In practice it is computed as (N_trucks x loading time per truck) / (N_loaders x truck cycle time). MF < 1 means under-trucking and an idle loader. MF > 1 means over-trucking and trucks queueing. MF = 1 is the theoretical balance. Case studies show the cost in both directions.

### Cited Findings
- Definition: MF is "the ratio of truck arrival rate to loader service time". It is used at equipment selection to size the truck fleet and in operations to measure fleet efficiency. The classic form assumes homogeneous fleets; Burt & Caccetta extend it to heterogeneous trucks, heterogeneous loaders and multiple locations. — [Burt & Caccetta, "Match factor extensions" (Studies in Systems, Decision and Control, 2018)](https://espace.curtin.edu.au/handle/20.500.11937/66822?show=full); [Burt 2008 PhD thesis, Curtin](https://curate.curtin.edu.au/articles/An_optimisation_approach_to_materials_handling_in_surface_mines/31461775)
- The term "match factor" is attributed to Morgan & Patterson/Peterson (1968), who reformulated the idea in terms of truck-shovel cycle time. Douglas (1964) is the earlier efficiency source. — [search summary of Burt/Caccetta literature](https://espace.curtin.edu.au/handle/20.500.11937/66822?show=full); [JME Shahrood article 3606](https://jme.shahroodut.ac.ir/article_3606.html) (reference list only)
- Interpretation: MF > 1 indicates redundant trucks; MF < 1 indicates redundant shovels or poor dispatch, with shovels idling. The paper also gives an LCM-based MF formula for heterogeneous fleets. — [OpenMines, arXiv 2404.00622](https://arxiv.org/pdf/2404.00622)
- Indonesian mine case study: the existing fleet had MF = 0.49 with significant shovel idle time. Six trucks gave MF = 0.987; seven trucks gave MF = 1.15 with truck queueing. Over-trucking raises loader utilisation and lowers truck utilisation; under-trucking lowers shovel utilisation and production. — [IIETA paper](https://iieta.org/download/file/fid/130620)
- Sungun copper mine, heterogeneous fleet: MF = 0.74 without and 0.85 with operational-efficiency coefficients. Sub-systems with MF < 1 needed more trucks. — [JME Shahrood article 3606](https://jme.shahroodut.ac.ir/article_3606.html)
- Queueing view: random truck inter-arrival and service times cause either truck queues at the shovel or an idle shovel (Elbrond 1990). Optimal fleet size is chosen by trading shovel idle cost against extra truck cost using cyclic (closed) queueing models. — [Ercelebi & Bascetin, J. SAIMM 109(7), 2009, pp. 433-439](https://www.scielo.org.za/pdf/jsaimm/v109n7/06.pdf)
- PHB29 analogue for scrapers: "Scraper cycle time divided by pusher cycle time indicates the number of scrapers which can be handled by each pusher". Example: 6.95 / 1.23 = 5.65. This is the same balance-number logic. — [PHB29 Mining & Earthmoving section](http://courses.washington.edu/esrm468/468%20Class%20material/PHB29.pdf)

### Inferences
- For the simulation, use:
  - MF = N_t x (n x t_c + t_ex) / (N_L x T_truck)
  - Balance truck count N* = T_truck / (n x t_c + t_ex)
  - Expected loader idle fraction is about 1 - MF for MF < 1 in a deterministic model. With stochastic cycle times both idle and queueing occur near MF = 1, which is why the queueing literature exists.
- Worked short-haul crawler-carrier example (derived): loading 2.0 min (8 x 14 s + exchange), a 100 m haul at about 6 km/h loaded and 9 km/h empty (about 1.0 + 0.7 min), dump plus rotate 0.5 min (assumed). Truck cycle T is about 4.2 min, so N* is about 2.1 carriers per excavator.

### Gaps
- No primary source was retrieved that states a recommended MF band (for example 0.8-1.2, or "slightly below 1"). The Peurifoy/Schexnayder and Caterpillar wording on recommended MF could not be verified online.
- Douglas (1964) efficiency-vs-MF curves were not retrieved.

## Q4. Crawler dump carriers: specifications

### Takeaway
Spec sheets give payload, body volume, travel speed and dump angle. They do **not** publish vessel tip/lower times, haul cycle times or hourly production. The only rotation rate found is the Komatsu CD110R-3 upper-structure slew of 4.7 rpm.

### Cited Findings
- **Komatsu CD110R-3** [P, Komatsu Japan brochure, Japanese]:
  - Payload 11,000 kg; machine mass 16,500 kg; net power 192 kW (261 PS) at 2050 rpm.
  - Travel speed, forward and reverse, high / mid / low: 10.0 / 8.4 / 4.6 km/h (3 speeds).
  - Upper structure slews 360 deg at **4.7 rpm**. Slew radius front/rear 3470 / 3270 mm.
  - Max dump angle 60 deg.
  - Body capacity, standard body: struck 3.7 m3, heaped 6.2 m3. Gated body: struck 4.7 m3; the ISO 6483 2:1 heaped figure would be 7.4 m3, but the brochure says to use the gated body struck only, to avoid overload.
  - Ground pressure 35 kPa.
  - Can dump in any direction over 360 deg; aligns the body to the excavator bucket for rear or side loading.
  - Source: [Komatsu CD110R-3 catalogue PDF](https://kcsj.komatsu/-/media/kcsj/contact/download/pdf/construction_machine/CD110R-3.pdf), spec page 14 and body page.
- **IHI / KATO IC120 and IC120-2**:
  - IC120-2 [P]: max payload 11,000 kg; operating weight 14,100 kg; Cummins QSL9 rated 224 kW at 2,200 rpm. — [KATO Works AU, IC120-2](https://katoworks.com.au/product/ic120-2/)
  - IC120 dealer data [S]: payload 24,250 lb [11.0 t]; transport weight 31,085 lb; travel 5.0 / 6.8 mph [8.0 / 10.9 km/h]; max dump angle 65 deg; dump clearance 3 ft 0 in.
  - The same dealer lists vessel capacity as "7.5 / 10.0 cu ft" struck/heaped. This is almost certainly a units error (probably yd3, about 5.7 / 7.6 m3); **do not use it unverified**. — [KATO CES IC120](https://katoces.com/product/ic120-crawler-carrier/)
- **Morooka MST-2200VD**:
  - Current US model [P]: max payload as dumper 11,000 kg (24,251 lb); carrier 12,880 kg; weight 14,433 kg.
  - Bed 3,607 x 2,440 mm; estimated bed volume 6.9 m3 (9 yd3) at 2,700 lb/yd3.
  - Travel 0-13.7 km/h (0-8.5 mph); Cat C7.1, 250 hp, HST.
  - Ground pressure 24.8 kPa empty / 43.4 kPa loaded.
  - Source: [Morooka Americas MST2200VD](https://morookaamericas.com/products/mst2200vd/)
  - Older spec sheet [P, approx. 2007]: carrying capacity 22,046 lb [10.0 t]; 1st gear 0-5.0 mph [8.0 km/h], 2nd gear 0-7.5 mph [12.1 km/h]; gradeability unloaded 30 deg; turning radius 11 ft 5 in [3.48 m]; dumping angle 60 deg; bed 11 ft 10 in x 8 ft 1 in, side height 1 ft 4 in; machine weight 29,762 lb; ground pressure 3.1 / 5.7 psi. — [MST-2200VD spec sheet (Alaska Excavators)](https://alaskaexcavators.com/wp-content/uploads/2021/04/MST-2200VDSpecs.pdf)
  - European market [S]: max speed 10 km/h. — [Lectura specs](https://www.lectura-specs.com/en/model/construction-machinery/tracked-dumpers-morooka/mst-2200-vd-11704525)
- **Morooka quick chart** [P, same older sheet], load / weight / power:
  - MST300VD/R: 5,000 lb / 5,000 lb / 46 hp
  - MST600VD: 7,260 lb [3.3 t] / 8,800 lb / 76 hp
  - MST800VD: 9,482 lb [4.3 t] / 14,200 lb / 115 hp
  - MST1500VD: 13,892 lb [6.3 t] / 20,282 lb / 204 hp
  - Source: [MST-2200VD spec sheet](https://alaskaexcavators.com/wp-content/uploads/2021/04/MST-2200VDSpecs.pdf)
- **Yanmar C30R-3** [S, dealer copies of Yanmar spec]:
  - Max payload 5,512 lb [2.5 t]; weight 5,843 lb [2.65 t] for the one-way dump type; 46 hp.
  - Travel 0-3.5 / 0-6.8 mph [5.6 / 10.9 km/h], 2-speed HST with auto shift.
  - Dump angle 58 deg (one-way) and 85 deg (swivel). Swivel bed rotates 180 deg.
  - Body 1.62 yd3 struck [1.24 m3] for one-way; 1.96 yd3 heaped [1.50 m3] for swivel.
  - Ground pressure 8.1 psi loaded.
  - One listing claims 6,614 lb and 6.2 mph, which conflicts with the above.
  - Sources: [HM Equipment C30R-3 spec PDF](https://www.hmequipment.com/assets/Uploads/C30R-3-2020-RED-Specs.pdf); [Wellington Implement](https://www.wellingtonimplement.com/new-equipment/yanmar-construction/tracked-carriers/Tracked-Carriers/C30R-3); conflicting [Freightsidekick](https://freightsidekick.com/quote/equipment/YANMAR-C30R-3)
- **Takeuchi TCR50-2** [P]: max load 3,700 kg (8,157 lb); operating weight 6,550 kg; dump body 2.06 m3 (2.68 yd3, struck/heaped not stated); dump angle 65 deg; 180 deg rotating bed. — [Takeuchi US TCR50-2 specs](https://www.takeuchi-us.com/tcr50-2-specs-and-dimensions-2/)

### Inferences
- Slew time for the CD110R-3 at 4.7 rpm (derived): 90 deg takes about 3.2 s and 180 deg about 6.4 s, excluding acceleration.
- Practical haul speeds on rough site tracks will be well below the rated maxima (8-14 km/h). Using about 50-70% of maximum is an engineering assumption, not a published figure.
- In this 11 t class, heaped body volume (6.2-6.9 m3) times typical loose soil density (1.6-1.8 t/m3) is about 10-12 t. Payload is therefore usually mass-limited near a full heap.

### Gaps
- No manufacturer publishes body raise/lower times, haul cycle times or hourly production for crawler carriers. Searches of IHI/KATO, Morooka, Takeuchi and Komatsu pages found none.
- The IHI IC120 operation manual on ManualsLib may list hydraulic timing; it was not retrieved.
- Komatsu CD110R-3 body raise time is not in the brochure.
- An English Komatsu spec sheet was not obtained. Figures above come from the Japanese brochure, translated by me.

## Q5. Dump time for rear-dump trucks/carriers and the effect of sticky material

### Takeaway
Caterpillar budgets 1.0-1.2 min for "maneuver and dump" at the dump point for haul trucks. One mine time study measured 52.5 s mean dumping plus 26.6 s spotting. Sticky material (carry-back) is documented mainly by vendors: carry-back occurred in more than 60% of cycles in one articulated-truck study, and claims of up to 50% of body volume circulate. There is no independent standard payload-loss percentage.

### Cited Findings
- PHB29 p. 9-7: maneuver and dump at dump point is typically 1.0-1.2 min (hauling units). — [PHB29 p. 9-7](http://courses.washington.edu/esrm468/468%20Class%20material/PHB29.pdf)
- PHB29 scraper/haul production examples use "maneuver and dump 0.5 min" for the units in that example (articulated/forestry haul context, section 23 example). The time-study example lists "Average dump time = 0.50 min". — [PHB29 section 23](http://courses.washington.edu/esrm468/468%20Class%20material/PHB29.pdf)
- Open-pit truck time study: mean dumping time 52.5 s; dumping spot time 10 / 26.6 / 56 s (min/mean/max). — [IJG 2023, Cycle Time Analysis of Open Pit Mining Dump Trucks](https://www.scirp.org/pdf/ijg_2023081610254581.pdf)
- Carry-back:
  - Loadscan study of four articulated trucks [S, vendor]: carry-back in more than 60% of haul cycles; more than 980 m3 of payload lost over the study period; one 65 t truck averaged just under 54 t payload. — [ForConstructionPros / Philippi-Hagenbuch article](https://www.forconstructionpros.com/equipment/fleet-maintenance/article/22751263/philippihagenbuch-inc-reducing-equipment-carryback-and-improving-hauling-efficiency); [Pit & Quarry: seeking solutions to combat carryback](https://www.pitandquarry.com/seeking-solutions-to-combat-carryback/)
  - Philippi-Hagenbuch [S, vendor]: operations often accumulate up to 50% of truck body volume in carry-back. Clean-out takes operators away from production for 20 min or more per truck. — [ForConstructionPros](https://www.forconstructionpros.com/equipment/fleet-maintenance/article/22751263/philippihagenbuch-inc-reducing-equipment-carryback-and-improving-hauling-efficiency)
  - BHP WAIO [S]: carry-back reduced by 54% after a hoist redesign allowed a higher tip angle. — [Equipment Journal: tackling haul truck carryback](https://www.equipmentjournal.com/on-the-job/solutions-for-sticky-situations-tackling-haul-truck-carryback/)
- An online calculator [S, low quality] states truck dump time is 2-4 min. This conflicts with the Cat and time-study values, so do not use it. — [Quantum Land Design haul calculator](https://www.quantumlanddesign.com/tools/haul-count-calculator)

### Inferences
- For small crawler carriers, dump time should be shorter than for haul trucks. The body is small, tip angles are 58-85 deg, and no reversing to the tip edge is needed when the body slews. A dump-plus-reposition of about 0.3-0.6 min is a reasonable sim prior, but it is an assumption, not a published figure.
- Model carry-back as a fractional retained volume per cycle. Plausible ranges from vendor data run from a few % up to tens of % in severe clay. It reduces effective payload and adds occasional clean-out events of about 20 min.

### Gaps
- No published dump-cycle times exist for crawler carriers.
- No peer-reviewed quantification links carry-back percentage to material plasticity or moisture.
- The PHB favourable/average/unfavourable spot and dump fixed-time table (bottom vs end dump) is reproduced only in garbled homework copies (Chegg) and could not be verified, so it is omitted.
