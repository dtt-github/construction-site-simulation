# Small crawler dozer (9–10 t) production reference figures — Komatsu D37PXi-24 / Cat D3–D4 class

Research date: 2026-11-04. Scope: blade capacity, short-distance dozing production, correction factors, speeds, drawbar pull, spreading in 0.3 m lifts, blade-force measurements.

Source-quality legend: **[P]** primary (OEM page or standard); **[D]** OEM dealer page or dealer-hosted spec sheet; **[S]** secondary (lecture slides, used-machine listing, third-party copy of a handbook, OCR scan). Several key primary PDFs could not be retrieved: both ringpower.com Cat Performance Handbook ed. 50 URLs returned 404, cat.com timed out, and the Komatsu brochure PDFs on lectura and archiexpo returned 403. The Gaps sections list what still needs checking against the originals.

---

## 1. Blade capacity (m³) for D37/D39 PAT and Cat D3/D4 blades, and estimating blade load from dimensions

### Takeaway
The D37PX(i)-24 power-angle-tilt (PAT) blade is rated at **2.13 m³ (2.79 yd³)**, about 3.2 m wide, on a 9.3–9.4 t machine. The Cat D3 (2020+) is rated at **2.19 m³ (standard) / 2.34 m³ (LGP)** under SAE, and the D4 VPAT at about **3.3–3.8 m³** under ISO 9246. Field blade load is estimated from a measured pile with **0.375·H·W·L (LCM)**. Komatsu also multiplies rated capacity by a blade factor of 0.4–1.1.

### Cited Findings
- **Komatsu D37PX-24 [P]:** operating weight 9,300 kg (20,503 lb); net power 66 kW (89 hp) at 2,200 rpm (ISO 9249 / SAE J1349); blade capacity 2.13 m³ (2.79 yd³); ground pressure 29.9 kPa (0.30 kg/cm², ISO 16754); standard shoe 600 mm single grouser. The page does not list blade width or height. — [Komatsu US D37PX-24 product page](https://www.komatsu.com/en-us/products/equipment/dozers/small-dozers/d37px-24)
- **D37PXi-24 [S]:** 9.38 t; 600 mm track; 6-way (PAT) blade 3.2 m wide; travel speed 8.5 km/h. — [Lectura-specs D37PXi-24](https://www.lectura-specs.com/en/model/construction-machinery/bulldozers-komatsu/d37pxi-24-11680423)
- **Narrower D37PX option [S]:** the Komatsu Europe D37EX/PX-24 datasheet lists a 1.95 m³ "narrow INPAT blade (PX)" and states that blade capacities follow SAE J1265. This is a search-snippet excerpt; fetching the PDF returned 403. — [Komatsu D37EX/PX-24 datasheet via lectura-specs](https://www.lectura-specs.es/datasheet-downloads/d-37-ex-px-24-en(1f3).pdf)
- **Cat D3 (current, C3.6 engine) [D/S]:** net power 77.6 kW (104 hp) at 2,200 rpm (ISO 9249:2007 / SAE J1349:2011); operating weight 9,362 kg standard, 9,693 kg LGP, 9,893 kg 762 mm LGP; ripper adds 554 kg; SAE blade capacity 2.19 m³ (2.86 yd³) standard and 2.34 m³ (3.06 yd³) wide/LGP; max speed 9 km/h forward and 10 km/h reverse. — [Barloworld D3 page](https://barloworld-equipment.com/new-equipment/106240-d3); [Ring Power D3](https://heavyequipment.ringpower.com/products/detail/106100-d3); [Cat D3 page](https://www.cat.com/en_US/products/new/equipment/dozers/small-dozers/106100.html) (timed out on fetch; values come from search snippets of dealer and spec-sheet copies)
- **Cat D4 (current) [P, snippet]:** VPAT blade 3,196 mm (10.5 ft) wide, LGP option 3,684 mm (12 ft); capacities **3.3 m³ (4.3 yd³)** and **3.8 m³ (4.9 yd³)** under ISO 9246. The mapping of each capacity to a blade is not explicit on the page. — [Cat D4 product page (en_ID)](https://www.cat.com/en_ID/products/new/equipment/dozers/medium-dozers/102980.html). Corroborating listings: 2023 D4-16 VP at 3.81 m³ [S] — [Boom & Bucket](https://www.boomandbucket.com/for-sale/2023-caterpillar-d4-16-vp-bulldozers/a8906014); 2024 D4 XL at 4.3 yd³ [S] — [Boom & Bucket](https://www.boomandbucket.com/for-sale/2024-caterpillar-d4-xl-bulldozers/a0759864)
- **Older Cat D3K LGP [S]:** VPAT blade 1.66 m³; max drawbar pull 122 kN. — [Boom & Bucket 2011 D3K LGP listing](https://www.boomandbucket.com/for-sale/2011-caterpillar-d3klgp-bulldozers/a4905384)
- **SAE J1265 / ISO 9246 scope [P, abstract]:** these standards cover straight, angling, semi-U and U blades. They assume a **flat, vertical blade face** and ignore the blade's included volume. They are "intended for relative comparisons of dozer blade capacity, and not for predicting capacities or productivities in actual field conditions." The J1265 2003 revision is listed as cancelled; ISO 9246 is the equivalent. — [SAE J1265_200305](https://saemobilus.sae.org/standards/j1265_200305-capacity-rating-dozer-blades); [ISO 9246 summary](https://asn.sn/en/node/49414)
- **Measured blade load (Nunnally method, reproduced in KSU CE419 slides) [S]:**
  - Doze a full blade, then lift the blade while moving forward on level ground until an even pile forms.
  - Measure the pile width W (perpendicular to the blade, in line with the inside of each track) and height H, then the length L parallel to the blade.
  - **Blade load (LCY) = 0.0139·H·W·L (ft)** and **Blade load (LCM) = 0.375·H·W·L (m)**. This is Eq. 4-10.
  - The same slides give the methods for average blade load: manufacturer rating, experience, or measuring typical loads.
  - Source: [KSU CE419 Ch. 4.2 slides, pp. 14–17](https://faculty.ksu.edu.sa/sites/default/files/4.CE419-Ch4.2.pdf). The slides follow Nunnally, *Construction Methods and Management*, Ch. 4, by table and example numbering.
- **Komatsu blade factor [S, OCR copy of Komatsu Specifications & Application Handbook Ed. 19, §13A, p. 13A-2]:** production per cycle q = q1 × a, where q1 is the rated blade capacity and a is the blade factor:

  | Dozing condition | Blade factor a |
  |---|---|
  | Easy dozing | 1.1–0.9 |
  | Average dozing | 0.9–0.7 |
  | Rather difficult | 0.7–0.6 |
  | Difficult | 0.6–0.4 |

  Source: [pdfcoffee copy of Komatsu Ed. 19 Productivity](https://pdfcoffee.com/komatsu-edition-19-productivity-pdf-free.html)

### Inferences
- The D37PX 2.13 m³ rating and the D3 2.19 m³ rating are nearly identical. For a 9–10 t class blade model, use **2.1–2.2 m³ loose rated** on a blade about 3.2 m wide.
- The 0.375 coefficient in 0.375·H·W·L is the volume of a triangular-prism-like pile: 0.5 × a pile-shape factor. The flat-face J1265 rating is explicitly not a field prediction. The simulation should therefore compare against **rated capacity × blade factor**: 0.9–1.1 for easy material, which gives roughly 1.9–2.3 m³ per pass on loose fill.
- If a published H is missing, back-solving with a J1265-style C ≈ k·W·H² gives an equivalent blade height. For W = 3.2 m, C = 2.13 m³ and an assumed k ≈ 0.8, H ≈ 0.91 m. This **is not sourced**, because the J1265 formula text could not be retrieved, so verify k before using it.

### Gaps
- D37PXi-24 blade height, max lift, max drop and exact width were not retrieved, because the brochure PDF returned 403. The J1265 / ISO 9246 formula coefficients (straight blade C = 0.8·W·H²?) are unconfirmed and need the standard's text. No D39PX/PXi-24 figures were retrieved.

---

## 2. Cat Performance Handbook production curves and correction factors for small dozers; Komatsu equivalents

### Takeaway
The Caterpillar method is production = max curve value (LCY/h or LCM/h vs. average dozing distance) × the product of correction factors. The D3/D4 sit on the straight-blade curve set. The correction-factor values were recovered only from secondary reproductions, and the numeric curve values for D3/D4 at 15 m and 30 m were **not** retrieved. Komatsu's method is cycle-time based: Q = q·60/Cm·e·E, with explicit speeds and shift times.

### Cited Findings
- **Curve families (Cat Performance Handbook as reproduced in ENCE 420 lecture slides) [S]:**
  - Production estimating curves exist for **straight-blade D3, D4, D5, D7, 814, 824, 834**, with a separate set for universal blades.
  - Production = maximum production from the curve × correction factors. The curve basis is a 60-min hour.
  - The total factor is the product of all applicable factors.
  - Sources: [search summary of ENCE 420 / Assakkaf slides](https://www.scribd.com/doc/234212998/Dozer); [Scribd ENCE 420 copy](https://www.scribd.com/document/488806669/434les02). The Scribd body was not readable on fetch.
- **Correction factors as reproduced from the Cat handbook [S]** (track-type / wheel-type):

  | Factor | Track-type | Wheel-type |
  |---|---|---|
  | Operator, excellent | 1.00 | 1.00 |
  | Operator, average | 0.75 | 0.75 |
  | Operator, poor | 0.60 | 0.50 |
  | Material, loose stockpile | 1.20 | 1.20 |
  | Material, hard to cut / frozen (with tilt cylinder) | 0.80 | 0.75 |
  | Material, dry non-cohesive (used in worked example) | 0.80 | — |
  | Slot dozing | 1.20 | 1.20 |
  | Side-by-side dozing | 1.15–1.25 | 1.15–1.25 |
  | Visibility: dust, rain, snow, fog, darkness | 0.80 | — |
  | Job efficiency, 50 min/h | 0.83 | — |

  Job efficiency is computed as working minutes per hour / 60. Sources: [Scribd 234212998](https://www.scribd.com/doc/234212998/Dozer); [Scribd 608-P2-Lec02-Dozer](https://www.scribd.com/document/505174566/608-P2-Lec02-Dozer); [allrounder.ai lecture transcript](https://allrounder.ai/construction-engineering-management-vol-2/production-value-of-the-bulldozer/next-lecture-on-scrapers-61-lesson-68835f). All are secondary; the visibility 0.80 came from a search snippet, not a fetched page.
- **Worked example in the same deck [S]:** a D11 example uses 600 LCY/h max × 0.75 (operator) × 0.83 (job efficiency) and other factors. It also cites a material weight correction line whose value was truncated. — [Scribd Dozer-hrs](https://www.scribd.com/doc/157927802/Dozer-hrs)
- **Slot dozing [S]:** under favorable conditions it may increase production by **up to 50%**. Blade-to-blade dozing is inefficient over short distances because of the extra maneuvering. — [KSU CE419 slides, pp. 24–26](https://faculty.ksu.edu.sa/sites/default/files/4.CE419-Ch4.2.pdf)
- **Komatsu method [S, OCR of Komatsu Specifications & Application Handbook Ed. 19, §13A, pp. 13A-2/3]:**
  - **Q (m³/h) = q × 60/Cm × e × E**, where q = q1 × a (see the blade factor table in §1), e is the grade factor (chart not extracted) and E is job efficiency.
  - **Cm (min) = D/F + D/R + Z**, where D is haul distance (m), F is forward speed (m/min), R is reverse speed (m/min) and Z is gear-shift time.
  - Speed rule: **forward 3–5 km/h, reverse 5–7 km/h**.
  - **Z = 0.10 min for direct drive and 0.05 min for TORQFLOW** (torque converter).
  - The general job-efficiency table (Good 0.83, Average 0.80, Rather poor 0.75, Poor 0.70) appears on p. 13A-5 in the loader section.
  - Source: [pdfcoffee Komatsu Ed. 19](https://pdfcoffee.com/komatsu-edition-19-productivity-pdf-free.html); [Scribd Komatsu Ed. 19](https://es.scribd.com/document/140175284/KOMATSU-Edition-19-Productivity)
- **Nunnally cycle-time method (KSU slides, Example 4-6) [S]:**
  - Inputs: 7.65 LCM blade; loose common earth; 61 m push; **fixed time 0.05 min** (Table 4-4, power-shift); **dozing speed 4.0 km/h** (Table 4-5); **reverse 8 km/h**; 50 min/h.
  - Cycle time = 0.05 + 0.91 + 0.45 = 1.41 min, so production = 7.65 × 50/60 × 60/1.41 = **271 LCM/h**.
  - Conversion: 1 km/h = 16.7 m/min.
  - Source: [KSU CE419 slides, pp. 19–23](https://faculty.ksu.edu.sa/sites/default/files/4.CE419-Ch4.2.pdf)
- **Model validation [S, abstract only]:** a KSCE J. Civil Eng. paper (2019) validated dozer productivity computation models. It was not fetched. — [Springer s12205-019-0180-5](https://link.springer.com/article/10.1007/s12205-019-0180-5); [KoreaScience](https://www.koreascience.or.kr/article/JAKO201926838277051.page)
- **Handbook status [S]:** Cat Performance Handbook Ed. 50 (2022) is hosted by Ring Power, but both URLs returned 404 on fetch. Its preface says the data is "intended for estimating purposes only". — [ringpower ed50](https://www.ringpower.com/media/2086/caterpillarperfhandbook_ed50.pdf); [WesTrac CPH page](https://www.westrac.com.au/resources/caterpillar-performance-handbook)

### Inferences (derived numbers for the D37PXi, Komatsu formula; not published values)
- Assumptions: q1 = 2.13 m³; a = 0.9 (easy, loose fill); E = 0.83; e = 1 (level); Z = 0.05 min (HST, treated like a torque-converter shift).

  | Speeds F / R (km/h) | D = 15 m: Cm → ideal → corrected | D = 30 m: Cm → ideal → corrected |
  |---|---|---|
  | 3 / 5 | 0.53 min → 241 m³/h → **≈180 LCM/h** | 1.01 min → 127 m³/h → **≈95 LCM/h** |
  | 4 / 7 | 0.404 min → 316 m³/h → **≈236 LCM/h** | 0.757 min → 169 m³/h → **≈126 LCM/h** |

- Applying Cat factors for an average operator (0.75) instead of a "good" one cuts these numbers by a further ~25%. **An envelope of ~70–240 LCM/h at 15–30 m** is a reasonable band for a D37/D3 on loose fill. It should be confirmed against the Cat D3/D4 curve values once the handbook is accessible.
- The Nunnally Example 4-6 scales well: 271 LCM/h for a 7.65 m³ blade at 61 m corresponds to ~35 m³/h per m³ of blade at 61 m.

### Gaps
- **Numeric Cat curve values for D3/D4 (straight or VPAT) at 15 m and 30 m were not obtained**; the handbook PDF was unavailable.
- The Cat grade correction curve (% grade vs. factor) was not obtained.
- The material weight correction is not confirmed. From memory it is believed to be 2300 lb/LCY ÷ actual (≈1370 kg/LCM ÷ actual), but this is **unverified**.
- The Cat "direct drive" factor and the Komatsu grade factor chart are missing, and there is no Komatsu D37-specific production curve.

---

## 3. Spreading fill in lifts (0.3 m), and typical forward/return speeds

### Takeaway
No published m²/h spreading-rate table specific to 9–10 t dozers was found. Speeds are well documented:
- Komatsu recommends forward 3–5 km/h and reverse 5–7 km/h for production estimates.
- Nunnally uses 4.0 km/h dozing loose common earth and 8 km/h return.
- D37PX HST ranges are 0–3.4 / 5.6 / 8.5 km/h forward and 0–4.1 / 6.5 / 8.5 km/h reverse.

### Cited Findings
- **D37PX-24 HST speed ranges [P]:** forward 0–3.4 km/h (1st), 0–5.6 (2nd), 0–8.5 (3rd); reverse 0–4.1, 0–6.5, 0–8.5 km/h. — [Komatsu US D37PX-24](https://www.komatsu.com/en-us/products/equipment/dozers/small-dozers/d37px-24)
- **Cat D3 [D]:** max 9 km/h forward and 10 km/h reverse. — [Ring Power D3](https://heavyequipment.ringpower.com/products/detail/106100-d3)
- **Komatsu estimating speeds [S]:** forward 3–5 km/h and reverse 5–7 km/h. — [Komatsu Ed. 19 §13A p. 13A-3](https://pdfcoffee.com/komatsu-edition-19-productivity-pdf-free.html)
- **Nunnally Table 4-5 example [S]:** dozing 4.0 km/h in loose common earth; return 8 km/h (max reverse, 3rd range); fixed time 0.05 min. — [KSU CE419 slides, p. 23](https://faculty.ksu.edu.sa/sites/default/files/4.CE419-Ch4.2.pdf)

### Inferences
- Spreading rate (m²/h) ≈ loose volume delivered per hour ÷ loose lift thickness:
  - At 0.3 m loose lift, **100 LCM/h → ~333 m²/h** and **200 LCM/h → ~667 m²/h**.
  - If 0.3 m is the *compacted* lift, divide by (0.3 × loose/compacted ratio). For example, with a 1.25 swell/shrink ratio the lift is ≈0.375 m loose, so 100 LCM/h → ~267 m²/h. The ratio is an assumption, not sourced.
- When spreading from dumped truck piles, the dozer works on a **loose stockpile**, so the Cat factor 1.20 applies. Push distances are very short (5–15 m), and fixed or maneuver time dominates.
- A blade 3.2 m wide spreading a 0.3 m lift at 3–4 km/h sweeps 9,600–12,800 m²/h of strip per pass. Production is volume-limited, not area-limited.

### Gaps
- No primary source gave spreading productivity (m²/h at a given lift) for D37 / D3 / D4. Peurifoy/Schexnayder and Nunnally spreading tables were not retrieved.

---

## 4. Drawbar pull vs. speed; usable pull (traction) and blade forces

### Takeaway
Only fragments were found:
- The D3K drawbar-pull chart spans 0–180 kN on its y-axis.
- The older D3K LGP is listed at 122 kN max drawbar pull.
- No D37PXi drawbar figure was retrieved.

For a 9.4 t machine, usable pull is traction-limited (weight × traction coefficient), not engine-limited, at dozing speeds.

### Cited Findings
- **Cat D3K/D4K/D5K drawbar pull chart [S]:** y-axis "Drawbar Pull × 1000 N" spans 0–180, and pull decreases with speed (HST curve). — [manuals.plus D3K/D4K/D5K](https://manuals.plus/m/4193a3749220933504bb23453bc5da4bb36698f04c7a99f517a558d49209c967)
- **D3K LGP [S]:** maximum drawbar pull 122 kN. — [Boom & Bucket listing](https://www.boomandbucket.com/for-sale/2011-caterpillar-d3klgp-bulldozers/a4905384)
- **D37PXi-24 [S]:** drawbar pull is listed on Lectura only behind a paywall; it was not on the Komatsu US page. — [Lectura detailed specs](https://www.lectura-specs.com/en/detailed-specs/d-37-pxi-24-komatsu/42020)
- **Crawler ground pressure (Nunnally) [S]:** 0.4–0.6 bar (6–9 psi), compared with the D37PX's 0.30 bar (29.9 kPa). — [KSU CE419 slides, p. 5](https://faculty.ksu.edu.sa/sites/default/files/4.CE419-Ch4.2.pdf); [Komatsu US](https://www.komatsu.com/en-us/products/equipment/dozers/small-dozers/d37px-24)

### Inferences
- 122 kN on a ~8–9 t D3K equals **≈1.4–1.5 × machine weight**. That exceeds any realistic traction coefficient, so the published "max drawbar pull" is a powertrain stall figure. Simulation should cap usable pull at μ·W.
- Coefficients of traction for steel tracks are commonly tabulated (Cat Performance Handbook, Peurifoy) at roughly 0.9 on firm earth, ~0.6 on loose earth, ~0.3 on dry sand and ~0.45 on wet clay. These values come **from memory and are NOT verified here**. With them, a 9.4 t dozer gives ~83 kN (firm), ~55 kN (loose) and ~28 kN (dry sand).
- Power check: 66 kW at 3 km/h (0.83 m/s) with ~80% transmission efficiency gives ≈63 kN. At dozing speeds of 2–4 km/h, the D37PXi is therefore near both its power limit and its traction limit, at about **50–80 kN horizontal blade force**.

### Gaps
- Official D37PXi-24 and D3 (2020+) drawbar-pull curves were not retrieved; the brochure PDF returned 403 and the Cat page timed out.
- Traction coefficients from the Cat handbook table were not retrieved. Their values above are unverified recollection and must be checked.

---

## 5. Measured dozer blade forces (soil-bin or field)

### Takeaway
No open-access paper with kN blade forces for a 9–10 t dozer was found. The relevant measured datasets are older SAE papers: model-scale blades in four soils, and field draft-power sensing via a strain-gauged push-arm trunnion. All are paywalled.

### Cited Findings
- **Garrity et al., SAE 680612 (1968):** five geometrically similar blades, 6.45–77.2 in (0.16–1.96 m) wide, tested in four soil types. Proposes scaling draft force from model to full size. — [SAE 680612](https://saemobilus.sae.org/content/680612)
- **SwRI Draft Power Sensor (US Bureau of Mines / DOE), US Patent 4,194,574:** measures blade draft force with a strain gauge on the push-arm trunnion and true ground speed by ultrasonic Doppler. Field tested. — [USPTO 4194574](https://image-ppubs.uspto.gov/dirsearch-public/print/downloadPdf/4194574)
- **Hambright & Goris, SAE 760650 (1976):** 42 field tests of the draft-power system, reporting production effects. — [SAE 760650](https://saemobilus.sae.org/papers/draft-power-optimization-improved-bulldozing-production-760650)
- **SAE 790510:** "Maximizing Dozer Work Rate with Draft Power Sensing Systems". — [SAE 790510](https://saemobilus.sae.org/papers/maximizing-dozer-work-rate-draft-power-sensing-systems-790510)
- **Alhimdani et al. (JEASD, ~2009):** a theoretical blade-force calculation method; the worked example uses φ = 32° and c = 8.63 kN/m². It is not a measurement. — [JEASD article](https://jeasd.uomustansiriyah.edu.iq/index.php/jeasd/article/view/1537)

### Inferences
- For simulation validation, the practical check is consistency: the simulated horizontal blade force at full blade should sit between soil-cutting/surcharge models (e.g. the Alhimdani / McKyes-type method) and the traction ceiling μ·W (≈30–85 kN for a 9.4 t dozer per §4, with unverified μ).

### Gaps
- No small-dozer (≈3 m blade) field force time-series was found in open literature. The SAE paper full texts were not accessible, and no recent (2015–2026) DEM/MBD dozer-validation paper with measured forces was retrieved in this pass.
