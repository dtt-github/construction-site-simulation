# Excavator cycle times, bucket fill factors and hourly production: 20 t class (ZX200 / PC200 / Cat 320 class, about 0.8 m3 heaped bucket) loading trucks

Scope note: I read three primary documents in full at the relevant pages: the Caterpillar Performance Handbook Ed. 29 (Oct 1998), the Komatsu Specifications & Application Handbook Ed. 27 (Aug 2006), and the KSU CE419 lecture that reproduces textbook tables. I could not open newer Cat editions (44 to 50); the ringpower.com Ed. 50 PDF returned HTTP 404. I found no Hitachi, Volvo or Liebherr production-estimating tables. Unit conventions: Lm3 = loose m3, Bm3 = bank m3, LCM = loose cubic metre. "Heaped" means SAE J296 / ISO 7451 heaped capacity at a 1:1 heap for hydraulic backhoes (lecture Table 3-1).

Sources used:
- **[CAT29]** Caterpillar Performance Handbook, Edition 29, October 1998 (PDF created 1998-10-06), Section 4 "Excavators", pp. 4-119 to 4-153, and the "Tables" section "Weight of Materials", p. 28-4/28-5. The copy is hosted by the University of Washington: http://courses.washington.edu/esrm468/468%20Class%20material/PHB29.pdf (PDF pages 246, 273 to 278, 984).
- **[KOM27]** Komatsu Specifications & Application Handbook, Edition 27, August 2006, Section 15A "Productivity", pp. 15A-2 to 15A-11. The copy on pdfcoffee is text-extracted: https://pdfcoffee.com/handbook-komatsu-edition27-pdf-free.html
- **[KSU]** King Saud University CE419 lecture "Ch. 3 Excavating and Lifting, Part 1". It reproduces textbook Tables 3-1 to 3-6 as scanned images. Based on the chapter and table numbering and Example 3-2, these are almost certainly Nunnally, *Construction Methods and Management*, Ch. 3, but the slides do not name the book, so treat this as a **secondary source**. https://faculty.ksu.edu.sa/sites/default/files/3.CE419-Ch3.1.pdf

---

## Q1. Manufacturer cycle times for 20 t class excavators (by digging difficulty, swing angle and depth)

### Takeaway
- **Komatsu PC200/PC210 standard cycle time:** 13 to 16 s at 45 to 90 deg swing and 16 to 19 s at 90 to 180 deg swing. This is multiplied by a depth/dump-condition factor of 0.7 to 1.8.
- **Cat 320B "typical":** 0.23 min (13.8 s) total at 60 to 90 deg swing, 2.3 m depth, above-average conditions and an average operator. Cat's chart band for the 320B runs from about 15 s (fastest practical) to about 26 s (severe); I read these values off a shaded bar graphic.
- For truck loading, Cat says to add about 0.7 min of truck exchange time per truck.

### Cited Findings

**Komatsu (KOM27, p. 15A-9/15A-10, Table 11 "Standard cycle time for backhoe", unit: s)**
- **Cycle definition:** Cycle time = excavating time + swing time (loaded) + dumping time + swing time (empty). The handbook computes it as Cm = (standard cycle time) x (conversion factor). — [KOM27](https://pdfcoffee.com/handbook-komatsu-edition27-pdf-free.html)
- **Table 11 values** (my reading of the extracted text, which lists 9 models against 9 value pairs in order):

  | Model | Swing 45 to 90 deg (s) | Swing 90 to 180 deg (s) |
  |---|---|---|
  | PC78 | 10 to 13 | 13 to 16 |
  | PC120/130 and PC160 | 11 to 14 | 14 to 17 |
  | **PC200/PC210** | **13 to 16** | **16 to 19** |
  | PC220/230 | 14 to 17 | 17 to 20 |
  | PC300/350 | 15 to 18 | 18 to 21 |
  | PC400/450 | 16 to 19 | 19 to 22 |

  — [KOM27](https://pdfcoffee.com/handbook-komatsu-edition27-pdf-free.html)
  - A Nepali government appendix that cites the Komatsu backhoe "Standard Cycle Time, Cm std (s)" table lists PC200 at about 13 to 16 s and 16 to 19 s. This is a search-snippet view only, and the PDF now returns 404. — [Indrasarowar appendix](https://www.indrasarowarmun.gov.np/sites/indrasarowarmun.gov.np/files/appendix-1.pdf)
- **Table 13 "Conversion factor for excavator"** (rows = dumping condition; columns = digging depth as % of specified max digging depth: below 40% / 40 to 75% / over 75%):

  | Dumping condition | Below 40% | 40 to 75% | Over 75% |
  |---|---|---|---|
  | Easy (dump onto spoil pile) | 0.7 | 0.8 | 0.9 |
  | Normal (large dump target) | 0.9 | 1.0 | 1.1 |
  | Rather difficult (small dump target) | 1.1 | 1.3 | 1.5 |
  | Difficult (small dump target requiring max dumping reach) | 1.4 | 1.6 | 1.8 |

  — [KOM27](https://pdfcoffee.com/handbook-komatsu-edition27-pdf-free.html). The source text flattens the table, so the row/column assignment is my inference. The matrix is consistent under either reading.
- **Loading-shovel configuration (Table 12):** PC400 16 to 20 s; PC750 18 to 22 s; PC1250 20 to 24 s; PC1800 27 to 31 s. — [KOM27](https://pdfcoffee.com/handbook-komatsu-edition27-pdf-free.html)
- **Mining backhoe rule of thumb (p. 15A-11):** "Every 10 degrees more swing will increase the cycle time by 1 second". It applies to a skilled operator, diggable material and standard attachments only. PC3000 backhoe: easy 23 to 25 s, average 26 to 28 s, severe 29 to 31 s. — [KOM27](https://pdfcoffee.com/handbook-komatsu-edition27-pdf-free.html). The same 1 s per 10 deg rule appears in the Komatsu Loading Tool Selection Guide. — [Komatsu LTSG](https://www.komatsu.com/content/dam/komatsu/sales-and-marketing-documents/brochures/mining-parts/loadingtoolselectionguide-brochure-english-en-ltsgbr01-0522-v3.pdf)

**Caterpillar (CAT29, pp. 4-147 to 4-148 "Cycle Time Estimating Charts", PDF pp. 273 to 274)**
- **Cycle segments:** 1. Load bucket; 2. Swing loaded; 3. Dump bucket; 4. Swing empty. "If a truck is located on the floor of the excavation beside material being moved, 10 to 17 second cycles are practical." — [CAT29](http://courses.washington.edu/esrm468/468%20Class%20material/PHB29.pdf)
- **Job-condition key for the chart (A to E):**
  - **A, Excellent:** easy digging (unpacked earth, sand, gravel, ditch cleaning), depth under 40% of max, swing under 30 deg, dump onto spoil pile or truck in excavation, no obstructions, good operator.
  - **B, Above average:** medium digging (packed earth, tough dry clay, under 25% rock), depth to 50% of max, swing to 60 deg, large dump target, few obstructions.
  - **C, Average:** medium-to-hard digging (hard packed soil with up to 50% rock), depth to 70% of max, swing to 90 deg, **loading trucks with truck spotted close to excavator**.
  - **D, Below average:** hard digging (shot rock or tough soil with up to 75% rock), depth to 90% of max, swing to 120 deg, shored trench, small dump target, working over a pipe crew.
  - **E, Severe:** toughest digging (sandstone, caliche, shale, certain limestones, hard frost), over 90% of max depth, swing over 120 deg, loading bucket in a man box, dump into a small target needing max reach, people and obstructions in the work area.
  - Bar labels: "Fastest possible", "Fastest practical", "Typical range", "Slow".
  - — [CAT29](http://courses.washington.edu/esrm468/468%20Class%20material/PHB29.pdf)
- **Conditions for the "typical" breakdown table:** no obstruction in the right of way, above-average job conditions, an operator of average ability, and a 60 to 90 deg swing angle. — [CAT29](http://courses.washington.edu/esrm468/468%20Class%20material/PHB29.pdf)
- **Typical breakdown, 20 t class and neighbours** (min; seconds in brackets):

  | Model | Bucket | Soil | Dig depth | Load | Swing loaded | Dump | Swing empty | **Total** |
  |---|---|---|---|---|---|---|---|---|
  | 315B/315B L | 520 L | packed earth | 3.0 m | 0.10 | 0.04 | 0.02 | 0.05 | 0.21 (12.6 s) |
  | **320B** | **800 L (1.05 yd3)** | **hard clay** | **2.3 m** | **0.09 (5.4 s)** | **0.06 (3.6 s)** | **0.03 (1.8 s)** | **0.05 (3.0 s)** | **0.23 (13.8 s)** |
  | 322B | 1000 L | hard clay | 3.2 m | 0.09 | 0.06 | 0.04 | 0.06 | 0.25 (15.0 s) |
  | 325B | 1100 L | hard clay | 3.2 m | 0.09 | 0.06 | 0.04 | 0.06 | 0.25 (15.0 s) |
  | 330B | 1400 L | hard clay | 3.4 m | 0.09 | 0.07 | 0.04 | 0.07 | 0.27 (16.2 s) |

  The 318B L/LN entry reads "Information not available at time of printing". — [CAT29](http://courses.washington.edu/esrm468/468%20Class%20material/PHB29.pdf)
  - A secondary web calculator also lists "320 = 13.8 s" from a "Caterpillar Cycle Time Estimating Chart", which matches. — [permacultureearthworks.com](https://permacultureearthworks.com/calculators/excavator-cost-calculator.html) (that page's TLS certificate had expired; I saw it only as a search snippet)
- **Graphical band, 320B column:** about 15 s at the top (fastest practical) to about 26 s at the bottom (slow/severe). The "typical" marker box sits at about 19 to 21 s. The 322B/325B bands run about 15/16 s to about 27 s, and the 330B about 17 to 28 s. — [CAT29](http://courses.washington.edu/esrm468/468%20Class%20material/PHB29.pdf) p. 4-148. **Caution:** I estimated these from shaded bars in the rendered image; they are not printed numbers.
- **Truck exchange time:** "For estimating truck loading production include approximately 0.7 minutes for truck exchange time." — [CAT29](http://courses.washington.edu/esrm468/468%20Class%20material/PHB29.pdf) p. 4-152
- **Mass-excavation guidance:** for maximum production, limit the work zone to 15 deg either side of the machine centreline. Bench height should be about equal to stick length for stable materials. The operator should begin boom-up when the bucket is 75% through the curl. — [CAT29](http://courses.washington.edu/esrm468/468%20Class%20material/PHB29.pdf) p. 4-149
- **NZ Forest Road Engineering Manual §5.9:** reproduces the Cat condition definitions (Excellent / Average / Severe: depth to 40/50/90%, swing under 30/to 70/to 120 deg). It gives no edition. Its "Average" swing (70 deg) differs from CAT29's "to 60 deg" for condition B. — [NZFOA](https://docs.nzfoa.org.nz/live/nz-forest-road-engineering-manual/5-road-and-landing-construction/5.9-estimating-machinery-production/)

**Textbook cycles per hour (KSU slides reproducing "Table 3-3 Standard cycles per hour for hydraulic excavators"; secondary, probably Nunnally)**

| Material class | Small excavator, 1 yd (0.76 m3) or less | Medium, 1.25 to 2.25 yd (0.94 to 1.72 m3) | Large, over 2.5 yd |
|---|---|---|---|
| Soft (sand, gravel, loam) | 250 cyc/h (14.4 s) | 200 cyc/h (18.0 s) | 150 cyc/h |
| Average (common earth, soft clay) | 200 cyc/h (18.0 s) | 160 cyc/h (22.5 s) | 120 cyc/h |
| Hard (tough clay, rock) | 160 cyc/h (22.5 s) | 130 cyc/h (27.7 s) | 100 cyc/h |

These rates apply at 90 deg swing and the "optimum" depth (factor 1.00 at 70% depth). — [KSU](https://faculty.ksu.edu.sa/sites/default/files/3.CE419-Ch3.1.pdf) slide 31
- A 0.8 m3 heaped bucket falls in the gap between the "small" (0.76 m3 or less) and "medium" (from 0.94 m3) classes.

### Inferences
- The three sources agree on a 20 t class central value of about **14 to 16 s at 60 to 90 deg swing in favourable conditions**, rising to **about 18 to 23 s** in average or hard conditions or at 90 to 180 deg swing:
  - Komatsu: 13 to 16 s (45 to 90 deg)
  - Cat: 13.8 s typical, 15 s fastest practical on the band
  - Nunnally: 14.4 s soft, 18 s average
- Komatsu's conversion factor can push PC200 to about 16 x 1.6 = 25.6 s, or 19 x 1.8 = 34 s in the worst case. This roughly matches Cat's "severe" end of about 26 s.

### Gaps
- I could not open newer Cat editions (44 to 50, 2014 to 2022) with 320D/320E/320 (2017+) breakdowns; the ringpower.com Ed. 50 PDF returned HTTP 404. The CAT29 values are for the 320B (1998).
- I found no Hitachi ZX200, Volvo EC200/EC220 or Liebherr R 920/R 922 published cycle-time tables. Hitachi brochures give swing speed and similar specs but no estimating cycle times.
- The Komatsu text table needs visual confirmation against a scanned page. The column mapping is consistent with the Nepali appendix snippet.

---

## Q2. Bucket fill factors by material, plus swell/load factors and loose/bank densities

### Takeaway
For backhoe buckets, the sources give fill factor as a fraction of heaped capacity:
- Moist loam / sandy clay: 1.00 to 1.10
- Sand and gravel: 0.90 to 1.10
- Hard clay: 0.65 to 0.90
- Wet clay: 0.50 to 0.90
- Well-blasted rock: 0.60 to 0.90
- Poorly blasted rock: 0.40 to 0.70

Komatsu gives a higher "easy" value (1.1 to 1.2) for clayey or soft natural ground. Swell runs about 11 to 12% for sand, about 23 to 25% for loam or common earth, and about 22 to 43% for clay, depending on the source.

### Cited Findings

**Cat CAT29 p. 4-119 (PDF p. 246), "Bucket Fill Factors", hydraulic excavators** (Average bucket payload = heaped bucket capacity x bucket fill factor; values as % of heaped capacity):

| Material | Fill factor range |
|---|---|
| Moist loam or sandy clay (A) | 100 to 110% |
| Sand and gravel (B) | 95 to 110% |
| Hard, tough clay (C) | 80 to 90% |
| Rock, well blasted | 60 to 75% |
| Rock, poorly blasted | 40 to 50% |

— [CAT29](http://courses.washington.edu/esrm468/468%20Class%20material/PHB29.pdf)
- A lecture by Habtamu Lulseged that cites Cat data gives sand and gravel as **95 to 100%**, so a later edition may have changed the upper bound. I saw this as a search snippet only; it is a secondary source. — via search summary of ENCE 420 / Lulseged lecture slides (no stable URL retrieved)
- Cat's web article on fill factors notes that buckets "often have fill factors over 100 percent" and points to the Performance Handbook for materials. The page denied access (HTTP 403). — [cat.com](https://www.cat.com/en_US/articles/ci-articles/bucket-fill-factors.html)

**Komatsu KOM27 p. 15A-9, Table 9 "Bucket fill factor (Backhoe), up to PC1800"** (K, applied to heaped capacity):

| Condition | Description | K |
|---|---|---|
| Easy | Excavating natural ground of clayey soil, clay, or soft soil | 1.1 to 1.2 |
| Average | Natural ground of soil such as sandy soil and dry soil | 1.0 to 1.1 |
| Rather difficult | Natural ground of sandy soil with gravel | 0.8 to 0.9 |
| Difficult | Loading blasted rock | 0.7 to 0.8 |

Table 10 gives the loading-shovel equivalents: 1.0 to 1.1, 0.95 to 1.0, 0.90 to 0.95 and 0.85 to 0.90. — [KOM27](https://pdfcoffee.com/handbook-komatsu-edition27-pdf-free.html)

**Textbook (KSU slide 24, "Table 3-2 Bucket fill factors for excavators", secondary, probably Nunnally):**

| Material | Fill factor |
|---|---|
| Common earth, loam | 0.80 to 1.10 |
| Sand and gravel | 0.90 to 1.00 |
| Hard clay | 0.65 to 0.95 |
| Wet clay | 0.50 to 0.90 |
| Rock, well-blasted | 0.70 to 0.90 |
| Rock, poorly blasted | 0.40 to 0.70 |

— [KSU](https://faculty.ksu.edu.sa/sites/default/files/3.CE419-Ch3.1.pdf)

**Densities and load factors, CAT29 "Weight of Materials" (Tables section, PDF p. 984)** (loose / bank kg/m3, load factor LF = bank-to-loose volume factor; swell% = 1/LF - 1, which I computed):

| Material | Loose kg/m3 | Bank kg/m3 | LF | Swell (computed) |
|---|---|---|---|---|
| Clay, natural bed | 1660 | 2020 | 0.82 | about 22% |
| Clay, dry | 1480 | 1840 | 0.81 | about 23% |
| Clay, wet | 1660 | 2080 | 0.80 | 25% |
| Clay & gravel, dry | 1420 | 1660 | 0.85 | |
| Clay & gravel, wet | 1540 | 1840 | 0.85 | |
| Earth, dry packed | 1510 | 1900 | 0.80 | 25% |
| Earth, wet excavated | 1600 | 2020 | 0.79 | 27% |
| Earth, loam | 1250 | 1540 | 0.81 | 23% |
| Sand, dry loose | 1420 | 1600 | 0.89 | 12% |
| Sand, damp | 1690 | 1900 | 0.89 | 12% |
| Sand, wet | 1840 | 2080 | 0.89 | 12% |
| Sand & clay, loose | 1600 | 2020 | 0.79 | 27% |
| Sand & gravel, dry | 1720 | 1930 | 0.89 | 12% |
| Sand & gravel, wet | 2020 | 2230 | 0.91 | 10% |
| Top soil | 950 | 1370 | 0.70 | 43% |
| Decomposed rock, 50% rock / 50% earth | 1720 | 2280 | 0.75 | 33% |

The handbook adds that values vary "with moisture content, grain size, degree of compaction", and that tests are needed to determine exact material characteristics. — [CAT29](http://courses.washington.edu/esrm468/468%20Class%20material/PHB29.pdf)

**Komatsu KOM27 p. 15A-2/15A-3, Table 1 "Earth volume conversion factor (f)"** (loose/bank and compacted/bank ratios):

| Material | Loose/bank | Compacted/bank |
|---|---|---|
| Sand | 1.11 | 0.95 |
| Sandy clay | 1.25 | 0.90 |
| Clay | 1.43 | 0.90 |
| Gravelly soil | 1.18 | 1.08 |
| Gravel | 1.13 | 1.03 |
| Solid/rugged gravel | 1.42 | 1.29 |
| Broken soft rock (limestone, sandstone) | 1.65 | 1.22 |
| Broken hard rock (granite, basalt) | 1.70 | 1.31 |
| Broken rocks | 1.75 | 1.40 |
| Blasted bulky rocks | 1.80 | 1.30 |

The handbook's worked example confirms "ordinary soil 1,000 m3 x 1.25 = 1,250 m3" loose, then "x 0.72 = 900 m3" compacted. — [KOM27](https://pdfcoffee.com/handbook-komatsu-edition27-pdf-free.html)

### Inferences
- For a sim default:
  - Moist sand / loam / common earth: fill about 1.0 to 1.05
  - Dry sand: about 0.95
  - Stiff clay: about 0.85
  - Sticky wet clay: about 0.7 (wide 0.5 to 0.9 band because of sticking/carry-back)
  - Blasted rock: about 0.65
- Komatsu's clay swell (1.43) is much higher than Cat's clay load factor implies (about 1.22 to 1.25). The two handbooks disagree on clay bulking, so carry both as a range.

### Gaps
- I did not retrieve the Peurifoy/Schexnayder (8th/9th ed.) fill-factor and swell tables directly, nor Day & Benjamin or Gransberg. I found only lecture reproductions of the Nunnally tables.
- I found no Hitachi, Volvo or Liebherr fill-factor tables.

---

## Q3. Hourly production formulas, job efficiency, swing/depth corrections, and typical outputs for a 0.8 m3 bucket

### Takeaway
All three methods reduce to Q = 3600 x q_heaped x K x E / Cm. For a 0.8 m3 heaped bucket on a 20 t machine loading trucks in average soil with a 50-min hour (E = 0.83), they give about **125 to 175 Lm3/h**, roughly **100 to 140 Bm3/h** or **190 to 270 t/h**. That range is before truck-exchange losses; Cat's 0.7 min per truck cuts it by about 15 to 25%.

### Cited Findings

**The three formulas**
- **Komatsu:** Q = q x 3600 x E / Cm, where q = q1 x K, q1 = heaped bucket capacity (m3), Cm in seconds. Q is loose m3/h. — [KOM27](https://pdfcoffee.com/handbook-komatsu-edition27-pdf-free.html) p. 15A-9
- **Cat:** m3/60-min hr = (60 / cycle time in min) x average bucket payload, where average bucket payload = heaped capacity x bucket fill factor. Actual m3/hr = m3/60-min hr x job efficiency factor. The handbook notes the 60-minute hour is "a condition that is never achieved in reality". — [CAT29](http://courses.washington.edu/esrm468/468%20Class%20material/PHB29.pdf) p. 4-149
- **Textbook:** Production (LCM/h) = C x S x V x B x E, where C = cycles/h (Table 3-3), S = swing-depth factor (Table 3-4), V = heaped volume, B = fill factor (Table 3-2), E = job efficiency. — [KSU](https://faculty.ksu.edu.sa/sites/default/files/3.CE419-Ch3.1.pdf) slide 30

**Job efficiency**
- **Cat "Job Efficiency Estimator":** 60 min = 100%; 55 min = 91%; **50 min = 83%**; 45 min = 75%; 40 min = 67%. — [CAT29](http://courses.washington.edu/esrm468/468%20Class%20material/PHB29.pdf) p. 4-151
- **Komatsu, hydraulic excavators:** good 0.83; average 0.75; rather poor 0.67; poor 0.58. — [KOM27](https://pdfcoffee.com/handbook-komatsu-edition27-pdf-free.html) p. 15A-10
  - The wheel-loader table on the preceding page uses a different scale: 0.83 / 0.80 / 0.75 / 0.70. — [KOM27](https://pdfcoffee.com/handbook-komatsu-edition27-pdf-free.html) p. 15A-8

**Swing-depth factor S for backhoes (textbook "Table 3-4", via KSU slide 31; secondary, probably Nunnally)**

| Depth of cut (% of max) | 45 deg | 60 deg | 75 deg | 90 deg | 120 deg | 180 deg |
|---|---|---|---|---|---|---|
| 30% | 1.33 | 1.26 | 1.21 | 1.15 | 1.08 | 0.95 |
| 50% | 1.28 | 1.21 | 1.16 | 1.10 | 1.03 | 0.91 |
| 70% | 1.16 | 1.10 | 1.05 | 1.00 | 0.94 | 0.83 |
| 90% | 1.04 | 1.00 | 0.95 | 0.90 | 0.85 | 0.75 |

— [KSU](https://faculty.ksu.edu.sa/sites/default/files/3.CE419-Ch3.1.pdf)
- **Hydraulic shovel swing adjustment (Table 3-6):** 45 deg 1.16; 60 deg 1.10; 75 deg 1.05; 90 deg 1.00; 120 deg 0.94; 180 deg 0.83. — [KSU](https://faculty.ksu.edu.sa/sites/default/files/3.CE419-Ch3.1.pdf) slide 50
- An Indonesian thesis attributes the swing-depth table to Peurifoy (1989). This is a secondary attribution from a search snippet. — [dewey.petra.ac.id](https://dewey.petra.ac.id/digital/get-file/18882)

**Published worked examples**
- **Textbook Example 3-2:** 0.57 m3 heaped, sand and gravel, B = 0.95, E = 50/60, depth 4.3 of 6.1 m max (70%), 90 deg swing, giving 250 x 1.00 x 0.57 x 0.95 x 0.833 = **113 LCM/h**. — [KSU](https://faculty.ksu.edu.sa/sites/default/files/3.CE419-Ch3.1.pdf) slide 36
- **Cat example, 320 with a 1.0 m3 bucket in wet sandy loam:**
  - 15 300 Bm3 (19 100 Lm3 at 25% swell), 2.4 m face depth, 60 to 90 deg swing, 50-min hour (83%).
  - Required rate: 190 Lm3/60-min hr, or 230 Lm3/50-min hr capability. For the 320 this means a 17.1 s cycle (60-min hr) or **15.0 s** (with job efficiency applied).
  - Cat concludes the 320 "would be working near its maximum capability"; a 330 with 1.9 m3 would need 35 s / 30 s.
  - — [CAT29](http://courses.washington.edu/esrm468/468%20Class%20material/PHB29.pdf) pp. 4-149 to 4-150
- **Cat Production Estimating Table** (Lm3 per 60-min hr; rows = cycle time, columns = bucket payload in Lm3):

  | Cycle time | 0.7 Lm3 | 0.9 Lm3 |
  |---|---|---|
  | 13.3 s (270 cyc/h) | 189 | 243 |
  | 15.0 s (240 cyc/h) | 168 | 216 |
  | 17.1 s (210 cyc/h) | 147 | 189 |
  | 20.0 s (180 cyc/h) | 126 | 162 |
  | 24.0 s (150 cyc/h) | 105 | 135 |

  The unshaded area marks "average production". — [CAT29](http://courses.washington.edu/esrm468/468%20Class%20material/PHB29.pdf) p. 4-151 (PDF p. 278)

**Computed outputs for a 0.8 m3 heaped bucket** (my arithmetic using the cited factors; not published figures):
- **Komatsu PC200, average soil (K = 1.0), 45 to 90 deg swing, normal dump to truck, depth 40 to 75% (factor 1.0), E = 0.83:**
  - Cm = 13 to 16 s, giving Q = 0.8 x 3600 x 0.83 / Cm = **184 to 149 Lm3/h** (mid, 14.5 s: 165 Lm3/h).
  - At 90 to 180 deg swing (16 to 19 s): **149 to 126 Lm3/h**.
  - Easy clay ground (K = 1.15, 13 s, factor 0.9 so Cm = 11.7 s): about 235 Lm3/h.
  - Rather difficult (K = 0.85, small target, factor 1.3 on 16 s so Cm = 20.8 s, E = 0.75): about 88 Lm3/h.
- **Cat 320B typical cycle 13.8 s, fill 1.0, E = 0.83:** 0.8 x 3600 / 13.8 x 0.83 = **173 Lm3/h**.
  - Adding truck exchange: for a 0.8 m3 bucket filling a roughly 10 Lm3 truck in about 12 to 13 passes, a 0.7 min exchange adds about 3.3 s per pass. That gives about 17.1 s effective and **about 140 Lm3/h**.
- **Textbook, small-class rates, 90 deg and 70% depth (S = 1.0), E = 0.833:**
  - Soft (250 cyc/h, B = 0.95): 158 LCM/h
  - Average (200 cyc/h, B = 0.95): 127 LCM/h
  - Hard (160 cyc/h, B = 0.80): 85 LCM/h
  - At 45 deg / 30% depth (S = 1.33), the soft case reaches 211 LCM/h.
- **Mass conversion** (my arithmetic): 165 Lm3/h of dry packed earth (1510 kg/m3 loose) is about 249 t/h, or about 132 Bm3/h at LF 0.80. 165 Lm3/h of wet sand (1840 kg/m3) is about 304 t/h.

### Inferences
- Sim validation targets for a 0.8 m3, 20 t excavator truck loading, 50-min hour:
  - Easy (loose sand/loam, low swing): about 170 to 210 Lm3/h
  - Average: about 125 to 165 Lm3/h
  - Hard clay / rocky: about 80 to 110 Lm3/h
- The Cat worked example shows that sustaining 230 Lm3/50-min hr with a 1.0 m3 bucket (15 s cycles) is "near maximum capability". So sim outputs much above about 200 Lm3/h for a 0.8 m3 bucket should be treated as suspect.

### Gaps
- I could not see the Peurifoy/Schexnayder text directly, so I did not confirm whether its swing-depth table is identical to the Nunnally table above. One snippet attributes the same table to Peurifoy (1989).
- I found no published manufacturer hourly-output figure stated specifically for a 0.8 m3 ZX200/PC200 in truck loading. All outputs above are computed from the published factors.

---

## Q4. Breakdown of the cycle into dig/load, swing loaded, dump and swing empty for this class

### Takeaway
The Cat handbook breakdown for the 320B (800 L bucket, hard clay, 2.3 m depth, 60 to 90 deg swing) is about **5.4 s dig, 3.6 s swing loaded, 1.8 s dump and 3.0 s swing empty, totalling 13.8 s**. That is roughly 39% dig, 26% swing loaded, 13% dump and 22% swing empty. Field data from a 30 t PC300-8 loading trucks at 60 to 180 deg swing show slower times: about 7.4 / 5.0 / 4.2 / 4.3 s, totalling about 21 s.

### Cited Findings
- **CAT29 Cycle Time Estimating Chart, 320B:** load bucket 0.09 min, swing loaded 0.06, dump 0.03, swing empty 0.05; total 0.23 min. 315B (packed earth, 3.0 m): 0.10 / 0.04 / 0.02 / 0.05 = 0.21 min. 322B and 325B: 0.09 / 0.06 / 0.04 / 0.06 = 0.25 min. — [CAT29](http://courses.washington.edu/esrm468/468%20Class%20material/PHB29.pdf) p. 4-148
- **Field study (Indonesian thesis appendix, "Lampiran C"), Komatsu PC300-8 (about 30 t class, not 20 t), 30 observed cycles at 60 to 180 deg swing (mostly 90 to 130 deg):**
  - Grouped-frequency means: digging **7.35 s**, swing loaded **4.96 s**, dumping **4.23 s**.
  - Raw ranges: dig 4.8 to 9.2 s; swing loaded 3.6 to 6.3 s; dump 3.0 to 5.3 s; swing empty 3.1 to 5.8 s.
  - The swing-empty mean was not shown in the extracted text. My arithmetic mean of the 30 raw values is **4.28 s**, making the total about **20.8 s**.
  - — [repository.itny.ac.id LAMPIRAN C](https://repository.itny.ac.id/id/eprint/4294/12/LAMPIRAN%20C.pdf)
- **Komatsu PC200 swing specification:** about 12.4 rpm on one PC200-7 model, per a spec-sheet search summary (secondary). — [machine.market PC200LC-7 spec](https://files.machine.market/specs/komatsu-pc200lc-7-excavators--dozers----motor-graders(1).pdf)

### Inferences
- At 12.4 rpm the pure-kinematic swing rate is about 74 deg/s, so a 90 deg swing is about 1.2 s at full speed. The published swing segments (3 to 3.6 s for about 75 deg in the Cat table; about 5 s for about 110 deg in the field data) are therefore dominated by acceleration/deceleration and combined boom raise, not peak swing speed. This is relevant for checking simulated hydraulics.
- Komatsu's mining rule adds 1 s of total cycle time per 10 deg of extra average swing. That is consistent with the jump of about 3 s between Komatsu's 45 to 90 deg and 90 to 180 deg bands for the PC200 (13 to 16 versus 16 to 19 s).

### Gaps
- I found no published 20 t class segment breakdown from Komatsu, Hitachi, Volvo or Liebherr. Komatsu gives only total standard cycle times.
- I did not retrieve newer Cat editions with 320D/320 (2017+) segment times. Modern machines with electrohydraulic controls may cycle faster than the 1998 320B figures.
- No PC200/ZX200 field time-study with a full segment breakdown was retrieved. The field data above is from a 30 t PC300-8.
