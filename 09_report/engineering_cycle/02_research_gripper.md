# Research: the gripper

Gate G0 (feeds G6) · Builds on [Stage 1](01_problem_identification.md) sections 8.5–8.6 · Fulfils groundwork for **R10** and **R11**

What the supplied gripper is, what's likely wrong with it, what other grippers do, and which improvements are worth considering for brief Task 10.

> **AI disclosure (C12):** this file was drafted with AI assistance (Claude). The STL measurements were made with a Python script on the files in `00_baseline/3d_models/`. Every number and source must be checked by the group before it goes into the report.

---

## 1. What the brief asks for (Task 10)

The brief describes the gripper as "a servo-driven gear and two racks" that move opposing jaws, and says to **evaluate it before redesigning it** [1, Task 10]. The improvement can target opening range, alignment, backlash, contact geometry, object retention, printability, material use, serviceability or servo load [1].

| Step | What we must produce [1, Task 10] |
| --- | --- |
| 1 | At least **two** tutor-approved light objects with different widths, shapes or surfaces |
| 2 | **Measurable** gripper requirements (opening, alignment, transfers, slip, damage, servo range, mass, print time, material) |
| 3 | Inspection of the original front plate, gear, racks and jaws: friction, backlash, interference, poor contact, tolerances |
| 4 | **At least two concepts**, chosen with a decision matrix (performance, printability, safety, cost, material, compatibility) |
| 5 | A **dimensioned CAD change** with clearance, mesh, wall thickness, servo travel, symmetry and slicer checks |
| 6 | The improved part printed and fitted (or a stated evidence limitation) |
| 7 | **5 trials per object** for the original and the improved design |
| 8 | A trade-off judgement: is it worth the extra mass, print time, material, complexity and maintenance? |

Marks: 5 for requirements, original analysis, concepts and CAD; 4 for repeated trials and trade-offs [1, Part E].

---

## 2. How the supplied gripper works

Measured from the STL files [3]. "Solid" volumes are as if printed 100 % full.

| Part | Size (mm) | Solid volume | What it is |
| --- | --- | --- | --- |
| `GripperGear` | Ø10.8 × 3 | 0.14 cm³ | Pinion, **10 teeth**. A profiled recess (≈ Ø4.2–5.6 mm, 2 mm deep) on one side, with a ≈ Ø1.5 mm hole through. It looks like it presses straight onto the servo's output spline and takes the horn screw. **Check on the real part.** |
| `GripperRackTop` / `Bottom` | 30 × 9.9 × 3.5 | 0.47 cm³ each | Straight racks with **8 rounded teeth** over ≈ 21 mm, plus a peg (2 × 4 mm) that locates the jaw. |
| `GripperLeft` / `Right` | 40 × 23.6 × 11.5 | 2.07 cm³ each | Jaw body with a slot for the rack. The part that touches the object is a **flat, plain finger about 10 mm long, 5.8 mm wide and 5.4–6.7 mm tall**. |
| `FrontPlate` | 20 × 43 × 2 | 1.12 cm³ | Plate with a 12.6 × 23.4 mm window (about the size of an SG90 body) and two Ø2 mm holes 27.5 mm apart, most likely for the gripper servo's screw tabs. The brief calls it the "front plate / sliding support" [1, Appendix B]. |

**How it moves:** the servo turns the pinion, which pushes the top rack one way and the bottom rack the other. Each rack carries one jaw, so the jaws close towards the middle by equal amounts. This is a **parallel** gripper: the contact faces stay parallel, so the object's centre doesn't shift with its size. An arc-swinging (angular) claw doesn't keep that property [14].

**Numbers that matter:**

| Quantity | Value | How |
| --- | --- | --- |
| Pinion pitch radius | ≈ 4.5 mm | Tip Ø10.8 mm, 10 teeth → module ≈ 0.9 mm (Stage 1, 8.6 [2]) |
| Jaw travel per servo degree | ≈ 0.079 mm per jaw | 4.5 mm × π/180 |
| Gap change over firmware range 30–150° | ≈ 19 mm | Stage 1 finding F5 [2] |
| Pinion tooth spacing | ≈ 2.83 mm | π × module 0.9 |
| Rack tooth spacing | **≈ 2.95 mm** | Measured between the 8 rack tooth tips [3] |

**Possible flaw found in the files:** the rack teeth are about **4 % further apart** than the pinion's. Over three teeth in mesh that adds up to ≈ 0.35 mm, which would show up as binding at some positions and play at others. The rack teeth are also rounded bumps, not a proper gear (involute) shape. This is **a measurement from the STL, not yet seen on the real part**. Check it in the step 3 inspection by turning the gear slowly by hand and feeling for tight and loose spots.

---

## 3. How hard does it need to grip?

**Force the gripper needs.** Each jaw must press hard enough for friction to hold the object's weight while the arm accelerates. The usual sizing formula is F = m (g + a) × S ÷ (n × μ) for n contact faces, μ friction and safety factor S [8, 9]. Parker recommends S = 4 for a friction grip [8].

With our worst case: m = 5 g (F4 [2]), a = g (pessimistic), S = 4, n = 2:

| Jaw surface | μ | Force needed per jaw |
| --- | --- | --- |
| Bare PLA, worst measured | 0.12 [7] | **1.6 N** |
| Bare PLA, best measured | 0.38 [7] | 0.5 N |
| Textured / rubbery pad | 0.3–0.5 [9] | 0.4–0.65 N |

**Force the servo can give.** SG90 stall torque is 1.2–1.8 kgf·cm (F3 [2]). At a 4.5 mm pitch radius that's 26–39 N of total rack force, or **≈ 13–20 N per jaw** at stall, before gear friction losses.

**Finding:** even the worst case needs **about 12 % of stall**. The SG90 has plenty of force. The gripper's problems are more likely to be:

1. **Slip on smooth, narrow faces.** PLA friction can be as low as 0.12 [7], and the contact face is only about 10 × 6 mm.
2. **Holding at stall.** If the firmware commands a closed angle past the point where the jaws meet the object, the servo keeps pushing at stall current. Users report a micro gripper servo getting very hot within 2–3 minutes of this [5]. Hobby servos have no protection against overheating [6]. Commanding a gap just smaller than the object is much safer than commanding fully closed [5].
3. **Play and alignment** in the rack and pinion (section 2).
4. **Round objects rolling** between two flat faces, which only touch along a line.

**Mass budget.** Each extra gram at the gripper (≈ 172 mm from the shoulder) uses ≈ 172 gf·mm of the 1530 gf·mm shoulder margin, so it costs about **1 g of payload** (Stage 1, 8.5 [2]). Any redesign should add **≤ 1 g**.

---

## 4. What other grippers do

### 4.1 Pololu Micro Gripper Kit (commercial)

- Same idea as ours: a micro servo, a pinion and two paddles with racks [4].
- Opens to about **32 mm** and weighs 30 g with the servo [4].
- Uses a servo that reports its position, so the paddles' position can be read back [4]. That's how it avoids guessing when the jaws have touched. We don't have that (C14 [2]).
- Even so, owners report the servo getting hot when it's left closed on an object [5].

**Take-away:** our mechanism is a normal, proven design. The main weakness is holding at stall, and we must deal with that in software because we have no feedback.

### 4.2 Friction pads and fingertip skins

- Berkeley researchers wrapped a commercial gripper's tips in tape. That alone raised the success rate on difficult printed objects from **28.7 % to 68.7 %** [10].
- Printed TPU pads on PLA fingers are a common way to add grip [11, 12]. Softer TPU (60A) is gentler; harder TPU (95A) holds more load [12].
- On printed TPU fingertips, the surface pattern (waves, circles, thickness) changes static friction as much as the material does [11].

**Take-away:** a high-friction skin is the cheapest big win. Tape, a rubber band or heat-shrink tube costs nothing (C3). A TPU pad needs a printer that can print TPU.

### 4.3 V-shaped jaws

- V-shaped jaw profiles make a parallel gripper **centre round objects** of different sizes on the same axis [14].
- On a parallel gripper the centre stays put as the size changes. On an angular claw the V shifts, so the control has to change for each size [14].

**Take-away:** a V-groove suits object B (the 12 mm cylinder) and still works on the flat cube. It's a jaw-only reprint, with no change to the racks or servo.

### 4.4 Fin Ray (soft, bending) fingers

- Fin Ray fingers bend around an object when pressed, spreading the grip. They're usually TPU 95A fingers on a PLA base, driven by one servo [13].
- Limits: they mainly adapt in one plane, so they suit symmetrical objects best [13]. They are larger and heavier than our 10 mm fingers.

**Take-away:** this is the "different concept" for the decision matrix. It's probably too big and heavy for our mass budget, but scoring it shows we compared it.

### 4.5 Printed gears and backlash

- Printed tooth flanks bulge slightly, so a zero-clearance mesh binds. Some clearance (backlash) is necessary and should be built into the tooth shape, not added by moving the shafts apart [15].
- Clearance values don't transfer between printers, so print a test piece and measure ours [16].

**Take-away:** if the step 3 inspection confirms the pitch mismatch, redraw the rack to match the pinion and print a small fit test first.

---

## 5. Proposed gripper requirements (step 2)

Suggested values for the group to agree. Objects are the Stage 1 ones: **A** = 15 mm printed cube, **B** = 12 mm × 20 mm printed cylinder [2].

| ID | Requirement | Target | How we measure |
| --- | --- | --- | --- |
| GR1 | Open gap | ≥ 20 mm (A + 5 mm) | Calipers, servo at open angle |
| GR2 | Closed gap | ≤ 10 mm (smaller than B) | Calipers |
| GR3 | Jaw play | ≤ 0.5 mm at the fingertips | Calipers, push each jaw both ways with the servo holding |
| GR4 | Jaw alignment | Finger faces parallel and level within 0.5 mm | Calipers / feeler on a flat surface |
| GR5 | Transfers | 5 / 5 per object, no drop | Brief Task 10 table |
| GR6 | Slip | ≤ 1 mm movement of the object in the jaws during transfer | Mark on object, photo before/after |
| GR7 | No stall when holding | Gripper servo not warm after 60 s hold; bench supply stays out of CC (current-limit) mode | Touch test + supply readout (D1) |
| GR8 | Added mass | ≤ 1 g over the original | Kitchen scale, or slicer estimate |
| GR9 | Print | No supports; ≤ 1 h | Slicer screenshot |
| GR10 | Objects | No visible damage | Visual check |

---

## 6. Improvement concepts (step 4)

| # | Concept | What changes | Fixes | Cost |
| --- | --- | --- | --- | --- |
| **C1** | **Friction pads** | Add rubber, heat-shrink, tape or a printed TPU pad to each finger face | Slip (GR6), object retention | Free, < 0.5 g |
| **C2** | **V-groove jaws** | Reprint both jaws with a shallow V cut into the finger face, plus a recess for a pad | Round objects rolling, centring, retention | 2 small reprints |
| **C3** | **Matched rack** | Redraw the racks with tooth spacing that matches the pinion, with tested clearance | Play, binding (GR3, GR4) | 2 small reprints |
| **C4** | **Fin Ray fingers** | Replace the fingers with bendy TPU fingers | Grip on odd shapes | Needs TPU; heavier and bigger |
| **C5** | **Grip-to-angle firmware** | Close to a per-object angle just past contact, never to fully closed | Holding at stall (GR7) | Free, software only |
| **C6** | **Mini geared claw** | A new angular claw like the Kitronik Klaw, scaled to the SG90: two jaws geared to each other, serrated edges, lightening holes. Design, checks and STLs in [`03_cad/mini_claw/`](../../03_cad/mini_claw/README.md) | Opening (≈ 32 mm), retention, mass (≈ 25 % less plastic) | 5 new prints + 2 M2 screws |

C5 isn't a CAD change, so it can't be the Task 10 improvement on its own. **Do it anyway**, whatever is chosen: it protects the gripper servo during every trial.

### Suggested decision matrix

Scores 1–5 (5 = best), weight in brackets. **These are suggestions to start the discussion. The group should score it together** and record the agreed version.

| Criterion (weight) | C1 Pads | C2 V-groove + pad | C3 Matched rack | C4 Fin Ray | C6 Mini claw |
| --- | --- | --- | --- | --- | --- |
| Performance (3) | 3 | 4 | 3 | 4 | 4 |
| Printability (2) | 5 | 4 | 3 | 2 | 3 |
| Safety (2) | 4 | 4 | 4 | 4 | 3 |
| Cost / material (1) | 5 | 4 | 4 | 2 | 4 |
| Compatibility with supplied arm (2) | 5 | 5 | 4 | 2 | 3 |
| Mass (2) | 5 | 4 | 5 | 2 | 5 |
| **Weighted total (/60)** | **52** | **50** | **45** | **34** | **44** |

**Reading it:** C1 scores best but is barely a CAD change, so it's weak for the "dimensioned CAD change" marks. **C2 with a pad recess** keeps most of C1's benefit and gives a real, measurable CAD change. Suggested choice: **C2 (V-groove jaws with a pad recess), plus C5 in firmware**. If the inspection confirms the pitch mismatch, add **C3** as a second fix, tested separately so each change's effect can be seen.

**C6** is the most ambitious option: a full redesign that is lighter, opens wider and gives the most CAD evidence. It loses points for its pointed tips (pinch risk), five new prints with small teeth, and the firmware changes it needs (a smaller gripper range and a new `L3`). It's the better choice if the group wants a bigger redesign and has printer time to tune the gears.

---

## 7. Test plan (steps 3 and 7)

1. **Inspect the original** with power off (step 3): turn the pinion by hand, note tight spots, measure jaw play (GR3) and alignment (GR4), photo everything. Log flaws in `03_cad/flaw_log.csv`.
2. **Find the contact angle** for each object: close in 2° steps until the object just stops moving freely. Record it. Command **a few degrees past contact**, not fully closed (C5).
3. **Baseline trials:** 5 per object with the original jaws. Record them in `08_tests/gripper_trials.csv` using the brief's columns (opening / command, trials, successful transfer, slip or damage, observation) [1].
4. Make the change, then repeat step 3 **with the same objects, angles and moves**, so the before/after comparison is fair.
5. Weigh the new parts, note print time and material, and write the trade-off (step 8).

---

## 8. Open questions

- Does the pinion really press onto the servo spline, or does it screw onto a horn? (Check the assembled part.)
- Is the 4 % pitch mismatch real on the printed parts?
- Which jaw face actually contacts the object once assembled? (The finger shape suggests the side faces, about 10 × 6 mm.)
- Can the lab printer print TPU? If not, C1 uses free rubber or heat-shrink instead.
- Does the tutor approve objects A and B?

---

## Sources

Accessed 2026-11-23.

1. CT6056 Group Case Study brief v3.0, `00_baseline/brief/`.
2. S. Ahmed, *Stage 1: Identification of the problem*, [`01_problem_identification.md`](01_problem_identification.md) (findings F3–F5, sections 8.5–8.6).
3. Measurements of `00_baseline/3d_models/Gripper*.stl` and `FrontPlate.stl` with the Python `trimesh` library, this repository.
4. Pololu, *Micro Gripper Kit with Position Feedback Servo*. https://www.pololu.com/product/3551 · opening width via Core Electronics: https://core-electronics.com.au/micro-gripper-kit-with-position-feedback-servo.html
5. Pololu Forum, *Micro gripper kit … servo is very hot when the gripper is closed*. https://forum.pololu.com/t/micro-gripper-kit-with-position-feedback-servo-servo-is-very-hot-when-the-gripper-is-closed/18765
6. Pololu Forum, *Servo burnout protection*. https://forum.pololu.com/t/servo-burnout-protection/5373
7. Static friction of printed PLA, 0.12–0.38 depending on filament direction (inclined-plane test), *Proc. IMechE Part L*, 2020. https://doi.org/10.1177/1464420720978997
8. Parker Hannifin, *Gripper sizing* catalogue (safety factor 4 for friction grip). https://parker.com/parkerimages/automation/cat/English/1835cint.pdf
9. FIRGELLI, *Pneumatic Gripper Force Calculator* (formula and typical μ). https://www.firgelliauto.com/blogs/engineering-calculators/pneumatic-gripper-force-calculator
10. Guo et al., *Design of parallel-jaw gripper tip surfaces for robust grasping*, ICRA 2017. https://goldberg.berkeley.edu/pubs/gripper-tip-surface-design-icra-2017-camera-ready.pdf
11. TU Delft MSc thesis on printed TPU fingertip patterns. https://repository.tudelft.nl/file/File_d71d1c5b-0dfa-4242-a32b-ce0a62690550
12. TPU hardness for grippers: *Soft Robotics for Advanced Handling of Delicate Fruit Products*, SCITEPRESS 2025, https://www.scitepress.org/publishedPapers/2025/139288/pdf/index.html · *Print-N-Grip*, arXiv:2401.16463, https://arxiv.org/pdf/2401.16463
13. Fin Ray grippers: *A 3D-Printed Fin Ray Effect Inspired Soft Robotic Gripper with Force Feedback*, Micromachines 2021, https://pmc.ncbi.nlm.nih.gov/articles/PMC8538355/ · one-plane limitation: *ARTiS*, https://arxiv.org/pdf/2609.03362
14. Pham & Yeo, V-shaped jaws for concentric handling of cylindrical parts by quasi-parallel grippers. https://link.springer.com/article/10.1007/BF02604336
15. Sovol, *3D Printing Gears That Actually Work: Backlash, Orientation, and Material Tips*. https://www.sovol3d.com/blogs/news/3d-printing-gears-that-actually-work-backlash-orientation-and-material-tips
16. All3DP, *3D Printing Tolerances: How to Test & Improve Them*. https://all3dp.com/2/3d-printing-tolerances/
