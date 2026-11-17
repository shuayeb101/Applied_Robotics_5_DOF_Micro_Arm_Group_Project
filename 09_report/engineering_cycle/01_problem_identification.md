# Stage 1: Identification of the problem

Gate: **G0** (roles, requirements, interfaces, Gantt plan and initial risk controls agreed) [2, Figure 2]
Author: Shuayeb Ahmed · Draft 2 · 2026-11-17

Lecture 3 splits this stage into two activities [1, slide 6]:

- **Define the problem statement:** say clearly what the challenge or need is.
- **Research and analysis:** find out the context, constraints and stakeholders.

Sections 1 to 3 define the problem: what we are building, what it has to do, and what limits we work within. Sections 4 to 9 are the research and analysis that back up those choices with sources and calculations. Section 10 lists what this stage hands on to Stage 2, and Section 11 lists the decisions the group still has to make.

---

## 1. Problem statement

> **We need to design and build a 5-DOF micro robotic arm, covering both its physical body and its firmware. The body is made from 3D-printed parts, five servo actuators and an Arduino Nano driving a PCA9685 servo controller. The firmware has to move the arm to commanded positions and coordinates within a stated accuracy, and pick up and place small, light objects. It has to do this repeatably and safely, using only the parts supplied plus wires and 3D-printed parts.**

### 1.1 Physical body: confirmed components

| Component | Qty | Confirmed by | Notes |
| --- | --- | --- | --- |
| Servo actuators | 5 | The brief maps PCA9685 channels 0–4 to base, swing, arm one, arm two and gripper [2, Task 3], and requires all five servos to be installed and calibrated [2, success definition; Task 4]. | The brief doesn't name a servo model. The group's kit uses **SG90** micro servos (repo README). Check the label on each servo. |
| Arduino Nano | 1 | The brief allows either an Arduino Nano or an STM32 controller [2, Task 3, Task 5]. The group chose the Nano. | ATmega328P, 5 V, 16 MHz, 32 KB flash, 2 KB SRAM [13]. |
| PCA9685 servo controller | 1 | Named in the brief's architecture and wiring table [2, Figure 1, Task 3]. | 16-channel, 12-bit PWM over I²C [11]. Only channels 0–4 are used. |
| 3D-printed parts | 11 | STL inventory [2, Appendix B]. Files in `00_baseline/3d_models/`. | Foot, Swing, Arm, ArmSpring, ArmFront, FrontPlate, GripperGear, GripperRackTop, GripperRackBottom, GripperLeft, GripperRight. |
| Wires | As needed | Group constraint (Section 3) | Dupont jumpers for I²C, and servo leads to the PCA9685 headers. |

### 1.2 Problem statement elements (Lecture 3, slide 7)

The lecture defines a problem statement as a short, precise description of the current situation, the gap between the current state and the desired state, and why closing that gap matters [1, slide 7].

| Element | This project |
| --- | --- |
| Current state | A kit of unassembled parts, plus starter code that hasn't been approved for physical motion and may contain unsafe assumptions or be missing fault handling [2, "Starter-code status"]. |
| Desired state | An assembled, calibrated arm whose motion and gripping meet measurable goals (Section 2). |
| Why it matters | An arm that moves unexpectedly can hurt people or break parts. The brief treats invalid input, communication loss, timeout, an unreachable target and a limit violation as stop/hold or fault conditions [2, "Safe and responsible use"]. |
| Scope and boundary | Section 5 |
| Stakeholders | Section 6 |
| Constraints | Section 3 |
| Measurable criteria | Section 2. The brief says one video of one successful move isn't enough: results must be reproducible, measured and linked to requirements [2, success definition]. |

**Why the statement is written this way:** it says *what* the arm must do and *with what parts*. It doesn't pick a design, e.g. a particular gripper shape or speed profile, because Stage 2 compares design options [1, slides 5, 9].

---

## 2. Goals

There are three groups of goals: your two main goals rewritten so they can be measured (G1, G2), and the other requirements found in the brief (G3–G12). Stage 3 turns each goal into one or more numbered requirements in `02_requirements/requirements.csv`.

### G1: Move to commanded joint angles and Cartesian coordinates, within a stated accuracy

**Why "exactly" has been replaced with a tolerance:** no physical system is exact, and "exactly" can't be tested, so it can't pass or fail. Three facts set a floor on what the arm can achieve:

1. **The servos have no position feedback to the Nano.** The controller only knows what angle it *asked for*, not what angle the joint reached. This also means the first HOME move snaps each joint from wherever it happens to be [18].
2. **The SG90 dead band is 10 µs** [3, 4]. That is about ±1° of commanded change that may not move the servo (Section 9.3).
3. **Backlash, horn alignment and link flex** all add error. The brief asks the group to measure and discuss them [2, Task 4 step 4; Task 7 step 6].

Section 9.4 works out that ±1° of dead band on each of the shoulder and elbow gives up to **~2.6 mm** of endpoint error, before backlash and calibration error are added. So the goal becomes:

| Sub-goal | Proposed acceptance criterion | Basis |
| --- | --- | --- |
| G1a: joint command | Each joint reaches its commanded angle within **±2°**, measured from the calibrated reference. | Dead band ±1° [3, 4], plus margin for backlash. Confirm at G2. |
| G1b: Cartesian command (numerical) | FK reconstruction residual **≤ 0.1 mm** for every accepted IK solution. | The brief requires a threshold to be set and justified before reviewing results [2, Task 7 step 4]. |
| G1c: Cartesian command (physical) | Measured endpoint within **±5 mm** of the target. | ~2.6 mm from dead band alone (Section 9.4), plus backlash and measurement uncertainty. Tighten if the G5 measurements allow. |
| G1d: invalid targets | An unreachable or limit-violating target produces **no motion** and an explicit error. | Mandatory in the brief [2, Task 5 "Mandatory behaviour"; Task 8 fault demonstrations]. |

### G2: Pick up and place defined objects

**What the brief says:** no specific size or weight. It requires:

- the objects to be **tutor-approved and lightweight** [2, success definition];
- **at least two** objects with different widths, shapes or surfaces [2, Task 10 step 1];
- no sharp, hot, biological, fragile or valuable objects [2, Task 10 step 1];
- at least **5 trials per object**, for both the original and the improved gripper [2, Task 10 step 7];
- at least **5 pick-and-place trials**, recording success rate and cycle time [2, Task 11, INT-09].

**Suggested size and weight, from what the parts can do (Sections 9.5 and 9.6):**

| Limit | Estimate | Basis |
| --- | --- | --- |
| Maximum payload | **≤ 5 g** | Shoulder torque budget: ~9 g calculated upper limit, halved for margin (Section 9.5). |
| Object width | **~10–20 mm** | The gripper's jaw gap changes by ~19 mm over the firmware's 30–150° gripper range, from the gear and rack geometry (Section 9.6). |

**Suggested objects:** print both on the same printers as the arm, so they cost nothing and their size is known.

| Object | Shape | Size | Mass if printed **solid** PLA | Why |
| --- | --- | --- | --- | --- |
| A | Cube | 15 mm | ≤ 4.2 g | Flat faces, inside the width and mass limits even at 100 % infill. |
| B | Cylinder | Ø12 mm × 20 mm | ≤ 2.8 g | Round surface and a different width, as Task 10 requires. |

Masses are volume × PLA density 1.24 g/cm³ [20]. Printing at normal infill only makes them lighter, so the solid figure is an upper bound. **Both objects still need tutor approval** [2]. Weigh them on a lab scale to confirm.

### G3–G12: other requirements from the brief

| ID | Requirement | Source |
| --- | --- | --- |
| G3 | Manual control of all five named joints, each limited to its calibrated range. | [2, Task 6 host-interface requirements; INT-02] |
| G4 | Command protocol with `S`, `XY`, `SPEED`, `HOME`, `STOP`, `STATUS` and `RESET_FAULT`, with ACK/ERR replies, units and bounds. | [2, Task 6 command table] |
| G5 | Never move on a parser or validation failure, and never silently clamp an unsafe Cartesian request. | [2, Task 5 "Mandatory behaviour"] |
| G6 | Safety state machine (INIT, IDLE, VALIDATE, MOVE, HOLD, FAULT). Communication watchdog. STOP that clears queued commands. A fault reset that doesn't replay the failed command. | [2, Task 8, Figure 5] |
| G7 | Joint, workspace, path and collision checks before motion, including intermediate points, not just the end point. | [2, Task 8 "Critical limitation"; Task 9] |
| G8 | At least three speed settings in explicit units. Synchronised multi-joint motion. STOP and timeout stay responsive during motion. | [2, Task 9] |
| G9 | A gripper improvement: evaluate the original, compare two concepts in a decision matrix, make a dimensioned CAD change and test it. | [2, Task 10] |
| G10 | A GUI with port and baud selection, HOME and STOP controls not distinguished by colour alone, a send-on-release or rate-limit strategy, status display and a timestamped log file. | [2, Task 6] |
| G11 | An emergency power cut that doesn't rely only on software. | [2, Task 3 "Emergency removal"; Task 8 electrical layer] |
| G12 | Calibration table for all five servos, a repeatable HOME pose, and backlash measured from both directions. | [2, Task 4] |

---

## 3. Constraints

### 3.1 Group constraints, with the reason for each

| ID | Constraint | Reason |
| --- | --- | --- |
| C1 | Use only the Arduino Nano, the PCA9685, the five servos and the other supplied parts, plus wires and 3D-printed parts. | The parts the module supplies. It keeps the build reproducible for every group. |
| C2 | Finish as quickly as possible. | Other modules compete for time. The brief carries 15 % of the module mark [2, "Mark allocation"], so time should go where the marks are (C11). |
| C3 | Little or no budget. Don't buy parts that weren't supplied. Only use extra items the university will reasonably provide for free (lab bench equipment, printing, consumables). | No funding for the project. |

**How these conflict with the brief, and how we resolve them:**

- **Servo power (C1, C3 vs. the brief).** The brief requires the servo rail to come from a **tutor-approved external supply with current limiting and a switch or fuse**, not the Nano's USB or logic pin [2, Task 3]. A lab bench power supply with a current limit meets this. It is standard teaching-lab equipment, so it falls under C3's "the uni will reasonably provide". If only the breadboard module (~0.7 A [19]) is available, firmware has to limit simultaneous motion instead (Section 9.2).
- **Emergency disconnect (C1 vs. G11).** The bench supply's output switch, or unplugging the servo lead, counts as "an accessible servo-power switch or disconnect" [2, Task 3]. No extra part is needed.
- **Bulk capacitor.** The brief asks for local bulk capacitance on the servo rail to be checked [2, Task 3]. First check whether our PCA9685 board already has a capacitor fitted. If it doesn't, ask the lab for one, because it's a common free consumable.
- **Fasteners.** SG90 servos come with horns and mounting hardware [3]. Check whether that's enough for the printed parts. If it isn't, ask the lab for screws.
- **Measurement tools.** Use the lab's oscilloscope or logic analyser, multimeter and scale. Section 9.3 explains why the scope is needed.
- **Vision.** No camera is supplied, so the optional vision target is out of scope [2, Figure 1].

**How "as quickly as possible" works with the gates (C2 vs. C9):** gates can't be skipped (C9), so speed has to come from doing independent work **in parallel**, not from shortcuts:

1. Firmware, GUI and IK simulation (software side of G3 and G4) run while parts print and are dry-fitted (G1). Firmware v1.0.0 already exists [18].
2. Order prints by longest print time first.
3. Run the cheap paper checks (Sections 9.2 and 9.3) before hardware tests, so faults are found before they can damage anything.

Physical motion still waits for G1 and G2 to pass [2, "Safe and responsible use"]. The brief has no deadline: WebLearn's submission instructions take precedence [2, "References and source acknowledgement"]. **Record the WebLearn deadline in the Gantt chart.**

### 3.2 Constraints from the brief

| ID | Constraint | Source |
| --- | --- | --- |
| C4 | Low-voltage, tutor-approved equipment only. | [2, "Safe and responsible use"] |
| C5 | No physical motion until assembly, power, polarity, common ground, current limit, servo orientation and software limits are checked. Commission one servo at a time, at low speed. | [2, "Safe and responsible use"; Task 3] |
| C6 | Servo rail on an external supply, never the Nano's USB or logic rail. | [2, Task 3] |
| C7 | Starter code compiled and audited before any servo is energised. Unchanged starter code doesn't count as our work. Keep an unchanged baseline copy. | [2, "Starter-code status"; Task 5] |
| C8 | Every change traceable through dated versions or a repo. Final tag matches the demonstration. | [2, "Case-study files"; Task 11] |
| C9 | Gates are conditional. A no-go repeats the current gate or an earlier one. Tutor approval is needed before bounded physical Cartesian moves. | [2, Figure 2; G5] |
| C10 | Objects must not be sharp, hot, biological, fragile or valuable. | [2, Task 10] |
| C11 | 100 marks, 15 % of the module: IK and safety 30, hardware 20, serial 20, speed and gripper 20, report and demo 10. | [2, "Mark allocation"] |
| C12 | Any AI-assisted code, text, CAD or analysis must be checked, understood, adapted and disclosed. Measured data must be real. | [2, Task 12 "Academic integrity"] |
| C13 | Every member keeps a dated logbook and must be able to explain all the interfaces. | [2, "Group organisation"] |

### 3.3 Constraints that follow from the parts (derived)

These aren't written in the brief. They follow from the facts about the parts we have to use.

| ID | Derived constraint | Reasoning |
| --- | --- | --- |
| C14 | **Open loop only.** The controller can't detect where a joint really is. | The SG90 has no feedback line to the controller [18], and no sensors are supplied (C1). Accuracy depends on calibration, so G12 is critical. |
| C15 | **Payload ≤ ~5 g.** | Shoulder torque budget (Section 9.5). |
| C16 | **Object width ~10–20 mm.** | Gripper gear and rack travel (Section 9.6). |
| C17 | **Positioning floor of ~±1° per joint.** | The SG90 dead band is coarser than the PCA9685's ~0.46° step (Section 9.3). |
| C18 | **One PWM frequency for all channels.** | The PCA9685 sets a single frequency for every output [11]. All five servos run at 50 Hz. |
| C19 | **Small firmware footprint.** | 2 KB SRAM [13]. Firmware v1.0.0 already uses 57.6 % of RAM [18], so new features must be lean. |
| C20 | **The GUI must run on uni PCs without installing anything.** | Hardware and upload work happens on uni PCs. The GUI uses built-in tkinter and bundled pyserial for this reason [18]. |

---

## 4. Research questions

Slide 8 starts research with *define your objectives* and *formulate research questions* [1, slide 8]. Each question below is answered in Section 9.

| ID | Question | Why it matters | Answered in |
| --- | --- | --- | --- |
| RQ1 | What are the real limits of the parts, and do the sources agree? | Decisions have to be justified from datasheets or reliable sources [2, LO2]. | 9.1 |
| RQ2 | Can the supply drive the servos without brownout? | A controller reset is a G2 no-go [2]. | 9.2 |
| RQ3 | Is the PWM signal accurate and fine-grained enough? | G1 accuracy depends on it. | 9.3 |
| RQ4 | What endpoint accuracy can we achieve? | Needed to set a testable G1 criterion. | 9.4 |
| RQ5 | How much can the arm lift? | Sets the G2 object mass. | 9.5 |
| RQ6 | How wide can the gripper open? | Sets the G2 object width. | 9.6 |
| RQ7 | Which safety principles apply? | Safety controls and a residual-risk statement are required [2, G7]. | 9.7 |

**Why questions come before data collection:** writing the questions first means only the data that answers them gets collected, and each answer can be checked against what it was meant to decide. This follows the lecture's order of *formulate research questions → … → collect data → analyse → validate* [1, slide 8].

---

## 5. System boundary

**Why:** ISO 12100, the base standard for risk assessment of machinery, starts by *determining the limits of the machinery*, covering use limits and space limits, before any hazard is identified [14]. The brief asks for the same step in Task 1, step 3 [2].

- **Inside the boundary:** the five servos, the Nano, the PCA9685, the printed body, the wiring, the firmware and the GUI (Section 1.1).
- **Outside the boundary:** the host PC; lab equipment (bench supply, scope, scale); the objects; the optional vision system, which is excluded (C1).
- **Use limits:** supervised lab demonstration, at low speed, with an immediate power-cut route [2]. Foreseeable misuse: a hand in the workspace while powered, a stale or malformed command, servos powered from USB [2].
- **Space limits:** the arm's reach above the table. `L1 = L2 = 49.6 mm` were measured pivot to pivot on the STLs. `L3 ≈ 73 mm` and the shoulder height `≈ 44 mm` are estimates still to be measured on the built arm [18]. The workspace is fixed at G4.

---

## 6. Stakeholders

**Why:** ISO/IEC/IEEE 29148 treats stakeholder needs as the input that gets transformed into requirements [16]. Every Stage 3 requirement should trace back to a need listed here.

| Stakeholder | Needs (source) |
| --- | --- |
| Operators (group members) | No unexpected motion, a clear STOP, hands outside the workspace [2]. HOME and STOP controls not distinguished by colour alone [2, Task 6]. |
| Tutor and assessor | Reproducible, measured results traced to requirements [2]. Approves objects and physical moves [2, G5, Task 10]. |
| Others in the lab | Low voltage, approved equipment, a power-disconnect route [2]. IEEE code: public safety comes first [17]. |
| University (H&S, IP) | Health and safety, licensing [2, LO5]. Bundled pyserial is BSD-3-Clause, and its licence is kept in the repo [18]. |
| Hypothetical customer or business | User, customer, business, cost, security and operational needs [2, LO4]. Low cost is directly supported by C3. |
| Environment | Lifecycle and end of life, sustainability [2, LO3, LO5]. Printing the test objects keeps material use small and known. |

---

## 7. Data collection method

**Process:** for each component, take the manufacturer's datasheet first, then check it against independent sources, and record any disagreement. Calculations use the **worst-case** value whenever sources disagree.

**Why:** the SG90 is sold by many vendors, and their figures don't agree (Section 9.1). Planning with the worst case means a design that works on paper will still work if our servos turn out to be at the weak end of the range. The figures are confirmed by measurement at G2.

---

## 8. Calculation conventions

- Torque is given in gram-force millimetres (gf·mm). 1 kgf·cm = 10 000 gf·mm.
- Angles are in degrees. Lengths are in mm.
- Every calculation can be repeated from the numbers in the cited sources and this repo.

---

## 9. Analysis

### 9.1 Component facts, with conflicts noted (RQ1)

| Parameter | Value(s) found | Sources | Value used for planning |
| --- | --- | --- | --- |
| SG90 supply voltage | 4.8 V nominal (~5 V) | [3, 4] | 5 V |
| SG90 stall torque | 1.8 kgf·cm at 4.8 V (TowerPro) · 1.2 kg·cm at 4.8 V (retailer) · 1.7 kg·cm (measured) | [3] · [5] · [6] | **1.2 kgf·cm** (lowest) |
| SG90 speed | 0.1 s/60° at 4.8 V, no load | [3, 4] | Upper bound only |
| SG90 current | Moving: 100–250 mA (measured) · Stall: 360 mA (measured), ~550 mA, 0.65 A | [6] · [6], [7], [8] | **0.65 A stall** (highest) |
| SG90 pulse range | 1–2 ms with 1.5 ms centre · 0.5–2.4/2.5 ms measured | [3, 10] · [9] | Measure for each servo (G12) |
| SG90 dead band | 10 µs | [3, 4] | 10 µs |
| SG90 mass | 9 g | [3] | 9 g |
| PCA9685 | 12-bit (4096 steps), ~24–1526 Hz, 25 MHz *typical* internal oscillator | [11] | See 9.3 |
| Arduino Nano | ATmega328P, 5 V, 16 MHz, 32 KB flash, 2 KB SRAM | [13] | — |
| Breadboard power module (MB102 type) | Max output 700 mA | [19] | 0.7 A |
| PLA density | 1.24 g/cm³ | [20] | 1.24 g/cm³ |

### 9.2 Power budget (RQ2)

- **All 5 servos moving, no stall:** 5 × 100–250 mA = **0.5–1.25 A** [6].
- **One servo stalled while the other 4 move:** 0.65 A + 4 × 0.25 A ≈ **1.65 A** [6, 8].
- **All 5 stalled (worst case):** 5 × 0.65 A ≈ **3.25 A** [8].
- **MB102-type module rating:** **0.7 A** [19].

**Finding F1:** a 0.7 A module can't reliably power all-joint motion, so brownout (R-01) is likely. **Preferred fix, at no cost:** a lab bench supply set to 5 V with a current limit. Start low for one-servo tests, as the brief requires [2, Task 3], then raise it once multi-servo current has been measured. **Fallback if only the module is available:** firmware moves one joint at a time and keeps the speed low, accepting that G8 synchronised motion is then limited, and records that limitation.

### 9.3 PWM resolution and accuracy (RQ3)

- **Resolution:** 20 000 µs / 4096 ≈ **4.9 µs per step** at 50 Hz [11]. Over the firmware's 500–2400 µs range (1900 µs / 180° ≈ 10.6 µs/°) [18], that's **≈ 0.46° per step**. The dead band of 10 µs [3, 4] is **≈ 0.95°**. The servo is therefore the limit, not the PCA9685 (C17).
- **Accuracy (finding F2):** the datasheet gives the oscillator as *typical* 25 MHz [11], but both the baseline code and our firmware assume 27 MHz [18; `00_baseline/sample_code/Arduino Code.cpp`, line 94]. If the real clock were 25 MHz:
  - prescale = round(27 MHz / (4096 × 50 Hz) − 1) = 131
  - actual frequency = 25 MHz / (4096 × 132) ≈ **46.2 Hz**
  - pulses shrink by 25/27, so a 1500 µs command comes out as ~1389 µs, about **10° of error**.

  **Measure the real PWM frequency with the lab scope before calibration**, and correct `PCA_OSC_HZ` to match.

### 9.4 Achievable endpoint accuracy (RQ4)

Arc length is radius × angle in radians, and 1° = 0.01745 rad:

- 1° at the shoulder moves the wrist point by up to (L1 + L2) × 0.01745 = 99.2 × 0.01745 ≈ **1.73 mm**.
- 1° at the elbow moves it by up to L2 × 0.01745 = 49.6 × 0.01745 ≈ **0.87 mm**.
- **Worst case, both errors in the same direction:** ≈ **2.6 mm** from dead band alone. Link lengths are from [18].

Backlash, horn misalignment and link flex add to this, and are measured at G2 and G5 [2, Task 4 step 4; Task 7 step 6]. That's why the G1c physical tolerance is proposed as ±5 mm, not ±2.6 mm. The proposal gets revised once real backlash is known.

### 9.5 Payload capacity (RQ5)

**Method:** add up the static torque at the shoulder with the arm stretched out horizontally (the worst case for gravity). The payload is whatever torque is left in the budget. Both the masses and the layout are worked out pessimistically:

- **Link masses:** STL volume (measured with trimesh) × 1.24 g/cm³ [20], as if printed **100 % solid**. Real prints with infill are lighter, so this overestimates.
- **Servo positions:** all three distal servos (elbow, wrist, gripper) are assumed to ride on the arm. If the design puts any of them at the base, the real torque is lower.

| Load | Mass (g) | Distance from shoulder (mm) | Torque (gf·mm) |
| --- | --- | --- | --- |
| Arm (5.89 cm³) | 7.3 | 24.8 (L1/2) | 181 |
| ArmSpring (6.73 cm³) | 8.3 | 24.8 | 206 |
| Elbow servo | 9 [3] | 49.6 (L1) | 446 |
| ArmFront (5.06 cm³) | 6.3 | 74.4 (L1 + L2/2) | 469 |
| Wrist servo | 9 | 99.2 (L1 + L2) | 893 |
| Gripper servo | 9 | ~130 | 1170 |
| Gripper parts (FrontPlate, gear, racks, jaws: 6.34 cm³) | 7.9 | ~140 | 1106 |
| **Arm's own weight, total** | | | **≈ 4470** |

**Torque budget:** a servo shouldn't be planned to work at its stall torque. ROBOTIS advises against planning to use stall torque continuously, and recommends staying within the continuous rating [21]. The SG90 has no published continuous rating, so this analysis uses **50 % of the lowest stall torque**. That's a planning assumption, not a sourced figure.

- Budget: 0.5 × 1.2 kgf·cm = 0.6 kgf·cm = **6000 gf·mm** [5].
- Left for payload: 6000 − 4470 = **1530 gf·mm**.
- At the tip (L1 + L2 + L3 = 172 mm [18]): 1530 / 172 ≈ **8.9 g**.

**Finding F4:** the **calculated upper limit is ~9 g**. Halving it again allows for acceleration loads and the uncertainty in this estimate, which gives the **≤ 5 g** design limit used in G2.

**Uncertainties, stated so they aren't hidden:** with the TowerPro figure of 1.8 kgf·cm [3], the limit rises to ~26 g. The real servo layout and infill will lower the arm's own weight. Stall-to-continuous ratios for small servos aren't published for the SG90. **Verify with a hold test:** hold a weighed object at full reach and watch for sag, buzz or heat (risk R-05) during G6.

### 9.6 Gripper opening (RQ6)

Measured from the STLs with trimesh:

- **GripperGear:** tip diameter 10.8 mm, root radius ~3.4 mm, **~10 teeth**.
  - With tip diameter = m(z + 2), the module m ≈ 10.8 / 12 = 0.9 mm.
  - Pitch radius r = m × z / 2 ≈ 4.5 mm.
- **Racks:** each is 30 mm long. Two racks on opposite sides of one gear move in opposite directions [2, Task 10].
- **Rack travel** per degree of gear rotation = r × π / 180 ≈ 0.0785 mm.
- Over the firmware's gripper limits of 30–150°, a 120° range [18]: each rack moves ≈ 9.4 mm, so **the jaw gap changes by ≈ 19 mm**.

**Finding F5:** the jaw gap changes by about 19 mm. The absolute gap depends on how the jaws are assembled, which the STLs can't tell us. So **10–20 mm wide objects** is the working estimate. Measure the real open and closed gap after assembly at G1, and adjust the object sizes then. The tooth count from the mesh is approximate (10 or 11), which changes r by less than 0.1 mm.

### 9.7 Safety principles (RQ7)

- **IEEE Code of Ethics:** members commit to hold paramount the safety, health and welfare of the public, and to disclose promptly factors that might endanger it [17]. Lecture 3 cites the code as the reason for rigorous testing [1, slide 37].
- **ISO 12100:** risk assessment is iterative. After every protective measure, go back to hazard identification, because a measure can introduce a new hazard [14]. The register gets reviewed at every gate.
- **ISO 10218-1:2025:** covers the safety of industrial robots [15]. We **don't claim compliance**. We only borrow its order: safe design first, then protective measures, then information for use.

---

## 10. Outputs to Stage 2

1. The problem statement, confirmed components, goals G1–G12 and constraints C1–C20.
2. Findings:
   - **F1** The supply is likely undersized → use a lab bench supply, or limit simultaneous motion.
   - **F2** Measure the PWM frequency before calibration (~10° risk).
   - **F3** SG90 figures conflict → measure each servo.
   - **F4** Payload ≤ 5 g.
   - **F5** Object width ~10–20 mm.
3. New risks for the register:
   - R-08 PWM frequency error from the unverified oscillator.
   - R-09 SG90 specification uncertainty.
   - R-10 Payload overload causing sag or stall at full reach.

## 11. Open decisions for the group (before G0)

- [ ] Ask the lab for a bench supply with a current limit, and check its availability for test sessions.
- [ ] Check whether the PCA9685 board has a bulk capacitor fitted.
- [ ] Check whether the servo hardware covers all the fasteners the printed parts need.
- [ ] Get tutor approval for Objects A and B.
- [ ] Check that a scope or logic analyser and a scale are available.
- [ ] Record the WebLearn deadline, and build the Gantt chart backwards from it.
- [ ] Assign roles in `01_management/team_charter.md`.

---

## References

Accessed 2026-11-17 unless stated.

1. D. Mariyanayagam, *Lecture 3: Engineering Cycle, Mixed Signal Processing and PCB Design, Part One: Idea to Prototype*, CT6056 Applied Robotics, London Metropolitan University, 2026/27. Slides 4–9, 20, 23, 37.
2. D. Mariyanayagam, *CT6056 Group Case Study: The Design and Control of a 5-DOF Micro Arm*, v3.0, London Metropolitan University, 2026/27. Copy in `00_baseline/brief/`.
3. TowerPro, *SG90 9 g Micro Servo* datasheet (university-hosted copies). https://tams.informatik.uni-hamburg.de/lehre/2023ss/vorlesung/es/doc/sg90.pdf; https://bit.nkust.edu.tw/~csshieh/teach/111A/mc/datasheet/SG90.pdf
4. Rapid Electronics, *Tower Pro SG90 Mini Servo, 37-1330*, datasheet. https://static.rapidonline.com/pdf/37-1330t.pdf
5. ThinkRobotics, *SG90 9G Micro Servo Motor*, product specifications. https://thinkrobotics.com/products/sg90-9g-micro-servo-motor
6. ProtoSupplies, *Servo Motor Micro SG90*, technical specifications (measured values). https://protosupplies.com/product/servo-motor-micro-sg90/
7. Circuit Rocks, *SG90 9g Servo*. https://docs.circuit.rocks/motors/sg90-servo/
8. ServoDatabase, *Yahboom SG90*. https://servodatabase.com/servo/yahboom/sg90
9. Arduino Forum, "Servo pulse widths". https://forum.arduino.cc/t/servo-pulse-widths/286525
10. Luxorparts / Kjell & Company, *SG90 Micro Servo* datasheet (2017). https://www.kjell.com/globalassets/mediaassets/701916_87897_datasheet_en.pdf
11. NXP Semiconductors, *PCA9685 16-channel, 12-bit PWM Fm+ I²C-bus LED controller*, Rev. 4, 16 April 2015. https://www.nxp.com/docs/en/data-sheet/PCA9685.pdf; https://www.nxp.com/products/PCA9685
12. NXP Semiconductors, *PCA9685PW product information*. https://nxp.com/part/PCA9685PW
13. Arduino, *Arduino Nano (A000005) tech specs*, as reproduced by PiShop. https://www.pishop.us/product/arduino-nano/. Cross-check against `tutorials/tutorial_1/NanoV3.3_sch.pdf`.
14. ISO 12100:2010, *Safety of machinery: general principles for design, risk assessment and risk reduction*. Structure as given in the ISO/DIS 12100:2025 draft contents (§5.3 limits of machinery). https://standardsdevelopment.elot.gr/drafts/2130. Iterative approach: Leuze, *Risk Assessment* white paper. https://files.leuze.com.cn/shopware/media/ee/2d/e7/1639045358/wp_risikobeurteilung_en_144dpi.pdf
15. ISO 10218-1:2025, *Robotics: safety requirements, Part 1: industrial robots*. https://www.iso.org/standard/73933.html
16. ISO/IEC/IEEE 29148:2018, *Requirements engineering*, §5.2.2–5.2.3, §6.3 (contents: https://www.din.de/de/mitwirken/normenausschuesse/nia/veroeffentlichungen/wdc-beuth:din21:299567544/toc-3025650/download). SEBoK summary: https://sebokwiki.org/draft/Special:Random/Absorption_(glossary)
17. IEEE, *IEEE Code of Ethics*. https://ieee-cas.org/about/ieee-code-ethics
18. S. Ahmed, *MicroArm firmware and control GUI v1.0.0*: `05_firmware/MicroArm_Firmware/MicroArm_Firmware.ino` and `05_firmware/MicroArm_v1.0_README.md`, this repository, commit `0b35f32`.
19. YwRobot, *MB102 breadboard power supply* specifications, via PETERVIS. https://www.petervis.com/Raspberry_PI/Breadboard_Power_Supply/YwRobot_Breadboard_Power_Supply.html
20. Prusa Polymers, *Prusament PLA technical datasheet*, v1.1, 2022 (density 1.24 g/cm³, ISO 1183). https://www.matterhackers.com/r/xFvGl8. Also Bambu Lab, *PLA Basic TDS*. https://cdn.shopify.com/s/files/1/0380/5149/files/Bambu_PLA_Basic_Technical_Data_Sheet.pdf
21. ROBOTIS, "Torque ratings: stall vs. continuous". https://robotis.us/robotis-blog/torque-ratings
