# Stage 1: Identification of the problem

**Gate:** G0, the planning gate: roles, requirements, interfaces, plan and first risk controls agreed [2, Figure 2]
**Author:** Shuayeb Ahmed · Draft 3 · 2026-11-17

## Summary

We are building a 5-DOF (five-joint) micro robotic arm: both its 3D-printed body and its firmware. It has to move to commanded positions within a stated accuracy and pick up small objects, using only the supplied parts, wires and 3D-printed parts. The research for this stage found five things that shape the rest of the project:

| # | Finding | What we do about it | Section |
| --- | --- | --- | --- |
| F1 | The breadboard power module (0.7 A) can't power all five servos moving at once (0.5–1.25 A). | Use a lab bench power supply, or move one joint at a time. | 8.2 |
| F2 | The code assumes the servo controller's clock is 27 MHz, but the datasheet says 25 MHz. If the datasheet is right, every joint is about 10° off. | Measure the signal with a scope before calibrating. | 8.3 |
| F3 | Published SG90 servo figures disagree. | Plan with the worst figure, then measure our own servos. | 8.1 |
| F4 | The arm can safely lift about **5 g**. | Objects of 5 g or less. | 8.5 |
| F5 | The gripper opens about **19 mm** over its range. | Objects about 10–20 mm wide. | 8.6 |

The lecture splits this stage into two parts [1, slide 6]:

- **The problem statement:** what needs solving (Sections 1–5).
- **Research and analysis:** the context, the stakeholders and the evidence (Sections 6–9).

Every fact has a numbered source `[n]`, listed at the end. Every process step has a **Why** that explains the reasoning.

---

## 1. Problem statement

> **We need to design and build a 5-DOF micro robotic arm, covering both its physical body and its firmware. The body is made from 3D-printed parts, five servo motors and an Arduino Nano that drives a PCA9685 servo controller board. The arm has to move to commanded joint angles and coordinates within a stated accuracy, and pick up and place small, light objects. It has to do this repeatably and safely, using only the supplied parts plus wires and 3D-printed parts.**

The lecture says a good problem statement describes where things are now, where they need to be, and why the gap matters [1, slide 7]:

| | |
| --- | --- |
| **Now** | A kit of unassembled parts. The starter code hasn't been approved for moving the arm, and may contain unsafe assumptions or be missing error handling [2, "Starter-code status"]. |
| **Needed** | An assembled, calibrated arm that meets the measurable goals in Section 3. |
| **Why it matters** | An arm that moves unexpectedly can hurt someone or break parts. The brief treats bad input, a lost connection, a timeout, an unreachable target and a limit being exceeded as reasons to stop or hold [2, "Safe and responsible use"]. |
| **How success is judged** | Results must be repeatable, measured and linked to requirements. One video of one good move isn't enough [2, success definition]. |

**Why the statement doesn't pick a design:** it says what the arm must do and with which parts. It doesn't say how, e.g. which gripper shape or speed method. Choosing a design now would skip Stage 2, where options get compared [1, slides 5, 9].

---

## 2. What we are building with

| Part | Qty | How we know | Notes |
| --- | --- | --- | --- |
| Servo motors | 5 | The brief assigns controller channels 0–4 to base, swing, arm one, arm two and gripper [2, Task 3], and requires all five to be fitted and calibrated [2, Task 4]. | The brief doesn't name a model. Our kit has **SG90** micro servos (repo README). Check the labels. |
| Arduino Nano | 1 | The brief allows a Nano or an STM32 board [2, Task 3, Task 5]. We chose the Nano. | ATmega328P chip, 5 V, 16 MHz, 32 KB program memory, 2 KB working memory (RAM) [13]. |
| PCA9685 servo controller | 1 | In the brief's system diagram and wiring table [2, Figure 1, Task 3]. | Makes 16 independent servo signals, each with 4096 steps, controlled over I²C (a two-wire bus) [11]. We use 5 channels. |
| 3D-printed parts | 11 | Listed in the brief [2, Appendix B]. Files are in `00_baseline/3d_models/`. | Foot, Swing, Arm, ArmSpring, ArmFront, FrontPlate, GripperGear, two racks, two jaws. |
| Wires | As needed | Our constraint C1 | Jumper wires for I²C, servo leads to the controller. |

---

## 3. Goals

G1 and G2 are our two main goals, written so they can be tested. G3–G12 are further requirements from the brief. Stage 3 turns each goal into numbered requirements in `02_requirements/requirements.csv`.

### G1: Move to commanded angles and coordinates, within a stated accuracy

**Why the goal says "within a tolerance" instead of "exactly":** no physical machine is exact, and a goal that can't be measured can't pass or fail. Three things limit how accurate this arm can be:

1. **The servos don't report their position.** The Nano only knows the angle it *asked for*, not the angle the joint actually reached [18].
2. **Each servo ignores very small changes.** Its *dead band* is 10 µs of signal [3, 4], which is about ±1° of movement (Section 8.3).
3. **Slack in the gears, horn alignment and bending in the printed links all add error.** The brief asks us to measure these [2, Task 4 step 4; Task 7 step 6].

The dead band alone gives up to about **2.6 mm** of error at the end of the arm (Section 8.4). So:

| Sub-goal | Pass if… | Why this number |
| --- | --- | --- |
| G1a: joint angle | Each joint reaches the commanded angle within **±2°**. | ±1° dead band, plus margin for gear slack. Confirm at G2. |
| G1b: coordinate maths | When an IK answer (joint angles worked out from a target point) is fed back through FK (target point worked out from joint angles), it lands within **0.1 mm** of the target. | Checks the maths separately from the hardware. The brief requires this limit to be set before testing [2, Task 7 step 4]. |
| G1c: real position | The measured position of the arm's end is within **±5 mm** of the target. | 2.6 mm from the dead band, plus gear slack and measuring error. Tighten after the G5 measurements. |
| G1d: bad targets | An unreachable or out-of-limits target causes **no movement** and a clear error. | Required by the brief [2, Task 5 "Mandatory behaviour"; Task 8]. |

### G2: Pick up and place defined objects

**What the brief says:** it gives no size or weight. It requires:

- the objects to be **lightweight and approved by the tutor** [2, success definition];
- **at least two** objects, with different widths, shapes or surfaces [2, Task 10 step 1];
- nothing sharp, hot, biological, fragile or valuable [2, Task 10 step 1];
- **5 trials per object** with both the original and the improved gripper [2, Task 10 step 7];
- **5 or more** full pick-and-place runs, recording success rate and time per cycle [2, Task 11, INT-09].

**What our parts can handle:**

| Limit | Value | Where it comes from |
| --- | --- | --- |
| Weight | **5 g or less** | Shoulder strength, worked out in Section 8.5. |
| Width | **about 10–20 mm** | How far the gripper jaws travel, Section 8.6. |

**Suggested objects:** print them. They cost nothing and their size is known exactly.

| Object | Shape | Size | Weight even if printed fully solid | Why |
| --- | --- | --- | --- | --- |
| A | Cube | 15 mm | 4.2 g or less | Flat sides. |
| B | Cylinder | 12 mm across × 20 mm tall | 2.8 g or less | Round, and a different width, as the brief requires. |

Weight = volume × PLA density 1.24 g/cm³ [20]. Normal printing with partial infill makes them lighter, so these are maximums. Weigh them on a lab scale, and **get the tutor's approval** [2].

### G3–G12: other requirements from the brief

| ID | Requirement | Source |
| --- | --- | --- |
| G3 | All five joints can be controlled by name, each within its own calibrated range. | [2, Task 6; INT-02] |
| G4 | A command set of `S` (one joint), `XY` (coordinates), `SPEED`, `HOME`, `STOP`, `STATUS` and `RESET_FAULT`. Every command gets an acknowledgement or an error, with units and limits. | [2, Task 6] |
| G5 | Never move after a bad or invalid command. Never quietly change an unsafe coordinate into a different one. | [2, Task 5 "Mandatory behaviour"] |
| G6 | A safety state machine (a fixed set of modes: start-up, idle, checking, moving, holding, fault). Stop if the PC connection is lost. STOP clears waiting commands. Clearing a fault never repeats the failed command. | [2, Task 8, Figure 5] |
| G7 | Check joint limits, the workspace and collisions along the whole path before moving, not just at the end point. | [2, Task 8 "Critical limitation"; Task 9] |
| G8 | At least three speed settings in real units (e.g. degrees per second). Joints that move together start and finish together. STOP still works during a move. | [2, Task 9] |
| G9 | Improve the gripper: test the original, compare two new designs in a scoring table, make a measured CAD change, then test it. | [2, Task 10] |
| G10 | A GUI with a port and speed selector, HOME and STOP buttons that don't rely on colour alone, sliders that send only on release (or are rate-limited), a status display and a timestamped log file. | [2, Task 6] |
| G11 | A way to cut servo power that doesn't depend on software. | [2, Task 3 "Emergency removal"; Task 8] |
| G12 | A calibration table for all five servos, a repeatable HOME position, and gear slack measured by approaching from both directions. | [2, Task 4] |

---

## 4. Constraints

### 4.1 Our constraints

| ID | Constraint | Why |
| --- | --- | --- |
| C1 | Use only the Nano, the PCA9685, the five servos and the other supplied parts, plus wires and 3D-printed parts. | These are what the module supplies. |
| C2 | Finish as quickly as possible. | Other modules need time too. This project is worth 15 % of the module [2, "Mark allocation"], so effort should go where the marks are (C11). |
| C3 | No budget. Don't buy anything that wasn't supplied. Only use extras the university would reasonably lend or give for free (lab equipment, printing, small consumables). | No funding. |

### 4.2 Where our constraints clash with the brief, and how we resolve them

| Clash | Resolution |
| --- | --- |
| **Servo power.** The brief requires a separate, current-limited supply with a switch or fuse, not the Nano's USB [2, Task 3]. | A **lab bench power supply** does this, and it's standard free lab equipment (C3). If only the breadboard module (0.7 A [19]) is available, the firmware moves one joint at a time (Section 8.2). |
| **Emergency power cut** (G11). | The bench supply's output switch, or pulling the servo power lead, counts as a disconnect [2, Task 3]. No extra part needed. |
| **Smoothing capacitor.** The brief asks for one on the servo power line to be checked [2, Task 3]. | Check whether the PCA9685 board already has one. If it doesn't, ask the lab; they're cheap, standard stock. |
| **Screws.** | SG90 servos come with horns and screws [3]. If those aren't enough for the printed parts, ask the lab. |
| **Test equipment.** | Borrow the lab's oscilloscope or logic analyser, multimeter and scale. |
| **Camera.** The brief allows an optional vision system [2, Figure 1]. | None is supplied, so it's out of scope. |
| **Speed vs. gates** (C2 vs. C9). The gates can't be skipped. | Save time by doing independent work **at the same time**: write and test firmware, GUI and IK in simulation while parts print. Firmware v1.0.0 already exists [18]. Print the longest parts first. Do the paper checks (Sections 8.2, 8.3) before hardware tests, to catch faults before they damage anything. |
| **Deadline.** The brief doesn't give one. The WebLearn instructions take priority [2, "References and source acknowledgement"]. | Put the WebLearn deadline in the Gantt chart and plan backwards from it. |

### 4.3 Constraints from the brief

| ID | Constraint | Source |
| --- | --- | --- |
| C4 | Low-voltage, tutor-approved equipment only. | [2, "Safe and responsible use"] |
| C5 | Don't move the arm until assembly, power, polarity, common ground, current limit, servo orientation and software limits are all checked. Test one servo at a time, slowly. | [2, "Safe and responsible use"; Task 3] |
| C6 | Servos are powered separately from the Nano's USB or logic supply. | [2, Task 3] |
| C7 | Compile and review the starter code before powering any servo. Unchanged starter code doesn't count as our work. Keep an untouched copy. | [2, "Starter-code status"; Task 5] |
| C8 | Every change must be traceable (we use git). The final tagged version must match what we demonstrate. | [2, "Case-study files"; Task 11] |
| C9 | A gate is only passed with its evidence. Failing sends us back to that gate or an earlier one. The tutor must approve physical coordinate moves. | [2, Figure 2; G5] |
| C10 | No sharp, hot, biological, fragile or valuable objects. | [2, Task 10] |
| C11 | Marks: IK and safety 30, hardware 20, serial control 20, speed and gripper 20, report and demo 10. Total 100, worth 15 % of the module. | [2, "Mark allocation"] |
| C12 | Any AI-assisted code, text, CAD or analysis must be checked, understood, adapted and declared. Test data must be real. | [2, Task 12 "Academic integrity"] |
| C13 | Every member keeps a dated logbook and can explain how the whole system connects. | [2, "Group organisation"] |

### 4.4 Constraints that follow from the parts

These aren't written in the brief. They follow from the facts about the parts.

| ID | Constraint | Reason |
| --- | --- | --- |
| C14 | **No position feedback.** The arm can't check where a joint really is. | SG90s don't report position [18], and no sensors are supplied (C1). Calibration (G12) is what makes it accurate. |
| C15 | **Payload 5 g or less.** | Section 8.5. |
| C16 | **Object width about 10–20 mm.** | Section 8.6. |
| C17 | **About ±1° is the best positioning per joint.** | The servo's dead band is coarser than the controller's smallest step (Section 8.3). |
| C18 | **All five servos share one signal frequency.** | The PCA9685 uses one frequency for all its outputs [11]. Ours is 50 Hz. |
| C19 | **Firmware must stay small.** | The Nano has 2 KB of RAM [13], and firmware v1.0.0 already uses 57.6 % of it [18]. |
| C20 | **The GUI must run on uni PCs without installing anything.** | Hardware work happens on uni PCs. The GUI uses Python's built-in tkinter and a bundled copy of pyserial for this reason [18]. |

---

## 5. What's inside and outside the system

**Why define this:** you can only find the hazards of a machine once you know what the machine is, who uses it and where it moves. ISO 12100, the main standard for machine risk assessment, starts with exactly this: the *limits of the machinery*, meaning its use limits and space limits [14]. The brief asks for the same thing [2, Task 1 step 3].

- **Inside:** the five servos, the Nano, the PCA9685, the printed body, the wiring, the firmware and the GUI.
- **Outside:** the PC; lab equipment (bench supply, scope, scale); the objects; a camera system (excluded, see 4.2).
- **How it's used:** a supervised lab demo, at low speed, with an immediate way to cut power [2].
- **Foreseeable misuse:** a hand in the way while the arm is powered; an old or garbled command; servos powered from USB [2].
- **Space it moves in:** its reach above the table. The two main links are 49.6 mm each, measured between pivot centres on the STL files. The gripper length (about 73 mm) and the shoulder height (about 44 mm) are estimates, still to be measured on the built arm [18]. The workspace is finalised at G4.

---

## 6. Who the arm is for (stakeholders)

**Why list them:** the requirements standard ISO/IEC/IEEE 29148 builds requirements from stakeholder needs [16]. Every requirement in Stage 3 should trace back to someone's need here, so nothing is added without a reason and nothing is missed.

| Stakeholder | What they need (source) |
| --- | --- |
| Operators (us, during tests and demos) | No surprise movement, an obvious STOP, hands kept clear [2]. HOME and STOP buttons that don't rely on colour [2, Task 6]. |
| Tutor and assessor | Repeatable, measured results tied to requirements [2]. Approval of objects and physical moves [2, G5, Task 10]. |
| Other people in the lab | Low voltage, approved equipment, a quick power cut [2]. The IEEE Code of Ethics puts public safety first [17]. |
| The university (health and safety, licensing) | Safe practice and correct licensing [2, LO5]. The bundled pyserial is BSD-3-Clause licensed, and its licence file is kept in the repo [18]. |
| A future customer or business | The brief asks us to consider user, customer, business, cost, security and operating needs [2, LO4]. Zero extra cost supports this (C3). |
| The environment | Lifecycle, end of life and sustainability [2, LO3, LO5]. Small printed test objects keep material use low. |

---

## 7. Research questions

**Why ask questions first:** deciding what we need to know before gathering data means we only collect what answers a question, and each answer can be checked against the decision it supports. This is the order the lecture gives: *define objectives → write research questions → … → collect data → analyse → validate* [1, slide 8].

| ID | Question | Why we need the answer | Answer |
| --- | --- | --- | --- |
| RQ1 | What are the real limits of each part, and do the sources agree? | The brief says decisions must be backed by datasheets or reliable sources [2, LO2]. | 8.1 |
| RQ2 | Can the power supply run the servos without the voltage dropping (brownout)? | A brownout resets the Nano, which fails G2 [2]. | 8.2 |
| RQ3 | Is the servo signal accurate and fine enough? | G1 accuracy depends on it. | 8.3 |
| RQ4 | How accurate can the arm's end position be? | To set a fair G1 target. | 8.4 |
| RQ5 | How much can the arm lift? | Sets the G2 object weight. | 8.5 |
| RQ6 | How wide does the gripper open? | Sets the G2 object width. | 8.6 |
| RQ7 | Which safety principles apply? | The brief requires safety controls and a statement of remaining risks [2, G7]. | 9 |

---

## 8. Analysis

**Method:** for each part, start with the manufacturer's datasheet, then check it against independent sources and note any disagreement. When sources disagree, plan with the **worst** value.

**Why:** SG90 servos are made by many vendors, and their figures differ (8.1). If the design works with the worst figure, it still works if our servos turn out to be weak. Measuring our own servos at G2 replaces these estimates with real values.

**Units:** lengths in mm, angles in degrees. Torque in gram-force millimetres (gf·mm): the turning force of 1 g of weight at 1 mm from the pivot. 1 kgf·cm = 10 000 gf·mm.

### 8.1 Part facts (RQ1)

| Fact | What the sources say | Sources | What we plan with |
| --- | --- | --- | --- |
| SG90 voltage | 4.8 V nominal (about 5 V) | [3, 4] | 5 V |
| SG90 maximum torque (stall torque: the most it can push before it stops turning) | 1.8 kgf·cm (manufacturer) · 1.2 kg·cm (retailer) · 1.7 kg·cm (measured) | [3] · [5] · [6] | **1.2 kgf·cm** (lowest) |
| SG90 speed | 0.1 s per 60° at 4.8 V, with no load | [3, 4] | Best case only. Slower under load. |
| SG90 current | Moving: 100–250 mA (measured). Stalled: 360 mA (measured), about 550 mA, 0.65 A | [6] · [6], [7], [8] | **0.65 A** when stalled (highest) |
| SG90 signal range (pulse width) | 1–2 ms with 1.5 ms centre · 0.5–2.4/2.5 ms measured | [3, 10] · [9] | Measure each servo (G12) |
| SG90 dead band | 10 µs | [3, 4] | 10 µs |
| SG90 weight | 9 g | [3] | 9 g |
| PCA9685 | 4096 steps per signal; frequency 24–1526 Hz; internal clock *typically* 25 MHz | [11] | See 8.3 |
| Arduino Nano | ATmega328P, 5 V, 16 MHz, 32 KB program memory, 2 KB RAM | [13] | — |
| Breadboard power module (MB102 type) | Up to 700 mA | [19] | 0.7 A |
| PLA plastic | Density 1.24 g/cm³ | [20] | 1.24 g/cm³ |

### 8.2 Power (RQ2)

Using the figures above:

| Situation | Current | Source |
| --- | --- | --- |
| All 5 servos moving | 5 × 100–250 mA = **0.5–1.25 A** | [6] |
| One servo stuck, 4 moving | 0.65 + 4 × 0.25 ≈ **1.65 A** | [6, 8] |
| All 5 stuck (worst case) | 5 × 0.65 ≈ **3.25 A** | [8] |
| What a breadboard module gives | **0.7 A** | [19] |

**Finding F1:** the breadboard module can't reliably run the arm with joints moving together, so the voltage dropping and resetting the Nano (risk R-01) is likely.

- **Fix, at no cost:** a lab bench supply at 5 V with its current limit set low for one-servo tests, as the brief requires [2, Task 3]. Raise the limit once we've measured how much current several servos really draw.
- **Fallback:** if only the module is available, move one joint at a time at low speed. This limits G8 (joints moving together), and we record that limitation.

### 8.3 Servo signal (RQ3)

**How fine the control is.** A servo reads one pulse every 20 ms (50 Hz). The PCA9685 divides each 20 ms into 4096 steps [11]:

- 20 000 µs ÷ 4096 ≈ **4.9 µs per step**.
- Our firmware spreads 0–180° over pulses of 500–2400 µs [18]: 1900 µs ÷ 180° ≈ 10.6 µs per degree.
- So one controller step ≈ **0.46°**.
- The servo ignores changes smaller than 10 µs [3, 4] ≈ **0.95°**.

The servo is the limit, not the controller (C17).

**How accurate the signal is (finding F2).** The PCA9685 times its pulses from an internal clock. The datasheet gives *typically* 25 MHz [11], but both the starter code and our firmware assume 27 MHz [18; `00_baseline/sample_code/Arduino Code.cpp`, line 94]. The firmware uses that number to set the PCA9685's clock divider (the prescaler). If the real clock is 25 MHz:

1. Divider = round(27 000 000 ÷ (4096 × 50) − 1) = 131.
2. Real frequency = 25 000 000 ÷ (4096 × 132) ≈ **46.2 Hz** instead of 50 Hz.
3. Every pulse is 25/27 as long as intended, so a 1500 µs command comes out at about 1389 µs. That's about **10° of error** on every joint.

We don't know which value is right for our board. **Measure the frequency with the lab scope before calibrating**, and correct `PCA_OSC_HZ` in the firmware. The firmware already reports the frequency it calculated, so the two can be compared [18].

### 8.4 End-of-arm accuracy (RQ4)

A small turn at a joint moves the end of the arm by *distance from the joint × angle in radians* (1° = 0.01745 rad). Using the link lengths from [18]:

| Joint | Distance to the end point | Movement from a 1° error |
| --- | --- | --- |
| Shoulder | 49.6 + 49.6 = 99.2 mm | 99.2 × 0.01745 ≈ **1.73 mm** |
| Elbow | 49.6 mm | 49.6 × 0.01745 ≈ **0.87 mm** |
| **Both wrong in the same direction** | | **≈ 2.6 mm** |

Gear slack, horn misalignment and link bending come on top of this. We measure them at G2 and G5 [2, Task 4 step 4; Task 7 step 6]. That's why G1c allows ±5 mm, to be tightened once those are known.

### 8.5 How much the arm can lift (RQ5)

**Method:** add up the turning force the shoulder servo has to resist with the arm held straight out sideways, which is the hardest position for it. Whatever strength is left over is what it can lift. Each assumption is pessimistic:

- **Part weights:** volume measured from the STL files × PLA density 1.24 g/cm³ [20], as if printed **fully solid**. Real prints are partly hollow, so lighter.
- **Servo positions:** the elbow, wrist and gripper servos are all assumed to be carried on the arm. If any sit at the base instead, the real load is lower.

| Load | Weight (g) | Distance from shoulder (mm) | Turning force (gf·mm) |
| --- | --- | --- | --- |
| Arm (5.89 cm³) | 7.3 | 24.8 (middle of link 1) | 181 |
| ArmSpring (6.73 cm³) | 8.3 | 24.8 | 206 |
| Elbow servo | 9 [3] | 49.6 (end of link 1) | 446 |
| ArmFront (5.06 cm³) | 6.3 | 74.4 (middle of link 2) | 469 |
| Wrist servo | 9 | 99.2 (end of link 2) | 893 |
| Gripper servo | 9 | about 130 | 1170 |
| Gripper parts (6.34 cm³) | 7.9 | about 140 | 1106 |
| **Arm's own weight in total** | | | **≈ 4470** |

**How much strength to plan on:** a servo shouldn't be planned to run at its maximum (stall) torque. ROBOTIS advises keeping planned motions within the continuous rating, not the stall rating [21]. The SG90 has no published continuous rating, so we **assume half of the lowest stall torque**. That 50 % is our own planning choice, not a published figure.

1. Available: 0.5 × 1.2 kgf·cm = **6000 gf·mm** [5].
2. Left after the arm's own weight: 6000 − 4470 = **1530 gf·mm**.
3. At the gripper tip, 49.6 + 49.6 + 73 = 172 mm out [18]: 1530 ÷ 172 ≈ **8.9 g**.

**Finding F4:** the arm can lift about 9 g at most. Halving that again covers the extra force when it accelerates and the uncertainty in these estimates, which gives a design limit of **5 g or less**.

**What could change this:** if the servos match the manufacturer's 1.8 kgf·cm [3], the limit rises to about 26 g. Real infill and servo positions also make the arm lighter. **Check with a hold test at G6:** hold a weighed object at full reach and watch for drooping, buzzing or heat (risk R-05).

### 8.6 How wide the gripper opens (RQ6)

The gripper turns a small gear that pushes two toothed racks in opposite directions, and each rack carries one jaw [2, Task 10]. Measured from the STL files:

- **Gear:** 10.8 mm across the tooth tips, about 10 teeth. That makes the tooth size (*module*) about 10.8 ÷ (10 + 2) = 0.9 mm. The working radius (*pitch radius*) is about 0.9 × 10 ÷ 2 = **4.5 mm**.
- **Rack movement:** 4.5 mm × π ÷ 180 ≈ **0.0785 mm per degree** of servo turn.
- **Over the firmware's gripper range of 30–150° (120°)** [18]: each rack moves about 9.4 mm. Both racks move, so **the gap changes by about 19 mm**.

**Finding F5:** the jaws travel about 19 mm. The exact open and closed gaps depend on how the jaws are fitted, which the files can't show. So **10–20 mm wide** is the working range. Measure the real gaps after assembly (G1). The tooth count read from the file could be 10 or 11, which changes the radius by under 0.1 mm.

---

## 9. Safety principles (RQ7)

- **IEEE Code of Ethics:** engineers must put the safety, health and welfare of the public first, and report anything that might endanger them [17]. Lecture 3 gives this as the reason for thorough testing [1, slide 37].
- **ISO 12100:** risk assessment is a loop. After adding any safety measure, check for hazards again, because the measure itself can create new ones [14]. So we review the risk register at every gate, not just once.
- **ISO 10218-1:2025:** covers the safety of industrial robots [15]. Our arm isn't one, so we **don't claim to meet it**. We follow its order: make the design safe first, then add protective measures, then give instructions.

---

## 10. Handed on to Stage 2

1. The problem statement (1), the parts (2), goals G1–G12 (3) and constraints C1–C20 (4).
2. Findings F1–F5 (Summary).
3. New risks for `01_management/risk_register.csv`:
   - **R-08** The servo signal frequency is wrong because the controller clock is unverified.
   - **R-09** The SG90 figures are uncertain (strength, current).
   - **R-10** Too heavy a load makes the arm droop or a servo stall at full reach.

## 11. To do before G0 can pass

- [ ] Ask the lab for a bench supply with a current limit, and book it for test sessions.
- [ ] Check whether the PCA9685 board has a smoothing capacitor fitted.
- [ ] Check whether the servo screws cover all the printed parts.
- [ ] Get the tutor's approval for objects A and B.
- [ ] Check that a scope or logic analyser and a scale are available.
- [ ] Find the WebLearn deadline and build the Gantt chart backwards from it.
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
