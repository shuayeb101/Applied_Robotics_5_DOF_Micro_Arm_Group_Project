# Stage 1: Identification of the problem

Gate: **G0** (roles, requirements, interfaces, Gantt plan and initial risk controls agreed) [2, Figure 2]
Author: Shuayeb Ahmed · Draft 1 · 2026-11-17

Lecture 3 splits this stage into two activities [1, slide 6]:

- **Define the problem statement:** say clearly what the challenge or need is.
- **Research and analysis:** find out the context, constraints and stakeholders.

This file does both. Section 1 is the problem statement, using the eight headings on slide 7 of the lecture. Sections 2 to 6 are the research and analysis, following the process on slide 8. Section 7 lists what this stage hands on to Stage 2, and Section 8 lists the decisions the group still has to make.

---

## 1. Problem statement

The lecture defines a problem statement as a short, precise description of the current situation, the gap between the current state and the desired state, and why closing that gap matters [1, slide 7].

> **The group has a kit of supplied parts and starter code for a 5-DOF micro arm. The parts haven't been assembled or calibrated, and the starter code hasn't been approved for physical motion. The group needs an integrated arm that accepts both joint commands and Cartesian target commands, validates every command before it moves, stays inside a defined safe workspace, and completes a repeatable, measured pick-and-place sequence with a tutor-approved lightweight object. Every result has to trace back to a requirement.**

The eight elements on slide 7, applied to this project:

| Element [1, slide 7] | This project | Source |
| --- | --- | --- |
| Description of the problem | Design, assemble, program, calibrate and critically evaluate the supplied 5-DOF micro arm. | [2, intro] |
| Context and background | The supplied firmware and GUI are baseline learning resources. They haven't been approved for unrestricted physical motion and may have incomplete initialisation, blocking behaviour, unsafe assumptions, missing validation or missing fault handling. | [2, "Starter-code status"] |
| Impact and consequences | Unsafe automation can hurt people or damage equipment. The brief treats invalid input, communication loss, timeout, an unreachable target and a limit violation as stop/hold or fault conditions. | [2, "Safe and responsible use"] |
| Scope and boundaries | See Section 3, the system boundary. | [2, Task 1 step 3] |
| Objectives and goals | Manual joint command, Cartesian target command, validation, smooth motion in a safe workspace, gripper operation and repeatable pick-and-place. | [2, "Project challenge and success definition"] |
| Stakeholders | See Section 5. | [2, LO4] |
| Constraints | See Section 4. | [2] |
| Measurable criteria | The result has to be reproducible, measured and linked to requirements. One video of one successful movement isn't enough. | [2, "Project challenge and success definition"] |

**Why the problem statement is written this way:** it states the *gap* (an uncalibrated kit and unapproved code versus a verified, repeatable arm) and doesn't name a solution. Naming one at this point, e.g. "upgrade the baseline Arduino code", would close off options before Stage 2 has compared them. That would break the order the lecture gives, where ideation comes after the problem is defined [1, slides 5 and 9].

---

## 2. Research objectives and questions

Slide 8 starts research with *define your objectives* and *formulate research questions* [1, slide 8]. Each question below comes from a risk or an unknown in the brief. Each has to be answered before a particular gate can pass.

| ID | Research question | Why it must be answered | Needed for |
| --- | --- | --- | --- |
| RQ1 | What are the real electrical and mechanical limits of the parts, and do the sources agree? | The brief asks for decisions justified by datasheets or reliable technical sources [2, LO2; "Case-study files"]. If the limits are wrong, so are the power design and the motion limits. | G0, G2 |
| RQ2 | Can the planned power supply drive the servos without brownout? | Brownout is already risk R-01 in the register. A controller reset counts as a no-go at G2 [2, G2]. | G0, G2 |
| RQ3 | Is the PWM signal chain accurate and fine-grained enough for the joint positions we need? | Kinematics (G4) and bounded Cartesian moves (G5) assume a commanded angle gives the physical angle. | G2, G4 |
| RQ4 | Who are the stakeholders and what does each one need? | LO4 asks for a design process that covers user, customer, business, cost, security and operational needs [2, LO4]. | G0 |
| RQ5 | Which safety principles and standards apply to an educational micro arm? | The brief requires safety controls and a residual-risk statement [2, G7]. The IEEE Code of Ethics makes public safety the first duty [1, slide 37; 17]. | G0, G7 |

**Why questions come before data collection:** writing the questions first means only the data that answers them gets collected, and each answer can be checked against what it was meant to decide. This follows the lecture's order of *formulate research questions → … → collect data → analyse → validate* [1, slide 8].

---

## 3. System boundary

**Process:** list what is inside the system (designed and verified by the group) and what is outside it (assumed, supplied or controlled by others). Then state the use limits and space limits.

**Why:** ISO 12100, the base standard for risk assessment of machinery, starts by *determining the limits of the machinery*, covering use limits and space limits, before any hazard is identified [14]. A hazard can only be analysed once it is clear what the machine is, who uses it and where it moves. The brief asks for the same step in Task 1, step 3 [2].

### 3.1 Inside the boundary

| Subsystem | Selected item | Source |
| --- | --- | --- |
| Controller | Arduino Nano (ATmega328P) | Group decision, repo README |
| Servo driver | PCA9685 16-channel PWM board, controlled over I²C | Repo README; [2, LO1] |
| Actuators | 5 × SG90 micro servos: base, shoulder, elbow, wrist, gripper | Repo README; firmware joint table [18] |
| Servo power | Breadboard 3.3 V / 5 V power module (exact model **to be confirmed**, see Section 6.2) | Repo README |
| Mechanical | 11 supplied STL parts, 3D-printed | [2, Appendix B]; `00_baseline/3d_models/` |
| Host interface | Python GUI over USB serial at 115200 baud | [18] |
| Firmware | MicroArm firmware v1.0.0 (state machine, validated protocol, planar IK) | [18] |

### 3.2 Outside the boundary (assumptions)

- The host PC, its operating system and the uni network. The group doesn't control these, so the GUI has to run without installing anything (pyserial is bundled for this reason [18]).
- Tutor-approved low-voltage lab equipment and the lab's own safety rules [2, "Safe and responsible use"].
- The pick-and-place object. The brief requires it to be tutor-approved, lightweight and benign [2, success definition; G6]. **Not chosen yet.**
- Vision is optional in the reference architecture, so it stays outside the boundary unless the group adds it [2, Figure 1]. If it is added, Task 1 needs a data and privacy risk [2, Task 1 step 6].

### 3.3 Use limits and space limits [14]

- **Intended use:** supervised lab demonstration of pick-and-place by group members, at low speed, with an immediate route to cut the power [2, "Safe and responsible use"].
- **Foreseeable misuse:** a hand inside the workspace while the arm is powered; a stale or malformed command sent from the GUI; servos powered from the controller's USB rail. The brief names all three as things to prevent [2].
- **Space limits:** the workspace is the arm's reachable envelope above the table surface. It is still an estimate: the link lengths `L1 = L2 = 49.6 mm` were measured pivot to pivot on the STLs, while `L3 ≈ 73 mm` and the shoulder height `≈ 44 mm` are estimates that still need measuring on the built arm [18]. The workspace gets fixed at G4, once the measurements are done.

---

## 4. Constraints

Each constraint comes straight from the brief, so it is fixed and not a design choice.

| ID | Constraint | Source | Consequence for design |
| --- | --- | --- | --- |
| C1 | No physical motion until the mechanical assembly, power architecture, polarity, common ground, current limit, servo orientation and software limits have been checked. | [2, "Safe and responsible use"] | Commissioning order G1 → G2 → G3, with no shortcuts. |
| C2 | Servo rail powered from a suitable external supply, not the controller's USB or logic rail. | [2, "Safe and responsible use"] | Separate power domains with a common ground (Section 6.2). |
| C3 | Commission one servo at a time, at low speed. | [2, "Safe and responsible use"; G2] | Calibration runs per joint. Firmware energises joints one at a time on HOME [18]. |
| C4 | Starter code must be compiled and audited before any servo is energised. Unchanged starter code isn't evidence of the group's work. | [2, "Starter-code status"] | Keep `05_firmware/code_audit_issue_log.md` and an unchanged baseline in `00_baseline/`. |
| C5 | Every change must be traceable (dated versions or a shared repo). | [2, "Case-study files"] | Git branches, small commits, a tagged release candidate. |
| C6 | Progression through gates is conditional. A no-go repeats the current gate or an earlier one. | [2, Figure 2] | The plan needs contingency time (Task 1 step 7). |
| C7 | 100 marks, scaled to 15 % of the module mark. Mainly assesses LO3 and LO4. | [2, "Mark allocation"] | Effort goes where the marks are: IK and safety (30), hardware (20), serial (20), speed and gripper (20), report (10). |

---

## 5. Stakeholders and their needs

**Process:** name everyone who affects the system or is affected by it, and write down what each of them needs. Requirements come in Stage 3.

**Why:** ISO/IEC/IEEE 29148 treats stakeholder needs as the input that gets transformed into requirements [16]. A requirement that doesn't trace back to a stakeholder need is unjustified, and a need with no requirement is a gap. Recording needs now means every requirement in Stage 3 can trace back to one.

| Stakeholder | Interest | Needs (source) |
| --- | --- | --- |
| Operator (group members during tests and demos) | Uses the arm directly | No unexpected motion, a clear STOP route, hands outside the workspace [2, "Safe and responsible use"]. HOME and STOP controls not distinguished by colour alone [2, Task 6, "Host-interface requirements"]. |
| Tutor and assessor | Approves physical tests and grades the evidence | Reproducible, measured results traced to requirements [2, success definition]. Tutor approval before bounded physical moves [2, G5]. |
| Other people in the lab | Bystanders | Low voltage only, approved equipment, a power-disconnect route [2]. IEEE code: safety, health and welfare of the public come first [17]. |
| University (H&S, IP) | Responsible for the environment and for licensing | Health and safety, IP, software and CAD licensing [2, LO5]. Bundled pyserial is BSD-3-Clause, and the licence is kept in the repo [18]. |
| Hypothetical customer or business | The brief's LO4 framing | User, customer, business, cost, security and operational needs [2, LO4]. |
| Environment and end of life | Lifecycle | Production, operation, maintenance and end-of-life effects, plus sustainability [2, LO3, LO5]. |
| Each group member as an individual | Accountability | A dated logbook, plus being able to explain the interfaces and safety decisions [2, "Group organisation"]. |

---

## 6. Data collection and analysis

### 6.1 Component facts, with conflicts noted (answers RQ1)

**Process:** for each component, collect the key figures from the manufacturer's datasheet first, then from independent sources, and record any disagreement.

**Why:** the SG90 is sold by many vendors, and their published figures don't agree (see the table). One source would hide that uncertainty. Several sources show where a figure has to be *measured* instead of trusted, and that becomes the G2 calibration plan.

| Parameter | Value(s) found | Sources | Conclusion |
| --- | --- | --- | --- |
| SG90 supply voltage | 4.8 V nominal (~5 V) | [3, 4] | Run the servos at ~5 V. |
| SG90 stall torque | 1.8 kgf·cm at 4.8 V (TowerPro) · 1.2 kg·cm at 4.8 V (retailer) · 1.7 kg·cm (independently measured) | [3] · [5] · [6] | **Conflicting.** Plan with the lowest figure, 1.2 kgf·cm, until it is measured. |
| SG90 speed | 0.1 s/60° at 4.8 V, no load | [3, 4] | Upper bound only. Under load the arm will be slower. |
| SG90 current | Moving: 100–250 mA (measured) · Stall: 360 mA (measured), ~550 mA, 0.65 A | [6] · [6], [7], [8] | **Conflicting.** Plan with the highest stall figure, ~0.65 A per servo. |
| SG90 pulse range | 1–2 ms over the full travel, 1.5 ms centre · 0.5–2.4/2.5 ms measured on an SG90 | [3, 10] · [9] | **Conflicting.** Measure the safe pulse range for each servo (G2). |
| SG90 dead band | 10 µs | [3, 4] | Commanded changes smaller than this may not move the servo. |
| SG90 temperature range | 0–55 °C | [4] | Not limiting in the lab. Matters for stall heating (R-05). |
| PCA9685 resolution | 12-bit (4096 steps) per channel | [11] | See Section 6.3. |
| PCA9685 PWM frequency | ~24–1526 Hz, set by the prescaler. 25 MHz typical internal oscillator | [11] | 50 Hz is inside the range. The oscillator is only "typical", see Section 6.3. |
| PCA9685 supply | 2.3–5.5 V logic. Outputs sink 25 mA / source 10 mA at 5 V | [11, 12] | Logic is powered from the Nano's 5 V. The servo power comes from V+, not from these outputs. |
| Arduino Nano | ATmega328P, 5 V, 16 MHz, 32 KB flash (2 KB bootloader), 2 KB SRAM | [13] | Firmware v1.0.0 uses 64.6 % of flash and 57.6 % of RAM, which leaves headroom [18]. |
| Breadboard power module (MB102 type) | Max output 700 mA | [19] | See Section 6.2. |

### 6.2 Power budget (answers RQ2)

**Process:** estimate the worst-case current from the figures in Section 6.1 and compare it with the supply rating.

**Why:** C2 requires an external servo supply. Risk R-01 identifies brownout as a hazard, and a controller reset is a G2 no-go [2]. A paper estimate costs nothing, so it is worth doing before any hardware test that could cause the fault.

Calculation, using the source figures:

- **All 5 servos moving, no stall:** 5 × 100–250 mA = **0.5–1.25 A** [6].
- **One servo stalled while the other 4 move:** 0.65 A + 4 × 0.25 A ≈ **1.65 A** [6, 8].
- **All 5 stalled (worst case):** 5 × 0.65 A ≈ **3.25 A** [8].
- **MB102-type module rating:** **0.7 A** [19].

**Finding:** if the module is an MB102 type, it can't guarantee even all-joint motion without stall, because the upper estimate of 1.25 A is above 0.7 A. Brownout (R-01) is therefore a likely failure, not just a possible one. This fits the ≥3 A recommendation for the full arm in the firmware guide [18]. This is the main input to Stage 2, where the options are compared, e.g. a ≥3 A bench supply with a current limit, or limiting simultaneous motion in firmware.

**Uncertainty:** the exact model of the power module still has to be confirmed (Section 8). The current figures come from different SG90 units, not ours, so the real current must be measured on our arm at G2 [2, G2 evidence: "current/behaviour observations"].

### 6.3 PWM signal accuracy and resolution (answers RQ3)

**Resolution:** at 50 Hz the PWM period is 20 ms, i.e. 20 000 µs. With 4096 steps [11], one step is 20 000 / 4096 ≈ **4.9 µs**. If the 0–180° travel spans 500–2400 µs (the firmware default [18]), that is 1900 µs / 180° ≈ 10.6 µs per degree, so one PCA9685 step ≈ **0.46°**. The SG90 dead band is 10 µs [3, 4], about **0.95°**. **Conclusion:** the servo's dead band limits the positioning resolution before the driver does. Positioning tolerances in Stage 3 shouldn't be tighter than about ±1° per joint until they are measured.

**Accuracy:** the PCA9685 internal oscillator is quoted as *typical* 25 MHz [11], but the baseline code and our firmware both assume 27 MHz [18; `00_baseline/sample_code/Arduino Code.cpp`, line 94]. The prescaler is calculated from the assumed frequency. If the real oscillator were 25 MHz:

- prescale = round(27 MHz / (4096 × 50 Hz) − 1) = 131
- actual frequency = 25 MHz / (4096 × 132) ≈ **46.2 Hz**, not 50 Hz
- every pulse is shortened by 25/27 ≈ 0.926, so a 1500 µs command comes out as ~1389 µs, an error of ~111 µs ≈ **10°** at the scale above.

**Conclusion:** a wrong oscillator assumption would cause a systematic joint error of around 10°. That is larger than any tolerance we would accept for pick-and-place. So the real PWM frequency has to be measured with a scope or logic analyser, and `PCA_OSC_HZ` corrected, *before* calibration at G2. This isn't a claim that the board is wrong, only that the value is unverified. The firmware already reports the computed `pwm_hz` so the two can be compared [18].

### 6.4 Applicable safety principles (answers RQ5)

- **IEEE Code of Ethics:** members commit to hold paramount the safety, health and welfare of the public, and to disclose promptly factors that might endanger it [17]. Lecture 3 cites the code as the basis for rigorous testing [1, slide 37].
- **ISO 12100:** risk assessment and risk reduction are iterative. After each protective measure, go back to hazard identification, because a measure can introduce a new hazard [14]. The register therefore gets reviewed at every gate, not filled in once.
- **ISO 10218-1:2025:** gives safety requirements for industrial robots. It covers inherently safe design, risk reduction measures and information for use [15]. Our arm isn't an industrial robot, so we **don't claim compliance**. We only borrow its structure: safe design first, then protective measures, then information for use. This limitation is stated so the report doesn't overclaim.

---

## 7. Outputs of Stage 1 (inputs to Stage 2)

1. **Problem statement** (Section 1). Ideation in Stage 2 aims to close this gap.
2. **System boundary and limits** (Section 3), for the G0 evidence and the start of the ISO 12100 hazard analysis.
3. **Constraints C1–C7** (Section 4). Every concept in Stage 2 has to meet all of them.
4. **Stakeholder needs** (Section 5). Stage 3 requirements trace back to these.
5. **Three analysis findings to carry forward:**
   - F1: The likely supply (≤0.7 A) is undersized for simultaneous motion (estimate 0.5–1.25 A, worst case ~3.25 A). → Stage 2: compare power concepts.
   - F2: The PWM frequency must be measured before calibration. A wrong oscillator assumption gives ~10° of error. → G2 procedure.
   - F3: The SG90 pulse range, torque and current figures conflict between sources, so they have to be measured per servo. → G2 calibration table.
6. **New risks for the register:** R-08 *PWM frequency error from the unverified oscillator* and R-09 *SG90 specification uncertainty (torque and current)*.

## 8. Open decisions for the group (needed before G0 passes)

- [ ] Confirm the exact servo power module (model or rating) and whether a bench supply with a current limit is available.
- [ ] Choose the pick-and-place object and get tutor approval [2].
- [ ] Assign the primary roles in `01_management/team_charter.md` [2, "Group organisation"].
- [ ] Check whether a scope or logic analyser is available for finding F2.

---

## References

Accessed 2026-11-17 unless stated.

1. D. Mariyanayagam, *Lecture 3: Engineering Cycle, Mixed Signal Processing and PCB Design, Part One: Idea to Prototype*, CT6056 Applied Robotics, London Metropolitan University, 2026/27. Slides 4–9, 20, 23, 37.
2. D. Mariyanayagam, *CT6056 Group Case Study: The Design and Control of a 5-DOF Micro Arm*, v3.0, London Metropolitan University, 2026/27. Copy in `00_baseline/brief/`.
3. TowerPro, *SG90 9 g Micro Servo* datasheet (university-hosted copy). https://tams.informatik.uni-hamburg.de/lehre/2023ss/vorlesung/es/doc/sg90.pdf; also https://bit.nkust.edu.tw/~csshieh/teach/111A/mc/datasheet/SG90.pdf
4. Rapid Electronics, *Tower Pro SG90 Mini Servo, 37-1330*, datasheet. https://static.rapidonline.com/pdf/37-1330t.pdf
5. ThinkRobotics, *SG90 9G Micro Servo Motor*, product specifications. https://thinkrobotics.com/products/sg90-9g-micro-servo-motor
6. ProtoSupplies, *Servo Motor Micro SG90*, technical specifications (measured values). https://protosupplies.com/product/servo-motor-micro-sg90/
7. Circuit Rocks, *SG90 9g Servo*, component documentation. https://docs.circuit.rocks/motors/sg90-servo/
8. ServoDatabase, *Yahboom SG90*. https://servodatabase.com/servo/yahboom/sg90
9. Arduino Forum, "Servo pulse widths" (measured 0.5–2.5 ms on an SG90). https://forum.arduino.cc/t/servo-pulse-widths/286525
10. Luxorparts / Kjell & Company, *SG90 Micro Servo* datasheet (2017). https://www.kjell.com/globalassets/mediaassets/701916_87897_datasheet_en.pdf
11. NXP Semiconductors, *PCA9685 16-channel, 12-bit PWM Fm+ I²C-bus LED controller*, product data sheet Rev. 4, 16 April 2015. https://www.nxp.com/docs/en/data-sheet/PCA9685.pdf; product page https://www.nxp.com/products/PCA9685
12. NXP Semiconductors, *PCA9685PW product information*. https://nxp.com/part/PCA9685PW
13. Arduino, *Arduino Nano (A000005) tech specs*, as reproduced by PiShop. https://www.pishop.us/product/arduino-nano/. Cross-check against the board schematic `tutorials/tutorial_1/NanoV3.3_sch.pdf`.
14. ISO 12100:2010, *Safety of machinery: general principles for design, risk assessment and risk reduction*. Structure as given in the ISO/DIS 12100:2025 draft contents (§5.3 limits of machinery: use and space limits; Figure 1, iterative process). https://standardsdevelopment.elot.gr/drafts/2130. Iterative approach: Leuze, *Risk Assessment* white paper, https://files.leuze.com.cn/shopware/media/ee/2d/e7/1639045358/wp_risikobeurteilung_en_144dpi.pdf
15. ISO 10218-1:2025, *Robotics: safety requirements, Part 1: industrial robots*, Edition 3, February 2025. https://www.iso.org/standard/73933.html
16. ISO/IEC/IEEE 29148:2018, *Systems and software engineering: life cycle processes, requirements engineering*, §5.2.2–5.2.3 and §6.3 (contents: https://www.din.de/de/mitwirken/normenausschuesse/nia/veroeffentlichungen/wdc-beuth:din21:299567544/toc-3025650/download). Definitions as summarised in SEBoK, "Stakeholder Needs and Requirements (glossary)", https://sebokwiki.org/draft/Special:Random/Absorption_(glossary)
17. IEEE, *IEEE Code of Ethics* (revisions through June 2020). https://ieee-cas.org/about/ieee-code-ethics
18. S. Ahmed, *MicroArm firmware and control GUI v1.0.0*: `05_firmware/MicroArm_Firmware/MicroArm_Firmware.ino`, `05_firmware/MicroArm_v1.0_README.md`, this repository, commit `0b35f32`.
19. YwRobot, *MB102 breadboard power supply* specifications (manual: max output current 700 mA), via PETERVIS. https://www.petervis.com/Raspberry_PI/Breadboard_Power_Supply/YwRobot_Breadboard_Power_Supply.html
