# Stage 2: Ideation and conceptualisation

**Gate:** G0 (planning) [2, Figure 2]
**Author:** Shuayeb Ahmed · Draft 1 · 2026-11-17
**Builds on:** [Stage 1](01_problem_identification.md): goals G1–G12, constraints C1–C20, findings F1–F5

## What this stage does

The lecture describes this stage as three steps [1, slide 9]:

1. **Brainstorm:** come up with many ideas.
2. **Develop concepts:** refine the ideas into workable options.
3. **Check feasibility:** is each option technically, economically and operationally possible?

The feasibility check includes a risk assessment. Each risk is rated by how likely it is and how bad it would be, and the two together give its severity [1, slide 24]. Each serious risk then gets one of four responses [1, slide 25]:

- **Avoid:** change the plan so the risk can't happen.
- **Reduce:** make it less likely or less harmful.
- **Transfer:** give it to someone else, e.g. through insurance or a contract.
- **Accept:** live with it and record it.

## Method used for every subsystem

| Step | What we do | Why |
| --- | --- | --- |
| 1. Options | List every realistic option, including weak ones. | The lecture says not to criticise ideas while brainstorming [1, slide 21]. Listing the weak options shows they were considered and why they lost. |
| 2. Criteria | Judge options against criteria taken from the Stage 1 goals and constraints. | Criteria then come from the requirements, not from preference, so every choice traces back to Stage 1. |
| 3. Must-haves first | Rule out any option that breaks a brief requirement or a hard constraint. | No score can make up for a broken safety rule. |
| 4. Feasibility | Check the options that are left: technical, economic, operational [1, slide 23]. | The lecture's three feasibility tests. |
| 5. Decision | Pick one and record why. | So the report can justify it [2, LO2]. |
| 6. Risks | Rate likelihood (L) and impact (I) from 1 to 3. Severity = L × I. Give each risk a response. | The lecture's risk method [1, slides 24–25]. Severity 6–9 = must be reduced before G2. 3–4 = reduce if cheap. 1–2 = accept and record. |

The 1–3 scale and the severity bands are our own choice. The lecture asks for likelihood × impact but doesn't set a scale.

## Subsystems

| # | Subsystem | Status |
| --- | --- | --- |
| 1 | Servo power supply | **Decided:** lab bench power supply |
| 2 | Electronics layout and wiring | Not started |
| 3 | Printing and assembly | Not started |
| 4 | Firmware structure | Not started |
| 5 | Host interface (GUI) | Not started |
| 6 | Kinematic model and IK solution choice | Not started |
| 7 | Speed and motion profile | Not started |
| 8 | Gripper improvement (the brief requires two concepts [2, Task 10]) | Not started |
| 9 | Safety approach | Not started |

---

## 1. Servo power supply

### 1.1 What the supply has to do

All of these come from Stage 1:

| ID | Criterion | Type | Source |
| --- | --- | --- | --- |
| P1 | Separate from the Nano's USB or logic supply | Must | C6; [2, Task 3] |
| P2 | Adjustable current limit | Must | [2, Task 3] asks for current limiting |
| P3 | Switch or disconnect that doesn't depend on software | Must | G11; [2, Task 3] |
| P4 | At least 1.25 A, enough for all five servos moving | Must | F1, Stage 1 §8.2 |
| P5 | Voltage between 4.8 and 6 V | Must | SG90 rated at 4.8 V (~5 V) [3]; PCA9685 servo rail up to 6 V [4] |
| P6 | Costs nothing | Must | C3 |
| P7 | Shows the current being drawn | Nice to have | G2 evidence asks for current observations [2, G2] |
| P8 | Available whenever we need it | Nice to have | C2 (speed) |

### 1.2 Options

| Option | Description |
| --- | --- |
| A | Nano 5 V pin or USB |
| B | Breadboard power module from the kit (MB102 type) |
| C | USB phone charger or power bank |
| D | Lab bench power supply |

### 1.3 Must-have check

| | P1 Separate | P2 Current limit | P3 Hardware switch | P4 ≥ 1.25 A | P5 4.8–6 V | P6 Free | Result |
| --- | --- | --- | --- | --- | --- | --- | --- |
| A Nano USB | ✗ | ✗ | ✗ | ✗ | ✓ | ✓ | **Out.** The brief bans it [2, Task 3]. |
| B Breadboard module | ✓ | ✗ (fixed) | ✓ (on/off switch) [5] | ✗ (0.7 A max [5]) | ✓ | ✓ | **Out as main supply.** Kept as a fallback (§1.6). |
| C Charger or power bank | ✓ | ✗ | Pull the plug only | Depends on model | ✓ | ✗ (not supplied) | **Out** |
| D Bench supply | ✓ | ✓ [6] | ✓ (output button, or pull the lead) | ✓ (3 A per channel covers most bench work [7]. Check our lab's model.) | ✓ (adjustable) | ✓ (lab equipment) | **Passes** |

### 1.4 Feasibility of option D

| Test [1, slide 23] | Result |
| --- | --- |
| Technical | It meets every must-have. It also shows the current draw (P7), which gives us the G2 current evidence for free. |
| Economic | £0. It's standard teaching-lab equipment (C3). |
| Operational | **Weak point:** it only exists in the lab, so all powered testing has to happen there (P8). We need to book sessions and take this into account in the Gantt chart. Software work isn't affected. |

**Decision: option D, a lab bench power supply** (group decision, 2026-11-17). It's the only option that passes every must-have. Its one weakness is that it's tied to the lab, which we handle through planning, with option B as a fallback (§1.6).

### 1.5 How we use it

**How the current limit works.** A bench supply normally holds the voltage you set (constant voltage, CV). If the circuit tries to draw more than the current limit, the supply switches to constant current (CC) mode: it lowers the voltage to hold the current at the limit [6]. A CC indicator on the front panel shows when this happens [8]. We use this in two ways:

- **As a stall alarm:** a jammed servo draws more current. If the limit sits above normal running current but below stall current, a jam lights the CC indicator and drops the voltage, instead of quietly overheating the servo.
- **As protection:** a short circuit just pulls the voltage close to zero at the set current, so nothing is damaged [7].

**Settings:**

| Setting | Value | Why |
| --- | --- | --- |
| Voltage | **5.0 V** | Close to the SG90's rated 4.8 V (~5 V) [3], and well below the PCA9685 rail's 6 V maximum [4]. |
| Current limit: **one servo, no load** (G2 start) | **0.3 A** | Above one servo's measured running current of 100–250 mA [9], so normal moves stay in CV. Below the stall figures of 0.36–0.65 A [9, 10, 11], so a jam shows up as CC. |
| Current limit: **all five servos, arm built** | **1.5 A** | Above five moving servos (5 × 0.25 = 1.25 A [9]). Well below five stalled servos (≈ 3.25 A [11]), so a jam still shows up as CC. |
| Current limit **after measuring** | Highest normal current measured + 20 % | Replaces the estimates with our own data (F3). The 20 % margin is our choice, so normal peaks don't trip CC. |

**If CC trips while a servo is moving freely:** it may be the brief start-up surge, not a jam. Record it, raise the limit in 0.1 A steps, and note the value where it stops tripping. That's real evidence for the report.

**Order for switching on and off:**

| Step | Action | Why |
| --- | --- | --- |
| 1 | With the output **off**, set 5.0 V and the current limit. Check the voltage with a multimeter. | A bumped knob could send far more than 6 V. Setting and checking with the output off means nothing is connected when the voltage is wrong. |
| 2 | Plug in the Nano over USB (logic power only). Check that the PCA9685 answers on I²C. | The brief requires logic first, and I²C confirmed before servo power is connected [2, Task 3]. |
| 3 | Switch the supply output **on**. | Servos only get power once the controller is in its safe INIT state, with all outputs off [12]. |
| Off | Supply output **off first**, then unplug USB. | Servos lose power while the controller still holds a safe state. |
| Emergency | Press the output button, or pull the servo lead. | Works even if the software has crashed (G11). |

**Wiring rules:**

| Rule | Why |
| --- | --- |
| Supply **+** goes to the PCA9685 **V+** terminal. Supply **−** goes to its **GND** terminal. | Servo power goes through the terminal block [13]. |
| Nano **5 V** goes to PCA9685 **VCC**. Nano **GND** goes to PCA9685 **GND**. | VCC only powers the PCA9685 chip, not the servos [4]. Joining the grounds gives the signal and power a shared reference, which the brief requires [2, Task 3]. |
| **Never connect V+ to VCC**, or the supply to the Nano's 5 V pin. | Otherwise two supplies would push against each other, and a servo surge could reach the Nano. |
| Use red for + and black for −. Check polarity before every switch-on. | Many PCA9685 boards have reverse-polarity protection on the terminal block [13, 14], but not all clones do. **Check ours.** |
| Check whether our board has a capacitor fitted on V+. | Many boards have a dedicated capacitor spot on the V+ line [15]. The brief asks for this to be checked [2, Task 3]. |

### 1.6 Fallback if the bench supply isn't available

Use option B, the breadboard module (0.7 A [5]), for **single-joint tests only**. Firmware moves one joint at a time at low speed, so the draw stays at one servo's running current (≤ 0.25 A [9]). The module has no current limit, so a jam won't show up on a CC indicator. Watch the servo closely and switch off at the first buzz or heat. Simultaneous motion (G8) and pick-and-place tests wait for the bench supply.

### 1.7 Risks

Rated L × I on the 1–3 scale above.

| ID | Risk | L | I | Severity | Response | Control |
| --- | --- | --- | --- | --- | --- | --- |
| PS-1 | The voltage is set too high (over 6 V) and damages the servos and board. | 2 | 3 | **6** | Reduce | Set and check with a multimeter with the output off (§1.5 step 1). Use the over-voltage protection if the supply has it. |
| PS-2 | Reversed polarity damages the board or servos. | 2 | 3 | **6** | Reduce | Colour-coded leads, a polarity check before switch-on, and confirm whether our board has polarity protection. |
| PS-3 | No common ground, so the servos jitter or behave unpredictably. | 2 | 2 | 4 | Reduce | Wiring checklist item, checked before step 3. |
| PS-4 | The limit is too low, so CC trips during normal moves and looks like a software bug. | 2 | 1 | 2 | Accept | Watch the CC indicator during every test and log it. |
| PS-5 | The limit is too high, so a jammed servo overheats. | 1 | 2 | 2 | Accept | Staged limits (§1.5). The firmware stops on fault. Feel for heat after every test. |
| PS-6 | Two supplies get connected together (V+ to VCC, or the supply to the Nano's 5 V). | 1 | 3 | 3 | Avoid | Wiring rule in §1.5, and a check before every switch-on. |
| PS-7 | The supply isn't available when we need it, and the schedule slips. | 2 | 2 | 4 | Reduce | Book lab sessions early. Do software work away from the lab. Fall back to option B (§1.6). |

**For the register:** PS-1 and PS-2 are severity 6, so they must be controlled before G2. They extend the existing R-03 (wrong polarity) and need a new R-11 (over-voltage). PS-7 is a schedule risk, which the brief asks for [2, Task 1 step 6].

---

## References

Accessed 2026-11-17 unless stated.

1. D. Mariyanayagam, *Lecture 3: Engineering Cycle, Mixed Signal Processing and PCB Design, Part One: Idea to Prototype*, CT6056 Applied Robotics, London Metropolitan University, 2026/27. Slides 9, 20, 21, 23–25.
2. D. Mariyanayagam, *CT6056 Group Case Study: The Design and Control of a 5-DOF Micro Arm*, v3.0, London Metropolitan University, 2026/27. Copy in `00_baseline/brief/`.
3. TowerPro, *SG90 9 g Micro Servo* datasheet (university-hosted copy). https://tams.informatik.uni-hamburg.de/lehre/2023ss/vorlesung/es/doc/sg90.pdf
4. Envistia Mall, *PCA9685 16-Channel 12-Bit I2C PWM Servo Motor Driver Module User Guide* (VCC logic 2.3–5.5 V, not servo power; V+ servo rail up to 6 V). https://envistiamall.com/blogs/learn/pca9685-16-channel-12-bit-i2c-pwm-servo-motor-driver-module-user-guide
5. YwRobot, *MB102 breadboard power supply* specifications (max output current 700 mA; locking on/off switch), via PETERVIS. https://www.petervis.com/Raspberry_PI/Breadboard_Power_Supply/YwRobot_Breadboard_Power_Supply.html
6. B&K Precision, "Understanding Constant Voltage (CV) and Constant Current (CC) Modes in Regulated Power Supplies", technical note, 2025. https://www.bkprecision.com/knowledge/technical-notes/5/understanding-constant-voltage-cv-and-constant-current-cc-modes-in-regulated-power-supplies
7. Applied EE Notebook, "Bench Power Supply" (CC into a short circuit; 3 A covers most bench work). https://applied-ee.github.io/ee-notebook/docs/measurement/test-instruments/bench-power-supply
8. B&K Precision, *Model 1735A instruction manual* (CV/CC crossover shown by front-panel indicators). https://www.manualsdir.com/manuals/53799/bk-precision-1735a-manual.html?page=17
9. ProtoSupplies, *Servo Motor Micro SG90*, technical specifications (measured: 100–250 mA moving, 360 mA stall). https://protosupplies.com/product/servo-motor-micro-sg90/
10. Circuit Rocks, *SG90 9g Servo* (stall current ~550 mA). https://docs.circuit.rocks/motors/sg90-servo/
11. ServoDatabase, *Yahboom SG90* (stall current 0.65 A). https://servodatabase.com/servo/yahboom/sg90
12. S. Ahmed, *MicroArm firmware v1.0.0* and `05_firmware/MicroArm_v1.0_README.md` (starts in INIT with all servo outputs off), this repository.
13. Adafruit, *Adafruit 16-Channel 12-bit PWM/Servo Driver, PCA9685*, product page (reverse-polarity protection on the terminal block input). https://www.adafruit.com/product/815
14. Adafruit, *16-Channel PWM/Servo Shield* guide (power through the terminal block; reverse-polarity protection). https://learn.adafruit.com/adafruit-16-channel-pwm-slash-servo-shield?view=all
15. 38-3D, *PWM Servo Driver 16-Channel 12-bit I2C (PCA9685)* (dedicated capacitor footprint on the V+ line; V+ 3.3–6 V). https://ja.38-3d.co.uk/products/16-channel-servo-expansion-module
