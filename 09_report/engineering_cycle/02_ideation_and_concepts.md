# Stage 2: Ideation and conceptualisation

Gate G0 · Builds on [Stage 1](01_problem_identification.md)

The lecture describes this stage as brainstorming ideas, developing them into concepts and checking that they're feasible (Lecture 3, slide 9). We keep it in three lists:

1. **Decisions:** what we've agreed to build.
2. **Ideas:** every idea anyone in the group has suggested. Each has its own file with the idea and its research.
3. **Requirements:** what we need to build, and which ideas cover it.

---

## 1. Decisions

| # | Decision | Idea | Fulfils | Date |
| --- | --- | --- | --- | --- |
| D1 | Power the servos from a lab bench supply at 5 V with a current limit. | [Bench power supply](02_ideas/bench_power_supply.md) | R1, R9 | 2026-11-17 |

---

## 2. Ideas

To add an idea, copy [`02_ideas/_template.md`](02_ideas/_template.md), fill it in, and add a row here.

Background: [what others have built](02_research_existing_arms.md).

| Idea | Proposed by | Fulfils | Status |
| --- | --- | --- | --- |
| [Bench power supply](02_ideas/bench_power_supply.md) | Shuayeb | R1, R9 | Decided |

---

## 3. Requirements

What we need to build, taken from the Stage 1 goals (G) and the brief.

| ID | What we need | From | Ideas that fulfil it |
| --- | --- | --- | --- |
| R1 | Servo power: separate from the Nano, current-limited, 5 V | C6, F1 | [Bench power supply](02_ideas/bench_power_supply.md) |
| R2 | Electronics layout and wiring: Nano, PCA9685, five servos, cable routes | G3, brief Task 3 | — |
| R3 | Printed and assembled arm body | Brief Task 2 | — |
| R4 | Calibration of all five joints and a repeatable HOME position | G12 | — |
| R5 | Firmware: command protocol, rejects bad commands, safety states, watchdog | G4, G5, G6 | — |
| R6 | GUI: port selection, joint control, HOME/STOP, status and log | G10 | — |
| R7 | Moves to coordinates: IK, choosing the elbow solution, workspace and path checks | G1, G7 | — |
| R8 | Speed control: three speed settings in real units, joints that finish together | G8 | — |
| R9 | Emergency power cut that doesn't depend on software | G11 | [Bench power supply](02_ideas/bench_power_supply.md) |
| R10 | Gripper improvement: two concepts, a CAD change, tested on two objects | G2, G9 | — |
| R11 | Repeatable pick-and-place, objects ≤ 5 g and 10–20 mm wide | G2, F4, F5 | — |
