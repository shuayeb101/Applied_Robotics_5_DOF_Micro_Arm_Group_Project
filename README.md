# Applied_Robotics_5_DOF_Micro_Arm_Group_Project

Group case study for **CT6056 Applied Robotics** (London Metropolitan University, 2026/27): design, assemble, program, calibrate and evaluate a 5-DOF 3D-printed micro robotic arm.

**Hardware:** Arduino Nano · PCA9685 16-channel PWM driver · 5 × SG90 micro servos · breadboard 3.3 V / 5 V power module · supplied printed parts.

**Completion guide:** [CT6056 Micro Arm — Step-by-Step Completion Guide](https://claude.ai/artifact/4LzATUBieDhHuErLtMR495)

## Team

| Member | Student ID | Primary role |
| --- | --- | --- |
| | | Hardware, power and mechanical integration |
| | | Embedded firmware and communication |
| | | Kinematics, calibration and safety |
| | | Host interface, motion and gripper |

## Repository structure

| Folder | Contents | Gate |
| --- | --- | --- |
| `00_baseline/` | Untouched supplied files: brief, starter code, original STLs. **Never edit.** | — |
| `01_management/` | Team charter, Gantt, risk register, contribution matrix, meeting notes | G0 |
| `02_requirements/` | Requirement list with IDs, system boundary, architecture diagram | G0 |
| `03_cad/` | Flaw log, CAD changes, new/modified STLs, slicer evidence | G1, G6 |
| `04_hardware/` | Wiring plan, pin table, power tests, calibration table, assembly photos | G1, G2 |
| `05_firmware/` | Arduino firmware, code-audit issue log, protocol spec | G3–G6 |
| `06_host_gui/` | Python GUI (PyQt5 + pyserial) | G3 |
| `07_kinematics/` | FK/IK derivation, Python validation scripts, test results | G4 |
| `08_tests/` | Test tables and timestamped logs (COM, KIN, SAFE, SPD, GRIP, INT) | G3–G7 |
| `09_report/` | Report drafts and figures | G7 |
| `10_submission/` | Final tagged submission package | G7 |

## Working rules

- Never edit `00_baseline/`. Copy a file into its working folder first, and commit the unchanged copy before changing it, so every change shows in the history.
- Commit small and often, with messages that say what changed and why (e.g. `firmware: reject out-of-range S commands instead of clamping`).
- Changes that affect another subsystem (protocol, pin mapping, calibration values, frames) need agreement from that subsystem's owner. Note the agreement in the commit message or the PR.
- Tag the release candidate used for final testing and the demo (e.g. `v1.0-rc1`). The submission must match that tag.
- Disclose any AI-assisted code or text in the commit message and the report.
