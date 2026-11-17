# 05_firmware (G3–G6)

Arduino Nano firmware for the arm.

- `code_audit_issue_log.md` — issues found in the baseline before any changes
- `protocol.md` — command protocol spec (units, bounds, replies, errors, timeout)
- `MicroArm_Firmware/MicroArm_Firmware.ino` — MicroArm firmware v1.0.0 (Shuayeb). Drives the PCA9685 directly over `Wire`, so it needs no extra libraries. Pairs with `06_host_gui/micro_arm_gui.py`
- `MicroArm_v1.0_README.md` — setup, wiring, calibration, serial protocol and communication tests for firmware v1.0.0 + GUI. Its paths `firmware/…` and `gui/…` refer to the original `MicroArm` folder layout: in this repo they are `05_firmware/MicroArm_Firmware/` and `06_host_gui/`

Status: compiled for the ATmega328P and tested against a desktop simulator. Not yet run on the physical arm.
