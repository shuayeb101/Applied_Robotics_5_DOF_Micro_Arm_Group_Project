# 05_firmware (G3–G6)

Arduino Nano firmware for the arm.

- `code_audit_issue_log.md` — issues found in the baseline before any changes
- `protocol.md` — command protocol spec (units, bounds, replies, errors, timeout)
- Firmware sketch goes in its own folder, e.g. `micro_arm/micro_arm.ino` (Arduino IDE) or a PlatformIO project

To start: copy `00_baseline/sample_code/Arduino Code.cpp` to `micro_arm/micro_arm.ino`, commit it unchanged, then make changes in later commits so the diff shows every change.

Dependencies: Adafruit PWM Servo Driver Library (Arduino Library Manager).
