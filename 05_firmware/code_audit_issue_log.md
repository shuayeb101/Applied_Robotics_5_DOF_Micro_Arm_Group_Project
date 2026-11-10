# Code audit issue log — Arduino baseline

Compile the untouched baseline with servo power **off** first and record every warning/error here.

| ID | File / line | Issue | Risk | Fix | Commit | Status |
| --- | --- | --- | --- | --- | --- | --- |
| A-01 | `setup()` | `pwm.setPWM(servoPins[0 - 4], …)` evaluates to `servoPins[-4]`: out-of-bounds read; servos never initialised | Undefined start-up position | | | Open |
| A-02 | `isSafePosition()` | Defined but never called | Workspace limits not enforced | | | Open |
| A-03 | `inverseKinematics()` | `acos` input not clamped; only one elbow branch; no offsets; link lengths placeholder 10.0 | NaN or wrong pose | | | Open |
| A-04 | `moveServoSmooth()` | Out-of-range targets silently clamped by `constrain` | Hides invalid commands | | | Open |
| A-05 | `moveServoSmooth()` | Blocking `while` + `delay()` loop | STOP / timeout can't be handled mid-move | | | Open |
| A-06 | `loop()` | No STOP, HOME, STATUS, ACK or timeout; `toInt()` returns 0 on bad input | Malformed input can command 0° | | | Open |
| A-07 | Pulse mapping | Generic `map(0–180 → 150–600)`; 600 counts ≈ 2.9 ms is past the SG90's typical range | Servo driven into end stop | | | Open |
