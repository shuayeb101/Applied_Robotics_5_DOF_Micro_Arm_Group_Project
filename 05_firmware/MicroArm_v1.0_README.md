# CT6056 5-DOF Micro Arm: firmware + control GUI (v1.0.0)

```
MicroArm/
├── firmware/MicroArm_Firmware/MicroArm_Firmware.ino   ← upload to the Arduino Nano
├── gui/micro_arm_gui.py                               ← run on the PC
├── gui/serial/                                        ← bundled pyserial 3.5 (no install needed)
└── README.md
```

The firmware needs **no extra Arduino libraries**: it drives the PCA9685 directly with `Wire`.
The GUI needs **only Python 3.8+**: tkinter is built in and pyserial is bundled.

---

## 1. On the uni PC

1. Unzip the whole `MicroArm` folder somewhere you can write to, e.g. your Documents or H: drive.
   Don't run it from inside the zip.
2. **Upload the firmware** (Arduino IDE)
   - Open `firmware/MicroArm_Firmware/MicroArm_Firmware.ino`.
   - Tools → Board → **Arduino Nano**. Tools → Processor → **ATmega328P**.
     If the upload fails with `stk500_getsync` / `not in sync`, choose **ATmega328P (Old Bootloader)**.
     Most clone Nanos need that.
   - Tools → Port → the Nano's COM port. Then Upload.
   - Optional check: Serial Monitor at **115200** with **Newline** should show
     `READY MicroArm fw=1.0.0 state=INIT (outputs OFF - send HOME)`.
     **Close the Serial Monitor before starting the GUI**, because only one program can use the port.
3. **Run the GUI**: open Command Prompt in the `gui` folder (type `cmd` in the Explorer address bar), then:
   ```
   py micro_arm_gui.py
   ```
   If `py` isn't found, try `python micro_arm_gui.py`, or launch Python from AppsAnywhere first.
4. In the GUI, pick the COM port (press **Refresh** if it's missing) and set baud to **115200**. Press **Connect**.
   The Nano resets when the port opens and comes back in **INIT** with every servo output **off**.
5. Press **HOME**. Joints are switched on one at a time, each straight at its HOME angle
   (wrist → elbow → shoulder → base → gripper, 0.7 s apart). Nothing else will move until HOME has run.

Logs are saved automatically to `gui/logs/microarm_<date>_<time>.log` (use **Open log folder**).

---

## 2. Wiring

| Nano | PCA9685 | |
|---|---|---|
| 5V | VCC | logic supply for the PCA9685 chip only |
| GND | GND | common ground (essential) |
| A4 | SDA | I2C data |
| A5 | SCL | I2C clock |
| – | OE | leave unconnected |

| Servo supply | PCA9685 |
|---|---|
| External 5–6 V, current-limited for first tests (≈1 A), ≥3 A for the full arm | green screw terminal **V+ / GND** |

**Never power the servos from the Nano 5 V pin or USB.** Fit an accessible switch on the servo supply for emergency removal (brief, Task 8).

| PCA9685 channel | Joint | Firmware name |
|---|---|---|
| 0 | Base (yaw) | `BASE` |
| 1 | Swing (shoulder) | `SWING` |
| 2 | Arm One (elbow) | `ARM1` |
| 3 | Arm Two (wrist) | `ARM2` |
| 4 | Gripper | `GRIP` |

Servo plugs: brown = GND, red = V+, orange = signal (PWM row).

---

## 3. First power-up and calibration (Task 4)

1. Logic only (USB, servo supply off): connect the GUI. State should read **INIT**. A **FAULT I2C_LOST** means
   the PCA9685 isn't answering, so check SDA/SCL/VCC/GND.
2. Plug in **one** servo, with no horn or link fitted. Switch the servo supply on and press HOME.
3. Move it in small steps with the Value box + Enter (e.g. 80, 100). Watch for buzzing, heat or Nano resets.
4. Edit the **CALIBRATION block** at the top of the `.ino` for each joint, then re-upload:
   - `minUs / maxUs`: pulse width at servo 0° and 180°. The defaults are 500–2400 µs.
     Narrow them if the servo buzzes at either end.
   - `dir`: −1 if a joint moves the wrong way. `offsetDeg`: trim for horn spline misalignment.
   - `lo / hi`: the safe soft limits you measured. `home`: a collision-free HOME pose.
   - The GUI reads the limits from the controller (`CONFIG`), so you only change them in one place.
5. Fit each horn with that servo powered at its HOME angle.
6. For XY / IK: set `KIN_ZERO` (servo angle where each link is straight/horizontal) and `KIN_SIGN`
   (+1 if increasing the angle lifts the link) once the horns are fitted. Also measure and update `SHOULDER_H` and `L3`.
   `L1 = L2 = 49.6 mm` was measured pivot-to-pivot from the supplied STLs.
7. `PCA_OSC_HZ` is 27 MHz (same as the baseline). If you have a scope, measure the actual pulse
   frequency and correct it. The `CONFIG` line reports the computed `pwm_hz`.

---

## 4. Serial protocol

115200 baud, ASCII, one command per line ending in `\n` (`\r` is also accepted), max 48 characters, case-insensitive.
An optional `#<id>` prefix is echoed in the reply. The GUI numbers every command, so the log
shows exactly which ACK/DONE/ERR belongs to which command.

| Command | Meaning | Reply |
|---|---|---|
| `S <joint> <deg>` | Move one joint. Joint = 0–4 or `BASE/SWING/ARM1/ARM2/GRIP` | `ACK S joint= target= eta=` then `DONE S pos=…` |
| `S<n> <deg>` | Baseline-compatible form (e.g. `S1 90`) | same |
| `XY <x> <y> [UP\|DOWN]` | Planar IK for shoulder+elbow. mm, origin at shoulder pivot, x forward, y up. Default branch UP | `ACK XY … SWING= ARM1= residual=` / `ERR UNREACHABLE` / `ERR JOINT_LIMIT` / `ERR UNSAFE_PATH` |
| `SPEED <deg/s>` | Joint speed, 5–180 deg/s (default 60) | `ACK SPEED speed=` |
| `HOME` | Energise any off joints at HOME, move the rest to HOME | `ACK HOME`, `EVENT HOME_ENERGISED …`, `DONE HOME` |
| `STOP` (or `X`) | Freeze every joint where it is, drop the current command | `ACK STOP state=HOLD` |
| `STATUS` (or `?`) | State, positions, targets, powered joints, speed, watchdog, last error, fault, rx count, uptime | `STATUS state=… pos=… tgt=… en=… spd=… wd=… err=… fault=… rx=… t=…` |
| `RESET_FAULT` | Clear a FAULT after the cause is fixed. **Never resumes** the old command | `ACK RESET_FAULT state=HOLD/INIT` |
| `WATCHDOG <ms\|0>` | Comm watchdog, 300–10000 ms, 0 = off (the default). The GUI sets 1500 ms | `ACK WATCHDOG ms=` |
| `CONFIG` | Joint table and kinematic constants | `CONFIG …` lines, then `CONFIG END` |
| `PING`, `HELP` | Liveness / command list | |

**States:** `INIT` (outputs off, needs HOME) → `HOMING` → `IDLE` ⇄ `MOVING` → `HOLD` (after STOP) · `FAULT`.

**Rules the firmware enforces:**
- A parse or validation failure **never moves the arm**. It answers `ERR <CODE> <reason>` and stays in its current state.
  Error codes: `UNKNOWN_CMD BAD_JOINT BAD_NUMBER MISSING_FIELD EXTRA_FIELD OUT_OF_RANGE
  UNREACHABLE JOINT_LIMIT UNSAFE_PATH BAD_BRANCH NOT_HOMED BUSY_HOMING IN_FAULT LINE_TOO_LONG NO_FAULT`.
- Nothing is clamped silently. Out-of-range angles and unreachable XY targets are rejected with the numbers shown.
  The IK never switches elbow branch on its own.
- Every S/XY move is checked along the **actual** motion profile (16 samples). The elbow, wrist and
  gripper tip must stay above the table, and the tip must stay out of the base keep-out zone.
- No command queue: a new move replaces the current one ("latest wins") and the firmware reports
  `EVENT SUPERSEDED #id`. A dragged slider can't build up a backlog of stale moves.
- Motion is non-blocking (20 ms ticks), so STOP and STATUS are handled mid-move.
- **FAULT** is entered on PCA9685/I2C loss (checked every 0.5 s and on every write) or a watchdog timeout during motion.
  The joints freeze and only `RESET_FAULT` clears it. After I2C loss the driver is re-initialised with outputs off,
  and you must send HOME again.

---

## 5. GUI features (Task 6 checklist)

- Port and baud selection, Refresh, Connect/Disconnect, with clear messages for a missing or busy port and for unplugging.
- Named controls for all five joints, with limits **read from the controller**. Values are checked on the host before
  sending, and checked again independently by the firmware.
- **Send-on-release** sliders, or type a value + Enter, so dragging can't flood the controller.
- **STOP** (Esc), **HOME** (Ctrl+H), **RESET FAULT** (Ctrl+R): large labelled buttons. The state banner shows the
  state as text, so it doesn't rely on colour alone.
- Live status panel: state, actual vs target, powered joints, speed, watchdog, last error, fault and link age.
  STATUS is polled every 0.4 s, which also feeds the 1.5 s firmware watchdog.
- Manual command box that sends **raw** text with no host validation, for the malformed-input tests.
- Timestamped log on screen and on disk.

## 6. Communication tests (brief, Task 6 table)

| Test | How | Expected |
|---|---|---|
| Valid joint command | Set Swing to 120 | `ACK #n S …`, then `DONE #n S` |
| Unknown command | Manual: `FOO` | `ERR UNKNOWN_CMD`, no motion |
| Invalid joint/value (≥3) | Manual: `S 9 10`, `S 1 abc`, `S 1 200`, `S 1`, `S1 90 5` | BAD_JOINT / BAD_NUMBER / OUT_OF_RANGE / MISSING_FIELD / EXTRA_FIELD |
| Rapid slider movement | Drag sliders fast | One command per release, `EVENT SUPERSEDED` for replaced moves, no backlog |
| Disconnect during motion | Close the GUI or pull USB mid-move | Pulling USB unpowers the Nano, so servos go limp. To test the **watchdog**, keep the Nano powered and stop the host instead (close the GUI mid-move, or in Serial Monitor send `WATCHDOG 500` then a long move and type nothing): `EVENT FAULT code=COMM_TIMEOUT` within 0.5–1.5 s |
| STOP during motion | Start a slow move (SPEED 10), press Esc | `ACK STOP state=HOLD`, the joint stops where it is |
| Reconnect/reset | Disconnect, reconnect | Nano restarts in **INIT** with outputs off and the GUI loads the current state. Nothing jumps to an old target |
| Unreachable / limit IK | `XY 200 0`, `XY 60 30` | `ERR UNREACHABLE` / `ERR JOINT_LIMIT` |

## 7. Known limits / to verify on the real arm

- The kinematic signs, zero angles, `SHOULDER_H` (≈44 mm) and `L3` (≈73 mm) are estimates from the STLs.
  Verify them before relying on XY.
- The floor/base checks only use the planar model (base yaw is ignored). Add task-specific no-go zones in `poseUnsafe()`.
- The first HOME snaps each joint from wherever it is to HOME, because hobby servos have no position feedback.
  Start with the arm resting near HOME, or support the links by hand.
- Tested by compiling with avr-gcc for the ATmega328P (64.6 % flash, 57.6 % RAM, no warnings) and by running the
  same source in a desktop simulator wired to the GUI over a virtual serial port. It hasn't been run on the physical arm yet.

---
pyserial 3.5 © Chris Liechti, BSD-3-Clause, bundled unmodified in `gui/serial/` (see `gui/serial/LICENSE_pyserial.txt`).
