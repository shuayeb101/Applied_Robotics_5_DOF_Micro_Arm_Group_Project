# Idea: Lab bench power supply for the servos

**Proposed by:** Shuayeb Ahmed · **Date:** 2026-11-17 · **Status:** Decided
**Fulfils:** R1 (servo power), R9 (emergency power cut)

## The idea

Power the five servos from a lab bench power supply set to 5 V, with an adjustable current limit. The Nano stays on USB.

## Why it works

- **It's what the brief asks for:** an external supply with current limiting and a switch, not the Nano's USB [1, Task 3].
- **It has enough current.** All five servos moving need 0.5–1.25 A [2]. The kit's breadboard module only gives 700 mA [3].
- **The voltage is safe.** SG90s are rated at 4.8 V (~5 V) [4]. The PCA9685 servo rail takes up to 6 V [5].
- **It costs nothing.** It's standard lab equipment.
- **The current limit acts as a jam alarm.** If a servo tries to draw more than the limit, the supply drops its voltage to hold the current at the limit (CC mode) [6], and a CC light comes on. A short circuit just pulls the voltage close to zero, so nothing is damaged [7].

## How we'd do it

| Setting | Value | Why |
| --- | --- | --- |
| Voltage | 5.0 V | Rated voltage, below the 6 V maximum. |
| Limit, one servo | 0.3 A | Above running current (100–250 mA [2]), below stall (0.36–0.65 A [2, 8]), so a jam lights CC. |
| Limit, full arm | 1.5 A | Above 5 servos moving (1.25 A), below 5 stalled (≈ 3.25 A [8]). |

1. With the output **off**, set the voltage and limit. Check them with a multimeter.
2. Plug in the Nano and check that the PCA9685 responds. The brief requires logic power first [1, Task 3].
3. Turn the supply output on.
4. To stop or in an emergency, turn the output **off** first.

**Wiring:**
- Supply + goes to PCA9685 V+, and supply − goes to GND.
- Nano 5 V goes to VCC, and Nano GND goes to GND.
- Never connect V+ to VCC.

## Risks

| Risk | What we do about it |
| --- | --- |
| Voltage set above 6 V damages the servos. | Set and check with the output off. |
| Polarity reversed. | Red/black leads, and check before switch-on. Some boards have reverse-polarity protection [9], but check ours. |
| No shared ground, so the servos jitter. | Check before switch-on. |
| The supply is only in the lab. | Book sessions. Use the breadboard module for single-joint tests if needed. |

## Sources

1. CT6056 Group Case Study brief v3.0, `00_baseline/brief/`.
2. ProtoSupplies, *Servo Motor Micro SG90* (measured current). https://protosupplies.com/product/servo-motor-micro-sg90/
3. YwRobot MB102 specifications, via PETERVIS. https://www.petervis.com/Raspberry_PI/Breadboard_Power_Supply/YwRobot_Breadboard_Power_Supply.html
4. TowerPro SG90 datasheet. https://tams.informatik.uni-hamburg.de/lehre/2023ss/vorlesung/es/doc/sg90.pdf
5. Envistia, *PCA9685 Module User Guide*. https://envistiamall.com/blogs/learn/pca9685-16-channel-12-bit-i2c-pwm-servo-motor-driver-module-user-guide
6. B&K Precision, *Understanding CV and CC Modes*. https://www.bkprecision.com/knowledge/technical-notes/5/understanding-constant-voltage-cv-and-constant-current-cc-modes-in-regulated-power-supplies
7. Applied EE Notebook, *Bench Power Supply*. https://applied-ee.github.io/ee-notebook/docs/measurement/test-instruments/bench-power-supply
8. ServoDatabase, *Yahboom SG90* (stall 0.65 A). https://servodatabase.com/servo/yahboom/sg90
9. Adafruit, *PCA9685 Servo Driver* product page. https://www.adafruit.com/product/815
