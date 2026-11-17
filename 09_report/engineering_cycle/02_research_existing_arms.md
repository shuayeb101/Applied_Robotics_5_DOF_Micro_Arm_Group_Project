# Research: existing 5-DOF micro arms

What others have built that's similar to our arm, and what we can learn from them.

**What counts as similar:** five joints (base, shoulder, elbow, wrist, gripper), or six at most; **only micro servos** (9 g class, SG90 or MG90S); an Arduino-class board; a printed or small kit body. Arms that use large servos (e.g. MG996R), stepper motors, or only four joints are left out, because we can't use those parts (C1).

## The list

### 1. ShillehTek: Arduino Nano + PCA9685 6-DOF arm

- **Hardware:** six MG90S micro servos (base, shoulder, elbow, wrist twist, wrist tilt, gripper), an Arduino Nano and a PCA9685 over I²C [1].
- **Power:** a separate 5 V, 2–3 A servo supply with a shared ground, because each servo can draw about 700 mA at peak [1].
- **Control:** a joystick. The code ignores small stick movements (a dead zone) and nudges the target angle a little every 20 ms [1].

| Advantages | Disadvantages |
| --- | --- |
| Same electronics as ours (Nano + PCA9685). | Only base and shoulder control is published. No emergency stop or collision checks [1]. |
| Correct power setup: external supply, shared ground [1]. | Angles are quietly clamped to 0–180°. A fixed 150–600 count pulse range may need calibrating for each servo [1]. Our brief forbids both (G5, G12). |
| Incremental moves give smooth motion [1]. | No reach, load or accuracy figures [1]. No IK [1]. |

### 2. Hackster: Bluetooth-controlled 5-DOF arm (Atharva Kaplay)

- **Hardware:** five joints (base, shoulder, elbow, wrist, gripper), an Arduino Uno, Nano or Mega, and a PCA9685 [2]. The servo type is left open: MG996R, SG90 or similar [2].
- **Power:** an external 5–6 V servo supply. The Arduino's 5 V powers the PCA9685 logic, and all grounds are joined [2].
- **Control:** smoothed (interpolated) motion with adjustable speed, over serial or Bluetooth [2].

| Advantages | Disadvantages |
| --- | --- |
| Same joint layout and electronics as ours [2]. | No measured results: no payload, accuracy or speed [2]. |
| Smoothed motion with adjustable speed [2], which is what our R8 needs. | IK, feedback and automatic calibration are only listed as future work [2]. |
| Wiring matches our bench-supply plan [2]. | Servo choice is left open, so the torque limits aren't known [2]. |

### 3. Cults3D: 5DOF robot arm with MG90S servos

- **Hardware:** a 5-DOF printed arm, an Arduino with a Sensor Shield V5.0 (no PCA9685), and a separate servo supply [3].
- **Servos:** MG90S at the shoulder, which needs the most torque. The author notes that 9 g servos may be fine for the gripper and wrist [3].
- **Built for:** small-scale training for students. The base is a cube filled with concrete to keep it steady on a desk [3].

| Advantages | Disadvantages |
| --- | --- |
| Made for students, at the same size and with the same kind of servos as ours [3]. | No PCA9685, so the Arduino makes every servo signal itself [3]. |
| Deals with tipping by weighing down the base [3]. | The author had to test several MG90S servos because cheap ones seemed too weak for the shoulder [3]. This backs up our payload worry (F4). |
| Updates added a stronger servo holder and a new gear for the SG90 [3]. | Basic gripper [3]. No control details or results. |

### 4. "Small Robot Arm 5 Motor SG90" kit

- **Hardware:** five SG90 servos for base, shoulder, elbow, wrist and gripper. Sold as an educational kit for pick-and-place and kinematics practice [4].

| Advantages | Disadvantages |
| --- | --- |
| Exactly our servo set and joint layout [4]. | It's a seller's product page: no technical data, controller details or test results [4]. A weak source. |
| Shows that an all-SG90 five-joint arm is a normal teaching design [4]. | — |

## Comparison

| | 1. ShillehTek | 2. Hackster | 3. Cults3D | 4. SG90 kit | **Our arm** |
| --- | --- | --- | --- | --- | --- |
| Joints | 6 | 5 | 5 | 5 | 5 |
| Servos | 6 × MG90S | Open choice | MG90S (+ 9 g) | 5 × SG90 | 5 × SG90 |
| Controller | Nano | Uno/Nano/Mega | Arduino + shield | Not stated | Nano |
| PCA9685 | ✓ | ✓ | ✗ | Not stated | ✓ |
| Separate servo power | ✓ | ✓ | ✓ | Not stated | ✓ (D1) |
| Smooth motion | Incremental | Interpolated | Not stated | Not stated | Non-blocking, speed in deg/s [5] |
| Coordinate control (IK) | ✗ | ✗ (future) | Not stated | Not stated | ✓ planar IK [5] |
| Rejects bad commands | ✗ (clamps) | Not stated | Not stated | Not stated | ✓ [5] |
| Measured results | ✗ | ✗ | ✗ | ✗ | Planned (G2–G7) |

## What we take from this

1. **Everyone powers the servos separately, with a shared ground** [1, 2, 3]. This confirms D1.
2. **SG90s are weak at the shoulder** [3]. Keep loads light (≤ 5 g, F4) and test holding a load early.
3. **Smooth motion with adjustable speed is common** [1, 2]. Our firmware already does this, and R8 needs it measured.
4. **Steady the base** [3]. Check whether our Foot tips over at full reach. If it does, add weight using only free or printed parts (C1, C3).
5. **None of them publish measured accuracy, payload or safety tests** [1–4]. Our measured results and safety features (G5–G7) would go beyond what's published.
6. **Avoid their shortcuts:** quietly clamping angles and using a fixed pulse range [1] both break our brief (G5, G12).

## Sources

Accessed 2026-11-17.

1. ShillehTek, *Arduino PCA9685: Joystick-Controlled 6-DOF Arm*. https://shillehtek.com/blogs/news/arduino-pca9685-robotic-arm-joystick-control
2. A. Kaplay, *Bluetooth Controlled 5 DOF Robotic Arm*, Hackster.io. https://www.hackster.io/atharva-kaplay/bluetooth-controlled-5-dof-robotic-arm-1436fc
3. *5DOF robotarm with MG90s servos*, Cults3D, via STLFinder. https://stlfinder.com/model/5dof-robotarm-with-mg90s-servos-9R3juYbv/2775700
4. Electrolik, *Small Robot Arm 5 Motor SG90*, product page. https://electrolik-eg.com/?p=47381
5. S. Ahmed, *MicroArm firmware v1.0.0* and README, `05_firmware/`, this repository.
