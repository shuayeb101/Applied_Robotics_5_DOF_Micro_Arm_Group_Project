# 04_hardware (G1, G2)

- Wiring plan: block diagram, pin/channel table, physical layout, lead-length check, power budget (G0/G1)
- `calibration_table.csv` — template F4, all five servos (G2)
- `power_test.csv` — rail voltage/current as more servos move (G2)
- `photos/` — numbered assembly and wiring photos, e.g. `01_foot_and_swing.jpg`

## Pin and channel allocation (Arduino Nano)

| Connection | Wiring |
| --- | --- |
| I2C | Nano A4 → PCA9685 SDA, A5 → SCL |
| PCA9685 logic | Nano 5V → VCC, GND → GND |
| Servo power | Power module 5 V rail → switch → PCA9685 V+ terminal |
| Bulk capacitor | 470–1000 µF across V+ / GND at the PCA9685 |
| Ch 0 / 1 / 2 / 3 / 4 | Base / Swing / Arm 1 / Arm 2 / Gripper |
