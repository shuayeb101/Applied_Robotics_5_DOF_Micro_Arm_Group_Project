# Component simulations: re-run check

The two gripper concepts each come with their own mechanism simulation. They were re-run unchanged, in a scratch copy, with the full-arm study, to confirm the committed results reproduce.

| Simulation | Command | Outcome |
| --- | --- | --- |
| Spanner gripper: rigid-body sweep, 141 steps, 12 checks | `python3 03_cad/spanner_gripper/simulate.py` | Report **identical** to the committed `spanner_gripper/sim/simulation_report.md`. All 12 checks pass. |
| Mini claw: open/close mechanism, 193 poses, 2,895 pair checks | `python3 03_cad/mini_claw/sim/mechanism_sim.py` (needs OpenSCAD) | Report **identical** to the committed `mini_claw/sim/results/mechanism_report.md`. No overlaps, 0.07 mm mesh gap. |
| Mini claw: MuJoCo grip, quick mode (1 trial per curl, cube, μ 0.12) | `python3 mini_claw/sim/grip_sim.py --quick` | Same trend as the full committed run (table below). The chosen 30° curl holds the cube with 0.07 mm slip. |

Quick grip re-run (cube 15 mm, 5 g, μ = 0.12, placement shifted across the jaws):

| Tip curl (deg) | Held | Secure (slip < 2 mm) | Median slip (mm) | Released | Grip angle (deg) |
| --- | --- | --- | --- | --- | --- |
| 5.44 (straight) | 0/1 | 0/1 | – | – | – |
| 12 | 1/1 | 0/1 | 6.46 | 1/1 | 17.6 |
| 18 | 1/1 | 0/1 | 3.29 | 1/1 | 15.5 |
| 24 | 1/1 | 1/1 | 0.23 | 1/1 | 13.8 |
| **30 (chosen)** | 1/1 | 1/1 | **0.07** | 1/1 | 13.9 |

The original rack-and-pinion gripper has no mechanism simulation. Its numbers in the main README come from the spanner README's comparison, which was measured from the supplied STLs.
