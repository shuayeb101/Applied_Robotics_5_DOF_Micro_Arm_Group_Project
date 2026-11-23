# Mini geared claw (gripper concept C6)

A small 3D-printed angular gripper for the SG90. It works like the Kitronik Klaw (the jaws are geared together), and its style follows the reference metal claw: curved pointed jaws, serrated gripping edges, round lightening holes and "stacked plate" grooves. It's sized for our 10–20 mm, ≤ 5 g objects. Background and the other concepts are in [`09_report/engineering_cycle/02_research_gripper.md`](../../09_report/engineering_cycle/02_research_gripper.md).

![Closed, open and 3D views](renders/overview.png)

> **Status: design only, not printed or tested.** Values marked MEASURE in `mini_claw.scad` are estimates for the SG90 and must be checked on our servo first.
> **AI disclosure (C12):** model, checks and this README were drafted with AI assistance (Claude). The group must check and understand them before using them as evidence.

## How it works

```
servo ──► pinion (10 T, layer 1) ──► right jaw gear (layer 1)
                                      right jaw gear (layer 2) ──► left jaw gear (layer 2)
```

- The **pinion** sits on the servo spline. It keeps the **original GripperGear's spline hub** (copied from the baseline STL), so it fits the SG90 the same way. Only the teeth are new: involute, module 0.9.
- The jaw gears sit on **two layers**. The pinion only meshes with the right jaw in the lower layer. The two jaws mesh with each other in the upper layer. This keeps the **grip centre on the servo shaft line**, as on the original gripper. With a single layer the servo would have to sit about 7 mm off-centre, which would shift the grip sideways.
- The jaws turn on **M2 screws** inside printed sleeves. The sleeves stand up from the bottom plate and also hold the top plate.
- The bottom plate contains the **original FrontPlate's 43 × 20 mm footprint, servo window (23.4 × 12.6 mm) and screw holes (27.5 mm apart)**, so the servo mounts exactly as before.

## Key numbers

| | Value |
| --- | --- |
| Gear ratio | 1.5 : 1 (servo turns 1.5° per jaw degree) |
| Servo travel used | ≈ 45° for fully closed → fully open |
| Tip travel | ≈ 0.36 mm per servo degree, per jaw |
| Grip force per jaw at SG90 stall | ≈ 3.0–4.4 N at the tips, 4.5–6.8 N mid-jaw (F = 0.75 T / L, before gear friction) |
| Grip force needed (worst case) | ≈ 1.6 N per jaw (research doc, section 3) |
| Gripping face height | 9 mm (6 mm gear stack + 3 mm extra finger depth) |
| Overall size | ≈ 43 × 49 × 21 mm without servo |

**Gap against jaw angle**, from the model geometry:

| Jaw angle θ | Servo change | Tip gap | Gap at mid-edge | Gap at edge start |
| --- | --- | --- | --- | --- |
| 0° | 0° | 0.4 mm | 2.4 mm | 4.4 mm |
| 10° | 15° | 11.0 mm | 9.3 mm | 7.7 mm |
| 20° | 30° | 21.7 mm | 16.4 mm | 11.1 mm |
| 30° | 45° | 32.2 mm | 23.4 mm | 14.6 mm |

So a 15 mm cube is gripped mid-jaw at θ ≈ 18° (servo ≈ 27° from closed), and a 12 mm cylinder at θ ≈ 14°.

## Mass

Volumes from the exported STLs, as if printed **100 % solid** (PLA 1.24 g/cm³). The original is measured the same way, so the comparison is fair.

| Part | Mini claw | | Original gripper parts | |
| --- | --- | --- | --- | --- |
| Pinion | 0.14 cm³ | 0.17 g | Gear 0.14 cm³ | 0.17 g |
| Jaws ×2 | 2.27 cm³ | 2.82 g | Jaws ×2 4.14 cm³ | 5.13 g |
| Bottom plate (+ sleeves) | 1.86 cm³ | 2.31 g | FrontPlate 1.12 cm³ | 1.39 g |
| Top plate | 0.49 cm³ | 0.61 g | Racks ×2 0.94 cm³ | 1.17 g |
| **Total plastic** | **4.77 cm³** | **5.9 g** | **6.34 cm³** | **7.9 g** |
| Screws | 2 × M2 ≈ 0.9 g | | — | |

It's about **2 g lighter in plastic** (25 %), or about 1 g lighter once the steel screws are included. With normal slicer settings both designs print lighter than these figures. Thin parts like these are mostly outer walls, so **infill makes little difference**. The saving comes from the shape: lightening holes and smaller jaws. Weigh both printed sets to confirm.

## What was checked (in OpenSCAD)

The `check_*` parts intersect the moving parts in 2D at a given jaw angle. An empty result means no overlap. They were run at θ = 0, 5, 10, 15, 20, 25 and 30°.

| Check | Result |
| --- | --- |
| Pinion vs right jaw gear (layer 1) | No overlap at any angle |
| Pinion vs left jaw (layer 1) | No overlap |
| Right vs left jaw, layer 1 | No overlap |
| Right vs left jaw gears, layer 2 | No overlap |
| Lowered finger section vs bottom plate and servo | No overlap |

To make sure the checks can fail, they were also run with deliberate errors: tips overlapping (tip gap −2 mm), the pinion turned tooth-on-tooth, and negative backlash. Each was correctly reported as an **overlap**.

```bash
openscad -D 'part="check_l2_jaws"' -D theta=20 -o check.dxf mini_claw.scad
```

"Current top level object is empty" means the check passed.

## Before printing: MEASURE

1. **Servo heights:** tab thickness (2.5 mm assumed) and tab top to spline top (10.5 mm assumed). These set how high the gear stack sits. If the spline is lower or higher, change `tab_top_to_spline_top`.
2. **Shaft position** in the window: the shaft centre to the near window end (5.9 mm assumed).
3. **How the original FrontPlate attaches to ArmFront.** The new bottom plate keeps FrontPlate's footprint, but anything outside that footprint must not hit the arm.
4. **Gear fit on our printer.** Print the pinion and one jaw, then turn them by hand. If they bind, raise `backlash` (0.15 mm now). If they rattle, lower it. Printer clearances differ (research doc, source 16).

## Printing

| Part | Orientation | Notes |
| --- | --- | --- |
| `mini_claw_jaw_right.stl`, `mini_claw_jaw_left.stl` | Flat top face down | No supports. Grooves are 0.5 mm deep and bridge fine. 0.12–0.16 mm layers for clean teeth. |
| `mini_claw_pinion.stl` | Spline recess up | Same as the original gear |
| `mini_claw_bottom_plate.stl` | Plate down, sleeves up | Sleeves are thin (2 mm radius): print slowly or use 100 % infill for them |
| `mini_claw_top_plate.stl` | Flat | |

4 perimeters, so the teeth and sleeves come out solid. Infill 15–20 % (it barely matters at these thicknesses).

## Parts list

- Printed: pinion, right jaw, left jaw, bottom plate, top plate
- 2 × M2 × 20 screws as the jaw pivots (into the sleeves). Self-tapping, or use a 1.75 mm filament pin instead.
- The SG90 and its mounting screws and horn screw (already in the kit)

## Assembly

1. Fit the servo through the bottom plate window, the same way it goes on FrontPlate.
2. **Set the servo angle first.** Send the gripper to its "closed" angle (e.g. `S GRIP 30`) with the servo connected but nothing fitted.
3. Press the pinion onto the spline and fit the horn screw.
4. Put both jaws on their sleeves **closed** (tips touching). Mesh the left jaw with the right jaw first, then turn the pinion gently until the right jaw meshes with it.
5. Fit the top plate and the two M2 screws. Snug them, not tight, so the jaws turn freely (as on the Klaw).
6. Open in small steps from the GUI (`S GRIP 35`, `40`, …) and check nothing rubs.

## Firmware changes needed

- **The gripper range shrinks to about 45°** of servo travel. Set `GRIP lo/hi/home` in the firmware calibration block from the measured closed and open angles. The current 30–150° would drive it past its stops.
- **Grip to an angle, not fully closed** (concept C5). Use the gap table above to find each object's angle, and command a few degrees past contact.
- **Update `L3`** (wrist pivot to gripper tip). The new tip position depends on how the plate attaches, so measure it once assembled.

## Trade-offs (for the decision matrix)

| For | Against |
| --- | --- |
| Lighter (≈ 25 % less plastic) | All new parts: 5 prints and 2 screws, against 2 jaw reprints for C2 |
| Opens wider (≈ 32 mm at the tips vs ≈ 19 mm change) | Jaws swing in an arc, so grip point and contact angle change with object size |
| Gear reduction gives finer control and more force mid-jaw | Coarser tip resolution (0.36 vs 0.08 mm per servo degree) |
| Serrations and grooves texture the grip face for free | Small printed teeth (module 0.9) need a well-tuned printer |
| Every part prints flat with no supports | The servo heights are estimates until measured |
