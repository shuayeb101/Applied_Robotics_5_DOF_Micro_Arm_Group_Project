# 03_cad (G1, G6)

- `flaw_log.csv` — flaws found in the supplied printed parts (G1)
- `stl/` — **new or modified** parts only (originals stay in `00_baseline/3d_models/`). Name them clearly, e.g. `GripperLeft_v2_tpu-pad.stl`
- CAD source files (Fusion 360 / Onshape exports), dimensioned drawings, slicer screenshots, gripper concept sketches and decision matrix (G6)

## Part redesigns (CAD only, not yet printed or tested)

- `mini_claw/` — gripper concept C6: parametric OpenSCAD mini geared claw, exported STLs, renders, collision checks and grip simulations (see its README)
- `spanner_gripper/` — spanner-inspired gripper (fixed datum jaw + pinion-driven sliding jaw in a dovetail). Parametric source (`spanner_gripper.py`), renders, simulation and test plan in its README. STLs in `stl/GripperSpanner*_v1.stl`
- `counterweight/` — swing counterweight to stop the arm tipping the foot over. STLs in `stl/Swing_v2_counterweight*.stl`
- `wide_base/` — wide, stiffer base: the supplied foot kept intact inside a Ø180 × 1.6 mm plate, with gusset ribs on the servo cradle and engraved pick/place targets. STL in `stl/Foot_v2_wide.stl`

## Chosen build

- `spanner_build/` — **the group's chosen build: spanner gripper + swing counterweight, on the wide base (D6).** Parts list, detailed simulation (pick-and-place map, dynamic tipping, joint limits), assembly order and firmware notes. Start here.

## Full arm designs

- `full_designs/` — the redesigns combined into seven complete arms (D0 baseline to D6; **D6 = spanner gripper + counterweight + wide base**), each assembled from the STLs and tested in simulation: collisions, tipping, servo torque, workspace and a pick-and-place task. Start with its README.
