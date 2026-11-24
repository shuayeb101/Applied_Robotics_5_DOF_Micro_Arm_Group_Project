# 03_cad (G1, G6)

- `flaw_log.csv` — flaws found in the supplied printed parts (G1)
- `stl/` — **new or modified** parts only (originals stay in `00_baseline/3d_models/`). Name them clearly, e.g. `GripperLeft_v2_tpu-pad.stl`
- CAD source files (Fusion 360 / Onshape exports), dimensioned drawings, slicer screenshots, gripper concept sketches and decision matrix (G6)

## Gripper redesign

- `spanner_gripper/` — spanner-inspired gripper (fixed datum jaw + pinion-driven sliding jaw in a dovetail). Parametric source (`spanner_gripper.py`), renders, rationale and test plan in its README. STLs in `stl/GripperSpanner*_v1.stl`. **CAD only — not yet printed or tested.**
