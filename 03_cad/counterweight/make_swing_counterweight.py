"""Generate Swing_v2_counterweight.stl and its lid from the untouched baseline Swing.stl.

The counterweight is fused to the Swing (the part the base servo rotates), on the
side opposite the arm. Because it is part of the Swing it always turns with the
base: whichever way the arm swivels, the weight points the other way.

Frame (same as Swing.stl, units mm):
  origin = base-servo (yaw) axis, z = 0 = underside of the swing disc
  +x     = direction the arm reaches (shoulder servo shaft sits at x ~ 0)
  -x     = behind the arm -> counterweight goes here

Run:  python3 make_swing_counterweight.py      (needs: pip install trimesh manifold3d)
"""
from pathlib import Path

import numpy as np
import trimesh
from trimesh.creation import box, cylinder

ROOT = Path(__file__).resolve().parents[2]
SRC = ROOT / "00_baseline" / "3d_models" / "Swing.stl"
OUT = ROOT / "03_cad" / "stl"

# ---------------- parameters (edit here) ----------------
POCKET_X = -52.0      # centre of the weight pocket (radius 52 mm from yaw axis)
POCKET_BORE = 26.6    # fits UK 2p coins (25.9 mm); 7 coins = 49.8 g, 14.2 mm tall
POCKET_WALL = 2.2
POCKET_FLOOR = 1.5
POCKET_TOP = 2.5      # rim height; lid (1.5 mm) brings it to z = 4.0
POCKET_DEPTH = 15.5   # internal depth: 7 coins + 1.3 mm for a packing shim
PUSH_HOLE = 10.0      # hole in the floor to push coins out from below
BOOM_W = 16.0         # boom width (y)
BOOM_T = POCKET_TOP   # boom plate z 0 .. 2.5
BOOM_X0 = -19.0       # boom starts inside the cradle wall so it fuses solidly
RIB_DEPTH = 6.0       # stiffening ribs under the boom
PIVOT_X = -30.0       # lid screw (M3 x 10 self-tapping, or M3 x 10 + nut)
PIVOT_PILOT = 2.6     # pilot for self-tapping M3 into PLA
PIVOT_BOSS_R = 4.0
LID_T = 1.5
LID_CLEAR = 0.3       # lid outline smaller than pocket outer by this
SECTIONS = 96


def cyl(r, z0, z1, x=0.0, y=0.0):
    c = cylinder(radius=r, height=z1 - z0, sections=SECTIONS)
    c.apply_translation([x, y, (z0 + z1) / 2])
    return c


def cube(x0, x1, y0, y1, z0, z1):
    b = box(extents=[x1 - x0, y1 - y0, z1 - z0])
    b.apply_translation([(x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2])
    return b


def build():
    swing = trimesh.load(SRC)
    r_out = POCKET_BORE / 2 + POCKET_WALL
    z_bot = POCKET_TOP - POCKET_DEPTH - POCKET_FLOOR

    boom = cube(POCKET_X, BOOM_X0, -BOOM_W / 2, BOOM_W / 2, 0, BOOM_T)
    # Ribs start at r = 24 mm so they clear the (static) foot post, max r ~ 23.4 mm.
    ribs = [cube(POCKET_X, -24.0, s * BOOM_W / 2 - (2.0 if s > 0 else 0), s * BOOM_W / 2 + (0 if s > 0 else 2.0),
                 -RIB_DEPTH, 0) for s in (-1, 1)]
    pocket_body = cyl(r_out, z_bot, POCKET_TOP, x=POCKET_X)
    boss = cyl(PIVOT_BOSS_R, -RIB_DEPTH, BOOM_T, x=PIVOT_X)

    add = trimesh.boolean.union([swing, boom, *ribs, pocket_body, boss], engine="manifold")

    bore = cyl(POCKET_BORE / 2, POCKET_TOP - POCKET_DEPTH, POCKET_TOP + 1, x=POCKET_X)
    push = cyl(PUSH_HOLE / 2, z_bot - 1, POCKET_TOP, x=POCKET_X)
    pilot = cyl(PIVOT_PILOT / 2, -RIB_DEPTH + 1.0, BOOM_T + 1, x=PIVOT_X)  # blind: open on top, 1 mm floor below
    part = trimesh.boolean.difference([add, bore, push, pilot], engine="manifold")

    # Lid: one M3 screw holds it; unscrew and lift off to add or remove coins.
    lid_outline = trimesh.boolean.union(
        [cyl(PIVOT_BOSS_R, 0, LID_T, x=PIVOT_X), cyl(r_out - LID_CLEAR, 0, LID_T, x=POCKET_X),
         cube(POCKET_X, PIVOT_X, -PIVOT_BOSS_R, PIVOT_BOSS_R, 0, LID_T)], engine="manifold")
    lid = trimesh.boolean.difference([lid_outline, cyl(3.4 / 2, -1, LID_T + 1, x=PIVOT_X)], engine="manifold")
    lid.apply_translation([0, 0, -lid.bounds[0, 2]])  # print flat
    return swing, part, lid


if __name__ == "__main__":
    swing, part, lid = build()
    OUT.mkdir(parents=True, exist_ok=True)
    part.export(OUT / "Swing_v2_counterweight.stl")
    lid.export(OUT / "Swing_v2_counterweight_lid.stl")
    for n, m in [("Swing (baseline)", swing), ("Swing_v2_counterweight", part), ("lid", lid)]:
        print(f"{n:24s} watertight={m.is_watertight} vol={m.volume:7.0f} mm^3 "
              f"bounds={np.round(m.bounds, 1).tolist()}")
