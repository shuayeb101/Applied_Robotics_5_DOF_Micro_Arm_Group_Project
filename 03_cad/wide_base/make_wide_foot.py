"""Wide base (Foot_v2_wide): a stable replacement for the supplied Foot.

The supplied Foot is a 110 mm "+" of 8 mm wide, 3 mm thick arms. Its support
polygon is a square rotated 45 deg: only ~39 mm from the yaw axis on the
diagonals, so the arm tips the robot over when it reaches out on a diagonal.

Foot_v2_wide = supplied Foot (kept exactly, so the base-servo cradle and screw
holes are unchanged)
  + a 1.6 mm disc round it, Ø180 mm by default. The plate is kept thin so the
    spanner head's low back corner clears its edge when picking just beyond it
    (a 3 mm plate blocked the 100 mm pick in simulation).
    The support polygon becomes a circle: ~90 mm from the axis in EVERY
    direction (2.3 x the old diagonal). The extra plastic sits low and central,
    so it adds restoring moment without adding any load to a servo.
  + a sloped buttress along each of the two thin servo-cradle walls (the walls
    are ~1.5 mm thick and 16 mm tall in the original). Sloped, not ribbed, so a
    servo cable dragged round by the swing slides over instead of catching.
  + a cable lane for the base servo wire: it leaves the SG90 at the shaft end
    (+x, the open end of the cradle), turns, and runs under 3 low clips to the
    plate edge on the +y side (lane_side=-1 for the -y side).
  + engraved (0.5 mm) target rings for the pick-and-place test at 60 and 80 mm,
    +/-45 deg, a FRONT arrow and 15-deg rim ticks: repeatable object placement
    for GRIP-05 / TASK-01 without tape marks.
  + 4 Ø3.4 mm holes to screw the base to a board (pan-head M3 or clamps).

Run:  python3 make_wide_foot.py            -> ../stl/Foot_v2_wide.stl
      python3 make_wide_foot.py --radius 80  (smaller beds; drops the 80 mm targets)
Needs: pip install trimesh manifold3d numpy
AI-assisted (Claude Code): disclose in the report.
"""
from __future__ import annotations

import argparse
import math
import os

import numpy as np
import trimesh
from manifold3d import CrossSection, Manifold, Mesh

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.abspath(os.path.join(HERE, "..", ".."))
SRC = os.path.join(REPO, "00_baseline", "3d_models", "Foot.stl")
OUT = os.path.join(REPO, "03_cad", "stl", "Foot_v2_wide.stl")

P = dict(
    radius=90.0,        # disc radius: Ø180 fits 180 mm beds and up
    thick=1.6,          # plate; the supplied 3 mm cross arms stand 1.4 mm proud of it
    chamfer_top=0.6,
    chamfer_bot=0.4,    # elephant-foot relief
    engrave=0.5,
    line=0.8,           # engraved line width
    target_r=(60.0, 80.0),
    target_yaw=(-45.0, 45.0),
    target_d=17.0,      # ring Ø: a 15 mm cube sits inside with 1 mm to spare
    hole_r=84.0, hole_d=3.4, hole_yaw=(22.5, 112.5, 202.5, 292.5),
    # sloped buttress along each cradle wall (x range = the walls); no vertical rib
    # faces for a dragged servo cable to catch on
    wedge_x=(-18.0, 7.0), wedge_wall_y=7.6, wedge_out_y=30.0, wedge_top=16.0,
    # cable lane for the base servo wire: the SG90 wire leaves the case at the
    # shaft end (+x, open end of the cradle), turns and runs out to one side
    lane_x=14.0, lane_side=1, clip_y=(30.0, 55.0, 80.0),
    clip_in_w=6.0, clip_in_h=2.6, clip_len=4.0, clip_wall=1.2,
)


def to_manifold(m: trimesh.Trimesh) -> Manifold:
    return Manifold(Mesh(vert_properties=np.asarray(m.vertices, np.float32),
                         tri_verts=np.asarray(m.faces, np.uint32)))


def to_trimesh(m: Manifold) -> trimesh.Trimesh:
    g = m.to_mesh()
    return trimesh.Trimesh(g.vert_properties[:, :3].astype(float), g.tri_verts, process=False)


def box(x0, x1, y0, y1, z0, z1):
    return Manifold.cube((x1 - x0, y1 - y0, z1 - z0)).translate((x0, y0, z0))


def disc(p):
    R, t, ct, cb = p["radius"], p["thick"], p["chamfer_top"], p["chamfer_bot"]
    n = 256
    bot = Manifold.cylinder(cb, R - cb, R, n)
    mid = Manifold.cylinder(t - cb - ct, R, R, n).translate((0, 0, cb))
    top = Manifold.cylinder(ct, R, R - ct, n).translate((0, 0, t - ct))
    return bot + mid + top


def wedge(side, p):
    """Sloped buttress in the y-z plane, extruded along the cradle wall (side = +/-1)."""
    y0, y1, h = p["wedge_wall_y"], p["wedge_out_y"], p["wedge_top"]
    x0, x1 = p["wedge_x"]
    z0 = p["thick"] - 0.01
    tri = [(y0, z0), (y1, z0), (y0, h)]
    if side < 0:
        tri = [(-y, z) for y, z in tri][::-1]
    cs = Manifold.extrude(CrossSection([tri]), x1 - x0)
    # map (u, v, w) -> (x = w, y = u, z = v)
    return cs.transform(np.array([[0, 0, 1, x0], [1, 0, 0, 0], [0, 1, 0, 0]], float))


def clips(p):
    """Low bridge clips that hold the base servo wire flat in its lane."""
    out = []
    w, h, L, t = p["clip_in_w"], p["clip_in_h"], p["clip_len"], p["clip_wall"]
    x, top = p["lane_x"], p["thick"]
    for y in p["clip_y"]:
        y = y * p["lane_side"]
        outer = box(x - w / 2 - t, x + w / 2 + t, y - L / 2, y + L / 2, top - 0.01, top + h + t)
        inner = box(x - w / 2, x + w / 2, y - L / 2 - 1, y + L / 2 + 1, top - 1, top + h)
        out.append(outer - inner)
    return out


def ring(cx, cy, d, w, depth, top):
    o = Manifold.cylinder(depth + 1, d / 2, d / 2, 96)
    i = Manifold.cylinder(depth + 2, d / 2 - w, d / 2 - w, 96).translate((0, 0, -0.5))
    return (o - i).translate((cx, cy, top - depth))


def radial_bar(yaw, r0, r1, w, depth, top):
    b = box(r0, r1, -w / 2, w / 2, top - depth, top + 1)
    return b.rotate((0, 0, yaw))


def engravings(p):
    top, dep, w = p["thick"], p["engrave"], p["line"]
    cuts = []
    for r in p["target_r"]:
        for yaw in p["target_yaw"]:
            a = math.radians(yaw)
            cx, cy = r * math.cos(a), r * math.sin(a)
            cuts.append(ring(cx, cy, p["target_d"], w, dep, top))
            # centre cross, 4 mm
            for ang in (yaw, yaw + 90):
                cuts.append(box(-2, 2, -w / 2, w / 2, top - dep, top + 1)
                            .rotate((0, 0, ang)).translate((cx, cy, 0)))
    R = p["radius"]
    for yaw in range(0, 360, 15):
        major = yaw % 45 == 0
        cuts.append(radial_bar(yaw, R - (9 if major else 5), R - 1.5, w, dep, top))
    # FRONT arrow on +x (the arm's zero-yaw direction)
    tri = Manifold.extrude(CrossSection([[(64, -4), (72, 0), (64, 4)]]), dep + 1).translate((0, 0, top - dep))
    cuts.append(tri)
    cuts.append(box(58, 64.5, -w / 2, w / 2, top - dep, top + 1))
    return cuts


def holes(p):
    cuts = []
    for yaw in p["hole_yaw"]:
        a = math.radians(yaw)
        x, y = p["hole_r"] * math.cos(a), p["hole_r"] * math.sin(a)
        cuts.append(Manifold.cylinder(p["thick"] + 2, p["hole_d"] / 2, p["hole_d"] / 2, 48)
                    .translate((x, y, -1)))
    return cuts


def build(p=P):
    foot = trimesh.load(SRC, force="mesh")
    m = to_manifold(foot) + disc(p)
    for s in (1, -1):
        m = m + wedge(s, p)
    for c in clips(p):
        m = m + c
    for c in engravings(p) + holes(p):
        m = m - c
    return to_trimesh(m)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--radius", type=float, default=P["radius"])
    ap.add_argument("--out", default=OUT)
    a = ap.parse_args()
    p = dict(P, radius=a.radius)
    if a.radius < 90:
        p["target_r"] = (60.0,)          # 80 mm ring would cut the rim
    p["hole_r"] = a.radius - 6
    m = build(p)
    os.makedirs(os.path.dirname(a.out), exist_ok=True)
    m.export(a.out)
    orig = trimesh.load(SRC)
    lost = trimesh.boolean.difference([orig, m], engine="manifold")
    print(f"{a.out}: watertight={m.is_watertight} volume={m.volume / 1000:.1f} cm3 "
          f"bounds={np.round(m.bounds, 1).tolist()}  original material removed "
          f"{abs(lost.volume) if len(lost.faces) else 0:.2f} mm3")


if __name__ == "__main__":
    main()
