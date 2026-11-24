"""Render Foot_v2_wide next to the supplied Foot (top view + 3D), with the
support polygon of each.  python3 render_foot.py -> preview.png"""
import os
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import trimesh
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from scipy.spatial import ConvexHull

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.abspath(os.path.join(HERE, "..", ".."))
old = trimesh.load(f"{REPO}/00_baseline/3d_models/Foot.stl")
new = trimesh.load(f"{REPO}/03_cad/stl/Foot_v2_wide.stl")
light = np.array([0.4, -0.6, 0.8]); light /= np.linalg.norm(light)


def shade(ax, m, col):
    s = 0.45 + 0.55 * np.clip(m.face_normals @ light, 0, 1)
    ax.add_collection3d(Poly3DCollection(m.vertices[m.faces], facecolors=np.clip(s[:, None] * col, 0, 1),
                                         edgecolor="none"))


fig = plt.figure(figsize=(15, 5.5))
ax = fig.add_subplot(1, 3, 1)
for m, c, lab, yc in ((new, "#C44E52", "Foot_v2_wide", -2.0), (old, "#4C72B0", "supplied Foot", -2.0)):
    for ycut, ls in ((yc, "-"),):
        sec = m.section(plane_origin=(-5.0, 0, 0), plane_normal=(1, 0, 0))
        for e in sec.discrete:
            ax.plot(e[:, 1], e[:, 2], color=c, lw=1, ls=ls)
    ax.plot([], [], color=c, label=lab)
ax.set_aspect("equal"); ax.set_ylim(-2, 22); ax.set_xlim(-40, 40); ax.grid(alpha=.3); ax.legend(fontsize=8)
ax.set_title("Section at x = -5 mm across the cradle:\n1.6 mm plate, original 3 mm cross arm, cradle walls + buttresses")
ax.set_xlabel("y (mm)"); ax.set_ylabel("z (mm)")
ax = fig.add_subplot(1, 3, 2, projection="3d")
crop = trimesh.boolean.intersection([new, trimesh.creation.box((60, 70, 30), trimesh.transformations.translation_matrix((0, 0, 15)))], engine="manifold")
shade(ax, crop, np.array([0.80, 0.25, 0.55]))
ax.set_xlim(-30, 30); ax.set_ylim(-35, 35); ax.set_zlim(0, 30); ax.set_box_aspect((1, 1, .5))
ax.view_init(25, -30); ax.set_title("Close-up: unchanged servo cradle, sloped buttresses, wire clips")
ax = fig.add_subplot(1, 3, 3)
for m, c, lab in ((new, "#C44E52", "Foot_v2_wide"), (old, "#4C72B0", "supplied Foot")):
    sec = m.section(plane_origin=(0, 0, 0.3), plane_normal=(0, 0, 1))
    for e in sec.discrete:
        ax.plot(e[:, 0], e[:, 1], color=c, lw=.8)
    h = ConvexHull(sec.vertices[:, :2]); v = sec.vertices[h.vertices, :2]
    d = np.min(np.abs(h.equations[:, 2]))
    ax.fill(v[:, 0], v[:, 1], color=c, alpha=.12, label=f"{lab}: support edge ≥ {d:.0f} mm from axis")
top = new.section(plane_origin=(0, 0, 1.35), plane_normal=(0, 0, 1))
for e in top.discrete:
    ax.plot(e[:, 0], e[:, 1], color="0.3", lw=.5)
ax.set_aspect("equal"); ax.legend(loc="lower center", fontsize=8); ax.grid(alpha=.3)
ax.set_title("Footprint (support polygon) and engraved targets, from above")
ax.set_xlabel("x: + = arm forward at zero yaw (mm)")
plt.tight_layout(); plt.savefig(os.path.join(HERE, "preview.png"), dpi=90)
print("preview.png")
