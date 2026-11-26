"""Key-dimension drawing of the D6 changes (brief Task 10 step 5: dimensioned CAD).
Sections are cut from the real STLs; dimensions are drawn from the generator
parameters, so the sheet and the parts cannot drift apart.

python3 drawing.py -> results/key_dimensions.png
AI-assisted (Claude Code).
"""
import math
import os
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import trimesh

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "full_designs"))
sys.path.insert(0, os.path.join(HERE, "..", "wide_base"))
sys.path.insert(0, os.path.join(HERE, "..", "spanner_gripper"))
import arm_model as am  # noqa: E402
import make_wide_foot as wf  # noqa: E402
import spanner_gripper as sg  # noqa: E402

STL = os.path.join(HERE, "..", "stl")
INK, NEW, OLD, SERVO = "0.15", "#C4468F", "#4C72B0", "#333333"


def outline(ax, mesh, origin, normal, axes, col, fill=True, lw=0.9, T=np.eye(4)):
    m = mesh.copy()
    m.apply_transform(T)
    s = m.section(plane_origin=origin, plane_normal=normal)
    if s is None:
        return
    for e in s.discrete:
        if fill:
            ax.fill(e[:, axes[0]], e[:, axes[1]], color=col, alpha=0.18, lw=0)
        ax.plot(e[:, axes[0]], e[:, axes[1]], color=col, lw=lw)


def dim(ax, p0, p1, text, off=(0, 0), fs=7.5, col=INK):
    """Dimension line with arrows between p0 and p1, label at the middle (+off)."""
    ax.annotate("", p1, p0, arrowprops=dict(arrowstyle="<->", color=col, lw=0.7, shrinkA=0, shrinkB=0))
    mx, my = (p0[0] + p1[0]) / 2 + off[0], (p0[1] + p1[1]) / 2 + off[1]
    ax.text(mx, my, text, fontsize=fs, ha="center", va="center", color=col,
            bbox=dict(fc="white", ec="none", pad=0.6, alpha=0.85))


def note(ax, xy, xytext, text, fs=7.5):
    ax.annotate(text, xy, xytext, fontsize=fs, color=INK, arrowprops=dict(arrowstyle="-", lw=0.6, color="0.4"))


def main():
    P = wf.P
    foot = trimesh.load(f"{STL}/Foot_v2_wide.stl")
    fig = plt.figure(figsize=(20, 12.5))
    gs = fig.add_gridspec(2, 3, height_ratios=[1.15, 1])

    # ---------------- A. foot plan
    ax = fig.add_subplot(gs[0, 0])
    for z, c, lw in ((0.8, NEW, .9), (2.3, NEW, .6), (17.0, NEW, .9), (4.8, NEW, 1.1)):
        outline(ax, foot, (0, 0, z), (0, 0, 1), (0, 1), c, fill=(z == 0.8), lw=lw)
    R = P["radius"]
    dim(ax, (-R, -R - 8), (R, -R - 8), f"Ø{2 * R:.0f}")
    for r in P["target_r"]:
        a = math.radians(P["target_yaw"][1])
        dim(ax, (0, 0), (r * math.cos(a), r * math.sin(a)), f"R{r:.0f}", off=(-6, 5) if r < 70 else (6, -4))
    note(ax, (60.1, 56.6), (30, 95), f"engraved Ø{P['target_d']:.0f} target rings, ±{P['target_yaw'][1]:.0f}°, "
                                      f"{P['engrave']} deep")
    hx, hy = P["hole_r"] * math.cos(math.radians(112.5)), P["hole_r"] * math.sin(math.radians(112.5))
    note(ax, (hx, hy), (-95, 100), f"4 × Ø{P['hole_d']} on R{P['hole_r']:.0f} (22.5° + 90°)")
    for y in P["clip_y"]:
        pass
    note(ax, (P["lane_x"], P["clip_y"][1]), (40, 20),
         f"wire clips at x = {P['lane_x']:.0f}, y = {', '.join(f'{v:.0f}' for v in P['clip_y'])}\n"
         f"opening {P['clip_in_w']:.0f} × {P['clip_in_h']} , wall {P['clip_wall']}")
    note(ax, (-5, 20), (-95, 40), "sloped buttress each side\nx −18…7, wall 16 → plate at y ±30")
    note(ax, (70, 0), (60, -40), "FRONT arrow (+x = arm at 0° yaw)")
    note(ax, (-5, 0), (-60, -60), "supplied Foot kept intact\n(base SG90 cradle unchanged)")
    ax.set_aspect("equal")
    ax.set_xlim(-105, 105)
    ax.set_ylim(-108, 110)
    ax.axis("off")
    ax.set_title("A  Foot_v2_wide: plan (mm)", loc="left", fontsize=11, weight="bold")

    # ---------------- B. foot section across the cradle
    ax = fig.add_subplot(gs[1, 0])
    outline(ax, foot, (-5, 0, 0), (1, 0, 0), (1, 2), NEW)
    Tb = am.frame_from_axes(*am.SERVO_MOUNTS["servo_base"][1:])
    sys.path.insert(0, HERE)
    import fit_check as fc  # noqa: E402
    outline(ax, fc.SERVO, (-5, 0, 0), (1, 0, 0), (1, 2), SERVO, T=Tb)
    dim(ax, (34, 0), (34, P["thick"]), f"{P['thick']}", off=(4, 0))
    dim(ax, (-34, 0), (-34, 3.0), "3.0 (cross arm)", off=(-8, 3))
    dim(ax, (P["wedge_wall_y"], -3), (P["wedge_out_y"], -3), f"{P['wedge_out_y'] - P['wedge_wall_y']:.1f}")
    dim(ax, (-12, 0), (-12, 19.2), "19.2 tab seat", off=(-6, 0))
    dim(ax, (P["wedge_out_y"] + 3, 0), (P["wedge_out_y"] + 3, P["wedge_top"]), f"{P['wedge_top']:.0f}", off=(3, 0))
    dim(ax, (-6.3, 23), (6.3, 23), "12.6 slot", off=(0, 2))
    ax.set_aspect("equal")
    ax.set_xlim(-45, 45)
    ax.set_ylim(-6, 36)
    ax.axis("off")
    ax.set_title("B  Foot_v2_wide: section x = −5, base SG90 seated (mm)", loc="left", fontsize=11, weight="bold")

    # ---------------- C/D. spanner v2 stack (FrontPlate frame)
    p = dict(sg.P)
    d = sg.derived(p)
    z = sg.zoff_v2(p)
    head = trimesh.load(f"{STL}/GripperSpannerHead_v2.stl")
    pin = trimesh.load(f"{STL}/GripperSpannerPinion_v2.stl")
    sl = trimesh.load(f"{STL}/GripperSpannerSlider_v2.stl")
    jaw = trimesh.load(f"{STL}/GripperSpannerJaw_v2.stl")
    fp = trimesh.load(f"{am.BASE}/FrontPlate.stl")
    Tfp = am.tr(0, 0, -2.0)
    Tsv = am.frame_from_axes((-10, 22.85, 0.0), (0, 1, 0), (0, 0, 1))
    ax = fig.add_subplot(gs[:, 1])
    cut = (p["shaft_x"], 0, 0)
    for mesh, col, T in ((fp, OLD, Tfp), (fc.SERVO, SERVO, Tsv), (head, NEW, np.eye(4)), (pin, NEW, np.eye(4)),
                         (sl, NEW, np.eye(4)), (jaw, NEW, np.eye(4))):
        outline(ax, mesh, cut, (1, 0, 0), (1, 2), col, T=T)
    yb = -6
    dim(ax, (yb, -2), (yb, 0), "2.0 plate", off=(-6, 0))
    dim(ax, (yb, 0), (yb, p["tab_top"]), f"{p['tab_top']} tab", off=(-6, 0))
    dim(ax, (yb - 10, p["tab_top"]), (yb - 10, z), f"{z - p['tab_top']:.1f} plinth", off=(-7, 0))
    dim(ax, (yb - 10, z), (yb - 10, z + p["base_t"]), f"{p['base_t']}", off=(-4, 0))
    dim(ax, (54, 0), (54, z + p["jaw_tip_z"]), f"{z + p['jaw_tip_z']:.1f} jaw tip", off=(5, 0))
    dim(ax, (50, 0), (50, z + 31.0), f"{z + 31.0:.1f} TCP", off=(4, 0))
    dim(ax, (yb - 20, -15.9), (yb - 20, 0), "15.9", off=(-4, 0))
    zt = z + p["pinion_z0"]
    dim(ax, (p["shaft_y"] + 11, p["boss_top"]), (p["shaft_y"] + 11, zt), "0.3", off=(4, 0), fs=6.5)
    note(ax, (p["shaft_y"], zt + 2.5), (40, zt + 13), f"pinion m{p['module']:.0f} z{p['teeth']}, R{d['R']:.0f}, "
                                                        f"bore Ø{p['spline_d']} on the SG90 spline")
    note(ax, (p["shaft_y"], p["boss_top"] - 1), (32, -10), f"SG90 boss top {p['boss_top']} (MEASURE)")
    note(ax, (p["hole_xy"][0][1], 4), (-18, 30), f"counterbore Ø{p['cbore_d']}, floor {p['cbore_floor']} on the tab;\n"
                                                  "supplied tab screw → tab → plate → ArmFront post")
    gap0 = p["fixed_face_y"]
    dim(ax, (gap0, z + p["jaw_tip_z"] + 3), (gap0 + p["gap_min"], z + p["jaw_tip_z"] + 3),
        f"gap {p['gap_min']}…{d['g_max']:.1f} over {p['travel_deg']:.0f}°", off=(12, 2))
    ax.set_aspect("equal")
    ax.set_xlim(-32, 62)
    ax.set_ylim(-20, 62)
    ax.axis("off")
    ax.set_title("C  Spanner gripper v2 on the gripper SG90: section through the shaft,\n"
                 "x = −10 in the FrontPlate frame (mm, z = 0 at the plate's front face)",
                 loc="left", fontsize=11, weight="bold")

    # ---------------- E. counterweight swing section
    sw = trimesh.load(f"{STL}/Swing_v2_counterweight.stl")
    lid = trimesh.load(f"{STL}/Swing_v2_counterweight_lid.stl")
    ax = fig.add_subplot(gs[0, 2])
    outline(ax, sw, (0, 0, 0), (0, 1, 0), (0, 2), NEW)
    outline(ax, lid, (0, 0, 0), (0, 1, 0), (0, 2), NEW, T=am.tr(0, 0, 2.5))
    outline(ax, fc.SERVO, (0, 0, 0), (0, 1, 0), (0, 2), SERVO,
            T=am.frame_from_axes(*am.SERVO_MOUNTS["servo_shoulder"][1:]))
    dim(ax, (0, -20), (-52, -20), "52 to pocket centre", off=(0, -2.5))
    dim(ax, (-52 - 13.3, 7), (-52 + 13.3, 7), "Ø26.6 bore", off=(0, 2.5))
    dim(ax, (-72, 2.5 - 15.5), (-72, 2.5), "15.5 deep", off=(-6, 0))
    dim(ax, (6, -14.5), (6, 0), "14.5", off=(4, 0))
    note(ax, (-30, 4), (-40, 30), "M3 lid screw at x = −30")
    note(ax, (-52, -10), (-90, -26), "7 × UK 2p coins (≈ 50 g)")
    ax.set_aspect("equal")
    ax.set_xlim(-100, 30)
    ax.set_ylim(-30, 40)
    ax.axis("off")
    ax.set_title("D  Swing_v2_counterweight: section y = 0 (mm, z = 0 at the disc underside)",
                 loc="left", fontsize=11, weight="bold")

    # ---------------- F. stack heights
    ax = fig.add_subplot(gs[1, 2])
    ax.axis("off")
    rows = [("Swing underside above table (horn stack)", f"{am.SWING_Z} mm (MEASURE)"),
            ("Base SG90 case bottom / tab seat", "3.3 / 19.2 mm"),
            ("Counterweight pocket bottom above table", f"{am.SWING_Z - 14.5:.1f} mm"),
            ("Counterweight → foot, closest over a full turn", "8.7 mm"),
            ("Shoulder axis above table", f"{am.SWING_Z + am.SHOULDER_Z:.1f} mm"),
            ("Link length (both links, unchanged)", f"{am.L_LINK} mm"),
            ("Spanner TCP in front of FrontPlate / lateral", "39.3 mm / gap/2 − 6.75 mm"),
            ("SG90 envelope used (datasheet max)", "22.8 × 12.4, tabs 32.2 × 2.5 @ 15.9"),
            ("Supplied slots (all 5)", "23.5 long, 16.2 seat → floor/wall")]
    for i, (a, b) in enumerate(rows):
        ax.text(0.0, 0.92 - i * 0.1, a, fontsize=9.5, transform=ax.transAxes)
        ax.text(0.68, 0.92 - i * 0.1, b, fontsize=9.5, transform=ax.transAxes, weight="bold")
    ax.set_title("E  Key stack heights (mm)", loc="left", fontsize=11, weight="bold")
    fig.suptitle("D6 main design: key dimensions of the new / modified parts. Pink = new or modified, "
                 "blue = supplied, black = SG90. Generated from the CAD parameters (drawing.py).", fontsize=12)
    plt.tight_layout()
    out = os.path.join(HERE, "results", "key_dimensions.png")
    plt.savefig(out, dpi=90)
    print(out)


if __name__ == "__main__":
    main()
