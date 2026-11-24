"""Cross-sections of the chosen build (D6) showing each SG90 in its slot, the
spanner v2 stack on the gripper servo, and the base servo wire lane.

python3 render_fit.py  -> results/fit_sections.png
AI-assisted (Claude Code).
"""
import os
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import trimesh

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(HERE, "..", "full_designs"))
import arm_model as am  # noqa: E402
import fit_check as fc  # noqa: E402

COL = {"original": "#4C72B0", "new": "#C4468F", "modified": "#C4468F", "servo": "#222222",
       "hardware": "#C9A227"}


def draw(ax, items, origin, normal, axes, title, lim):
    for mesh, T, kind, label in items:
        m = mesh.copy()
        m.apply_transform(T)
        s = m.section(plane_origin=origin, plane_normal=normal)
        if s is None:
            continue
        for e in s.discrete:
            ax.fill(e[:, axes[0]], e[:, axes[1]], color=COL[kind], alpha=0.35 if kind != "servo" else 0.55,
                    lw=0)
            ax.plot(e[:, axes[0]], e[:, axes[1]], color=COL[kind], lw=0.8)
    ax.set_aspect("equal")
    ax.set_xlim(*lim[0])
    ax.set_ylim(*lim[1])
    ax.grid(alpha=.25)
    ax.set_title(title, fontsize=9.5)


def main():
    P = fc.P
    I = np.eye(4)
    Tb, _ = fc.servo_T("servo_base")
    Ts, _ = fc.servo_T("servo_shoulder")
    Tg, _ = fc.servo_T("servo_gripper")
    Tsw = am.tr(0, 0, am.SWING_Z)
    fig, axs = plt.subplots(2, 3, figsize=(19, 11.5))

    # 1 foot + base servo + swing, side section through the yaw axis
    items = [(P["Foot_v2_wide"].mesh, I, "modified", ""), (fc.SERVO, Tb, "servo", ""),
             (P["Swing_v2_counterweight"].mesh, Tsw, "modified", ""),
             (P["Swing_v2_counterweight_lid"].mesh, Tsw, "new", ""),
             (P["coins_7x2p"].mesh, Tsw, "hardware", ""), (fc.SERVO, Tsw @ Ts, "servo", "")]
    draw(axs[0, 0], items, (0, 0, 0), (0, 1, 0), (0, 2),
         "Base: section y = 0 through the yaw axis\nbase SG90 in the unchanged cradle; swing on its "
         f"horn at z = {am.SWING_Z}; counterweight", ((-95, 40), (-2, 60)))
    axs[0, 0].set_xlabel("x (mm)")
    axs[0, 0].set_ylabel("z (mm)")
    axs[0, 0].annotate("wire exits\nshaft end", (7, 5), (18, 14), fontsize=8,
                       arrowprops=dict(arrowstyle="->"))

    # 2 foot cross section (across the cradle) showing the wedges
    draw(axs[0, 1], items[:2], (-5, 0, 0), (1, 0, 0), (1, 2),
         "Base: section x = -5 across the cradle\nsloped buttresses brace both walls; servo lowers in "
         "from above", ((-35, 35), (-2, 40)))
    axs[0, 1].set_xlabel("y (mm)")

    # 3 foot from above: wire lane and clips
    ax = axs[0, 2]
    foot = P["Foot_v2_wide"].mesh
    for z, c, lw in ((0.8, "0.5", .6), (2.3, "#C4468F", 1.0), (4.8, "#C4468F", 1.4)):
        s = foot.section(plane_origin=(0, 0, z), plane_normal=(0, 0, 1))
        for e in s.discrete:
            ax.plot(e[:, 0], e[:, 1], color=c, lw=lw)
    sb = fc.SERVO.copy()
    sb.apply_transform(Tb)
    s = sb.section(plane_origin=(0, 0, 10), plane_normal=(0, 0, 1))
    for e in s.discrete:
        ax.fill(e[:, 0], e[:, 1], color="#222", alpha=.5)
    lane = np.array([(4, 0), (14, 0), (14, 93)])
    ax.plot(lane[:, 0], lane[:, 1], color="#E39B1B", lw=3, label="base SG90 wire")
    ax.legend(loc="lower left", fontsize=8)
    ax.set_aspect("equal")
    ax.set_xlim(-60, 95)
    ax.set_ylim(-60, 95)
    ax.grid(alpha=.25)
    ax.set_title("Base from above: wire leaves the open +x end of the cradle,\nturns and runs under 3 "
                 "clips to the plate edge", fontsize=9.5)

    # 4 swing + shoulder servo section across the bay
    items = [(P["Swing_v2_counterweight"].mesh, I, "modified", ""), (fc.SERVO, Ts, "servo", ""),
             (P["ArmSpring"].mesh, am.link_transforms((0, 90, -70, -80))["link1"]
              @ np.linalg.inv(am.link_transforms((0, 90, -70, -80))["swing"]), "original", "")]
    draw(axs[1, 0], items, (-5, 0, 0), (1, 0, 0), (1, 2),
         "Swing: section x = -5 across the shoulder bay\n(swing frame) SG90 on its posts, back wall "
         "behind", ((-25, 30), (-16, 30)))
    axs[1, 0].set_xlabel("y (mm)")

    # 5 gripper stack: section through the shaft axis (tool frame)
    yax = trimesh.transform_points([(-10, 22.85, 0)], am.FPS_TO_AF)[0]
    items = [(P["ArmFront"].mesh, I, "original", ""), (P["FrontPlate"].mesh, I, "original", ""),
             (fc.SERVO, Tg, "servo", ""), (P["GripperSpannerHead_v2"].mesh, I, "new", ""),
             (P["GripperSpannerPinion_v2"].mesh, I, "new", ""),
             (P["GripperSpannerSlider_v2"].mesh, I, "new", ""), (P["GripperSpannerJaw_v2"].mesh, I, "new", "")]
    draw(axs[1, 1], items, yax, (0, 1, 0), (0, 2),
         "Gripper: section through the SG90 shaft (tool frame)\nArmFront post | FrontPlate | tabs | "
         "spanner v2 plinth; pinion on the spline", ((20, 100), (-14, 26)))
    axs[1, 1].set_xlabel("x along the arm (mm)")
    axs[1, 1].set_ylabel("z (mm)")

    # 6 gripper stack across: section at the screw line
    draw(axs[1, 2], items, (0, 0, yax[2]), (0, 0, 1), (0, 1),
         "Gripper: section z = shaft height\nhead screws pass head -> tab -> plate -> post",
         ((20, 100), (-25, 40)))
    axs[1, 2].set_xlabel("x along the arm (mm)")
    for k, c in (("supplied part", "original"), ("new / modified part", "new"), ("SG90 (datasheet max)", "servo"),
                 ("coins", "hardware")):
        axs[1, 2].fill([], [], color=COL[c], alpha=.5, label=k)
    axs[1, 2].legend(fontsize=8, loc="lower right")
    fig.suptitle("D6 fit check: every SG90 in its slot (sections). Pink = new or modified, blue = supplied, "
                 "black = SG90", fontsize=12)
    plt.tight_layout()
    os.makedirs(os.path.join(HERE, "results"), exist_ok=True)
    out = os.path.join(HERE, "results", "fit_sections.png")
    plt.savefig(out, dpi=85)
    print(out)


if __name__ == "__main__":
    main()
