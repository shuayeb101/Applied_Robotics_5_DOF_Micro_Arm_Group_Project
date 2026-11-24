"""
Open/close mechanism simulation for the mini geared claw.

Moves the real CAD outlines (exported from mini_claw.scad) through the
whole stroke, in small steps, and checks the mechanics at every step:

  * interference and minimum clearance between every pair of parts that
    share a height band (gear layer 1, gear layer 2, lowered fingers),
  * gear backlash in both meshes and the resulting play at the jaw tips,
  * whether the gear sectors stay in mesh over the whole stroke,
  * where the mechanism hits its end-stops when driven past its range,
  * jaw gap against servo angle, and stroke time at SG90 speed.

It is a kinematic simulation: parts move exactly as the gears dictate
(pinion 1.5x the jaw angle, jaws mirrored) and are tested for overlap.
It does not model forces, friction or part flex.

Usage:  python3 -I sim/mechanism_sim.py
Needs:  openscad on PATH, pip packages: shapely>=2.0, ezdxf, matplotlib, pillow
AI-assisted (Claude). Check and understand before using as evidence (C12).
"""
import math, os, subprocess, tempfile
from functools import reduce

import ezdxf
import numpy as np
from shapely import affinity
from shapely.geometry import LineString, Point
from shapely.ops import polygonize, unary_union

HERE = os.path.dirname(os.path.abspath(__file__))
SCAD = os.path.join(HERE, "..", "mini_claw.scad")
OUT = os.path.join(HERE, "results")

# ---- must match mini_claw.scad --------------------------------------------
M, Z_P, Z_J = 0.9, 10, 15
A = M * Z_J / 2
H = math.sqrt((M * (Z_P + Z_J) / 2) ** 2 - A ** 2)
RATIO = Z_J / Z_P
SLEEVE_R, PILLAR_R, HUB_CLEAR = 2.0, 3.0, 0.25
THETA_MAX = 30.0
TIP_LOCAL = (0.2 - A, 30.0)                  # tip_gap/2 - a, jaw_len
R_TIP = math.hypot(*TIP_LOCAL)
SCREW_HEAD_R = 2.0      # ASSUMED: SG90 horn screw head ~4 mm across, sits on the pinion (layer 2 height)
SG90_S_PER_60 = 0.10    # SG90 datasheet no-load speed at 4.8 V, s per 60 deg

PIV_R, PIV_L = (A, H), (-A, H)


# ---- geometry from the CAD -----------------------------------------------------
def load_2d(part):
    with tempfile.TemporaryDirectory() as td:
        f = os.path.join(td, "p.dxf")
        subprocess.run(["openscad", "-D", f'part="{part}"', "-o", f, SCAD], check=True, capture_output=True)
        lines = [LineString([(e.dxf.start.x, e.dxf.start.y), (e.dxf.end.x, e.dxf.end.y)])
                 for e in ezdxf.readfile(f).modelspace() if e.dxftype() == "LINE"]
    faces = list(polygonize(lines))
    return reduce(lambda a, b: a.symmetric_difference(b), faces).buffer(0)   # even-odd: keeps holes


def pose_R(g, th, extra=0.0):
    return affinity.translate(affinity.rotate(g, -th + extra, origin=(0, 0)), *PIV_R)


def pose_L(g, th, extra=0.0):
    return affinity.translate(affinity.rotate(g, th + extra, origin=(0, 0)), *PIV_L)


def pose_P(g, th):
    return affinity.rotate(g, RATIO * th, origin=(0, 0))


class Claw:
    def __init__(self):
        print("exporting CAD outlines ...", flush=True)
        self.R1, self.R2 = load_2d("jawR_l1_2d"), load_2d("jawR_l2_2d")
        self.L1, self.L2 = load_2d("jawL_l1_2d"), load_2d("jawL_l2_2d")
        self.P = load_2d("pinion_2d")
        self.dropR = load_2d("drop_2d")
        self.dropL = affinity.scale(self.dropR, -1, 1, origin=(0, 0))
        self.servo = load_2d("servo_2d")
        from shapely.geometry import box
        self.fingerR = load_2d("finger_2d").intersection(box(-50, 9.0, 50, 60))   # gripping part only
        self.fingerL = affinity.scale(self.fingerR, -1, 1, origin=(0, 0))
        self.sleeveR, self.sleeveL = Point(PIV_R).buffer(SLEEVE_R, 64), Point(PIV_L).buffer(SLEEVE_R, 64)
        self.pillarR, self.pillarL = Point(PIV_R).buffer(PILLAR_R, 64), Point(PIV_L).buffer(PILLAR_R, 64)
        self.screw = Point(0, 0).buffer(SCREW_HEAD_R, 64)

    def posed(self, th):
        return dict(
            P=pose_P(self.P, th),
            R1=pose_R(self.R1, th), R2=pose_R(self.R2, th),
            L1=pose_L(self.L1, th), L2=pose_L(self.L2, th),
            dR=pose_R(self.dropR, th), dL=pose_L(self.dropL, th),
        )

    def pairs(self, th):
        """(band, name, geom a, geom b, is_gear_mesh) for every pair that shares a height band."""
        g = self.posed(th)
        return [
            ("layer 1 (z 11-14)", "pinion / right jaw gear", g["P"], g["R1"], True),
            ("layer 1 (z 11-14)", "pinion / left jaw", g["P"], g["L1"], False),
            ("layer 1 (z 11-14)", "right jaw / left jaw", g["R1"], g["L1"], False),
            ("layer 1 (z 11-14)", "right jaw / left sleeve", g["R1"], self.sleeveL, False),
            ("layer 1 (z 11-14)", "left jaw / right sleeve", g["L1"], self.sleeveR, False),
            ("layer 2 (z 14-17)", "right jaw gear / left jaw gear", g["R2"], g["L2"], True),
            ("layer 2 (z 14-17)", "right jaw / left sleeve", g["R2"], self.sleeveL, False),
            ("layer 2 (z 14-17)", "left jaw / right sleeve", g["L2"], self.sleeveR, False),
            ("layer 2 (z 14-17)", "right jaw / pinion screw head", g["R2"], self.screw, False),
            ("layer 2 (z 14-17)", "left jaw / pinion screw head", g["L2"], self.screw, False),
            ("fingers (z 8-11)", "right finger / left finger", g["dR"], g["dL"], False),
            ("fingers (z 8-11)", "right finger / servo case", g["dR"], self.servo, False),
            ("fingers (z 8-11)", "left finger / servo case", g["dL"], self.servo, False),
            ("fingers (z 8-11)", "right finger / left pillar", g["dR"], self.pillarL, False),
            ("fingers (z 8-11)", "left finger / right pillar", g["dL"], self.pillarR, False),
        ]

    # ---- backlash: free rotation of the driven gear with its driver held still ----
    def _free(self, hit, sign, lim=25.0):
        if hit(0.0):
            return 0.0
        lo, hi, step = 0.0, None, 0.25
        d = step
        while d <= lim:
            if hit(sign * d):
                hi = d
                break
            lo = d
            d += step
        if hi is None:
            return math.inf                 # never touches: out of mesh
        for _ in range(30):
            mid = (lo + hi) / 2
            lo, hi = (mid, hi) if not hit(sign * mid) else (lo, mid)
        return lo

    def backlash(self, th):
        P = pose_P(self.P, th)
        hit_pr = lambda d: pose_R(self.R1, th, d).intersects(P)          # right jaw vs held pinion
        R2 = pose_R(self.R2, th)
        hit_rl = lambda d: pose_L(self.L2, th, d).intersects(R2)         # left jaw vs held right jaw
        pr = (self._free(hit_pr, +1), self._free(hit_pr, -1))
        rl = (self._free(hit_rl, +1), self._free(hit_rl, -1))
        return pr, rl

    def tip_gap(self, th):
        return 2 * (A + TIP_LOCAL[0] * math.cos(math.radians(th)) + TIP_LOCAL[1] * math.sin(math.radians(th)))

    def min_gap(self, th):
        """Smallest gap between the two gripping fingers (the gear hubs excluded)."""
        return pose_R(self.fingerR, th).distance(pose_L(self.fingerL, th))


# ---- run -----------------------------------------------------------------------
def main():
    os.makedirs(OUT, exist_ok=True)
    c = Claw()
    step = 0.25
    sweep = np.round(np.arange(-3.0, 45.0 + step / 2, step), 3)

    print("sweeping clearances ...", flush=True)
    rows = []
    for th in sweep:
        for band, name, a, b, mesh in c.pairs(th):
            ov = a.intersection(b).area
            rows.append((th, band, name, mesh, ov, 0.0 if ov > 1e-6 else a.distance(b)))

    in_range = [r for r in rows if 0 <= r[0] <= THETA_MAX]
    names = list(dict.fromkeys((r[1], r[2], r[3]) for r in rows))
    pair_tab = []
    for band, name, mesh in names:
        rr = [r for r in in_range if r[2] == name and r[1] == band]
        worst = min(rr, key=lambda r: r[5])
        ov = max(r[4] for r in rr)
        pair_tab.append((band, name, mesh, ov, worst[5], worst[0]))

    # end stops: first collision outside the design range, by pair
    def first_hit(angles):
        for th in angles:
            for r in (x for x in rows if x[0] == th):
                if r[4] > 1e-6:
                    return th, r[2]
        return None, None
    close_stop = first_hit(sorted({r[0] for r in rows if r[0] < 0}, reverse=True))
    open_stop = first_hit(sorted({r[0] for r in rows if r[0] > THETA_MAX}))

    print("measuring backlash and engagement ...", flush=True)
    bl = []
    for th in np.arange(-2.0, 44.0 + 0.1, 1.0):
        (pr_p, pr_m), (rl_p, rl_m) = c.backlash(th)
        bl.append((th, pr_p, pr_m, rl_p, rl_m))
    eng_lost = [b[0] for b in bl if math.isinf(b[1]) or math.isinf(b[2]) or math.isinf(b[3]) or math.isinf(b[4])]
    bl_in = [b for b in bl if 0 <= b[0] <= THETA_MAX]
    pr_tot = [b[1] + b[2] for b in bl_in]
    rl_tot = [b[3] + b[4] for b in bl_in]
    tipR = [p * math.pi / 180 * R_TIP for p in pr_tot]
    tipL = [(p + q) * math.pi / 180 * R_TIP for p, q in zip(pr_tot, rl_tot)]
    centring = [abs(b[1] - b[2]) / 2 for b in bl_in] + [abs(b[3] - b[4]) / 2 for b in bl_in]

    # kinematics
    kin = [(th, RATIO * th, c.tip_gap(th), c.min_gap(th)) for th in np.arange(0, THETA_MAX + 0.1, 5)]
    stroke_s = RATIO * THETA_MAX / 60 * SG90_S_PER_60
    tip_speed = (60 / SG90_S_PER_60) / RATIO * math.pi / 180 * R_TIP   # servo deg/s -> jaw deg/s -> tip mm/s

    # ---- report -------------------------------------------------------------
    L = ["# Mechanism simulation: opening and closing", "",
         f"Kinematic sweep of the CAD outlines from {sweep[0]:g}° to {sweep[-1]:g}° jaw angle in {step}° steps "
         f"({len(sweep)} poses, {len(rows)} pair checks). Design range 0–{THETA_MAX:g}° (servo 0–{RATIO*THETA_MAX:g}°). "
         "Generated by `sim/mechanism_sim.py`.", "",
         "## 1. Interference and clearance over the design range", "",
         "| Height band | Pair | Type | Overlap | Smallest clearance | At jaw angle |",
         "| --- | --- | --- | --- | --- | --- |"]
    for band, name, mesh, ov, mn, at in pair_tab:
        L.append(f"| {band} | {name} | {'gear mesh' if mesh else 'clearance'} | "
                 f"{'**YES ' + format(ov, '.3f') + ' mm²**' if ov > 1e-6 else 'none'} | {mn:.2f} mm | {at:g}° |")
    any_ov = any(p[3] > 1e-6 for p in pair_tab)
    non_mesh = [p for p in pair_tab if not p[2]]
    tight = min(non_mesh, key=lambda p: p[4])
    L += ["", f"**Result:** {'INTERFERENCE FOUND' if any_ov else 'no part overlaps anywhere in the design range'}. "
          f"Tightest non-gear clearance: {tight[1]}, {tight[4]:.2f} mm at {tight[5]:g}°.", "",
          "## 2. End-stops (driving past the design range)", "",
          f"- **Closing past 0°:** first contact at {close_stop[0]:g}° ({close_stop[1]})." if close_stop[0] is not None
          else "- **Closing past 0°:** no contact down to -3°.",
          f"- **Opening past {THETA_MAX:g}°:** first contact at {open_stop[0]:g}° ({open_stop[1]})." if open_stop[0] is not None
          else f"- **Opening past {THETA_MAX:g}°:** no contact up to {sweep[-1]:g}°.",
          (f"- **Gear sectors leave mesh** at jaw angle {min(eng_lost):g}° and beyond." if eng_lost
           else f"- **Gears stay in mesh** over the whole sweep (-2° to 44°)."), "",
          "## 3. Backlash (free play)", "",
          "Measured by holding the driving gear still and turning the driven gear each way until its teeth touch.", "",
          "| | Min | Max |", "| --- | --- | --- |",
          f"| Pinion → right jaw, jaw degrees | {min(pr_tot):.2f}° | {max(pr_tot):.2f}° |",
          f"| Right jaw → left jaw, jaw degrees | {min(rl_tot):.2f}° | {max(rl_tot):.2f}° |",
          f"| Play at the right tip | {min(tipR):.2f} mm | {max(tipR):.2f} mm |",
          f"| Play at the left tip (both meshes) | {min(tipL):.2f} mm | {max(tipL):.2f} mm |",
          f"| Play in the tip gap (both tips) | {min(a+b for a,b in zip(tipR,tipL)):.2f} mm | {max(a+b for a,b in zip(tipR,tipL)):.2f} mm |",
          f"| Off-centre mesh (phase error) | | {max(centring):.3f}° |", "",
          f"Plus up to {HUB_CLEAR} mm radial play of each jaw on its sleeve (pivot hole clearance).", "",
          "## 4. Kinematics", "",
          "| Jaw angle | Servo angle | Tip gap | Smallest gap between the fingers |", "| --- | --- | --- | --- |"]
    for th, sv, tg, mg in kin:
        L.append(f"| {th:g}° | {sv:g}° | {tg:.1f} mm | {mg:.1f} mm |")
    L += ["", f"At SG90 no-load speed ({SG90_S_PER_60} s per 60°) a full stroke takes about **{stroke_s*1000:.0f} ms**, "
          f"and the tips move at about **{tip_speed:.0f} mm/s**. That's fast enough to pinch a finger, so close slowly with SPEED in the firmware.", "",
          "## Limits", "",
          "- Kinematic only: no forces, friction, bending or wear. Teeth are perfect CAD shapes; printed teeth will be slightly different.",
          "- Parts are checked in three height bands, using the heights in `mini_claw.scad` (which include estimated servo dimensions, see MEASURE).",
          f"- The pinion screw head is assumed to be {2*SCREW_HEAD_R:g} mm across. Measure the real screw.",
          "- The servo case is a box keep-out; the real case is slightly smaller."]
    with open(os.path.join(OUT, "mechanism_report.md"), "w") as f:
        f.write("\n".join(L) + "\n")
    print("\n".join(L))

    plots(c, rows, bl, names)
    animate(c, bl)


def plots(c, rows, bl, names):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig, ax = plt.subplots(1, 3, figsize=(18, 5.2))
    for band, name, mesh in names:
        if mesh:
            continue
        rr = [r for r in rows if r[2] == name and r[1] == band]
        ax[0].plot([r[0] for r in rr], [min(r[5], 8) for r in rr], label=f"{name} ({band.split(' (')[0]})", lw=1.4)
    ax[0].axvspan(0, THETA_MAX, color="0.93", zorder=0)
    ax[0].set_xlabel("jaw angle (deg)"); ax[0].set_ylabel("clearance (mm, capped at 8)")
    ax[0].set_title("Clearance between non-meshing parts"); ax[0].legend(fontsize=7); ax[0].set_ylim(-0.2, 8.3)
    th = [b[0] for b in bl]
    clip = lambda v: v if not math.isinf(v) else np.nan
    ax[1].plot(th, [clip(b[1] + b[2]) for b in bl], label="pinion → right jaw")
    ax[1].plot(th, [clip(b[3] + b[4]) for b in bl], label="right jaw → left jaw")
    ax[1].axvspan(0, THETA_MAX, color="0.93", zorder=0)
    ax[1].set_xlabel("jaw angle (deg)"); ax[1].set_ylabel("backlash (jaw deg)")
    ax[1].set_title("Gear backlash (gaps in the line = out of mesh)"); ax[1].legend(fontsize=8)
    t = np.arange(0, THETA_MAX + 0.01, 0.5)
    ax[2].plot(RATIO * t, [c.tip_gap(x) for x in t], label="tip gap")
    ax[2].plot(RATIO * t, [c.min_gap(x) for x in t], label="smallest gap between the fingers")
    for w, lab in ((15, "cube A 15 mm"), (12, "cylinder B 12 mm")):
        ax[2].axhline(w, ls="--", color="0.6", lw=1); ax[2].text(1, w + 0.4, lab, fontsize=8, color="0.4")
    ax[2].set_xlabel("servo angle from closed (deg)"); ax[2].set_ylabel("mm")
    ax[2].set_title("Jaw opening against servo angle"); ax[2].legend(fontsize=8)
    plt.tight_layout(); plt.savefig(os.path.join(OUT, "mechanism_plots.png"), dpi=90); plt.close(fig)


def animate(c, bl):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.animation import FuncAnimation, PillowWriter
    angles = list(np.arange(0, THETA_MAX + 0.01, 1.0)) + list(np.arange(THETA_MAX - 1, -0.01, -1.0))
    blmap = {round(b[0]): b for b in bl}
    fig, axs = plt.subplots(1, 2, figsize=(11, 6), gridspec_kw={"width_ratios": [1.25, 1]})

    def fill(ax, g, **kw):
        for p in getattr(g, "geoms", [g]):
            if p.is_empty:
                continue
            x, y = p.exterior.xy
            ax.fill(x, y, **kw)
            for hole in p.interiors:
                hx, hy = hole.xy
                ax.fill(hx, hy, color="white", zorder=kw.get("zorder", 1) + 0.1)

    def outline(ax, g, color, lw=1.3, z=3):
        for p in getattr(g, "geoms", [g]):
            if p.is_empty:
                continue
            for ring in [p.exterior, *p.interiors]:
                x, y = ring.xy
                ax.plot(x, y, color=color, lw=lw, zorder=z)

    def draw(i):
        th = angles[i]
        g = c.posed(th)
        d1 = g["P"].distance(g["R1"])
        d2 = g["R2"].distance(g["L2"])
        for k, ax in enumerate(axs):
            ax.clear()
            fill(ax, c.servo, color="#d6e2f2", zorder=0)
            if k == 0:
                fill(ax, g["dR"], color="0.6", zorder=1); fill(ax, g["dL"], color="0.6", zorder=1)
                fill(ax, g["R2"], color="#3d4f66", zorder=2); fill(ax, g["L2"], color="#3d4f66", zorder=2)
                fill(ax, g["P"], color="#e0b000", zorder=1.5)
                ax.set_xlim(-22, 22); ax.set_ylim(-8, 42)
                ax.set_title(f"jaw {th:4.1f}°  |  servo {RATIO*th:4.1f}°  |  tip gap {c.tip_gap(th):4.1f} mm", fontsize=10)
            else:
                fill(ax, g["P"], color="#e0b000", alpha=0.55, zorder=1)
                outline(ax, g["P"], "#9a7a00", 1.2, 2)
                outline(ax, g["R1"], "#d2691e", 1.6, 3); outline(ax, g["L1"], "#d2691e", 1.0, 3)
                outline(ax, g["R2"], "#1f5fbf", 1.6, 4); outline(ax, g["L2"], "#1f5fbf", 1.6, 4)
                ax.set_xlim(-11, 11); ax.set_ylim(-7, 16)
                ax.set_title("gear mesh close-up (outlines)\n"
                             f"orange = layer 1, pinion → right jaw: gap {d1:.2f} mm\n"
                             f"blue = layer 2, right jaw → left jaw: gap {d2:.2f} mm", fontsize=9)
            for sl in (c.sleeveR, c.sleeveL):
                fill(ax, sl, color="0.25", zorder=6)
            ax.set_aspect("equal"); ax.axis("off")
        fig.suptitle("Mini claw open/close cycle (kinematic, CAD outlines)", fontsize=11)

    anim = FuncAnimation(fig, draw, frames=len(angles))
    anim.save(os.path.join(OUT, "mechanism_cycle.gif"), writer=PillowWriter(fps=12))
    plt.close(fig)


if __name__ == "__main__":
    main()
