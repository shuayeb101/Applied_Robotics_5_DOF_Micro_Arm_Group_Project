"""Detailed simulation of the group's chosen build (D6):
spanner gripper + swing counterweight + wide base (Foot_v2_wide).

Uses the same model and checks as ../full_designs (collisions, table/plate
contact, tipping, servo torque), and adds tests that only matter now that a
single build is chosen:

  MAP   pick-and-place map: every (radius, yaw) object position on the work
        area, for a 15 mm cube and a 12 x 20 mm cylinder. Can the arm reach
        30 mm above it, grasp it, lift it and return HOME with every check
        passing?
  TASK  the standard pick-and-place tasks (pick at one target, place at
        another), every leg sampled 24 times
  DYN   dynamic tipping: how fast can the base and shoulder move (min-jerk
        profile) before inertial loads use up the tip margin
  LIM   joint limits from HOME (first collision / plate contact per joint)
  BACK  leaning back: stability with the shoulder past vertical

Run:   python3 simulate_build.py     (~3 min)  -> results/
AI-assisted (Claude Code). Simulation is not test evidence.
"""
from __future__ import annotations

import json
import math
import os
import sys

import numpy as np
import trimesh

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "full_designs"))
import arm_model as am  # noqa: E402
import simulate_designs as sd  # noqa: E402

OUT = os.path.join(HERE, "results")
KEY = "D6"
CUBE = dict(name="15 mm cube", half=7.5, h=15.0)
CYL = dict(name="12 x 20 mm cylinder", half=6.0, h=20.0)
SAMPLES = 24


def design():
    d = [x for x in am.DESIGNS if x.key == KEY][0]
    return d.build()


# ---------------------------------------------------------------- helpers
def check_q(d, w, st, q, payload):
    hits, zmin = w.check(q)
    if hits:
        return "collision " + hits[0].replace("~", "/")
    if zmin < sd.TABLE_CLEAR - 0.5:
        return f"hits table (z={zmin:.1f})"
    if st.stability(q, payload)[0] < 0:
        return "tips over"
    if max(st.torques(q, payload).values()) > sd.TRQ_FAIL * am.SG90_STALL:
        return "servo torque > 80 % stall"
    return None


def grasp_poses(d, w, st, R, yaw, obj, payload):
    """Lowest grasp on the object and the pose 30 mm above it, or a reason."""
    floor = d.floor_at(R, obj["half"])
    if floor is None:
        return None, "straddles plate edge"
    for z in floor + np.arange(obj["h"] / 2, obj["h"] + 0.01, 1.0):
        g = sd.solve(d, w, R, yaw, z, payload, st)
        if g:
            a = sd.solve(d, w, R, yaw, z + 30, payload, st)
            if a:
                return (g, a, float(z), floor), None
            return None, "cannot reach 30 mm above"
    return None, "no collision-free grasp"


def run_legs(d, w, st, legs):
    """legs = [(qa, qb, payload)]. Returns (fail reason or None, stats)."""
    margins, loads, sides = [], [], []
    for a, b, pl in legs:
        a, b = np.array(a), np.array(b)
        for t in np.linspace(0, 1, SAMPLES):
            q = tuple(a + t * (b - a))
            why = check_q(d, w, st, q, pl)
            if why:
                return why, None
            margins.append(st.stability(q, pl)[0])
            loads.append(max(st.torques(q, pl).values()) / am.SG90_STALL)
            sides.append(st.side_moment(q, pl))
    return None, dict(min_margin_mm=round(min(margins), 1), peak_load_pct=round(100 * max(loads)),
                      peak_side_gmm=round(max(sides)))


# ---------------------------------------------------------------- tests
def test_map(d, w, st, home, obj):
    radii = list(range(50, 131, 10))
    yaws = list(range(-90, 91, 15))
    grid = {}
    for R in radii:
        for yaw in yaws:
            p, why = grasp_poses(d, w, st, R, yaw, obj, 0.0)
            if p is None:
                grid[(R, yaw)] = dict(ok=False, why=why)
                continue
            (g, _), (a, _), z, floor = p
            why, stats = run_legs(d, w, st, [(home, a, 0), (a, g, 0), (g, a, am.PAYLOAD),
                                             (a, home, am.PAYLOAD)])
            grid[(R, yaw)] = dict(ok=why is None, why=why or "", grasp_z=z, surface=floor,
                                  tilt=abs(g[1] - sd.approach_angles(d)[0]), **(stats or {}))
    return radii, yaws, grid


def test_tasks(d, w, st, home):
    tasks = [("cube", CUBE, (60, -45), (60, 45)), ("cube", CUBE, (80, -45), (80, 45)),
             ("cube", CUBE, (100, -45), (100, 45)), ("cube", CUBE, (80, -90), (80, 90)),
             ("cube", CUBE, (60, 0), (100, 0)), ("cube", CUBE, (100, -60), (60, 60)),
             ("cylinder", CYL, (60, -45), (60, 45)), ("cylinder", CYL, (80, -45), (80, 45)),
             ("cylinder", CYL, (80, 30), (80, -60)), ("cylinder", CYL, (100, -45), (100, 45))]
    out = []
    for kind, obj, pick, place in tasks:
        rec = dict(object=obj["name"], pick=pick, place=place)
        pg, why = grasp_poses(d, w, st, *pick, obj, 0.0)
        pp, why2 = grasp_poses(d, w, st, *place, obj, am.PAYLOAD)
        if not (pg and pp):
            rec.update(result="FAIL", reason=why or why2)
            out.append(rec)
            continue
        (G, _), (AG, _), _, _ = pg
        (P, _), (AP, _), _, _ = pp
        legs = [(home, AG, 0), (AG, G, 0), (G, AG, am.PAYLOAD), (AG, AP, am.PAYLOAD),
                (AP, P, am.PAYLOAD), (P, AP, 0), (AP, home, 0)]
        why, stats = run_legs(d, w, st, legs)
        rec.update(result="PASS" if not why else "FAIL", reason=why or "", **(stats or {}))
        if not why and (stats["min_margin_mm"] < 5 or stats["peak_load_pct"] > 50):
            rec["result"] = "MARGINAL"
        out.append(rec)
    return out


def dyn_margin(d, st, q, joint, theta_deg, T, payload=am.PAYLOAD):
    """Worst tip margin during a min-jerk move of `joint` through theta in T s,
    quasi-static: each moving mass adds an inertial force m*a at its CoM,
    which shifts the effective centre of pressure by sum(m*a*h)/(M*g)."""
    g = 9810.0                                     # mm/s^2
    th = math.radians(theta_deg)
    alpha = 5.7735 * th / T ** 2                   # peak angular accel (min-jerk)
    omega = 1.875 * th / T                         # peak angular velocity
    pts, ms, links, Tm = st.world_masses(q, payload)
    M = ms.sum()
    com = (ms[:, None] * pts).sum(0) / M
    worst = 1e9
    if joint == "base":
        moving = np.array([l != "foot" for l in links])
        for yaw in np.arange(0, 360, 10.0):
            c, s = math.cos(math.radians(yaw)), math.sin(math.radians(yaw))
            Rz = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])
            P = pts.copy()
            P[moving] = P[moving] @ Rz.T
            cxy = (ms[:, None] * P[:, :2]).sum(0) / M
            # peak tangential accel (braking) and centripetal at peak speed act on
            # the moving masses; take the worse of the two, in the worst direction
            for acc in ("tan", "cen"):
                F = np.zeros(2)
                mom = np.zeros(2)
                for i in np.nonzero(moving)[0]:
                    r = P[i, :2]
                    rn = np.linalg.norm(r)
                    if rn < 1e-6:
                        continue
                    if acc == "tan":
                        a = alpha * rn * np.array([-r[1], r[0]]) / rn
                    else:
                        a = omega ** 2 * r          # inertial (outward) for centripetal
                    mom += ms[i] * a * P[i, 2]
                shift = mom / (M * g)
                for sgn in (1, -1):
                    worst = min(worst, st.margin_xy(cxy + sgn * shift if acc == "tan" else cxy + shift))
    else:                                          # shoulder: masses on link1 and beyond
        axis_pt = trimesh.transform_points([[0, 0, am.SHOULDER_Z]], Tm["swing"])[0]
        moving = np.array([l in ("link1", "link2", "tool", "payload") for l in links])
        for yaw in np.arange(0, 360, 10.0):
            c, s = math.cos(math.radians(yaw)), math.sin(math.radians(yaw))
            Rz = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])
            P = pts @ Rz.T
            ax = Rz @ axis_pt
            cxy = (ms[:, None] * P[:, :2]).sum(0) / M
            for sgn in (1, -1):
                mom = np.zeros(2)
                for i in np.nonzero(moving)[0]:
                    r = P[i] - ax
                    # rotation about the horizontal axis perpendicular to the arm plane
                    nvec = Rz @ np.array([0, 1.0, 0])
                    a_t = sgn * alpha * np.cross(nvec, r)
                    a_c = -omega ** 2 * r
                    a = a_t + a_c
                    # horizontal inertial force -m*a at height z
                    mom += -ms[i] * a[:2] * P[i, 2]
                    # vertical accel changes effective weight: ignore (second order)
                worst = min(worst, st.margin_xy(cxy + mom / (M * g)))
    return worst


def test_dyn(d, st, home):
    poses = {"arm straight out": (0, 0, 0, 0), "HOME": tuple(home)}
    out = []
    for name, q in poses.items():
        for joint, theta in (("base", 90), ("base", 180), ("shoulder", 90)):
            row = dict(pose=name, joint=joint, move_deg=theta, static_mm=round(st.stability(q)[1], 1))
            for T in (2.0, 1.0, 0.6, 0.4, 0.3, 0.2):
                row[f"T={T}s"] = round(dyn_margin(d, st, q, joint, theta, T), 1)
            # fastest move time with >= 5 mm margin
            fast = None
            for T in np.arange(2.0, 0.09, -0.02):
                if dyn_margin(d, st, q, joint, theta, T) < 5:
                    break
                fast = T
            row["fastest_T_s_margin5"] = round(float(fast), 2) if fast else None
            out.append(row)
    return out


def test_limits(d, w, st, home):
    names = ("base", "shoulder", "elbow", "wrist")
    win = {"base": (-180, 180), **sd.WINDOWS}
    out = {}
    for j, jn in enumerate(names):
        for end in (0, 1):
            q = list(home)
            target = win[jn][end]
            sgn = 1 if target > q[j] else -1
            v, why = q[j], "end of servo window"
            while (target - v) * sgn > 0:
                nv = v + sgn * 1.0
                q[j] = nv
                r = check_q(d, w, st, tuple(q), 0.0)
                if r:
                    why = r
                    break
                v = nv
            out[f"{jn}_{'min' if end == 0 else 'max'}"] = dict(limit_deg=v, stop=why)
    return out


def test_back(st):
    rows = []
    for sh in range(90, 181, 10):
        worst = min(st.stability((0, sh, el, 0))[1] for el in range(-180, 1, 10))
        rows.append(dict(shoulder=sh, worst_margin_mm=round(worst, 1)))
    return rows


# ---------------------------------------------------------------- plots
def plot_map(radii, yaws, grids, path):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig, axs = plt.subplots(1, len(grids), figsize=(7 * len(grids), 6.5), subplot_kw=dict(polar=False))
    for ax, (title, grid) in zip(np.atleast_1d(axs), grids.items()):
        circ = plt.Circle((0, 0), am.WIDE_FOOT_R, fc="#F4E1EA", ec="#C4468F", lw=1.2, zorder=0)
        ax.add_patch(circ)
        for (R, yaw), g in grid.items():
            a = math.radians(yaw)
            x, y = R * math.cos(a), R * math.sin(a)
            if g["ok"]:
                col = "#2E9E4F" if g["min_margin_mm"] >= 5 and g["peak_load_pct"] <= 50 else "#E39B1B"
                ax.plot(x, y, "o", color=col, ms=10, mec="k", mew=.5)
                ax.annotate(f"{g['peak_load_pct']}", (x, y), ha="center", va="center", fontsize=6,
                            color="white")
            else:
                ax.plot(x, y, "x", color="#D62728", ms=8, mew=2)
        ax.plot(0, 0, "k+", ms=12)
        ax.annotate("", (40, 0), (0, 0), arrowprops=dict(arrowstyle="->"))
        ax.set_aspect("equal"); ax.set_xlim(-60, 140); ax.set_ylim(-140, 140); ax.grid(alpha=.3)
        ax.set_title(f"{title}: grasp + lift + return HOME\n"
                     "green = pass (number = peak servo load %), red x = fail", fontsize=10)
        ax.set_xlabel("x (mm), + = forward at zero yaw"); ax.set_ylabel("y (mm)")
    plt.tight_layout(); plt.savefig(path, dpi=90); plt.close(fig)


def plot_dyn(dyn, back, path):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig, axs = plt.subplots(1, 2, figsize=(14, 5))
    Ts = [2.0, 1.0, 0.6, 0.4, 0.3, 0.2]
    for r in dyn:
        axs[0].plot(Ts, [r[f"T={T}s"] for T in Ts], "-o", ms=3,
                    label=f"{r['joint']} {r['move_deg']}°, {r['pose']}")
    axs[0].axhline(5, color="k", ls="--", lw=1); axs[0].axhline(0, color="#D62728", lw=1)
    axs[0].invert_xaxis(); axs[0].legend(fontsize=7)
    axs[0].set(title="Dynamic tip margin (min-jerk move, 5 g payload, worst yaw)",
               xlabel="move duration T (s): faster to the right", ylabel="tip margin (mm)")
    axs[1].plot([b["shoulder"] for b in back], [b["worst_margin_mm"] for b in back], "-o", color="#C4468F")
    axs[1].axhline(0, color="#D62728", lw=1)
    axs[1].set(title="Leaning back: worst tip margin over every elbow angle and yaw",
               xlabel="shoulder angle (°), 90 = upright, 180 = pointing back", ylabel="tip margin (mm)")
    plt.tight_layout(); plt.savefig(path, dpi=90); plt.close(fig)


# ---------------------------------------------------------------- main
def main():
    os.makedirs(OUT, exist_ok=True)
    d = design()
    w, st = sd.World(d), sd.Statics(d)
    home = sd.find_home(d, w, st)[0]
    print("HOME", home, flush=True)
    grids = {}
    for obj in (CUBE, CYL):
        radii, yaws, grids[obj["name"]] = test_map(d, w, st, home, obj)
        ok = sum(g["ok"] for g in grids[obj["name"]].values())
        print(f"MAP {obj['name']}: {ok}/{len(grids[obj['name']])} positions pass", flush=True)
    tasks = test_tasks(d, w, st, home)
    for t in tasks:
        print("TASK", t["object"], t["pick"], "->", t["place"], t["result"], t.get("reason", ""), flush=True)
    dyn = test_dyn(d, st, home)
    for r in dyn:
        print("DYN", r["pose"], r["joint"], r["move_deg"], "fastest", r["fastest_T_s_margin5"], flush=True)
    lim = test_limits(d, w, st, home)
    back = test_back(st)
    plot_map(radii, yaws, grids, os.path.join(OUT, "pick_place_map.png"))
    plot_dyn(dyn, back, os.path.join(OUT, "dynamics_and_lean_back.png"))
    res = dict(design=d.key, title=d.title, home=home,
               map={k: {f"{R},{y}": v for (R, y), v in g.items()} for k, g in grids.items()},
               map_radii=radii, map_yaws=yaws, tasks=tasks, dynamic=dyn, limits=lim, lean_back=back)
    with open(os.path.join(OUT, "build_results.json"), "w") as f:
        json.dump(res, f, indent=1, default=float)
    print(json.dumps(lim, indent=1))
    print(json.dumps(back))


if __name__ == "__main__":
    main()
