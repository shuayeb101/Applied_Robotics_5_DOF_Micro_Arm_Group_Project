"""Render MP4 videos of full arm designs running in the simulation.

Each video shows, for one design:
  1. Range-of-motion limit test: from HOME, each joint (base, shoulder, elbow,
     wrist) is driven towards each end of its servo window in 3 deg steps.
     Every frame is checked for collisions, table contact and tipping; the
     joint stops at the first failing step and the reason is shown.
  2. Gripper open/close over its full range.
  3. Pick-and-place of two objects (15 mm cube, 12 x 20 mm cylinder), with the
     same checks on every frame plus servo load.

The right-hand panels show the view from above (foot outline, centre of mass,
tipping margin) and the live status.

Run:   python3 animate.py              -> videos/D3_*.mp4, D5_*.mp4, D6_*.mp4
       python3 animate.py --only D3 --fps 15
Needs: the simulate_designs.py packages + ffmpeg + fast-simplification
AI-assisted (Claude Code). A simulation video is not test evidence.
"""
from __future__ import annotations

import argparse
import math
import multiprocessing as mp
import os
import shutil
import subprocess
import sys
import tempfile

import numpy as np
import trimesh

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import arm_model as am  # noqa: E402
import simulate_designs as sd  # noqa: E402

OUT = os.path.join(am.HERE, "videos")
VIDEO_DESIGNS = ("D3", "D5", "D6")
STEP = 3.0                 # deg per frame in the limit test
LEG_FRAMES = 18            # frames per pick-and-place leg
JOINT_NAMES = ("base", "shoulder", "elbow", "wrist")
WIN = {"base": (-90, 90), **sd.WINDOWS}

CUBE = dict(name="15 mm cube, 5 g", kind="box", size=15.0, h=15.0, grip=15.0)
CYL = dict(name="12 x 20 mm cylinder, 5 g", kind="cyl", size=12.0, h=20.0, grip=12.0)


# ======================================================================
# gripper jaw motion (for display; statics use the closed geometry)
# ======================================================================
def jaw_transforms(d: am.Design, gap: float):
    """Extra transform (in the tool link frame) for each moving gripper part."""
    out = {}
    if d.gripper == "spanner":
        t = am.tr(0, gap - 1.0, 0)            # FPs +y == ArmFront +y; STL is at gap 1 mm
        out["GripperSpannerSlider_v1"] = out["GripperSpannerJaw_v1"] = t
    elif d.gripper == "mini_claw":
        full = am.FLIP_X @ am.FPS_TO_AF @ am.CLAW_TO_FPS
        inv = np.linalg.inv(full)
        th = max(0.0, (gap - 0.4) / 1.06)      # tip gap vs jaw angle, from the claw README
        a, h = 6.75, 9.0
        for nm, px, sgn in (("mini_claw_jaw_right", a, -1), ("mini_claw_jaw_left", -a, 1)):
            r = am.tr(px, h, 0) @ am.rz(sgn * th) @ am.tr(-px, -h, 0)
            out[nm] = full @ r @ inv
        out["mini_claw_pinion"] = full @ am.rz(1.5 * th) @ inv
    elif d.gripper == "original":
        out["GripperLeft"] = out["GripperRackTop"] = am.tr(0, gap / 2, 0)
        out["GripperRight"] = out["GripperRackBottom"] = am.tr(0, -gap / 2, 0)
    return out


GAP_RANGE = {"spanner": (1.0, 20.5), "mini_claw": (0.4, 32.0), "original": (0.0, 18.8)}


# ======================================================================
# timeline
# ======================================================================
class Planner:
    def __init__(self, d):
        self.d, self.w, self.st = d, sd.World(d), sd.Statics(d)
        self.home = sd.find_home(d, self.w, self.st)[0]
        self.closed, self.open = GAP_RANGE[d.gripper]

    def status(self, q, payload=0.0):
        hits, zmin = self.w.check(q)
        m = self.st.stability(q, payload)[0]
        tq = self.st.torques(q, payload)
        load = max(tq.values()) / am.SG90_STALL
        reason = None
        if hits:
            reason = "collision: " + hits[0].replace("~", "/")
        elif zmin < 0:
            reason = f"hits the table ({zmin:.1f} mm)"
        elif m < 0:
            reason = f"robot tips over (margin {m:.1f} mm)"
        return dict(reason=reason, margin=m, load=load, torques=tq)

    # ------------------------------------------------------------------
    def limit_test(self):
        frames = []
        hold = lambda q, n, **kw: frames.extend(dict(q=q, **kw) for _ in range(n))
        found = {}
        for j, jn in enumerate(JOINT_NAMES):
            for end in (0, 1):
                target = WIN[jn][end]
                q = list(self.home)
                sgn = 1 if target > q[j] else -1
                path, stop, last_ok = [], None, tuple(q)
                v = q[j]
                while True:
                    nv = v + sgn * STEP
                    if (nv - target) * sgn > 0:
                        nv = target
                    q[j] = nv
                    s = self.status(tuple(q))
                    if s["reason"]:
                        stop = (nv, s["reason"])
                        break
                    path.append(tuple(q))
                    last_ok = tuple(q)
                    v = nv
                    if nv == target:
                        break
                lim = last_ok[j]
                why = stop[1] if stop else "end of servo window"
                found[(jn, end)] = (lim, why)
                title = f"Range test: {jn} {'down' if sgn < 0 else 'up'}"
                for p in path:
                    frames.append(dict(q=p, phase=title, note=f"{jn} = {p[j]:.0f} deg"))
                note = (f"{jn} limit {lim:.0f} deg: {why}" if stop
                        else f"{jn} reached {lim:.0f} deg (end of servo window)")
                hold(last_ok, 12, phase=title, note=note, alert=bool(stop))
                for p in reversed(path[:-1]):
                    frames.append(dict(q=p, phase=title + " (return)", note=note, alert=bool(stop)))
                frames.append(dict(q=tuple(self.home), phase=title + " (return)", note=note))
        # gripper open / close
        for g in list(np.linspace(self.closed, self.open, 14)) + list(np.linspace(self.open, self.closed, 14)):
            frames.append(dict(q=tuple(self.home), gap=float(g), phase="Range test: gripper",
                               note=f"jaw gap {g:.1f} mm (range {self.closed}-{self.open} mm)"))
        return frames, found

    # ------------------------------------------------------------------
    def plan_pick_place(self, obj, pick, place):
        """pick/place = (radius mm, yaw deg). Returns frames or (None, reason)."""
        d, w, st = self.d, self.w, self.st
        fg, fp = d.floor_at(pick[0], obj["size"] / 2), d.floor_at(place[0], obj["size"] / 2)
        if fg is None or fp is None:
            return None, "object would straddle the base plate edge"
        g = p = None
        for z in np.arange(obj["h"] / 2, obj["h"] + 0.01, 1.0):
            g = sd.solve(d, w, pick[0], pick[1], fg + z, 0, st)
            p = sd.solve(d, w, place[0], place[1], fp + z, am.PAYLOAD, st)
            if g and p:
                break
        if not (g and p):
            return None, "no collision-free grasp pose"
        ag = sd.solve(d, w, pick[0], pick[1], fg + z + 30, 0, st)
        ap = sd.solve(d, w, place[0], place[1], fp + z + 30, am.PAYLOAD, st)
        if not (ag and ap):
            return None, "cannot reach 30 mm above the object"
        H, G, P, AG, AP = tuple(self.home), g[0], p[0], ag[0], ap[0]
        og = min(self.open, obj["grip"] + 8)
        legs = [("move above object", H, AG, 0, self.closed, og), ("lower", AG, G, 0, og, og),
                ("grip", G, G, 0, og, obj["grip"] - 0.5), ("lift", G, AG, 1, None, None),
                ("carry", AG, AP, 1, None, None), ("lower", AP, P, 1, None, None),
                ("release", P, P, 0, obj["grip"] - 0.5, og), ("lift clear", P, AP, 0, og, og),
                ("return HOME", AP, H, 0, og, self.closed)]
        frames = []
        for name, a, b, carry, g0, g1 in legs:
            n = 10 if a == b else LEG_FRAMES
            for i in range(n):
                t = i / (n - 1)
                s = 10 * t ** 3 - 15 * t ** 4 + 6 * t ** 5          # min-jerk
                q = tuple(np.array(a) + s * (np.array(b) - np.array(a)))
                gap = obj["grip"] - 0.5 if g0 is None else g0 + s * (g1 - g0)
                frames.append(dict(q=q, gap=gap, carry=carry, leg=name))
        for f in frames:
            stt = self.status(f["q"], am.PAYLOAD if f["carry"] else 0)
            if stt["reason"]:
                return None, f"{f['leg']}: {stt['reason']}"
        return frames, None


# ======================================================================
# rendering
# ======================================================================
_MESH = {}


def simplified(p):
    if p.name not in _MESH:
        m = p.mesh
        if len(m.faces) > 1600:
            try:
                m = m.simplify_quadric_decimation(face_count=1600)
            except Exception:
                pass
        _MESH[p.name] = m
    return _MESH[p.name]


def part_colour(p):
    if p.name.startswith("servo"):
        return np.array([0.16, 0.16, 0.19])
    if p.status == "hardware":
        return np.array([0.78, 0.62, 0.22])
    if p.status in ("new", "modified"):
        return np.array([0.82, 0.27, 0.56])
    if p.link == "foot":
        return np.array([0.55, 0.55, 0.57])
    return np.array([0.32, 0.53, 0.80])


def obj_mesh(obj):
    if obj["kind"] == "box":
        m = trimesh.creation.box((obj["size"],) * 3)
    else:
        m = trimesh.creation.cylinder(radius=obj["size"] / 2, height=obj["h"], sections=20)
    return m


def render_frame(args):
    idx, d_key, frame, path, meta = args
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.patches import Polygon as MPoly
    from mpl_toolkits.mplot3d.art3d import Poly3DCollection

    d = _DESIGNS[d_key]
    st = _STATICS[d_key]
    q = frame["q"]
    T = am.link_transforms(q)
    jt = jaw_transforms(d, frame.get("gap", GAP_RANGE[d.gripper][0]))
    light = np.array([0.35, -0.55, 0.75])
    light /= np.linalg.norm(light)

    fig = plt.figure(figsize=(12.8, 7.2), dpi=100, facecolor="white")
    ax = fig.add_axes([-0.06, -0.08, 0.76, 1.08], projection="3d")
    polys, cols = [], []
    for p in d.parts:
        if p.link == "foot" and p.printed:      # flat base: its own collection, drawn behind
            m = simplified(p)
            v, n = m.vertices, m.face_normals
            shade = 0.42 + 0.58 * np.clip(n @ light, 0, 1)
            pc = Poly3DCollection(v[m.faces], facecolors=np.clip(shade[:, None] * part_colour(p), 0, 1),
                                  edgecolor="none")
            pc.set_sort_zpos(-500)
            ax.add_collection3d(pc)
            continue
        m = simplified(p)
        M = T[p.link] @ jt.get(p.name, np.eye(4))
        v = trimesh.transform_points(m.vertices, M)
        n = m.face_normals @ M[:3, :3].T
        shade = 0.42 + 0.58 * np.clip(n @ light, 0, 1)
        polys.append(v[m.faces])
        cols.append(np.clip(shade[:, None] * part_colour(p), 0, 1))
    for o in frame.get("objects", []):
        m = obj_mesh(o["obj"])
        v = trimesh.transform_points(m.vertices, o["T"])
        n = m.face_normals @ o["T"][:3, :3].T
        shade = 0.45 + 0.55 * np.clip(n @ light, 0, 1)
        polys.append(v[m.faces])
        cols.append(np.clip(shade[:, None] * np.array([0.95, 0.55, 0.12]), 0, 1))
    ax.add_collection3d(Poly3DCollection(np.concatenate(polys), facecolors=np.concatenate(cols),
                                         edgecolor="none"))
    # table, foot hull, targets
    xx, yy = np.meshgrid([-95, 165], [-120, 120])
    ax.plot_surface(xx, yy, np.zeros_like(xx), color="0.88", alpha=0.35, shade=False)
    hull = np.vstack([st.foot_xy, st.foot_xy[:1]])
    ax.plot(hull[:, 0], hull[:, 1], 0.2, color="0.45", lw=0.8, ls="--")
    for tgt in meta.get("targets", []):
        a = math.radians(tgt[1])
        ax.scatter(tgt[0] * math.cos(a), tgt[0] * math.sin(a), 0.3, marker="x", color="0.3", s=30)
    ax.set_xlim(-95, 165); ax.set_ylim(-130, 130); ax.set_zlim(0, 200)
    ax.set_box_aspect((1, 1, 0.83))
    ax.set_proj_type("persp", focal_length=0.6)
    ax.view_init(22, -62 + 18 * math.sin(2 * math.pi * idx / 600))
    ax.set_axis_off()

    # --- right panel: view from above --------------------------------
    ax2 = fig.add_axes([0.665, 0.40, 0.31, 0.50])
    ax2.add_patch(MPoly(st.foot_xy, closed=True, fc="0.9", ec="0.4", lw=1))
    pts = [trimesh.transform_points([[0, 0, 0]], T["swing"])[0],
           trimesh.transform_points([[0, 0, am.PIVOT_Z]], T["link1"])[0],
           trimesh.transform_points([[0, 0, am.PIVOT_Z]], T["link2"])[0],
           trimesh.transform_points([[0, 0, am.PIVOT_Z]], T["tool"])[0],
           trimesh.transform_points([d.tool["tcp"]], T["tool"])[0]]
    pts = np.array(pts)
    ax2.plot(pts[:, 0], pts[:, 1], "-o", color="#4C72B0", ms=3, lw=2)
    if d.counterweight:
        c = trimesh.transform_points([[-52, 0, 0]], T["swing"])[0]
        ax2.add_patch(plt.Circle(c[:2], 15, color="#C4468F", alpha=0.6))
    payload = am.PAYLOAD if frame.get("carry") else 0.0
    ps, ms, links, _ = st.world_masses(q, payload)
    com = (ms[:, None] * ps[:, :2]).sum(0) / ms.sum()
    margin = st.margin_xy(com)
    mc = "#2E9E4F" if margin >= 5 else ("#E39B1B" if margin >= 0 else "#D62728")
    ax2.plot(*com, "o", color=mc, ms=11, mec="k")
    for o in frame.get("objects", []):
        ax2.plot(o["T"][0, 3], o["T"][1, 3], "s", color="#F28C1E", ms=6)
    ax2.set_xlim(-100, 190); ax2.set_ylim(-140, 140); ax2.set_aspect("equal")
    ax2.set_title("From above: foot, arm, centre of mass", fontsize=10)
    ax2.tick_params(labelsize=7)

    # --- status text -------------------------------------------------
    tq = st.torques(q, payload)
    lines = [f"base {q[0] + 0.0:6.0f}°   shoulder {q[1]:5.0f}°",
             f"elbow {q[2]:5.0f}°   wrist {q[3]:7.0f}°",
             f"jaw gap {frame.get('gap', GAP_RANGE[d.gripper][0]):5.1f} mm",
             f"tip margin {margin:6.1f} mm",
             "servo load  " + "  ".join(f"{k[:2]} {100 * v / am.SG90_STALL:3.0f}%" for k, v in tq.items())]
    fig.text(0.675, 0.33, "\n".join(lines), fontsize=10.5, family="monospace", va="top")
    import textwrap
    fig.text(0.675, 0.16, textwrap.fill(frame.get("note", ""), 52), fontsize=10, va="top",
             color="#D62728" if frame.get("alert") else "0.15", weight="bold" if frame.get("alert") else None)
    fig.text(0.015, 0.965, f"{d.key}: {d.title}", fontsize=15, weight="bold", va="top")
    fig.text(0.015, 0.925, frame.get("phase", ""), fontsize=12, va="top", color="0.25")
    fig.text(0.015, 0.02, "Simulation only (static checks on every frame), not test evidence. "
                          "Pink = new/modified part, orange = object. AI-assisted.",
             fontsize=8, color="0.4")
    fig.savefig(path, dpi=100)
    plt.close(fig)
    return idx


_DESIGNS, _STATICS = {}, {}


def _init(keys):
    for d in am.DESIGNS:
        if d.key in keys:
            d.build()
            _DESIGNS[d.key] = d
            _STATICS[d.key] = sd.Statics(d)


# ======================================================================
def build_frames(d):
    pl = Planner(d)
    frames, limits = pl.limit_test()
    tasks = [(CUBE, None, None), (CYL, (80, 30), (80, -60))]
    res = json_results(d.key)
    best = max((r["radius_mm"] for r in res["pnp"] if r["result"] == "PASS"), default=80)
    tasks[0] = (CUBE, (best, -45), (best, 45))
    targets, summary = [], []
    world_objs = []                     # objects resting on the table
    for obj, pick, place in tasks:
        fr, why = pl.plan_pick_place(obj, pick, place)
        if fr is None:   # try nearer/other side before giving up
            for alt in ((90, 35), (100, 20), (80, 0)):
                fr, why2 = pl.plan_pick_place(obj, alt, (alt[0], -alt[1]))
                if fr:
                    pick, place = alt, (alt[0], -alt[1])
                    break
        if fr is None:
            summary.append(f"{obj['name']}: FAILED ({why})")
            continue
        targets += [pick, place]
        a = math.radians(pick[1])
        T_obj = am.tr(pick[0] * math.cos(a), pick[0] * math.sin(a),
                      d.floor_at(pick[0], obj["size"] / 2) + obj["h"] / 2) @ am.rz(pick[1])
        if obj["kind"] == "cyl":
            T_obj = T_obj  # upright cylinder
        rel = None
        title = f"Pick and place: {obj['name']}, {pick[0]} mm @ {pick[1]}° -> {place[0]} mm @ {place[1]}°"
        for f in fr:
            Tt = am.link_transforms(f["q"])["tool"]
            if f["carry"] and rel is None:
                rel = np.linalg.inv(Tt) @ T_obj
            if f["carry"]:
                T_obj = Tt @ rel
            elif rel is not None:
                rel = None
            s = pl.status(f["q"], am.PAYLOAD if f["carry"] else 0)
            frames.append(dict(q=f["q"], gap=f["gap"], carry=f["carry"], phase=title,
                               note=f"{f['leg']} - all checks pass (margin {s['margin']:.0f} mm, "
                                    f"peak servo load {100 * s['load']:.0f}%)",
                               objects=[*world_objs, dict(obj=obj, T=T_obj.copy())]))
        world_objs.append(dict(obj=obj, T=T_obj.copy()))
        summary.append(f"{obj['name']}: picked at {pick[0]} mm and placed, all checks passed")
    # final card
    lim_txt = "; ".join(f"{k[0]} {'min' if k[1] == 0 else 'max'} {v[0]:.0f}°"
                        for k, v in limits.items())
    for _ in range(30):
        frames.append(dict(q=tuple(pl.home), phase="Summary", objects=world_objs,
                           note="Joint limits found: " + lim_txt + ".  " + "  ".join(summary)))
    return frames, dict(targets=targets), limits, summary


def json_results(key):
    import json
    return json.load(open(os.path.join(am.HERE, "results", f"{key}_results.json")))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--only", nargs="*", default=list(VIDEO_DESIGNS))
    ap.add_argument("--fps", type=int, default=15)
    ap.add_argument("--max-frames", type=int, default=0, help="for quick previews")
    args = ap.parse_args()
    os.makedirs(OUT, exist_ok=True)
    for key in args.only:
        _init([key])
        d = _DESIGNS[key]
        frames, meta, limits, summary = build_frames(d)
        if args.max_frames:
            frames = frames[:: max(1, len(frames) // args.max_frames)]
        print(f"{key}: {len(frames)} frames ({len(frames) / args.fps:.0f} s)", flush=True)
        for k, v in limits.items():
            print(f"   {k[0]:8s} {'min' if k[1] == 0 else 'max'} {v[0]:6.0f}  {v[1]}")
        for s in summary:
            print("  ", s)
        tmp = tempfile.mkdtemp(prefix=f"anim_{key}_")
        jobs = [(i, key, f, os.path.join(tmp, f"f{i:05d}.png"), meta) for i, f in enumerate(frames)]
        with mp.Pool(os.cpu_count(), initializer=_init, initargs=([key],)) as pool:
            for n, _ in enumerate(pool.imap_unordered(render_frame, jobs, chunksize=4)):
                if n % 100 == 0:
                    print(f"   rendered {n}/{len(jobs)}", flush=True)
        name = d.title.lower().replace(" + ", "_").replace(" ", "_")
        out = os.path.join(OUT, f"{key}_{name}.mp4")
        subprocess.run(["ffmpeg", "-y", "-loglevel", "error", "-framerate", str(args.fps),
                        "-i", os.path.join(tmp, "f%05d.png"), "-c:v", "libx264", "-pix_fmt",
                        "yuv420p", "-crf", "26", "-preset", "slow", out], check=True)
        shutil.rmtree(tmp)
        print(f"   -> {out} ({os.path.getsize(out) / 1e6:.1f} MB)", flush=True)


if __name__ == "__main__":
    main()
