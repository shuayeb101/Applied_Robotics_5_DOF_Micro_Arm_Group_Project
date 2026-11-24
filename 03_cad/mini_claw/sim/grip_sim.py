"""
Grip simulation for the mini geared claw (MuJoCo physics).

What it does, for each tip-curl angle, object and friction value:
  1. builds the jaw collision shape from the real CAD outline
     (OpenSCAD `finger_2d` export, split into convex prisms),
  2. runs a pick cycle with the claw pointing down:
       open -> lower over the object -> close (servo torque-limited)
       -> lift 40 mm -> shake sideways in the jaw plane -> shake
       out of the jaw plane -> shake up/down -> lower -> open,
  3. reports whether the object was held, how far it slipped in the
     jaws, and whether it released cleanly.

Model assumptions (state these with the results):
  * the two jaws are rigidly coupled 1:1 (ideal gears, no backlash);
  * the servo is a position controller limited to SG90 stall torque
    (default 1.2 kgf.cm, the pessimistic figure) through the 1.5:1 gearing;
  * PLA-on-PLA friction is varied over the measured 0.12-0.38 range;
  * contact is MuJoCo's soft contact model, not a measured one.
It checks grip and retention, not part strength.

Usage:  python3 -I sim/grip_sim.py [--quick] [--gif]
Needs:  openscad on PATH, pip packages: mujoco, shapely>=2.1, ezdxf, matplotlib
AI-assisted (Claude). Check and understand before using as evidence (C12).
"""
import argparse, itertools, json, math, os, subprocess, sys, tempfile

import ezdxf
import mujoco
import numpy as np
import shapely
from shapely.geometry import LineString, Polygon, box
from shapely.ops import polygonize, unary_union

HERE = os.path.dirname(os.path.abspath(__file__))
SCAD = os.path.join(HERE, "..", "mini_claw.scad")
OUT = os.path.join(HERE, "results")

# ---- geometry that must match mini_claw.scad --------------------------
M, Z_P, Z_J = 0.9, 10, 15
A = M * Z_J / 2                              # pivot half-spacing, mm
H = math.sqrt((M * (Z_P + Z_J) / 2) ** 2 - A ** 2)  # pivot y, mm
Z0, Z2, DROP = 11.0, 17.0, 3.0               # gear stack and finger depth, mm
JAW_LEN = 30.0
RATIO = Z_J / Z_P

# ---- servo ---------------------------------------------------------------
STALL_KGFCM = 1.2                            # pessimistic SG90 stall torque
T_STALL = STALL_KGFCM * 0.0980665            # N.m
SAT_DEG = 4.0                                # jaw error at which torque saturates


def finger_outline(tip_curl):
    """Right-jaw finger outline (local frame, mm) from the OpenSCAD model."""
    with tempfile.TemporaryDirectory() as td:
        dxf = os.path.join(td, "f.dxf")
        subprocess.run(["openscad", "-D", 'part="finger_2d"', "-D", f"tip_curl={tip_curl}",
                        "-o", dxf, SCAD], check=True, capture_output=True)
        lines = [LineString([(e.dxf.start.x, e.dxf.start.y), (e.dxf.end.x, e.dxf.end.y)])
                 for e in ezdxf.readfile(dxf).modelspace() if e.dxftype() == "LINE"]
    polys = list(polygonize(lines))
    outer = max(polys, key=lambda p: p.area)
    return Polygon(outer.exterior)            # holes filled: they never touch the object


def inner_edge(tip_curl, curl_len=9.0, inner_start=9.0, inner_off=2.2, tip_gap=0.4):
    """Gripping-edge polyline of the right jaw, same construction as mini_claw.scad."""
    x_tip = tip_gap / 2 - A
    c = math.radians(tip_curl)
    tip = np.array([x_tip, JAW_LEN])
    bend = tip - curl_len * np.array([-math.sin(c), math.cos(c)])
    p0 = np.array([-A + inner_off, inner_start])
    return p0, bend, tip


def convex_pieces(poly, tip_curl, y_min=10.0, pitch=2.0):
    """Split the gripping part of the finger into chunky convex pieces.

    The finger is sliced perpendicular to the gripping edge through every
    serration notch, so each slice is one tooth plus the body behind it
    (convex). Anything left that is not convex is triangulated. Thin
    sliver triangles are avoided because a box corner can tunnel through them.
    """
    from shapely.ops import split
    region = poly.intersection(box(-50, y_min, 50, 60))
    p0, bend, tip = inner_edge(tip_curl)
    pieces = [region]
    for a, b, margin in ((p0, bend, 1.0), (bend, tip, 2.5)):
        L = np.linalg.norm(b - a)
        d = (b - a) / L
        nrm = np.array([d[1], -d[0]])
        n = int((L - margin) // pitch)
        for i in range(1, n + 1):
            c = a + d * (i * pitch)
            cut = LineString([c - nrm * 12, c + nrm * 12])
            nxt = []
            for pc in pieces:
                nxt += list(split(pc, cut).geoms) if cut.intersects(pc) else [pc]
            pieces = nxt
        # also cut at the bend so the straight and curled parts are separate
    cut = LineString([bend - np.array([0, 0]) + np.array([-12, -12 * math.tan(math.radians(tip_curl / 2))]),
                      bend + np.array([12, 12 * math.tan(math.radians(tip_curl / 2))])])
    nxt = []
    for pc in pieces:
        nxt += list(split(pc, cut).geoms) if cut.intersects(pc) else [pc]
    out = []
    for pc in nxt:
        if pc.area < 1e-3:
            continue
        if pc.convex_hull.area / pc.area < 1.02:
            out.append(Polygon(pc.convex_hull.exterior))
        else:
            out += [t for t in shapely.constrained_delaunay_triangles(pc).geoms if t.area > 1e-3]
    return out


def edge_boxes(tip_curl, thick=3.0, ext=0.6):
    """Jaw contact model: one solid box behind each part of the gripping edge
    (straight part and curled tip). Single convex pieces with no internal
    seams, so a box corner cannot tunnel between pieces. Serration teeth are
    not modelled geometrically; their effect is covered by the friction sweep.
    Returns (centre_xy, angle_deg, half_length) per box, right jaw, pivot frame."""
    p0, bend, tip = inner_edge(tip_curl)
    out = []
    for p, q in ((p0, bend), (bend, tip)):
        L = np.linalg.norm(q - p)
        d = (q - p) / L
        n = np.array([d[1], -d[0]])              # into the jaw material (+x side)
        c = (p + q) / 2 + n * thick / 2
        out.append((c, math.degrees(math.atan2(d[1], d[0])), L / 2 + ext))
    return out


def mjcf(tris_R, obj, mu, tip_curl=None, obj_dx=0.0, obj_yaw=0.0):
    """MJCF model. Units: metres, kg. Claw frame = mm drawing frame / 1000,
    rotated so the jaws point down (claw +y -> world -z, claw +z -> world +y)."""
    s = 1e-3
    zlo, zhi = (Z0 - DROP) * s, Z2 * s
    zc, zh = (zlo + zhi) / 2, (zhi - zlo) / 2
    assets, geomsR, geomsL = [], [], []
    for c, ang, hl in edge_boxes(tip_curl):
        for side, sign in (("R", 1), ("L", -1)):
            cx, cy = sign * c[0], c[1]
            a = ang if sign > 0 else 180 - ang
            g = (f'<geom type="box" pos="{cx*s:.6f} {cy*s:.6f} {zc:.6f}" euler="0 0 {a:.4f}" '
                 f'size="{hl*s:.6f} {1.5*s:.6f} {zh:.6f}" friction="{mu} 0.005 0.0001" '
                 f'solref="-100000 -40" solimp="0.95 0.99 0.0001" condim="4" rgba=".2 .2 .2 1" mass="0.0007"/>')
            (geomsR if side == "R" else geomsL).append(g)
    obj_geom = {
        "cube": '<geom name="obj" type="box" size="0.0075 0.0075 0.0075" mass="0.005" '
                f'friction="{mu} 0.005 0.0001" solref="-100000 -40" solimp="0.95 0.99 0.0001" condim="4" rgba=".9 .5 .1 1"/>',
        "cylinder": '<geom name="obj" type="cylinder" size="0.006 0.010" mass="0.005" '
                    f'friction="{mu} 0.005 0.0001" solref="-100000 -40" solimp="0.95 0.99 0.0001" condim="4" rgba=".9 .5 .1 1"/>',
    }[obj]
    obj_h = {"cube": 0.0075, "cylinder": 0.010}[obj]
    kp = RATIO * T_STALL / math.radians(SAT_DEG)
    return f"""
<mujoco model="mini_claw_grip">
  <compiler angle="degree"/>
  <option timestep="0.0001" cone="elliptic" impratio="10" noslip_iterations="3" gravity="0 0 -9.81"/>
  <worldbody>
    <geom name="table" type="plane" size="0.2 0.2 0.01" friction="0.5 0.005 0.0001" solref="-100000 -40" solimp="0.95 0.99 0.0001" rgba=".8 .8 .8 1"/>
    <body name="carriage" pos="0 -0.0125 0.08" gravcomp="1">
      <joint name="cx" type="slide" axis="1 0 0" damping="20"/>
      <joint name="cy" type="slide" axis="0 1 0" damping="20"/>
      <joint name="cz" type="slide" axis="0 0 1" damping="20"/>
      <inertial pos="0 0 0" mass="0.05" diaginertia="1e-5 1e-5 1e-5"/>
      <body name="claw" euler="-90 0 0" gravcomp="1">
        <inertial pos="0 0 0" mass="0.005" diaginertia="1e-6 1e-6 1e-6"/>
        <body name="jawR" pos="{A*s} {H*s} 0" gravcomp="1">
          <joint name="qR" type="hinge" axis="0 0 -1" damping="0.01" armature="1e-5" range="-2 35" limited="true"/>
          {''.join(geomsR)}
        </body>
        <body name="jawL" pos="{-A*s} {H*s} 0" gravcomp="1">
          <joint name="qL" type="hinge" axis="0 0 1" damping="0.01" armature="1e-5" range="-2 35" limited="true"/>
          {''.join(geomsL)}
        </body>
      </body>
    </body>
    <body name="object" pos="{obj_dx} 0 {obj_h + 0.0002}" euler="0 0 {obj_yaw}">
      <freejoint/>
      {obj_geom}
    </body>
  </worldbody>
  <contact><exclude body1="jawR" body2="jawL"/></contact>
  <equality><joint joint1="qL" joint2="qR" polycoef="0 1 0 0 0" solref="0.0004 1" solimp="0.99 0.999 0.0001"/></equality>
  <actuator>
    <position name="servo" joint="qR" kp="{kp:.4f}" forcelimited="true" forcerange="{-RATIO*T_STALL:.5f} {RATIO*T_STALL:.5f}"/>
    <position name="ax" joint="cx" kp="3000"/>
    <position name="ay" joint="cy" kp="3000"/>
    <position name="az" joint="cz" kp="3000"/>
  </actuator>
</mujoco>"""


def smooth(t, t0, t1):
    if t <= t0: return 0.0
    if t >= t1: return 1.0
    u = (t - t0) / (t1 - t0)
    return u * u * (3 - 2 * u)


TIP_DEPTH = (H + JAW_LEN) * 1e-3     # carriage-to-tip distance, m
Z_GRIP = TIP_DEPTH + 0.0015          # carriage height with tips 1.5 mm above the table
Z_UP = Z_GRIP + 0.040
OPEN = math.radians(30)


def targets(t):
    """Command schedule. Returns (jaw angle, x, y, z) targets."""
    q = OPEN * smooth(t, 0.0, 0.2)                     # open
    q -= OPEN * smooth(t, 0.6, 1.0)                    # close (to 0 = squeeze at stall)
    q += OPEN * smooth(t, 4.4, 4.7)                    # release
    z = Z_GRIP + 0.025 * (1 - smooth(t, 0.2, 0.6))     # lower
    z += 0.040 * smooth(t, 1.1, 1.5)                   # lift
    z -= 0.040 * smooth(t, 4.0, 4.35)                  # put down
    x = y = 0.0
    w, amp = 2 * math.pi * 4, 0.008                    # 4 Hz, +-8 mm  (~0.5 g)
    if 1.5 <= t < 2.5: x = amp * math.sin(w * (t - 1.5))
    if 2.5 <= t < 3.5: y = amp * math.sin(w * (t - 2.5))
    if 3.5 <= t < 4.0: z += 0.005 * math.sin(w * (t - 3.5))
    return q, x, y, z


def run(tip_curl, obj, mu, tris_cache, record=False, obj_dx=0.0, obj_yaw=0.0):
    if tip_curl not in tris_cache:
        tris_cache[tip_curl] = (finger_outline(tip_curl),)
        tris_cache[tip_curl] += (convex_pieces(tris_cache[tip_curl][0], tip_curl),)
    outline, tris = tris_cache[tip_curl]
    model = mujoco.MjModel.from_xml_string(mjcf(tris, obj, mu, tip_curl, obj_dx, obj_yaw))
    data = mujoco.MjData(model)
    data.qpos[model.jnt_qposadr[model.joint("cz").id]] = Z_GRIP + 0.025 - 0.08
    obj_b = model.body("object").id
    claw_b = model.body("claw").id
    qR = model.jnt_qposadr[model.joint("qR").id]
    qL = model.jnt_qposadr[model.joint("qL").id]
    worst = 0.0

    def rel():  # object position in the claw frame, mm
        R = data.xmat[claw_b].reshape(3, 3)
        return R.T @ (data.xpos[obj_b] - data.xpos[claw_b]) * 1e3

    frames, log = [], {}
    dt = model.opt.timestep
    n = int(4.8 / dt)
    for k in range(n):
        t = k * dt
        q, x, y, z = targets(t)
        data.ctrl[:] = [q, x, y, z - 0.08]
        mujoco.mj_step(model, data)
        worst = max(worst, abs(data.qpos[qR] - data.qpos[qL]))
        if abs(t - 1.1) < dt / 2:
            log["p_grip"] = rel().tolist()
            log["jaw_deg_grip"] = math.degrees(data.qpos[qR])
        if abs(t - 4.0) < dt / 2:
            log["p_end"] = rel().tolist()
            log["obj_height_mm"] = data.xpos[obj_b][2] * 1e3
        if record and k % 200 == 0:
            frames.append((t, math.degrees(data.qpos[qR]),
                           data.xpos[claw_b].copy(), data.xpos[obj_b].copy(),
                           data.xquat[obj_b].copy()))
    log["obj_final_height_mm"] = data.xpos[obj_b][2] * 1e3
    log["jaw_mismatch_deg"] = math.degrees(worst)
    log["jaw_deg_final"] = math.degrees(data.qpos[qR])
    p0, p1 = np.array(log["p_grip"]), np.array(log["p_end"])
    log["slip_mm"] = float(np.linalg.norm(p1 - p0))
    held = log["obj_height_mm"] > 30           # still up after lift + shaking
    log["held"] = bool(held)
    log["released"] = bool(log["obj_final_height_mm"] < 12 and log["jaw_deg_final"] > 25)
    return log, frames, outline


def make_gif(frames, outline, obj, path, title):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.animation import FuncAnimation, PillowWriter
    from shapely import affinity
    fig, ax = plt.subplots(figsize=(5, 6))
    hub = shapely.Point(0, 0).buffer(5.4)
    jaw = unary_union([outline, hub])

    def draw(i):
        t, qdeg, cpos, opos, oquat = frames[i]
        ax.clear()
        cx, cz = cpos[0] * 1e3, cpos[2] * 1e3
        for sign in (1, -1):
            g = affinity.rotate(jaw, -qdeg, origin=(0, 0))
            if sign < 0:
                g = affinity.scale(g, -1, 1, origin=(0, 0))
            g = affinity.translate(g, sign * A, H)
            xs, ys = g.exterior.xy
            ax.fill([cx + x for x in xs], [cz - y for y in ys], color="0.2")
        # object (side view in the jaw plane)
        w = 15 if obj == "cube" else 12
        hh = 15 if obj == "cube" else 20
        R = np.zeros(9); mujoco.mju_quat2Mat(R, oquat); R = R.reshape(3, 3)
        ang = math.degrees(math.atan2(R[2, 0], R[0, 0]))
        o = affinity.rotate(box(-w / 2, -hh / 2, w / 2, hh / 2), ang, origin=(0, 0))
        o = affinity.translate(o, opos[0] * 1e3, opos[2] * 1e3)
        xs, ys = o.exterior.xy
        ax.fill(xs, ys, color="tab:orange")
        ax.axhline(0, color="0.5")
        ax.set_xlim(-35, 35); ax.set_ylim(-5, 110); ax.set_aspect("equal")
        ax.set_title(f"{title}\nt = {t:.2f} s   jaw = {qdeg:.0f} deg", fontsize=9)
        ax.set_xlabel("mm")

    anim = FuncAnimation(fig, draw, frames=len(frames))
    anim.save(path, writer=PillowWriter(fps=25))
    plt.close(fig)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--quick", action="store_true", help="cube only, mu = 0.12, 1 trial")
    ap.add_argument("--gif", action="store_true", help="save animations (first trial, mu = 0.12)")
    ap.add_argument("--trials", type=int, default=5, help="trials per case (object placement varied)")
    ap.add_argument("--drop", type=float, default=3.0, help="finger_drop in the CAD (mm): grip face depth = 6 + drop")
    ap.add_argument("--curls", type=str, default="5.44,12,18,24,30", help="tip_curl values to test")
    ap.add_argument("--tag", type=str, default="", help="suffix for the result files")
    args = ap.parse_args()
    global DROP
    DROP = args.drop
    os.makedirs(OUT, exist_ok=True)
    curls = [float(c) for c in args.curls.split(",")]   # 5.44 deg = straight jaw (the first version)
    objs = ["cube", "cylinder"] if not args.quick else ["cube"]
    mus = [0.12, 0.25, 0.38] if not args.quick else [0.12]
    # Trial design: 5 placements shifted across the jaws (no rotation), and for
    # the cube 4 more rotated about the vertical axis (a cylinder looks the
    # same at any rotation). Fixed values, so every run is repeatable.
    shift_trials = [(dx * 1e-3, 0.0) for dx in (-1.5, -0.75, 0.0, 0.75, 1.5)]
    yaw_trials = [(0.0, yaw) for yaw in (2.0, 4.0, 6.0, 8.0)]
    if args.quick:
        shift_trials, yaw_trials = [(0.0, 0.0)], []
    cache, rows = {}, []
    for c, o, mu in itertools.product(curls, objs, mus):
        offsets = shift_trials + (yaw_trials if o == "cube" else [])
        for k, (dx, yaw) in enumerate(offsets):
            rec = args.gif and mu == 0.12 and k == 0
            log, frames, outline = run(c, o, mu, cache, record=rec, obj_dx=dx, obj_yaw=yaw)
            row = dict(tip_curl=c, object=o, mu=mu, trial=k + 1, obj_dx_mm=dx * 1e3, obj_yaw_deg=yaw,
                       **{kk: log[kk] for kk in ("held", "slip_mm", "released", "jaw_deg_grip",
                                                 "obj_height_mm", "jaw_mismatch_deg")})
            row["secure"] = bool(row["held"] and row["slip_mm"] < 2.0)
            rows.append(row)
            print(f"curl {c:>5} {o:<8} mu {mu:.2f} trial {k+1}  held={row['held']!s:<5} "
                  f"slip={row['slip_mm']:7.2f} mm  released={row['released']!s:<5} "
                  f"grip={row['jaw_deg_grip']:5.1f} deg", flush=True)
            if rec:
                make_gif(frames, outline, o, os.path.join(OUT, f"grip_{o}_curl{c:g}_mu{mu}{args.tag}.gif"),
                         f"tip curl {c:g} deg, {o}, mu = {mu}")
    with open(os.path.join(OUT, f"results{args.tag}.json"), "w") as f:
        json.dump(rows, f, indent=1)
    # summary tables
    def stats(r):
        held = [x for x in r if x["held"]]
        return (f"{len(held)}/{len(r)} | {sum(x['secure'] for x in r)}/{len(r)} | "
                f"{np.median([x['slip_mm'] for x in held]) if held else float('nan'):.2f} | "
                f"{sum(x['released'] for x in held)}/{len(held)} | "
                f"{np.median([x['jaw_deg_grip'] for x in held]) if held else float('nan'):.1f}")
    lines = ["## Placement shifted across the jaws (-1.5 to +1.5 mm, not rotated)", "",
             "| Tip curl (deg) | Object | mu | Held | Secure (slip < 2 mm) | Median slip of held (mm) | Released (of held) | Grip angle (deg) |",
             "| --- | --- | --- | --- | --- | --- | --- | --- |"]
    for c, o, mu in itertools.product(curls, objs, mus):
        r = [x for x in rows if x["tip_curl"] == c and x["object"] == o and x["mu"] == mu and x["obj_yaw_deg"] == 0]
        lines.append(f"| {c:g} | {o} | {mu} | " + stats(r) + " |")
    if "cube" in objs and not args.quick:
        lines += ["", "## Cube rotated on the table (centred): largest rotation still held securely", "",
                  "| Tip curl (deg) | mu | Secure at 0 / 2 / 4 / 6 / 8 deg |", "| --- | --- | --- |"]
        for c, mu in itertools.product(curls, mus):
            r = {x["obj_yaw_deg"]: x for x in rows if x["tip_curl"] == c and x["object"] == "cube" and x["mu"] == mu
                 and x["obj_dx_mm"] == 0}
            lines.append(f"| {c:g} | {mu} | " + " ".join("✓" if r[y]["secure"] else "✗" for y in (0.0, 2.0, 4.0, 6.0, 8.0)) + " |")
    with open(os.path.join(OUT, f"summary{args.tag}.md"), "w") as f:
        f.write("\n".join(lines) + "\n")
    print("\n".join(lines))
    print("worst jaw coupling error (deg):", max(x["jaw_mismatch_deg"] for x in rows))


if __name__ == "__main__":
    main()
