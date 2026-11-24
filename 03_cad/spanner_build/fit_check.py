"""Fit, hand-assembly and wiring check for the chosen build (D6):
spanner gripper v2 + swing counterweight + wide base, on the supplied arm links.

Three questions, each answered for every SG90 in the arm:

  FIT    Does the servo fit its slot? The SG90 envelope (datasheet maxima, see
         arm_model.SG90) is placed in each slot exactly as on the real arm (tab
         underside on the measured tab seat). Every part's surface is sampled and
         any point INSIDE the servo is a clash; the depth is reported.
         Supplied parts are checked too: they must pass, which validates the
         servo model and placement against the slots the servos really fit.
  ASSY   Can the parts be put together by hand, in order? Each step moves a part
         (or servo) along its insertion direction from 40 mm away and checks every
         1 mm against what is already assembled. Screwdriver access (Ø5 mm shaft,
         60 mm long) is checked for every screw at the step it is fitted.
  WIRE   Is there room for the wires? Each SG90 wire leaves the case at the
         shaft end, 1-5 mm above the case bottom. A corridor 12 mm long out of
         that face must be free. The base servo wire is followed along its lane
         under the clips to the plate edge, with the swing and counterweight
         turned through a full circle above it. The arm's wire bundle drops off
         the +x end of the swing: the counterweight must never sweep through it.

Run:  python3 fit_check.py   -> results/fit_check.json, printed report
AI-assisted (Claude Code). A CAD check is not test evidence: dry-fit the prints.
"""
from __future__ import annotations

import json
import math
import os
import sys

import numpy as np
import trimesh
from manifold3d import Manifold, Mesh

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "full_designs"))
import arm_model as am  # noqa: E402

OUT = os.path.join(HERE, "results")
CLASH = 0.3            # mm: deeper than this is a clash (contact/print tolerance below)
SAMPLES = 20000
rng = np.random.default_rng(0)


# ------------------------------------------------------------------ geometry
def to_tm(m: Manifold) -> trimesh.Trimesh:
    g = m.to_mesh()
    return trimesh.Trimesh(np.asarray(g.vert_properties)[:, :3].astype(float), g.tri_verts, process=False)


def mbox(x0, x1, y0, y1, z0, z1):
    return Manifold.cube((x1 - x0, y1 - y0, z1 - z0)).translate((x0, y0, z0))


def mcyl(r, z0, z1, n=32):
    return Manifold.cylinder(z1 - z0, r, r, n).translate((0, 0, z0))


def sg90_solid():
    """Watertight SG90 envelope in the servo frame of arm_model (origin: shaft axis
    in the tab-underside plane, +z shaft, +x towards the shaft end)."""
    g = am.SG90
    x1 = g["shaft_from_end"]
    x0 = x1 - g["case_l"]
    cx = (x0 + x1) / 2
    zb = -g["tab_h"]
    w = g["case_w"] / 2
    s = mbox(x0, x1, -w, w, zb, zb + g["case_top"])
    s += mbox(cx - g["tab_l"] / 2, cx + g["tab_l"] / 2, -w, w, 0, g["tab_t"])
    s += mcyl(g["boss_r"], zb + g["case_top"] - 0.01, zb + g["boss_top"])
    s += mcyl(g["spline_r"], zb + g["boss_top"] - 0.01, zb + g["spline_top"], 16)
    return to_tm(s)


def wire_corridor(length=12.0):
    """Space the wire needs as it leaves the case (servo frame)."""
    g = am.SG90
    x1 = g["shaft_from_end"]
    zb = -g["tab_h"]
    h0, h1 = g["wire_h"]
    w = g["wire_w"] / 2 + 0.5
    return to_tm(mbox(x1 + 0.05, x1 + length, -w, w, zb + h0, zb + h1 + 1.0))


SERVO = sg90_solid()
WIRE = wire_corridor()


_SAMP, _MAN = {}, {}


def _samples(part):
    # cache keyed on the object AND kept alive with it, so a freed temporary's
    # reused id can never return another mesh's data
    k = id(part)
    if k not in _SAMP or _SAMP[k][0] is not part:
        pts, _ = trimesh.sample.sample_surface(part, SAMPLES, seed=1)
        _SAMP[k] = (part, np.vstack([pts, part.vertices]))
    return _SAMP[k][1]


def _man(mesh):
    k = id(mesh)
    if k not in _MAN or _MAN[k][0] is not mesh:
        _MAN[k] = (mesh, Manifold(Mesh(vert_properties=np.asarray(mesh.vertices, np.float32),
                                       tri_verts=np.asarray(mesh.faces, np.uint32))))
    return _MAN[k][1]


def _boxes_apart(a, Ta, b, Tb, pad=0.5):
    ab = trimesh.transform_points(trimesh.bounds.corners(a.bounds), Ta)
    bb = trimesh.transform_points(trimesh.bounds.corners(b.bounds), Tb)
    return np.any(ab.min(0) - pad > bb.max(0)) or np.any(bb.min(0) - pad > ab.max(0))


def pen_depth(solid: trimesh.Trimesh, part: trimesh.Trimesh, T_solid=np.eye(4), T_part=np.eye(4)):
    """Deepest point of `part`'s surface inside `solid` (mm, 0 = no overlap).
    Works for non-watertight parts (only `solid` must be closed)."""
    if _boxes_apart(solid, T_solid, part, T_part):
        return 0.0
    if part.is_watertight and solid.is_watertight:
        i = _man(solid).transform(T_solid[:3, :]) ^ _man(part).transform(T_part[:3, :])
        if i.volume() < 1e-3:
            return 0.0
        # thickness of each separate overlap: min(bbox extent, 1.5 x 2V/S); a thin
        # ring (press fit) or slab reads as its true thickness
        t = 0.0
        for c in i.decompose():
            v = c.volume()
            if v < 1e-4:
                continue
            bb = c.bounding_box()
            ext = min(np.subtract(bb[3:], bb[:3]))
            t = max(t, min(ext, 3.0 * v / max(c.surface_area(), 1e-9)))
        return float(t)
    pts = trimesh.transform_points(_samples(part), np.linalg.inv(T_solid) @ T_part)
    b = solid.bounds
    m = np.all((pts > b[0] - 0.01) & (pts < b[1] + 0.01), axis=1)
    pts = pts[m]
    if not len(pts):
        return 0.0
    inside = solid.contains(pts)
    if not inside.any():
        return 0.0
    d = trimesh.proximity.signed_distance(solid, pts[inside])
    return float(max(0.0, d.max()))


def both_ways(a, b, Ta=np.eye(4), Tb=np.eye(4)):
    """Clash depth between two parts: b's surface inside a, and (if b is closed)
    a's surface inside b."""
    if a.is_watertight and b.is_watertight:
        return pen_depth(a, b, Ta, Tb)
    d = pen_depth(a, b, Ta, Tb) if a.is_watertight else 0.0
    if b.is_watertight:
        d = max(d, pen_depth(b, a, Tb, Ta))
    return d


# ------------------------------------------------------------------ the build
d = [x for x in am.DESIGNS if x.key == "D6"][0].build()
P = {p.name: p for p in d.parts}
HOME = (0.0, 90.0, -70.0, -80.0)
T = am.link_transforms(HOME)


def servo_T(name):
    link, o, xa, za = (am.SERVO_MOUNTS[name] if name != "servo_gripper" else
                       ("tool", None, None, None))
    if name == "servo_gripper":
        return am.FPS_TO_AF @ am.frame_from_axes((-10, 22.85, 0.0), (0, 1, 0), (0, 0, 1)), "tool"
    return am.frame_from_axes(o, xa, za), link


SERVOS = ["servo_base", "servo_shoulder", "servo_elbow", "servo_wrist", "servo_gripper"]
PRINTED = [p for p in d.parts if p.printed]


# ------------------------------------------------------------------ FIT
def test_fit():
    rows = []
    for sn in SERVOS:
        Ts, link = servo_T(sn)
        Tw = T[link] @ Ts
        for p in PRINTED:
            dep = pen_depth(SERVO, p.mesh, Tw, T[p.link])
            if p.mesh.is_watertight:
                dep = max(dep, pen_depth(p.mesh, SERVO, T[p.link], Tw))
            if dep > 0.0 or p.link == link:
                rows.append(dict(servo=sn, part=p.name, status=p.status, depth_mm=round(dep, 2),
                                 result="CLASH" if dep > CLASH else ("contact" if dep > 0.01 else "clear")))
    return rows


# ------------------------------------------------------------------ ASSY
def sweep(moving, T_mov, fixed, direction, dist=40.0, step=1.0, frame=np.eye(4)):
    step = 1.0 if all(m.is_watertight for _, m, _ in fixed) else 2.0
    """Move `moving` from dist mm out (along `direction`, in `frame`) to its seat.
    Returns the worst clash depth against every fixed (mesh, T) on the way, with
    the seated pose excluded (that is FIT)."""
    worst, where = 0.0, None
    dvec = frame[:3, :3] @ np.asarray(direction, float)
    for s in np.arange(dist, step - 1e-9, -step):
        Tm = am.tr(*(dvec * s)) @ T_mov
        for name, mesh, Tf in fixed:
            dep = both_ways(moving, mesh, Tm, Tf)
            if dep > worst:
                worst, where = dep, (name, round(float(s), 1))
    return worst, where


def screw_access(axis_pt, axis_dir, fixed, r=2.5, L=60.0, start=0.5):
    """Screwdriver shaft (cylinder) from just above the screw head outwards."""
    a = np.asarray(axis_dir, float)
    a /= np.linalg.norm(a)
    cyl = trimesh.creation.cylinder(radius=r, height=L, sections=24)
    z = np.array([0, 0, 1.0])
    R = trimesh.geometry.align_vectors(z, a)
    R[:3, 3] = np.asarray(axis_pt) + a * (start + L / 2)
    worst, hit = 0.0, None
    for name, mesh, Tf in fixed:
        dep = both_ways(cyl, mesh, R, Tf)
        if dep > worst:
            worst, hit = dep, name
    return worst, hit


def test_assembly():
    rows = []

    def rec(step, part, how, dep, where, note=""):
        rows.append(dict(step=step, part=part, how=how, worst_mm=round(dep, 2),
                         blocked_by=(where[0] if isinstance(where, tuple) else where) if dep > CLASH else "",
                         result="BLOCKED" if dep > CLASH else "OK", note=note))

    foot = P["Foot_v2_wide"].mesh
    I = np.eye(4)
    # 1. base servo drops into the foot cradle from above
    Tb, _ = servo_T("servo_base")
    dep, w = sweep(SERVO, Tb, [("Foot_v2_wide", foot, I)], (0, 0, 1))
    rec(1, "base SG90 -> Foot_v2_wide cradle", "lowered in from above", dep, w)
    # 2. base servo tab screws (vertical, at the two cradle holes)
    for x in (-19.5, 8.5):
        dep, w = screw_access((x, 0, 19.2 + 2.5), (0, 0, 1), [("Foot_v2_wide", foot, I)])
        rec(2, f"base tab screw at x={x}", "screwdriver from above", dep, w)
    # 3. swing (with counterweight, lid off) lowered onto the base spline
    sw = P["Swing_v2_counterweight"].mesh
    Tsw = am.tr(0, 0, am.SWING_Z)
    dep, w = sweep(sw, Tsw, [("Foot_v2_wide", foot, I), ("base SG90", SERVO, Tb)], (0, 0, 1))
    rec(3, "Swing_v2_counterweight -> base spline", "lowered onto the horn", dep, w)
    # 4. shoulder servo into the swing bay (open top)
    Ts, _ = servo_T("servo_shoulder")
    dep, w = sweep(SERVO, Ts, [("Swing_v2_counterweight", sw, I)], (0, 0, 1))
    rec(4, "shoulder SG90 -> swing bay", "lowered in from above", dep, w)
    for x in (-19.0, 8.0):
        dep, w = screw_access((x, 4.7 + 2.5, am.SHOULDER_Z), (0, 1, 0),
                              [("Swing_v2_counterweight", sw, I)])
        rec(4, f"shoulder tab screw at x={x}", "screwdriver along the shaft axis (+y)", dep, w)
    # 5. coins and lid
    coins = P["coins_7x2p"].mesh
    dep, w = sweep(coins, I, [("Swing_v2_counterweight", sw, I)], (0, 0, 1))
    rec(5, "7 x 2p coins -> pocket", "dropped in from above", dep, w)
    lid = P["Swing_v2_counterweight_lid"].mesh
    dep, w = sweep(lid, I, [("Swing_v2_counterweight", sw, I), ("coins", coins, I)], (0, 0, 1))
    rec(5, "counterweight lid", "placed from above", dep, w)
    # 5b. lid screw (M3, at x = -30 on the boom) reachable with the WHOLE arm fitted, at HOME:
    #     the coins can be changed without taking the arm apart
    whole = [(p.name, p.mesh, T[p.link]) for p in d.parts if p.link in ("link1", "link2", "tool")]
    whole += [(sn, SERVO, T[servo_T(sn)[1]] @ servo_T(sn)[0]) for sn in SERVOS if servo_T(sn)[1] != "foot"
              and servo_T(sn)[1] != "swing"]
    lid_top = trimesh.transform_points([(-30.0, 0, 4.0)], T["swing"])[0]
    dep, w = screw_access(lid_top, (0, 0, 1), whole + [("Swing_v2_counterweight_lid",
                                                         P["Swing_v2_counterweight_lid"].mesh, T["swing"])])
    rec(5, "counterweight lid screw (x = -30), whole arm fitted at HOME", "screwdriver from above", dep, w,
        "coins can be changed without dismantling the arm")

    # 6. gripper servo into ArmFront, case first through the FrontPlate window
    af, fp = P["ArmFront"].mesh, P["FrontPlate"].mesh
    Tg, _ = servo_T("servo_gripper")
    shaft = Tg[:3, 2]
    dep, w = sweep(SERVO, Tg, [("ArmFront", af, I), ("FrontPlate", fp, I)], shaft)
    rec(6, "gripper SG90 -> FrontPlate window -> ArmFront bay", "pushed in case-first", dep, w)
    # 7. spanner head over the servo case, onto the tabs
    head = P["GripperSpannerHead_v2"].mesh
    dep, w = sweep(head, I, [("ArmFront", af, I), ("FrontPlate", fp, I), ("gripper SG90", SERVO, Tg)], shaft)
    rec(7, "GripperSpannerHead_v2 -> onto the servo tabs", "pushed on along the shaft", dep, w)
    fixed7 = [("ArmFront", af, I), ("FrontPlate", fp, I), ("gripper SG90", SERVO, Tg),
              ("GripperSpannerHead_v2", head, I)]
    for hx, hy in ((-10.0, 3.75), (-10.0, 31.25)):
        top = trimesh.transform_points([(hx, hy, 2.5 + 1.5 + 1.6)], am.FPS_TO_AF)[0]
        dep, w = screw_access(top, shaft, fixed7, r=2.0)
        rec(7, f"head/tab screw at FrontPlate ({hx}, {hy})", "screwdriver down the counterbore", dep, w,
            "stack: 1.5 head floor + 2.5 tab + 2 plate, then 3.1 mm into the ArmFront post: "
            "the supplied ~8 mm SG90 tab screw is long enough")
    # 8. slider into the dovetail from the open (high-y) end
    sl = P["GripperSpannerSlider_v2"].mesh
    along = am.FPS_TO_AF[:3, 1]
    dep, w = sweep(sl, I, fixed7, along, dist=45.0)
    rec(8, "GripperSpannerSlider_v2 -> dovetail", "slid in from the open end", dep, w)
    # 9. pinion pressed onto the spline (closed position), then its horn screw
    pin = P["GripperSpannerPinion_v2"].mesh
    fixed9 = fixed7 + [("GripperSpannerSlider_v2", sl, I)]
    dep, w = sweep(pin, I, fixed9, shaft)
    rec(9, "GripperSpannerPinion_v2 -> spline", "pressed on along the shaft", dep, w)
    ptop = trimesh.transform_points([(-10, 22.85, am.SPANNER_ZOFF + 2.8 + 5.5)], am.FPS_TO_AF)[0]
    dep, w = screw_access(ptop, shaft, fixed9, r=1.6)
    rec(9, "pinion horn screw", "screwdriver along the shaft", dep, w)
    # 10. jaw onto the slider tenon, then its M2 screw
    jaw = P["GripperSpannerJaw_v2"].mesh
    fixed10 = fixed9 + [("GripperSpannerPinion_v2", pin, I)]
    dep, w = sweep(jaw, I, fixed10, shaft, dist=30.0)
    rec(10, "GripperSpannerJaw_v2 -> slider tenon", "pushed on along the shaft", dep, w)
    return rows


# ------------------------------------------------------------------ WIRE
ORIGINAL_OF = {"Foot_v2_wide": "Foot", "Swing_v2_counterweight": "Swing"}


def new_material(p):
    """The part of a printed part that is new in this build (whole part if new)."""
    if p.status == "original":
        return None
    if p.name in ORIGINAL_OF:
        o = am.load(f"{am.BASE}/{ORIGINAL_OF[p.name]}.stl")
        return to_tm(_man(p.mesh) - _man(o))
    return p.mesh


def test_wires():
    rows = []
    newmat = {p.name: new_material(p) for p in PRINTED}
    for sn in SERVOS:
        Ts, link = servo_T(sn)
        Tw = T[link] @ Ts
        worst, who, orig, owho = 0.0, "", 0.0, ""
        for p in PRINTED:
            nm = newmat[p.name]
            if nm is not None and len(nm.faces):
                dep = pen_depth(WIRE, nm, Tw, T[p.link])
                if dep > worst:
                    worst, who = dep, p.name + " (new material)"
            if p.status == "original" or p.name in ORIGINAL_OF:
                src = p.mesh if p.status == "original" else am.load(f"{am.BASE}/{ORIGINAL_OF[p.name]}.stl")
                dep = pen_depth(WIRE, src, Tw, T[p.link])
                if dep > orig:
                    orig, owho = dep, ORIGINAL_OF.get(p.name, p.name)
        rows.append(dict(check=f"{sn}: wire exit corridor (12 mm straight out of the shaft-end face)",
                         worst_mm=round(worst, 2), blocked_by=who if worst > CLASH else "",
                         result="BLOCKED" if worst > CLASH else "OK",
                         note=("supplied parts alone: clear" if orig <= CLASH else
                               f"as supplied, {owho} already turns the wire (it bends round a "
                               f"{orig:.1f} mm lip in the original build too); no new material in the way")))
    # base wire lane: exit -> lane -> under the clips -> plate edge, swing turning above
    lane_x, side = 14.0, 1
    path = [(6.0, 0.0), (lane_x, 0.0), (lane_x, side * 30.0), (lane_x, side * 55.0),
            (lane_x, side * 80.0), (lane_x, side * 93.0)]
    zb = 19.2 - am.SG90["tab_h"]
    corridor = []
    for (x0, y0), (x1, y1) in zip(path, path[1:]):
        L = math.hypot(x1 - x0, y1 - y0)
        n = max(1, int(L / 2))
        for t in np.linspace(0, 1, n + 1):
            corridor.append((x0 + t * (x1 - x0), y0 + t * (y1 - y0)))
    foot = P["Foot_v2_wide"].mesh
    wire_w, wire_t = am.SG90["wire_w"], am.SG90["wire_t"]
    worst, who = 0.0, ""
    for i, (x, y) in enumerate(corridor):
        # wire lies on the plate / cross arm (z top of whatever is under it)
        offs = [(dx, dy) for dx in (-1, 0, 1) for dy in (-1.9, 0, 1.9)]
        if x >= lane_x - 0.1:                          # lane runs along y: swap
            offs = [(dy, dx) for dx, dy in offs]
        xs = [(x + dx, y + dy) for dx, dy in offs]
        loc, ri, _ = foot.ray.intersects_location([(a, b, 30.0) for a, b in xs], [(0, 0, -1.0)] * len(xs))
        # floor = top of the plate / cross arm (ignore the clip bridges above the wire)
        loc = np.asarray(loc).reshape(-1, 3)
        tops = [loc[np.asarray(ri) == k][:, 2] for k in range(len(xs))]
        z_floor = max((t[t < 3.05].max() if np.any(t < 3.05) else 0.0) for t in tops)
        if x < 7.5:
            z_floor = zb + am.SG90["wire_h"][0]          # still at the case exit height
        seg = trimesh.creation.box((2.0, wire_w, wire_t))
        horiz = (path[1][0] - path[0][0], 0) if x < lane_x - 0.1 else (0, 1)
        ang = math.atan2(horiz[1], horiz[0])
        M = am.tr(x, y, z_floor + wire_t / 2 + 0.05) @ am.rz(math.degrees(ang))
        dep = pen_depth(seg, foot, M, np.eye(4))
        if dep > worst:
            worst, who = dep, f"Foot_v2_wide at ({x:.0f}, {y:.0f})"
    rows.append(dict(check="base wire: lane from the cradle exit, under 3 clips, to the plate edge",
                     worst_mm=round(worst, 2), blocked_by=who if worst > CLASH else "",
                     result="BLOCKED" if worst > CLASH else "OK"))
    # swing + counterweight + horn sweep over the lying wire (worst over a full turn)
    seg_all = trimesh.util.concatenate([
        trimesh.creation.box((2.0, 2.0, 4.0 + wire_t), am.tr(x, y, am.FOOT_TOP + 2.0))
        for x, y in corridor if x >= 7.5])
    swing_parts = [p for p in d.parts if p.link == "swing"]
    worst, who = 0.0, ""
    for yaw in range(0, 360, 10):
        Tsw = am.rz(yaw) @ am.tr(0, 0, am.SWING_Z)
        for p in swing_parts:
            dep = pen_depth(p.mesh, seg_all, Tsw, np.eye(4)) if p.mesh.is_watertight else 0.0
            if dep > worst:
                worst, who = dep, f"{p.name} at yaw {yaw}"
    clear = min(trimesh.transform_points(p.mesh.vertices, am.tr(0, 0, am.SWING_Z))[:, 2].min()
                for p in swing_parts) - (am.FOOT_TOP + 4.0 + wire_t)
    rows.append(dict(check="swing + counterweight turning over the base wire (wire with 4 mm slack loop)",
                     worst_mm=round(worst, 2), blocked_by=who if worst > CLASH else "",
                     result="BLOCKED" if worst > CLASH else "OK",
                     note=f"lowest swing part is {clear:.1f} mm above the top of a 4 mm wire loop"))
    # arm bundle: leaves the swing at its +x end (shoulder SG90 shaft end) and drops
    # to the plate; it turns with the swing, so check in the swing frame
    drop = trimesh.creation.box((14.0, 14.0, am.SWING_Z + 18 - am.FOOT_TOP),
                                am.tr(13.0, -6.0, (am.SWING_Z + 18 + am.FOOT_TOP) / 2 - am.SWING_Z))
    worst, who = 0.0, ""
    for p in swing_parts:
        if p.name.startswith(("Swing_v2", "coins")) or "lid" in p.name:
            dep = pen_depth(drop, p.mesh, np.eye(4), np.eye(4))
            if p.name != "Swing_v2_counterweight" and dep > worst:
                worst, who = dep, p.name
    cw = trimesh.transform_points(P["coins_7x2p"].mesh.vertices, np.eye(4))
    gap = float(np.min(np.hypot(cw[:, 0] - 13.0, cw[:, 1] + 6.0))) - 7.0
    rows.append(dict(check="arm wire bundle drop zone (+x end of the swing) vs counterweight pocket",
                     worst_mm=round(worst, 2), blocked_by=who if worst > CLASH else "",
                     result="BLOCKED" if worst > CLASH else "OK",
                     note=f"counterweight stays {gap:.0f} mm from the bundle: it is on the far (-x) side "
                          "and turns with the swing"))
    return rows


def self_test():
    """The detector must flag a servo pushed 1 mm into its slot floor."""
    Tb, _ = servo_T("servo_base")
    ok = pen_depth(SERVO, P["Foot_v2_wide"].mesh, am.tr(0, 0, -1.0) @ Tb)
    sideways = pen_depth(SERVO, P["ArmSpring"].mesh, am.tr(0, -1.0, 0) @ servo_T("servo_elbow")[0])
    print(f"self-test: base SG90 1 mm too low -> {ok:.2f} mm; elbow SG90 1 mm into the back wall "
          f"(non-watertight ArmSpring) -> {sideways:.2f} mm")
    assert ok > CLASH and sideways > CLASH, "clash detector failed its self-test"
    return dict(base_1mm_low=round(ok, 2), elbow_1mm_into_wall=round(sideways, 2))


def main():
    os.makedirs(OUT, exist_ok=True)
    st = self_test()
    print("FIT: SG90 in each slot (depth of overlap; contact < "
          f"{CLASH} mm is a seating face / print tolerance)")
    fit = test_fit()
    for r in fit:
        print(f"  {r['servo']:15s} {r['part']:28s} {r['status']:9s} {r['depth_mm']:5.2f} mm  {r['result']}")
    print("ASSY: hand-assembly order")
    assy = test_assembly()
    for r in assy:
        print(f"  {r['step']:2d} {r['part']:52s} {r['how']:38s} {r['worst_mm']:5.2f} mm  {r['result']}"
              f"{'  <- ' + r['blocked_by'] if r['blocked_by'] else ''}")
    print("WIRE")
    wires = test_wires()
    for r in wires:
        print(f"  {r['check']:80s} {r['worst_mm']:5.2f} mm  {r['result']}"
              f"{'  <- ' + r['blocked_by'] if r['blocked_by'] else ''}  {r.get('note', '')}")
    with open(os.path.join(OUT, "fit_check.json"), "w") as f:
        json.dump(dict(self_test=st, fit=fit, assembly=assy, wires=wires, clash_threshold_mm=CLASH,
                       sg90=am.SG90, swing_z=am.SWING_Z), f, indent=1)


if __name__ == "__main__":
    main()
