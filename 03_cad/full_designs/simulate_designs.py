"""Simulate and test every full arm design (D0-D5) defined in arm_model.py.

Tests (each design):
  ASM   part interfaces: STL integrity, gripper screw holes on the ArmFront
        holes, Swing_v2 keeps every Swing feature, swing/foot clearance
  MASS  printed + hardware mass, moving mass, print volume
  COL   self-collision and table-collision sweep over the joint space
  STAB  tipping margin of the whole robot (foot not clamped), worst yaw
  TRQ   static gravity torque on the shoulder / elbow / wrist / base servos
  WS    reachable, stable, torque-safe workspace
  PNP   pick-and-place task: 15 mm, 5 g cube at 60-160 mm, -45 -> +45 deg
  YAW   base inertia and side load on the base servo shaft

Run:   python3 simulate_designs.py           (~5 min)  -> results/
       python3 simulate_designs.py --quick   (coarse grid, ~1 min)
Needs: pip install numpy scipy trimesh manifold3d python-fcl matplotlib
AI-assisted (Claude Code): disclose in the report. Simulation is not test evidence.
"""
from __future__ import annotations

import argparse
import csv
import gzip
import json
import math
import os
import sys
import time

import fcl
import numpy as np
import trimesh
from scipy.spatial import ConvexHull

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import arm_model as am  # noqa: E402

OUT = os.path.join(am.HERE, "results")
LINK_IDX = {k: i for i, k in enumerate(am.LINKS)}

# servo windows used for the workspace (each SG90 gives ~180 deg)
WINDOWS = {"shoulder": (0, 180), "elbow": (-180, 0), "wrist": (-90, 90)}
TRIM_R = 13.0          # mm around a shared pivot treated as the joint itself
TABLE_CLEAR = 0.0      # nothing but the foot may go below the table top (z = 0)
TRQ_PASS, TRQ_FAIL = 0.50, 0.80   # fraction of SG90 stall torque


# ======================================================================
# collision world
# ======================================================================
def bvh(mesh):
    m = fcl.BVHModel()
    m.beginModel(len(mesh.vertices), len(mesh.faces))
    m.addSubModel(mesh.vertices.astype(float), mesh.faces.astype(np.int32))
    m.endModel()
    return m


def fcl_obj(mesh):
    return fcl.CollisionObject(bvh(mesh), fcl.Transform())


def set_tf(obj, T):
    obj.setTransform(fcl.Transform(T[:3, :3], T[:3, 3]))


def trim_near_axis(mesh, point, axis, r):
    """Drop faces within r of a pivot axis (the pin, horn and fork boss)."""
    c = mesh.triangles_center - np.asarray(point)
    a = np.asarray(axis, float) / np.linalg.norm(axis)
    d = np.linalg.norm(c - np.outer(c @ a, a), axis=1)
    keep = d > r
    sub = mesh.submesh([np.nonzero(keep)[0]], append=True)
    return sub


# joint axes in the frame of the PROXIMAL and DISTAL link (point on axis)
JOINTS = {
    "shoulder": ("swing", (0, 0, am.SHOULDER_Z), "link1", (0, 0, am.PIVOT_Z)),
    "elbow": ("link1", (am.L_LINK, 0, am.PIVOT_Z), "link2", (0, 0, am.PIVOT_Z)),
    "wrist": ("link2", (am.L_LINK, 0, am.PIVOT_Z), "tool", (0, 0, am.PIVOT_Z)),
}
STRUCT = {"swing": ("Swing", "Swing_v2_counterweight"), "link1": ("ArmSpring",),
          "link2": ("Arm",), "tool": ("ArmFront",)}


class World:
    def __init__(self, design: am.Design):
        self.d = design
        self.parts = design.parts
        self.objs = {p.name: fcl_obj(p.mesh) for p in self.parts}
        self.hull_pts = {p.name: p.mesh.convex_hull.vertices for p in self.parts}
        self.link_of = {p.name: p.link for p in self.parts}
        # pairs that touch in the assembled neutral pose are joined parts
        # (screwed together, servo in its bay, fork round a pivot): exclude.
        self.joined = set()
        # Only parts on the same or adjacent links can be joined: a contact
        # between non-adjacent links (e.g. the tool on a wide foot) is a real hit.
        adj = {frozenset((a, b)) for a, b in zip(am.LINKS, am.LINKS[1:])}
        for q in [(0, 0, 0, 0), (0, 90, -90, 0), (0, 45, -45, -45), (30, 120, -120, 30)]:
            for pr in self._pairs_hit(q, all_pairs=True):
                la = {self.link_of[n] for n in pr}
                if len(la) == 1 or frozenset(la) in adj:
                    self.joined.add(pr)
        # adjacent structural links: check them with the pivot region trimmed
        self.trimmed = []
        for jn, (pl, pp, dl, dp) in JOINTS.items():
            for a in self.parts:
                for b in self.parts:
                    if a.link == pl and b.link == dl and a.name in STRUCT[pl] and b.name in STRUCT[dl]:
                        ta = trim_near_axis(a.mesh, pp, (0, 1, 0), TRIM_R)
                        tb = trim_near_axis(b.mesh, dp, (0, 1, 0), TRIM_R)
                        self.trimmed.append((jn, a, b, fcl_obj(ta), fcl_obj(tb)))
        self.pair_list = [(a, b) for i, a in enumerate(self.parts) for b in self.parts[i + 1:]
                          if a.link != "foot" or b.link != "foot"]
        self.pair_list = [(a, b) for a, b in self.pair_list
                          if frozenset((a.name, b.name)) not in self.joined
                          and not (a.link == b.link)]
        self.neutral_trim_hits = self.trim_hits((0, 0, 0, 0))

    def _pairs_hit(self, q, all_pairs=False):
        T = am.link_transforms(q)
        for p in self.parts:
            set_tf(self.objs[p.name], T[p.link])
        hits = set()
        for i, a in enumerate(self.parts):
            for b in self.parts[i + 1:]:
                if fcl.collide(self.objs[a.name], self.objs[b.name],
                               fcl.CollisionRequest(), fcl.CollisionResult()):
                    hits.add(frozenset((a.name, b.name)))
        return hits

    def trim_hits(self, q):
        T = am.link_transforms(q)
        out = []
        for jn, a, b, oa, ob in self.trimmed:
            set_tf(oa, T[a.link])
            set_tf(ob, T[b.link])
            if fcl.collide(oa, ob, fcl.CollisionRequest(), fcl.CollisionResult()):
                out.append(f"{a.name}~{b.name}")
        return out

    def check(self, q, links=("swing", "link1", "link2", "tool")):
        """Return (list of collisions, lowest point z of the given links)."""
        T = am.link_transforms(q)
        for p in self.parts:
            set_tf(self.objs[p.name], T[p.link])
        hits = []
        for a, b in self.pair_list:
            if fcl.collide(self.objs[a.name], self.objs[b.name],
                           fcl.CollisionRequest(), fcl.CollisionResult()):
                hits.append(f"{a.name}/{b.name}")
        hits += [h for h in self.trim_hits(q) if h not in self.neutral_trim_hits]
        zmin = min((trimesh.transform_points(self.hull_pts[p.name], T[p.link])[:, 2].min()
                    for p in self.parts if p.link in links))
        return hits, zmin


# ======================================================================
# mass, CoM, torque, stability
# ======================================================================
class Statics:
    def __init__(self, design: am.Design):
        self.d = design
        self.items = [(p.link, p.mass, np.asarray(p.com, float)) for p in design.parts]
        foot = [p for p in design.parts if p.link == "foot" and p.printed][0].mesh
        sec = foot.section(plane_origin=(0, 0, 0.3), plane_normal=(0, 0, 1))
        self.hull = ConvexHull(sec.vertices[:, :2])
        self.eq = self.hull.equations              # n.x + c <= 0 inside
        self.foot_xy = sec.vertices[:, :2][self.hull.vertices]

    def world_masses(self, q, payload=0.0):
        T = am.link_transforms(q)
        pts, ms, links = [], [], []
        for link, m, c in self.items:
            pts.append(trimesh.transform_points([c], T[link])[0])
            ms.append(m)
            links.append(link)
        if payload:
            pts.append(trimesh.transform_points([self.d.tool["tcp"]], T["tool"])[0])
            ms.append(payload)
            links.append("payload")
        return np.array(pts), np.array(ms), links, T

    def margin_xy(self, p):
        return float(np.min(-(self.eq[:, :2] @ p + self.eq[:, 2])))

    def stability(self, q, payload=am.PAYLOAD):
        """Tipping margin (mm, + = stable) at the given yaw and at the worst yaw."""
        pts, ms, links, _ = self.world_masses((0,) + tuple(q[1:]), payload)
        rot = np.array([l != "foot" for l in links])
        cs = (ms[~rot, None] * pts[~rot, :2]).sum(0)
        cr = (ms[rot, None] * pts[rot, :2]).sum(0)
        M = ms.sum()
        out = []
        for yaw in np.arange(0, 360, 3.0):
            a = math.radians(yaw)
            R = np.array([[math.cos(a), -math.sin(a)], [math.sin(a), math.cos(a)]])
            out.append(self.margin_xy((cs + R @ cr) / M))
        a = math.radians(q[0])
        R = np.array([[math.cos(a), -math.sin(a)], [math.sin(a), math.cos(a)]])
        at_yaw = self.margin_xy((cs + R @ cr) / M)
        return at_yaw, min(out)

    def side_moment(self, q, payload=0.0):
        """Horizontal moment of everything the base servo carries (g.mm)."""
        pts, ms, links, _ = self.world_masses((0,) + tuple(q[1:]), payload)
        rot = np.array([l != "foot" for l in links])
        return float(np.linalg.norm((ms[rot, None] * pts[rot, :2]).sum(0)))

    def torques(self, q, payload=am.PAYLOAD):
        """Static gravity torque (g.mm) about the shoulder, elbow and wrist axes."""
        pts, ms, links, T = self.world_masses(q, payload)
        out = {}
        order = ["link1", "link2", "tool", "payload"]
        for jn, (pl, pp, dl, _) in JOINTS.items():
            axis_pt = trimesh.transform_points([pp], T[pl])[0]
            fwd = T[pl][:3, 0]                    # arm plane direction
            hor = np.array([fwd[0], fwd[1], 0.0])
            hor /= np.linalg.norm(hor) or 1
            dist = [i for i, l in enumerate(links) if l in order[order.index(dl):]]
            out[jn] = float(abs(sum(ms[i] * ((pts[i] - axis_pt) @ hor) for i in dist)))
        return out

    def yaw_inertia(self, q=(0, 0, 0, 0)):
        pts, ms, links, _ = self.world_masses(q)
        rot = np.array([l != "foot" for l in links])
        return float((ms[rot] * (pts[rot, 0] ** 2 + pts[rot, 1] ** 2)).sum())   # g.mm^2


# ======================================================================
# individual tests
# ======================================================================
def test_asm(d: am.Design, world: World):
    res = {"parts": [], "checks": []}
    for p in d.parts:
        if not p.printed:
            continue
        m = am.load(p.source)
        res["parts"].append(dict(part=p.name, status=p.status, watertight=bool(m.is_watertight),
                                 volume_cm3=round(abs(m.volume) / 1000, 2),
                                 print_mass_g=round(p.mass - p.extra_mass, 2),
                                 solid_mass_g=round(am.solid_mass(m), 2)))
    bad = [r["part"] for r in res["parts"] if not r["watertight"]]
    res["checks"].append(dict(
        id="ASM-1", check="Every printed STL is a closed solid",
        result="PASS" if not bad else "WARN",
        detail="all watertight" if not bad else
        f"not watertight: {', '.join(bad)} (supplied files; repair in the slicer)"))

    # gripper screw holes line up with the ArmFront holes
    af_holes = np.array([[am.AF_FACE_X, -13.24, 8.0], [am.AF_FACE_X, 14.26, 8.0]])
    mount = [p for p in d.parts if p.name in ("FrontPlate", "GripperSpannerHead_v1",
                                               "mini_claw_bottom_plate")]
    clear = []
    for p in mount:
        for h in af_holes:
            o = h + np.array([40.0, 0, 0])
            hits = p.mesh.ray.intersects_any([o], [[-1.0, 0, 0]])[0]
            # also confirm there is material round the hole (ring 2.5 mm out)
            ring = p.mesh.ray.intersects_any([o + [0, 0, 2.6]], [[-1.0, 0, 0]])[0]
            clear.append((p.name, bool(not hits), bool(ring)))
    ok = all(c[1] and c[2] for c in clear)
    res["checks"].append(dict(
        id="ASM-2", check="Gripper mount: screw path through the plate/head lines up with "
                          "both ArmFront end-face holes",
        result="PASS" if ok else "FAIL",
        detail="; ".join(f"{n}: hole {'clear' if c else 'BLOCKED'}"
                         f"{'' if r else ' (no material round it)'}" for n, c, r in clear)))

    if d.counterweight:
        import manifold3d  # noqa: F401  (engine for the boolean)
        sw = am.load(f"{am.BASE}/Swing.stl")
        v2 = am.load(f"{am.NEW}/Swing_v2_counterweight.stl")
        lost = trimesh.boolean.difference([sw, v2], engine="manifold")
        lv = abs(lost.volume) if len(lost.faces) else 0.0
        res["checks"].append(dict(
            id="ASM-3", check="Swing_v2 keeps every feature of Swing (servo cradle, hub, "
                              "pivot boss): material removed from the original",
            result="PASS" if lv < 5 else "FAIL", detail=f"{lv:.2f} mm^3 removed"))

    if d.foot == "wide":
        import manifold3d  # noqa: F401
        ft = am.load(f"{am.BASE}/Foot.stl")
        v2 = am.load(f"{am.NEW}/Foot_v2_wide.stl")
        lost = trimesh.boolean.difference([ft, v2], engine="manifold")
        lv = abs(lost.volume) if len(lost.faces) else 0.0
        res["checks"].append(dict(
            id="ASM-5", check="Foot_v2_wide keeps every feature of Foot (servo cradle, "
                              "screw ears, cross arms): material removed from the original",
            result="PASS" if lv < 5 else "FAIL", detail=f"{lv:.2f} mm^3 removed"))

    # swing (and counterweight) vs foot and base servo over a full turn
    sw_parts = [p for p in d.parts if p.link == "swing"]
    foot = [p for p in d.parts if p.link == "foot" and p.printed][0]
    fo = fcl_obj(foot.mesh)
    dmin = 1e9
    for yaw in range(0, 360, 5):
        T = am.link_transforms((yaw, 0, 0, 0))
        for p in sw_parts:
            o = world.objs[p.name]
            set_tf(o, T["swing"])
            r = fcl.DistanceResult()
            dmin = min(dmin, fcl.distance(o, fo, fcl.DistanceRequest(), r))
    res["checks"].append(dict(
        id="ASM-4", check="Swing (with counterweight if fitted) clears the static foot "
                          "over a full turn",
        result="PASS" if dmin > 1.0 else "FAIL",
        detail=f"closest approach {dmin:.1f} mm (swing underside assumed "
               f"{am.SWING_Z - 19.2:.0f} mm above the foot top: MEASURE)"))
    res["neutral_contacts"] = sorted("/".join(sorted(p)) for p in world.joined)
    return res


def test_mass(d: am.Design, st: Statics):
    printed = sum(p.mass - p.extra_mass for p in d.parts if p.printed)
    solid = sum(am.solid_mass(p.mesh) for p in d.parts if p.printed)
    hw = sum(p.mass for p in d.parts if not p.printed) + sum(p.extra_mass for p in d.parts if p.printed)
    total = printed + hw
    moving = sum(p.mass for p in d.parts if p.link != "foot")
    beyond_sh = sum(p.mass for p in d.parts if p.link in ("link1", "link2", "tool"))
    tool = sum(p.mass for p in d.parts if p.link == "tool")
    vol = sum(abs(am.load(p.source).volume) for p in d.parts if p.printed) / 1000
    return dict(printed_g=round(printed, 1), printed_solid_g=round(solid, 1),
                hardware_g=round(hw, 1), total_g=round(total, 1), moving_g=round(moving, 1),
                beyond_shoulder_g=round(beyond_sh, 1), tool_g=round(tool, 1),
                print_volume_cm3=round(vol, 2),
                printed_parts=sum(1 for p in d.parts if p.printed),
                new_or_modified_parts=len(d.new_parts))


def sweep(d: am.Design, world: World, st: Statics, quick=False):
    s_step, e_step, w_step = (15, 15, 30) if quick else (5, 5, 15)
    rows = []
    for sh in range(0, 181, s_step):
        for el in range(-180, 61, e_step):
            for wr in range(-135, 136, w_step):
                q = (0, sh, el, wr)
                hits, zmin = world.check(q)
                rows.append(dict(q=q, hits=hits, zmin=zmin))
    # statics for every collision-free, table-clear pose inside the servo windows
    for r in rows:
        _, sh, el, wr = r["q"]
        r["in_window"] = (WINDOWS["shoulder"][0] <= sh <= WINDOWS["shoulder"][1] and
                          WINDOWS["elbow"][0] <= el <= WINDOWS["elbow"][1] and
                          WINDOWS["wrist"][0] <= wr <= WINDOWS["wrist"][1])
        r["free"] = not r["hits"] and r["zmin"] >= TABLE_CLEAR
        if r["free"] and r["in_window"]:
            r["stab_worst"] = st.stability(r["q"])[1]
            r["stab_worst_empty"] = st.stability(r["q"], payload=0)[1]
            r["trq"] = st.torques(r["q"])
            r["side"] = st.side_moment(r["q"], am.PAYLOAD)
            T = am.link_transforms(r["q"])
            r["tcp"] = trimesh.transform_points([d.tool["tcp"]], T["tool"])[0]
    return rows


def summarise_sweep(rows):
    tot = len(rows)
    win = [r for r in rows if r["in_window"]]
    free = [r for r in win if r["free"]]
    hitcount = {}
    for r in win:
        for h in r["hits"]:
            hitcount[h] = hitcount.get(h, 0) + 1
        if not r["hits"] and r["zmin"] < TABLE_CLEAR:
            hitcount["table"] = hitcount.get("table", 0) + 1
    stab = [r for r in free if r["stab_worst"] >= 0]
    torque_ok = [r for r in stab if max(r["trq"].values()) <= TRQ_PASS * am.SG90_STALL]
    usable = torque_ok
    tcp = np.array([r["tcp"] for r in usable]) if usable else np.zeros((0, 3))
    fwd = tcp[:, 0] if len(tcp) else np.array([0])          # + = in front of the yaw axis
    low = fwd[(tcp[:, 2] >= 0) & (tcp[:, 2] <= 30)] if len(tcp) else np.array([0])
    front = [r for r in free if r["tcp"][0] >= 0]
    back = [r for r in free if r["tcp"][0] < 0]
    worst = min(free, key=lambda r: r["stab_worst"]) if free else None
    tmax = {j: max(r["trq"][j] for r in free) for j in JOINTS} if free else {}
    return dict(
        poses_swept=tot, poses_in_servo_windows=len(win), collision_free=len(free),
        collision_free_pct=round(100 * len(free) / max(len(win), 1), 1),
        top_collisions=sorted(hitcount.items(), key=lambda kv: -kv[1])[:6],
        stable_pct_worst_yaw=round(100 * len(stab) / max(len(free), 1), 1),
        stable_pct_front=round(100 * sum(r["stab_worst"] >= 0 for r in front) / max(len(front), 1), 1),
        stable_pct_back=round(100 * sum(r["stab_worst"] >= 0 for r in back) / max(len(back), 1), 1),
        max_stable_forward_reach_any_yaw_mm=round(max((r["tcp"][0] for r in front
                                                       if r["stab_worst"] >= 0), default=0), 1),
        stable_pct_worst_yaw_empty=round(100 * sum(r["stab_worst_empty"] >= 0 for r in free)
                                         / max(len(free), 1), 1),
        min_margin_mm=round(worst["stab_worst"], 1) if worst else None,
        min_margin_pose=worst["q"] if worst else None,
        max_torque_gmm={j: round(v) for j, v in tmax.items()},
        max_torque_pct={j: round(100 * v / am.SG90_STALL) for j, v in tmax.items()},
        max_side_moment_gmm=round(max(r["side"] for r in free)) if free else None,
        usable_poses=len(usable),
        max_reach_mm=round(float(fwd.max()), 1),
        max_reach_table_mm=round(float(low.max()), 1) if len(low) else 0.0,
    )


# ======================================================================
# pick and place
# ======================================================================
def approach_angles(d):
    # ArmFront pitch that points the tool straight down, then progressively tilted
    down = -90.0 if d.gripper != "mini_claw" else 0.0
    return [down + s for s in (0, 15, -15, 30, -30, 45)]


def solve(d, world, rho_target, yaw_target, z, payload, st):
    """IK with the tool as close to vertical as the arm allows."""
    py = d.tool["tcp"][1]
    if abs(py) >= rho_target:
        return None
    rho = math.sqrt(rho_target ** 2 - py ** 2)
    yaw = yaw_target - math.degrees(math.atan2(py, rho))
    for phi in approach_angles(d):
        s = am.ik_planar(d, rho, z, phi)
        if s is None:
            continue
        q = (yaw,) + tuple(s)
        if not all(WINDOWS[k][0] - 1e-6 <= v <= WINDOWS[k][1] + 1e-6
                   for k, v in zip(("shoulder", "elbow", "wrist"), s)):
            continue
        hits, zmin = world.check(q)
        if hits or zmin < TABLE_CLEAR:
            continue
        return q, phi
    return None


def find_home(d, world, st):
    """HOME: link 1 upright, link 2 level, tool pointing down, compact and stable.
    First candidate that is collision-free, off the table and stable at any yaw."""
    down = approach_angles(d)[0]
    for sh, el in ((90, -90), (100, -100), (80, -80), (110, -110), (90, -70), (100, -80)):
        for tilt in (0, 15, 30, 45, -15):
            wr = down + tilt - sh - el
            q = (0.0, float(sh), float(el), float(wr))
            if not WINDOWS["wrist"][0] <= wr <= WINDOWS["wrist"][1]:
                continue
            hits, _ = world.check(q)
            _, zarm = world.check(q, links=("link1", "link2", "tool"))
            if not hits and zarm >= 20 and st.stability(q, 0)[1] > 0:
                return q, down + tilt
    return None


def pick_place(d, world, st, radii=(60, 80, 100, 120, 140, 160), cube=15.0):
    out = []
    home = find_home(d, world, st)
    for R in radii:
        rec = dict(radius_mm=R, result="PASS", reason="", grasp_z=None, tilt_deg=None,
                   min_margin_mm=None, max_torque_pct=None, max_side_gmm=None,
                   surface_z=d.floor_at(R, cube / 2))
        if rec["surface_z"] is None:
            rec.update(result="FAIL", reason="object would straddle the base plate edge")
            out.append(rec)
            continue
        # lowest grasp height (TCP) that keeps every part off the table, within the cube
        g = p = None
        for z in rec["surface_z"] + np.arange(cube / 2, cube + 0.01, 1.0):
            g = solve(d, world, R, -45, z, 0, st)
            p = solve(d, world, R, 45, z, am.PAYLOAD, st)
            if g and p:
                rec["grasp_z"] = float(z)
                break
        if not (g and p):
            rec.update(result="FAIL", reason="no collision-free tool pose at grasp height")
            out.append(rec)
            continue
        above_g = solve(d, world, R, -45, rec["grasp_z"] + 30, 0, st)
        above_p = solve(d, world, R, 45, rec["grasp_z"] + 30, am.PAYLOAD, st)
        if not (above_g and above_p and home):
            rec.update(result="FAIL", reason="cannot reach 30 mm above the object")
            out.append(rec)
            continue
        rec["tilt_deg"] = abs(g[1] - approach_angles(d)[0])
        legs = [(home[0], above_g[0], 0), (above_g[0], g[0], 0), (g[0], above_g[0], 1),
                (above_g[0], above_p[0], 1), (above_p[0], p[0], 1), (p[0], above_p[0], 0),
                (above_p[0], home[0], 0)]
        margins, trq, side, fails = [], [], [], []
        for a, b, carrying in legs:
            for t in np.linspace(0, 1, 16):
                q = tuple(np.array(a) + t * (np.array(b) - np.array(a)))
                pl = am.PAYLOAD if carrying else 0.0
                hits, zmin = world.check(q)
                if hits:
                    fails.append("collision " + hits[0])
                if zmin < TABLE_CLEAR - 0.5:
                    fails.append(f"hits table (z={zmin:.1f})")
                m = st.stability(q, pl)[0]
                margins.append(m)
                if m < 0:
                    fails.append("tips over")
                tq = st.torques(q, pl)
                trq.append(max(tq.values()) / am.SG90_STALL)
                side.append(st.side_moment(q, pl))
        rec["min_margin_mm"] = round(min(margins), 1)
        rec["max_torque_pct"] = round(100 * max(trq))
        rec["max_side_gmm"] = round(max(side))
        if max(trq) > TRQ_FAIL:
            fails.append("servo torque > 80 % of stall")
        if fails:
            rec.update(result="FAIL", reason=sorted(set(fails))[0])
        elif max(trq) > TRQ_PASS or min(margins) < 5:
            rec.update(result="MARGINAL",
                       reason="torque 50-80 % of stall" if max(trq) > TRQ_PASS
                       else "tip margin < 5 mm")
        out.append(rec)
    return out, home


# ======================================================================
# figures
# ======================================================================
def render(d, q, path, title):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d.art3d import Poly3DCollection
    T = am.link_transforms(q)
    fig = plt.figure(figsize=(11, 5.2))
    light = np.array([0.4, -0.6, 0.7])
    light /= np.linalg.norm(light)
    for k, (el, az) in enumerate([(12, -95), (28, -55)]):
        ax = fig.add_subplot(1, 2, k + 1, projection="3d")
        for p in d.parts:
            m = p.mesh.copy()
            m.apply_transform(T[p.link])
            if p.name.startswith("servo"):
                base = np.array([0.15, 0.15, 0.18])
            elif p.status in ("new", "modified"):
                base = np.array([0.80, 0.25, 0.55])
            elif p.status == "hardware":
                base = np.array([0.75, 0.6, 0.2])
            else:
                base = np.array([0.35, 0.55, 0.80]) if p.link != "foot" else np.array([.55, .55, .55])
            shade = 0.45 + 0.55 * np.clip(m.face_normals @ light, 0, 1)
            pc = Poly3DCollection(m.vertices[m.faces],
                                  facecolors=np.clip(shade[:, None] * base, 0, 1), edgecolor="none")
            if p.link == "foot" and p.printed:
                pc.set_sort_zpos(-500)       # flat base: always draw it behind the arm
            ax.add_collection3d(pc)
        xx, yy = np.meshgrid([-100, 190], [-110, 110])
        ax.plot_surface(xx, yy, np.zeros_like(xx), color="0.9", alpha=0.35)
        ax.set_xlim(-100, 190); ax.set_ylim(-130, 130); ax.set_zlim(0, 260)
        ax.set_box_aspect((1, 1, 1))
        ax.view_init(el, az)
        ax.set_xticks([0, 100]); ax.set_yticks([-100, 0, 100]); ax.set_zticks([0, 100, 200])
    fig.suptitle(title, fontsize=11)
    plt.tight_layout()
    plt.savefig(path, dpi=90)
    plt.close(fig)


def comparison_plots(results, rows_by, path):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    keys = [r["key"] for r in results]
    cols = ["#4C72B0", "#55A868", "#C44E52", "#8172B2", "#CCB974", "#64B5CD", "#E377C2", "#7F7F7F"]
    mid = (len(keys) - 1) / 2
    fig, axs = plt.subplots(2, 3, figsize=(16, 9))
    # 1 workspace side view (usable poses)
    for i, k in enumerate(keys):
        rows = [r for r in rows_by[k] if r.get("tcp") is not None and r["stab_worst"] >= 0
                and max(r["trq"].values()) <= TRQ_PASS * am.SG90_STALL]
        if not rows:
            continue
        t = np.array([r["tcp"] for r in rows])
        axs[0, 0].scatter(t[:, 0], t[:, 2], s=3, color=cols[i], alpha=.35, label=k)
    axs[0, 0].set(title="Usable TCP positions (collision-free, stable at worst yaw,\n"
                        "torque <= 50 % stall), 5 g payload", xlabel="TCP x: + in front of the yaw axis, - behind (mm)",
                  ylabel="height above table (mm)")
    axs[0, 0].legend(markerscale=4, fontsize=8)
    # 2 stability margin vs reach
    for i, k in enumerate(keys):
        rows = [r for r in rows_by[k] if r.get("tcp") is not None]
        t = np.array([r["tcp"] for r in rows])
        m = np.array([r["stab_worst"] for r in rows])
        rho = t[:, 0]
        bins = np.arange(-180, 190, 10)
        idx = np.digitize(rho, bins)
        y = [m[idx == j].min() if np.any(idx == j) else np.nan for j in range(1, len(bins))]
        axs[0, 1].plot(bins[:-1] + 5, y, "-o", ms=3, color=cols[i], label=k)
    axs[0, 1].axhline(0, color="k", lw=1)
    axs[0, 1].set(title="Worst tipping margin vs TCP reach\n(foot not clamped, any yaw, 5 g payload)",
                  xlabel="TCP x: + in front of the yaw axis, - behind (mm)",
                  ylabel="margin to foot edge (mm), <0 = tips")
    axs[0, 1].legend(fontsize=8)
    # 3 torques
    w = 0.8 / len(keys)
    for i, r in enumerate(results):
        v = [r["sweep"]["max_torque_pct"].get(j, 0) for j in JOINTS]
        axs[0, 2].bar(np.arange(3) + (i - mid) * w, v, w, color=cols[i], label=r["key"])
    axs[0, 2].axhline(50, color="k", ls="--", lw=1)
    axs[0, 2].set_xticks(range(3), list(JOINTS))
    axs[0, 2].set(title="Peak static torque over the usable workspace\n(% of SG90 stall, 1.8 kgf.cm)",
                  ylabel="% of stall")
    axs[0, 2].legend(fontsize=8)
    # 4 mass
    m = [r["mass"] for r in results]
    axs[1, 0].bar(keys, [x["printed_g"] for x in m], color="#4C72B0", label="printed (est.)")
    axs[1, 0].bar(keys, [x["hardware_g"] for x in m], bottom=[x["printed_g"] for x in m],
                  color="#CCB974", label="servos, screws, coins")
    axs[1, 0].set(title="Mass", ylabel="g")
    axs[1, 0].legend(fontsize=8)
    # 5 pick and place
    for i, r in enumerate(results):
        R = [p["radius_mm"] for p in r["pnp"]]
        v = [{"PASS": 2, "MARGINAL": 1, "FAIL": 0}[p["result"]] for p in r["pnp"]]
        axs[1, 1].plot(R, np.array(v) + (i - mid) * 0.05, "-o", color=cols[i], label=r["key"])
    axs[1, 1].set_yticks([0, 1, 2], ["FAIL", "MARGINAL", "PASS"])
    axs[1, 1].set(title="Pick-and-place: 15 mm 5 g cube, -45 to +45 deg yaw",
                  xlabel="object distance from yaw axis (mm)")
    axs[1, 1].legend(fontsize=8)
    # 6 base side load
    x = np.arange(len(keys))
    for j, (lab, f) in enumerate((("arm straight out", lambda r: r["yaw"]["side_moment_straight_out_gmm"]),
                                  ("HOME", lambda r: r["yaw"]["side_moment_home_gmm"] or 0),
                                  ("peak, any pose", lambda r: r["sweep"]["max_side_moment_gmm"]))):
        axs[1, 2].bar(x + (j - 1) * 0.27, [f(r) / 1000 for r in results], 0.27,
                      color=["#4C72B0", "#55A868", "#C44E52"][j], label=lab)
    axs[1, 2].set_xticks(x, keys)
    axs[1, 2].legend(fontsize=8)
    axs[1, 2].set(title="Side (bending) moment on the base servo shaft, 5 g payload",
                  ylabel="kg.mm")
    plt.tight_layout()
    plt.savefig(path, dpi=90)
    plt.close(fig)


# ======================================================================
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--quick", action="store_true")
    ap.add_argument("--only", nargs="*")
    args = ap.parse_args()
    os.makedirs(OUT, exist_ok=True)
    results, rows_by = [], {}
    for d in am.DESIGNS:
        if args.only and d.key not in args.only:
            continue
        t0 = time.time()
        d.build()
        world, st = World(d), Statics(d)
        asm = test_asm(d, world)
        mass = test_mass(d, st)
        rows = sweep(d, world, st, args.quick)
        rows_by[d.key] = rows
        sw = summarise_sweep(rows)
        pnp, home = pick_place(d, world, st)
        reach_pose = max((r for r in rows if r.get("tcp") is not None and r["stab_worst"] >= 0),
                         key=lambda r: r["tcp"][0])["q"]
        yaw = dict(inertia_gmm2=round(st.yaw_inertia((0, 0, 0, 0))),   # arm straight out
                   side_moment_straight_out_gmm=round(st.side_moment((0, 0, 0, 0), am.PAYLOAD)),
                   tip_margin_straight_out_worst_yaw_mm=round(st.stability((0, 0, 0, 0))[1], 1),
                   tip_margin_home_worst_yaw_mm=round(st.stability(home[0], 0)[1], 1) if home else None,
                   side_moment_home_gmm=round(st.side_moment(home[0], 0)) if home else None)
        # base torque for a 90 deg min-jerk move in 0.6 s (peak accel 5.77 th/T^2)
        alpha = 5.77 * math.pi / 2 / 0.6 ** 2
        yaw["base_torque_90deg_0.6s_pct"] = round(100 * yaw["inertia_gmm2"] * 1e-9 * alpha
                                                  / (am.SG90_STALL * 9.81e-6), 1)
        res = dict(key=d.key, title=d.title, gripper=d.tool["name"], counterweight=d.counterweight,
                   foot=d.foot,
                   tool_notes=d.tool["notes"], opening_mm=d.tool["opening_mm"],
                   grip_force=d.tool["grip_force_N"], asm=asm, mass=mass, sweep=sw,
                   pnp=pnp, home=home[0] if home else None, reach_pose=reach_pose, yaw=yaw,
                   parts=[dict(name=p.name, link=p.link, status=p.status,
                               source=os.path.relpath(p.source, am.REPO) if os.path.exists(p.source) else p.source,
                               mass_g=round(p.mass, 2)) for p in d.parts],
                   runtime_s=round(time.time() - t0, 1))
        results.append(res)
        with open(os.path.join(OUT, f"{d.key}_results.json"), "w") as f:
            json.dump(res, f, indent=1, default=float)
        with gzip.open(os.path.join(OUT, f"{d.key}_sweep.csv.gz"), "wt", newline="") as f:
            w = csv.writer(f)
            w.writerow(["shoulder", "elbow", "wrist", "collisions", "lowest_z_mm", "in_servo_window",
                        "tip_margin_worst_yaw_mm", "tau_shoulder_gmm", "tau_elbow_gmm",
                        "tau_wrist_gmm", "tcp_x", "tcp_y", "tcp_z"])
            for r in rows:
                t = r.get("trq", {})
                tcp = r.get("tcp", [None] * 3)
                w.writerow([*r["q"][1:], ";".join(r["hits"]), round(r["zmin"], 1), int(r["in_window"]),
                            round(r["stab_worst"], 1) if "stab_worst" in r else "",
                            *(round(t[j]) if t else "" for j in JOINTS),
                            *(round(float(v), 1) if v is not None else "" for v in tcp)])
        if home:
            render(d, home[0], os.path.join(OUT, f"{d.key}_home.png"), f"{d.key} {d.title}: HOME")
        render(d, reach_pose, os.path.join(OUT, f"{d.key}_reach.png"),
               f"{d.key} {d.title}: furthest stable reach")
        print(f"{d.key} {d.title}: {res['runtime_s']} s  free {sw['collision_free_pct']} %  "
              f"stable {sw['stable_pct_worst_yaw']} %  reach {sw['max_reach_table_mm']} mm  "
              f"PnP {[p['result'][0] for p in pnp]}", flush=True)
    if not args.only:
        comparison_plots(results, rows_by, os.path.join(OUT, "comparison.png"))
        with open(os.path.join(OUT, "summary.json"), "w") as f:
            json.dump(results, f, indent=1, default=float)


if __name__ == "__main__":
    main()
