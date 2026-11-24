"""Full-arm model of the CT6056 5-DOF micro arm, for every combination of parts.

Builds each full design from the supplied baseline STLs plus the team's
replacement parts (two gripper concepts, one swing counterweight), places
every part with the joint kinematics, and gives mass properties for the
simulations in simulate_designs.py.

Frames and numbers below were measured from the STLs (see README, section
"How the parts fit together"). Values marked MEASURE are assumptions that must
be checked on the real build.

Units: mm, g, degrees.  AI-assisted (Claude Code): disclose in the report.
"""
from __future__ import annotations

import math
import os
from dataclasses import dataclass, field

import numpy as np
import trimesh

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.abspath(os.path.join(HERE, "..", ".."))
BASE = os.path.join(REPO, "00_baseline", "3d_models")
NEW = os.path.join(REPO, "03_cad", "stl")
CLAW = os.path.join(REPO, "03_cad", "mini_claw", "stl")

# ----------------------------------------------------------------- geometry
L_LINK = 49.5          # pivot -> next servo shaft, ArmSpring and Arm (STL: 49.5)
PIVOT_Z = 8.0          # pivot height in every link's own frame
SHOULDER_Z = 12.9      # shoulder axis above the swing's underside (STL)
# Swing underside above the table. Set by the base servo stack, measured from the
# supplied Foot slot and the SG90 datasheets (see SG90 below): tab underside on the
# cradle top (19.2) -> case bottom 3.3 -> horn plate (under the swing disc) top at
# case bottom + 30.4 = 33.7. MEASURE on the real build.
SWING_Z = 33.7
AF_FACE_X = 43.3       # ArmFront end face (FrontPlate screws into it)

# material and hardware
PLA_DENSITY = 1.24e-3  # g/mm^3
WALL = 1.2             # 3 perimeters x 0.4 mm
INFILL = 0.20
SG90_MASS = 9.0
HORN_MASS = 0.6        # horn + horn screw
M2_MASS = 0.3
COIN_2P = 7.12
COINS = 7
PAYLOAD = 5.0          # brief: lightweight object, <= 5 g
SG90_STALL = 18000.0   # g.mm (1.8 kgf.cm at 4.8 V, datasheet)

# wide base (03_cad/wide_base): a Ø180 x 1.6 mm plate round the supplied foot
WIDE_FOOT_R = 90.0
FOOT_TOP = 1.6         # top of the plate (the original cross arms stand at 3.0)
EDGE_CHAMFER = 0.6


# ------------------------------------------------------------ transforms
def tr(x=0.0, y=0.0, z=0.0):
    m = np.eye(4)
    m[:3, 3] = (x, y, z)
    return m


def rz(deg):
    a = math.radians(deg)
    m = np.eye(4)
    m[:2, :2] = [[math.cos(a), -math.sin(a)], [math.sin(a), math.cos(a)]]
    return m


def ry(deg):
    a = math.radians(deg)
    m = np.eye(4)
    m[0, 0], m[0, 2], m[2, 0], m[2, 2] = math.cos(a), math.sin(a), -math.sin(a), math.cos(a)
    return m


def pitch(deg):
    """Rotate about the joint's y axis so that +x tilts UP by `deg`."""
    return ry(-deg)


def rot(rows, t=(0, 0, 0)):
    m = np.eye(4)
    m[:3, :3] = np.array(rows, float)
    m[:3, 3] = t
    assert abs(np.linalg.det(m[:3, :3]) - 1) < 1e-9, "not a proper rotation"
    return m


# FrontPlate frame used by the spanner CAD ("FPs"): x across the plate
# (-20..0), y along it (0..43), z = 0 on the front face, +z forward.
# FPs -> ArmFront frame (checked: both FrontPlate holes land on ArmFront's
# two end-face holes at y = -13.24 / 14.26, z = 8).
FPS_TO_AF = rot([[0, 0, 1], [0, 1, 0], [-1, 0, 0]], (AF_FACE_X + 2.0, -16.95, -2.0))
FP_STL_TO_FPS = tr(0, 0, -2.0)              # FrontPlate.stl sits at z 0..2
GRIP_SPLINE_Z = 10.8   # SG90 boss top above the FrontPlate front face (tabs on the plate)
# Mini claw frame: servo shaft at the origin, plate long axis = x.
# Rotating +90 deg about z puts its two holes on the FrontPlate holes.
CLAW_TO_FPS = rot([[0, -1, 0], [1, 0, 0], [0, 0, 1]], (-10.0, 23.3, 0.0))
# The claw is mounted with the plate turned 180 deg about the arm axis, so the
# jaws point DOWN when the wrist is level (as in its grip simulation). Same holes.
HOLE_MID_Y = (-13.24 + 14.26) / 2
FLIP_X = rot([[1, 0, 0], [0, -1, 0], [0, 0, -1]], (0, 2 * HOLE_MID_Y, 2 * PIVOT_Z))


# -------------------------------------------------------------- SG90 model
# Datasheets (Luxorparts 87897; TowerPro SG90): case 22.2-22.8 x 11.8-12.4, tabs
# 32.0-32.2 long, tab underside 15.6-16 above the case bottom, case + boss 26.7,
# horn top ~31. Checked against every supplied slot: 23.5 mm long, 12.6 mm wide,
# 16.2 mm from the tab seat to the floor / back wall. Values below are the
# largest of the datasheets (worst case for clearance).
SG90 = dict(case_l=22.8, case_w=12.4, tab_l=32.2, tab_t=2.5, tab_h=15.9,   # tab underside above case bottom
            case_top=22.7, boss_r=5.9, boss_top=26.7, spline_r=2.4, spline_top=29.4,
            shaft_from_end=5.9, wire_w=3.8, wire_t=1.4, wire_h=(1.0, 5.0))


def sg90_mesh():
    """SG90 envelope. Origin on the shaft axis in the plane of the tab UNDERSIDE
    (the face that rests on the slot), +z = shaft direction, +x = along the body
    towards the shaft end. Includes boss and spline, not the horn."""
    g = SG90
    cx = g["shaft_from_end"] - g["case_l"] / 2           # case centre x (shaft at 0)
    zb = -g["tab_h"]                                      # case bottom
    body = trimesh.creation.box((g["case_l"], g["case_w"], g["case_top"]))
    body.apply_translation((cx, 0, zb + g["case_top"] / 2))
    tabs = trimesh.creation.box((g["tab_l"], g["case_w"], g["tab_t"]))
    tabs.apply_translation((cx, 0, g["tab_t"] / 2))
    boss = trimesh.creation.cylinder(radius=g["boss_r"], height=g["boss_top"] - g["case_top"], sections=24)
    boss.apply_translation((0, 0, zb + (g["case_top"] + g["boss_top"]) / 2))
    spl = trimesh.creation.cylinder(radius=g["spline_r"], height=g["spline_top"] - g["boss_top"], sections=16)
    spl.apply_translation((0, 0, zb + (g["boss_top"] + g["spline_top"]) / 2))
    return trimesh.util.concatenate([body, tabs, boss, spl])


SERVO_BODY_COM = np.array([SG90["shaft_from_end"] - SG90["case_l"] / 2, 0.0,
                           -SG90["tab_h"] + SG90["case_top"] / 2])

# where each servo sits, in its mount part's frame (x_axis, z_axis = shaft)
SERVO_MOUNTS = {
    # name: (parent link, origin = shaft axis in the tab-seat plane, body x axis, shaft axis)
    # tab seats measured from the STLs: Foot cradle top z 19.2; Swing / ArmSpring /
    # Arm post faces y 4.7 (back wall y -11.5, floor z 6.6 / 1.7)
    "servo_base": ("foot", (0, 0, 19.2), (1, 0, 0), (0, 0, 1)),
    "servo_shoulder": ("swing", (0, 4.7, SHOULDER_Z), (1, 0, 0), (0, 1, 0)),
    "servo_elbow": ("link1", (L_LINK, 4.7, PIVOT_Z), (1, 0, 0), (0, 1, 0)),
    "servo_wrist": ("link2", (L_LINK, 4.7, PIVOT_Z), (1, 0, 0), (0, 1, 0)),
}


def frame_from_axes(origin, xa, za):
    xa, za = np.array(xa, float), np.array(za, float)
    ya = np.cross(za, xa)
    m = np.eye(4)
    m[:3, 0], m[:3, 1], m[:3, 2], m[:3, 3] = xa, ya, za, origin
    return m


# ------------------------------------------------------------------ parts
@dataclass
class Part:
    name: str
    link: str                  # foot / swing / link1 / link2 / tool
    mesh: trimesh.Trimesh      # already in the link frame
    source: str                # STL path, or "SG90 model", ...
    printed: bool = True
    status: str = "original"   # original / new / modified
    extra_mass: float = 0.0    # hardware attached to this part (g)
    mass_override: float | None = None

    @property
    def mass(self) -> float:
        if self.mass_override is not None:
            return self.mass_override + self.extra_mass
        return printed_mass(self.mesh) + self.extra_mass

    @property
    def com(self) -> np.ndarray:
        if self.name.startswith("servo"):
            return self.mesh.metadata["com"]
        m = self.mesh
        return m.center_mass if m.is_watertight and m.volume > 0 else m.bounds.mean(0)


def printed_mass(mesh: trimesh.Trimesh) -> float:
    """Shell + infill estimate: WALL-thick skin on every surface, INFILL inside."""
    v = abs(mesh.volume)
    shell = min(v, mesh.area * WALL / 2.0)   # /2: opposite faces share the wall
    return PLA_DENSITY * (shell + INFILL * (v - shell))


def solid_mass(mesh):
    return PLA_DENSITY * abs(mesh.volume)


_cache: dict[str, trimesh.Trimesh] = {}


def load(path, xf=None):
    if path not in _cache:
        m = trimesh.load(path, force="mesh")
        m.merge_vertices()
        _cache[path] = m
    m = _cache[path].copy()
    if xf is not None:
        m.apply_transform(xf)
    return m


def servo_part(name, link, xf, extra=HORN_MASS):
    m = sg90_mesh()
    m.apply_transform(xf)
    m.metadata["com"] = trimesh.transform_points([SERVO_BODY_COM], xf)[0]
    return Part(name, link, m, "SG90 envelope", printed=False, status="hardware",
                mass_override=SG90_MASS, extra_mass=extra)


# -------------------------------------------------------- gripper options
def gripper_original():
    """Supplied rack-and-pinion gripper. Placement of the racks and jaws is an
    APPROXIMATION from the part shapes (good for mass/CoM and envelope, not for
    the gripper's own internal fit): gear on the shaft, racks either side of
    it, jaws reaching forward 2..42 mm in front of the plate."""
    a = FPS_TO_AF
    parts = [Part("FrontPlate", "tool", load(f"{BASE}/FrontPlate.stl", a @ FP_STL_TO_FPS),
                  f"{BASE}/FrontPlate.stl", extra_mass=2 * M2_MASS)]
    # gear pressed on the spline, just above the boss (FPs z 10.8); racks level with it
    gz = GRIP_SPLINE_Z
    parts.append(Part("GripperGear", "tool",
                      load(f"{BASE}/GripperGear.stl", a @ tr(-10, 22.85, gz)),
                      f"{BASE}/GripperGear.stl"))
    for nm, dx in (("GripperRackTop", -6.5), ("GripperRackBottom", 6.5)):
        xf = a @ tr(-10 + dx - 0.55, 22.85 - 15, gz - 0.5)
        if dx > 0:
            xf = a @ tr(-10 + dx + 0.55, 22.85 + 15, gz - 0.5) @ rz(180)
        parts.append(Part(nm, "tool", load(f"{BASE}/{nm}.stl", xf), f"{BASE}/{nm}.stl"))
    jaw = rot([[0, 0, -1], [0, 1, 0], [1, 0, 0]])   # jaw x -> forward, jaw z -> across
    for nm in ("GripperLeft", "GripperRight"):
        xf = a @ tr(-4.25, 22.85 - 11.8, gz - 8.0) @ jaw
        parts.append(Part(nm, "tool", load(f"{BASE}/{nm}.stl", xf), f"{BASE}/{nm}.stl"))
    gs = servo_part("servo_gripper", "tool",
                    a @ frame_from_axes((-10, 22.85, 0.0), (0, 1, 0), (0, 0, 1)))
    tcp = trimesh.transform_points([(-10, 22.85, 32.0 + gz - 2.8)], a)[0]
    tool_axis = a[:3, :3] @ np.array([0, 0, 1.0])
    info = dict(name="Original rack-and-pinion", tcp=tcp, tool_axis=tool_axis,
                opening_mm=(0.0, 18.8), servo_travel_deg=120,
                grip_force_N="~20 (ideal, both jaws, at stall)",
                notes="Supplied design. Jaw placement approximate.")
    return parts + [gs], info


SPANNER_ZOFF = 8.3     # v2 lift (spanner_gripper.py zoff_v2): pinion on the real spline


def gripper_spanner(gap=13.5):
    a = FPS_TO_AF
    parts = [Part("FrontPlate", "tool", load(f"{BASE}/FrontPlate.stl", a @ FP_STL_TO_FPS),
                  f"{BASE}/FrontPlate.stl")]
    for nm, hw in (("GripperSpannerHead_v2", 2 * M2_MASS), ("GripperSpannerSlider_v2", 0),
                   ("GripperSpannerJaw_v2", M2_MASS), ("GripperSpannerPinion_v2", 0)):
        parts.append(Part(nm, "tool", load(f"{NEW}/{nm}.stl", a), f"{NEW}/{nm}.stl",
                          status="new", extra_mass=hw))
    gs = servo_part("servo_gripper", "tool",
                    a @ frame_from_axes((-10, 22.85, 0.0), (0, 1, 0), (0, 0, 1)))
    tcp = trimesh.transform_points([(-10, 10.75 + gap / 2, 31.0 + SPANNER_ZOFF)], a)[0]
    info = dict(name="Spanner (fixed datum jaw + sliding jaw)", tcp=tcp,
                tool_axis=a[:3, :3] @ np.array([0, 0, 1.0]),
                opening_mm=(1.0, 20.5), servo_travel_deg=140,
                grip_force_N="~22 (ideal, at stall)",
                notes=f"Grasp centre is off the arm axis by gap/2 - 6.75 mm "
                      f"(shown for a {gap} mm object).")
    return parts + [gs], info


def gripper_mini_claw():
    a = FPS_TO_AF @ CLAW_TO_FPS
    full = FLIP_X @ a
    parts = []
    for nm, hw in (("mini_claw_bottom_plate", 0), ("mini_claw_top_plate", 2 * 0.45),
                   ("mini_claw_jaw_left", 0), ("mini_claw_jaw_right", 0),
                   ("mini_claw_pinion", 0)):
        parts.append(Part(nm, "tool", load(f"{CLAW}/{nm}.stl", full), f"{CLAW}/{nm}.stl",
                          status="new", extra_mass=hw))
    gs = servo_part("servo_gripper", "tool",
                    full @ frame_from_axes((0, 0, -4.5), (1, 0, 0), (0, 0, 1)))   # mini claw: interface not re-verified
    tcp = trimesh.transform_points([(0, 24.0, 12.5)], full)[0]
    tool_axis = full[:3, :3] @ np.array([0, 1.0, 0])
    info = dict(name="Mini geared claw (30 deg curled tips)", tcp=tcp, tool_axis=tool_axis,
                opening_mm=(0.4, 32.2), servo_travel_deg=45,
                grip_force_N="3.0-4.4 per jaw at the tips",
                notes="Plate mounted turned 180 deg about the arm axis so the jaws "
                      "point down with the wrist level (same two screw holes).")
    return parts + [gs], info


GRIPPERS = {"original": gripper_original, "spanner": gripper_spanner,
            "mini_claw": gripper_mini_claw}


# ----------------------------------------------------------------- swings
def swing_parts(counterweight: bool):
    if not counterweight:
        return [Part("Swing", "swing", load(f"{BASE}/Swing.stl"), f"{BASE}/Swing.stl")]
    sw = Part("Swing_v2_counterweight", "swing", load(f"{NEW}/Swing_v2_counterweight.stl"),
              f"{NEW}/Swing_v2_counterweight.stl", status="modified")
    lid = Part("Swing_v2_counterweight_lid", "swing",
               load(f"{NEW}/Swing_v2_counterweight_lid.stl", tr(0, 0, 2.5)),   # lid on the pocket rim
               f"{NEW}/Swing_v2_counterweight_lid.stl", status="new", extra_mass=0.5)
    # coins: 7 x UK 2p in the 26.6 mm bore, centre 52 mm behind the yaw axis
    coins = trimesh.creation.cylinder(radius=12.95, height=7 * 1.85, sections=32)
    coins.apply_translation((-52.0, 0, -13.0 + 7 * 1.85 / 2))   # on the pocket floor
    c = Part("coins_7x2p", "swing", coins, "7 x UK 2p coins (or ~50 g filler)",
             printed=False, status="hardware", mass_override=COINS * COIN_2P)
    return [sw, lid, c]


# ------------------------------------------------------------------ feet
def foot_part(kind: str):
    if kind == "wide":
        return Part("Foot_v2_wide", "foot", load(f"{NEW}/Foot_v2_wide.stl"),
                    f"{NEW}/Foot_v2_wide.stl", status="modified")
    return Part("Foot", "foot", load(f"{BASE}/Foot.stl"), f"{BASE}/Foot.stl")


# ---------------------------------------------------------------- designs
@dataclass
class Design:
    key: str
    title: str
    gripper: str
    counterweight: bool
    foot: str = "original"     # original / wide
    parts: list = field(default_factory=list)
    tool: dict = field(default_factory=dict)

    def floor_at(self, r, half=7.5):
        """Height of the surface an object of half-width `half` rests on, centred
        at distance r from the yaw axis. None if it would straddle the plate edge."""
        if self.foot != "wide":
            return 0.0
        if r + half <= WIDE_FOOT_R - EDGE_CHAMFER:
            return FOOT_TOP
        if r - half >= WIDE_FOOT_R + 1.0:
            return 0.0
        return None

    def build(self):
        parts = [foot_part(self.foot)]
        parts += swing_parts(self.counterweight)
        parts.append(Part("ArmSpring", "link1", load(f"{BASE}/ArmSpring.stl"),
                          f"{BASE}/ArmSpring.stl"))
        parts.append(Part("Arm", "link2", load(f"{BASE}/Arm.stl"), f"{BASE}/Arm.stl"))
        parts.append(Part("ArmFront", "tool", load(f"{BASE}/ArmFront.stl"),
                          f"{BASE}/ArmFront.stl"))
        for nm, (link, o, xa, za) in SERVO_MOUNTS.items():
            parts.append(servo_part(nm, link, frame_from_axes(o, xa, za)))
        gparts, info = GRIPPERS[self.gripper]()
        self.parts = parts + gparts
        self.tool = info
        return self

    @property
    def new_parts(self):
        return [p for p in self.parts if p.status in ("new", "modified")]


DESIGNS = [
    Design("D0", "Baseline (as supplied)", "original", False),
    Design("D1", "Baseline + swing counterweight", "original", True),
    Design("D2", "Mini claw gripper", "mini_claw", False),
    Design("D3", "Mini claw gripper + swing counterweight", "mini_claw", True),
    Design("D4", "Spanner gripper", "spanner", False),
    Design("D5", "Spanner gripper + swing counterweight", "spanner", True),
    Design("D6", "Spanner gripper + swing counterweight + wide base", "spanner", True, "wide"),
]


# ------------------------------------------------------------- kinematics
LINKS = ("foot", "swing", "link1", "link2", "tool")


def link_transforms(q):
    """q = (yaw, shoulder, elbow, wrist) in degrees.
    yaw: swing about the vertical axis; shoulder: link 1 pitch above horizontal;
    elbow, wrist: relative pitch of the next link (positive = up)."""
    yaw, sh, el, wr = q
    T = {"foot": np.eye(4)}
    T["swing"] = rz(yaw) @ tr(0, 0, SWING_Z)
    T["link1"] = T["swing"] @ tr(0, 0, SHOULDER_Z) @ pitch(sh) @ tr(0, 0, -PIVOT_Z)
    T["link2"] = T["link1"] @ tr(L_LINK, 0, PIVOT_Z) @ pitch(el) @ tr(0, 0, -PIVOT_Z)
    T["tool"] = T["link2"] @ tr(L_LINK, 0, PIVOT_Z) @ pitch(wr) @ tr(0, 0, -PIVOT_Z)
    return T


def shoulder_point():
    return np.array([0.0, SWING_Z + SHOULDER_Z])


def ik_planar(design: Design, rho, z, phi, elbow_up=True):
    """Joint angles that put the TCP at horizontal distance rho (in the arm's
    plane) and height z, with the ArmFront x axis pitched at phi degrees.
    Returns (shoulder, elbow, wrist) or None if out of reach."""
    tcp = design.tool["tcp"]
    px, pz = tcp[0], tcp[2] - PIVOT_Z
    c, s = math.cos(math.radians(phi)), math.sin(math.radians(phi))
    wx, wz = rho - (c * px - s * pz), z - (s * px + c * pz)
    sx, sz = shoulder_point()
    dx, dz = wx - sx, wz - sz
    d = math.hypot(dx, dz)
    if d > 2 * L_LINK - 1e-6 or d < 1e-6:
        return None
    a = math.degrees(math.acos(d / (2 * L_LINK)))
    base = math.degrees(math.atan2(dz, dx))
    sh, el = (base + a, -2 * a) if elbow_up else (base - a, 2 * a)
    return sh, el, phi - sh - el
