#!/usr/bin/env python3
"""
Spanner-inspired gripper for the CT6056 5-DOF micro arm (parametric CAD).

Concept: an adjustable spanner has ONE fixed jaw that is part of the body and
ONE sliding jaw that runs in a guided slot, driven by a rack. This gripper
copies that layout:

  * GripperSpannerHead   - body + integral fixed jaw + dovetail slide channel.
                           Bolts onto the ORIGINAL FrontPlate using the same two
                           M2 screws that hold the gripper servo (longer screws).
  * GripperSpannerSlider - rack bar that runs in the dovetail channel
                           (the spanner's sliding-jaw rack).
  * GripperSpannerJaw    - moving jaw finger, screwed to the slider.
  * GripperSpannerPinion - m1 z16 pinion on the SG90 spline (the spanner's worm,
                           replaced by a pinion because the servo only turns ~180 deg).

Coordinate frame = the frame of 00_baseline/3d_models/FrontPlate.stl:
  x across the plate (-20..0), y along the plate (0..43, opening direction),
  z = 0 on the plate's FRONT face, +z pointing away from the arm (jaw reach).
The arm's centre line passes through (x=-10, y=17.5).

Run:  python3 spanner_gripper.py            -> writes STLs to ../stl/
      python3 spanner_gripper.py --shaft-low -> mirrored set, see README
Requires: pip install manifold3d trimesh numpy
"""
import argparse, math, os, sys
import numpy as np
import manifold3d as mf
import trimesh

# ------------------------------------------------------------------ parameters
P = dict(
    # interface to the original front plate / SG90
    hole_xy=[(-10.0, 3.75), (-10.0, 31.25)],   # M2 holes in FrontPlate.stl
    window=(-16.35, 5.75, -3.75, 29.25),       # servo window in FrontPlate.stl
    axis_y=17.5,                               # arm centre line (between screws)
    shaft_x=-10.0, shaft_y=22.85,              # SG90 output shaft (measure!)
    servo_top=2.3,                             # servo case top above plate front face
    # drive
    module=1.0, teeth=16, pressure=20.0, backlash=0.15,
    pinion_z0=2.8, pinion_b=5.5, spline_d=4.7, spline_depth=3.6,
    # body
    base_t=2.0, x_left=-20.0, wall_out=9.0,
    slider_top=8.5, slider_right_bot=6.0, slider_right_top=4.25, slide_clear=0.3,
    # jaws
    fixed_face_y=10.75, jaw_t=5.0, jaw_x=(-18.0, 4.2), jaw_tip_z=38.0,
    jaw_bottom_z=9.2, gap_min=1.0, travel_deg=140.0,
    vgroove_depth=2.0, serr_pitch=2.0, serr_h=0.5, serr_len=14.0,
)

def cs(poly):
    """CrossSection from a simple polygon, forced counter-clockwise."""
    a = sum(x0 * y1 - x1 * y0 for (x0, y0), (x1, y1) in zip(poly, poly[1:] + poly[:1]))
    return mf.CrossSection([poly if a > 0 else poly[::-1]])

def ext_xy(poly, z0, z1):
    """Extrude a 2-D (x, y) polygon between z0 and z1."""
    return mf.Manifold.extrude(cs(poly), z1 - z0).translate((0, 0, z0))

def ext_xz(poly, y0, y1):
    """Extrude a 2-D (x, z) polygon between y0 and y1."""
    m = mf.Manifold.extrude(cs(poly), y1 - y0).rotate((90, 0, 0))
    return m.translate((0, y1, 0))   # Rx(90): (u,v,w)->(u,-w,v)

def ext_yz(poly, x0, x1):
    """Extrude a 2-D (y, z) polygon between x0 and x1."""
    # (u,v,w) -> want x=w, y=u, z=v
    m = mf.Manifold.extrude(cs(poly), x1 - x0)
    m = m.rotate((90, 0, 0)).rotate((0, 0, 90))  # (u,v,w)->(u,-w,v)->(w,u,v)
    return m.translate((x0, 0, 0))

def box(x0, y0, z0, x1, y1, z1):
    return mf.Manifold.cube((x1 - x0, y1 - y0, z1 - z0)).translate((x0, y0, z0))

def cyl(x, y, z0, z1, r, n=48):
    return mf.Manifold.cylinder(z1 - z0, r, r, n).translate((x, y, z0))

def cone(x, y, z0, z1, r0, r1, n=48):
    return mf.Manifold.cylinder(z1 - z0, r0, r1, n).translate((x, y, z0))

def to_trimesh(m):
    mesh = m.to_mesh()
    return trimesh.Trimesh(np.asarray(mesh.vert_properties)[:, :3],
                           np.asarray(mesh.tri_verts), process=True)

# ------------------------------------------------------------------ gear maths
def pinion_profile(p):
    m, z, a = p['module'], p['teeth'], math.radians(p['pressure'])
    r = m * z / 2; rb = r * math.cos(a); ra = r + m; rf = r - 1.25 * m
    inv = lambda t: math.tan(t) - t
    # half tooth angle at pitch circle, thinned by half the backlash
    half = (math.pi / 2 - p['backlash'] / 2 / m) / z
    pts = []
    for k in range(z):
        c = 2 * math.pi * k / z
        flank = []
        for rr in np.linspace(max(rb, rf), ra, 10):
            ang = half + inv(a) - inv(math.acos(min(1, rb / rr)))
            flank.append((rr, ang))
        root_ang = flank[0][1]
        side = [(rf, root_ang + 0.0)] + flank
        left = [(rr, c - t) for rr, t in side]
        right = [(rr, c + t) for rr, t in reversed(side)]
        for rr, t in left + right:
            pts.append((rr * math.cos(t), rr * math.sin(t)))
    return pts, r, ra, rf

def rack_profile(p, y0, y1, x_pitch, x_back, phase=0.0):
    """Rack teeth (pointing -x) as an (x, y) polygon, y0..y1."""
    m, a = p['module'], math.radians(p['pressure'])
    pitch = math.pi * m; ha, hf = m, 1.25 * m
    s = pitch / 2 - p['backlash']               # tooth thickness at pitch line
    xt, xr = x_pitch - ha, x_pitch + hf         # tip, root
    pts = [(x_back, y0)]
    k = math.floor((y0 - phase) / pitch) - 1
    while phase + k * pitch < y1 + pitch:
        yc = phase + k * pitch
        dt = s / 2 - ha * math.tan(a); dr = s / 2 + hf * math.tan(a)
        for (x, y) in [(xr, yc - dr), (xt, yc - dt), (xt, yc + dt), (xr, yc + dr)]:
            if y0 < y < y1:
                pts.append((x, y))
        k += 1
    pts += [(xr, y1), (x_back, y1)]
    pts.insert(1, (xr, y0))
    return pts

# ------------------------------------------------------------------ parts
def derived(p):
    d = {}
    _, r, ra, rf = pinion_profile(p)
    d['R'], d['ra'], d['rf'] = r, ra, rf
    d['x_pitch'] = p['shaft_x'] + r
    d['travel'] = r * math.radians(p['travel_deg'])
    d['g_max'] = p['gap_min'] + d['travel']
    yf = p['fixed_face_y']
    face_lo, face_hi = yf + p['gap_min'], yf + d['g_max']
    rel_lo, rel_hi = p['shaft_y'] - face_hi, p['shaft_y'] - face_lo
    d['slider_back'] = math.ceil(-(rel_lo - 4.5))      # slider extends this far -y of face
    d['slider_fwd'] = math.ceil(rel_hi + 4.5)          # and this far +y of face
    d['y_lo'] = face_lo - d['slider_back'] - 1.0
    d['y_hi'] = face_hi + d['slider_fwd'] + 1.0
    return d

def slider_section(p, clear=0.0):
    """(x, z) cross-section of the slider body without teeth (dovetail on +x)."""
    c = clear
    return [(-0.75 - 0.0, p['base_t'] - c * 0), (p['slider_right_bot'] + c, p['base_t']),
            (p['slider_right_top'] + c, p['slider_top'] + c), (-0.75, p['slider_top'] + c)]

def make_head(p, d):
    b = p['base_t']; xl, xr = p['x_left'], p['wall_out']
    head = box(xl, d['y_lo'], 0, xr, d['y_hi'], b)
    # dovetail wall
    sc = p['slide_clear']
    wall = [(p['slider_right_bot'] + sc, 0), (xr, 0), (xr, p['slider_top'] + 0.1),
            (p['slider_right_top'] + sc, p['slider_top'] + 0.1)]
    head += ext_xz(wall, d['y_lo'], d['y_hi'])
    # fixed jaw: full-depth block, then keep-outs remove what must stay clear
    yf, t = p['fixed_face_y'], p['jaw_t']
    head += box(xl, yf - t, 0, xr, yf, p['jaw_tip_z'])
    # rear buttress behind fixed jaw (stiffens the datum jaw)
    but = [(yf - t - 4.0, b), (yf - t, b), (yf - t, 22.0)]
    head += ext_yz(but, xl, -16.9) + ext_yz(but, p['slider_right_bot'] + sc + 0.5, xr)
    # ---- keep-outs
    wx0, wy0, wx1, wy1 = p['window']
    head -= box(wx0 - 0.2, wy0 - 0.2, -1, wx1 + 0.2, wy1 + 0.2, p['servo_top'] + 0.7)
    chan = [(-3.6, b), (p['slider_right_bot'] + sc, b),
            (p['slider_right_top'] + sc, p['slider_top'] + 0.5), (-3.6, p['slider_top'] + 0.5)]
    head -= ext_xz(chan, d['y_lo'] - 1, d['y_hi'] + 1)
    head -= cyl(p['shaft_x'], p['shaft_y'], b, p['pinion_z0'] + p['pinion_b'] + 0.6, d['ra'] + 0.6, 96)
    for hx, hy in p['hole_xy']:                        # M2 countersunk
        head -= cyl(hx, hy, -1, 10, 1.15, 24)
        head -= cone(hx, hy, b - 1.0, b + 0.01, 1.15, 2.05, 24)
        head -= cyl(hx, hy, b, 60, 2.2, 24) if hy > yf else mf.Manifold()
    head = jaw_face(head, p, yf, +1)
    return head

def jaw_face(part, p, yface, sign):
    """V-groove along the jaw length + serrations near the tip, on face y=yface.
    sign=+1: face looks toward +y (fixed jaw); -1: toward -y (moving jaw)."""
    x0, x1 = p['jaw_x']; xc = (x0 + x1) / 2; dv = p['vgroove_depth']
    v = [(xc - dv, yface + sign * 0.01), (xc + dv, yface + sign * 0.01), (xc, yface - sign * dv)]
    if sign < 0:
        v = v[::-1]
    part -= ext_xy(v, p['jaw_bottom_z'] + 3, p['jaw_tip_z'] + 1)
    z = p['jaw_tip_z'] - 1.0
    while z > p['jaw_tip_z'] - p['serr_len']:
        tri = [(yface, z), (yface, z + p['serr_pitch'] / 2), (yface + sign * 1.0, z + p['serr_pitch'] / 4)]
        tri = [(yy - sign * p['serr_h'], zz) if i == 2 else (yy, zz) for i, (yy, zz) in enumerate(tri)]
        # grooves (cut) rather than ridges keep the face plane = datum
        g = [(yface + sign * 0.01, z), (yface + sign * 0.01, z + 0.9), (yface - sign * p['serr_h'], z + 0.45)]
        if sign < 0:
            g = g[::-1]
        part -= ext_yz(g if sign > 0 else g, x0 - 1, x1 + 1)
        z -= p['serr_pitch']
    return part

def make_slider(p, d, face):
    y0, y1 = face - d['slider_back'], face + d['slider_fwd']
    body = ext_xz(slider_section(p), y0, y1)
    # tooth phase fixed to the slider: with the jaw closed, a tooth SPACE sits
    # on the pinion centre line so the pinion (tooth at 0 deg) meshes.
    face0 = p['fixed_face_y'] + p['gap_min']
    phase = (p['shaft_y'] - math.pi * p['module'] / 2 - face0) % (math.pi * p['module'])
    teeth = ext_xy(rack_profile(p, y0, y1, d['x_pitch'], -0.5, face + phase),
                   p['base_t'], p['slider_top'])
    s = body + teeth
    s -= box(*MORTISE(p, face, 0.0))                               # jaw tenon socket
    s -= cyl(1.75, face + 9.5, p['base_t'] + 1.5, 20, 0.8, 16)     # M2 self-tap pilot
    return s

def MORTISE(p, face, c):
    """Socket in the slider (c=0) / tenon on the jaw (c=clearance)."""
    return (0.25 + c, face + 0.5 + c, 3.5 + c, 3.25 - c, face + 4.5 - c, p['slider_top'] + 0.01)

def make_jaw(p, d, face):
    x0, x1 = p['jaw_x']; t = p['jaw_t']; st = p['slider_top']
    j = box(x0, face, p['jaw_bottom_z'], x1, face + t, p['jaw_tip_z'])      # finger
    j += box(-0.75, face, st, x1, face + 12.0, st + 3.0)                    # foot on slider
    j += box(*MORTISE(p, face, 0.15)[:5], st + 0.5)                         # tenon
    j += ext_yz([(face + t, st + 3.0), (face + t + 2.5, st + 3.0),
                 (face + t, 24.0)], -0.75, x1)                              # gusset
    j -= cyl(1.75, face + 9.5, 0, 40, 1.15, 16)                             # M2 clearance
    j = jaw_face(j, p, face, -1)
    return j

def make_pinion(p, d):
    pts, r, ra, rf = pinion_profile(p)
    z0, z1 = p['pinion_z0'], p['pinion_z0'] + p['pinion_b']
    g = ext_xy(pts, z0, z1)
    g -= cyl(0, 0, z0 - 1, z0 + p['spline_depth'], p['spline_d'] / 2, 42)   # spline bore
    g -= cyl(0, 0, z0, z1 + 1, 1.15, 20)                                     # horn screw
    g -= cyl(0, 0, z1 - 1.3, z1 + 1, 2.1, 24)                                # screw head
    return g.translate((p['shaft_x'], p['shaft_y'], 0))

# ------------------------------------------------------------------ checks
def gap_for_angle(p, d, frac):
    return p['gap_min'] + frac * d['travel']

def build(p, frac):
    d = derived(p)
    face = p['fixed_face_y'] + gap_for_angle(p, d, frac)
    head = make_head(p, d)
    slider = make_slider(p, d, face)
    jaw = make_jaw(p, d, face)
    # rotate pinion with the rack so tooth phase stays meshed
    pin = make_pinion(p, d)
    ang = math.degrees((face - p['fixed_face_y'] - p['gap_min']) / d['R'])
    pin = pin.translate((-p['shaft_x'], -p['shaft_y'], 0)).rotate((0, 0, ang)).translate((p['shaft_x'], p['shaft_y'], 0))
    parts = dict(head=head, slider=slider, jaw=jaw, pinion=pin)
    for k, m in parts.items():
        assert m.status() == mf.Error.NoError and m.volume() > 100, f'{k} failed: {m.status()}'
    return d, parts

def interference(parts):
    names = list(parts); out = []
    for i in range(len(names)):
        for j in range(i + 1, len(names)):
            a, b = parts[names[i]], parts[names[j]]
            v = (a ^ b).volume()
            out.append((names[i], names[j], v))
    return out

def mirror_y(m, p):
    return m.mirror((0, 1, 0)).translate((0, 2 * p['axis_y'], 0))

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--out', default=os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'stl'))
    ap.add_argument('--shaft-low', action='store_true',
                    help='servo shaft is on the LOW-y side of the window: mirror the set')
    ap.add_argument('--check', action='store_true', help='interference sweep only')
    a = ap.parse_args()
    p = dict(P)
    d = derived(p)
    print(f"pinion m{p['module']} z{p['teeth']}: pitch R {d['R']:.2f} mm, tip R {d['ra']:.2f} mm")
    print(f"opening {p['gap_min']:.1f} .. {d['g_max']:.1f} mm over {p['travel_deg']:.0f} deg "
          f"({d['R']*math.pi/180:.3f} mm/deg); head y {d['y_lo']:.1f} .. {d['y_hi']:.1f}")
    worst = 0
    for frac in np.linspace(0, 1, 6):
        _, parts = build(p, frac)
        for n1, n2, v in interference(parts):
            if v > 1e-3:
                print(f"  frac {frac:.1f}: {n1} x {n2} overlap {v:.3f} mm^3"); worst = max(worst, v)
    print('interference sweep:', 'CLEAR' if worst <= 1e-3 else f'max overlap {worst:.3f} mm^3')
    if a.check:
        return
    os.makedirs(a.out, exist_ok=True)
    _, parts = build(p, 0.0)
    tag = '_shaftlow' if a.shaft_low else ''
    names = dict(head='GripperSpannerHead_v1', slider='GripperSpannerSlider_v1',
                 jaw='GripperSpannerJaw_v1', pinion='GripperSpannerPinion_v1')
    for k, m in parts.items():
        if k == 'pinion':
            m = make_pinion(p, d)
        if a.shaft_low and k != 'pinion':
            m = mirror_y(m, p)
        tm = to_trimesh(m)
        fn = os.path.join(a.out, names[k] + tag + '.stl')
        tm.export(fn)
        print(f"{os.path.basename(fn):40s} {tm.volume/1000:6.2f} cm^3  watertight={tm.is_watertight}  "
              f"bbox {np.round(tm.extents,1)}")

if __name__ == '__main__':
    main()
