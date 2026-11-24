#!/usr/bin/env python3
"""
Rigid-body kinematic + static check of the spanner gripper over its full stroke.

The parts are built ONCE at the closed position and then moved as rigid bodies,
the way the real mechanism moves: the servo turns the pinion by theta, and the
slider + jaw translate by R*theta along y. At every step:

  1. interference  - overlap volume for every pair of parts (must be 0)
  2. engagement    - the pinion is turned both ways with the rack held still
                     until the teeth touch: finite in both directions means the
                     gear is meshed and can drive both ways; the sum is backlash
  3. rack margin   - distance from the pinion centre line to the ends of the rack
  4. capture       - the slider is nudged 0.4 mm up (+z) and sideways (+x):
                     it must hit the dovetail, i.e. it cannot fall out or twist
  5. statics       - SG90 stall and nominal grip loads -> tooth, jaw and joint stresses

Outputs (sim/): simulation_log.csv, simulation_report.md, simulation_plots.png,
spanner_open_close.gif.   Run:  python3 simulate.py
"""
import csv, math, os
import numpy as np
import manifold3d as mf
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation, PillowWriter
from shapely.geometry import Polygon
from shapely.ops import unary_union
import spanner_gripper as sg

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(HERE, 'sim'); os.makedirs(OUT, exist_ok=True)
P = dict(sg.P); D = sg.derived(P)
CX, CY, R = P['shaft_x'], P['shaft_y'], D['R']
EPS = 1e-3                                    # mm^3 treated as "touching"

# ------------------------------------------------------------------ build once
_, base = sg.build(P, 0.0)                    # closed position
HEAD, PIN0, SLIDER0, JAW0 = base['head'], base['pinion'], base['slider'], base['jaw']

def rot_pin(m, deg):
    return m.translate((-CX, -CY, 0)).rotate((0, 0, deg)).translate((CX, CY, 0))

def pose(theta_deg):
    """Rigid-body pose for a servo rotation theta (0 = closed)."""
    s = R * math.radians(theta_deg)
    return dict(head=HEAD, pinion=rot_pin(PIN0, theta_deg),
                slider=SLIDER0.translate((0, s, 0)), jaw=JAW0.translate((0, s, 0))), s

def ov(a, b):
    return (a ^ b).volume()

def play(slider, vec, lim=2.0):
    """How far the slider can move along vec before it touches the head."""
    lo, hi = 0.0, lim
    for _ in range(12):
        mid = (lo + hi) / 2
        if ov(slider.translate(tuple(v * mid for v in vec)), HEAD) < EPS: lo = mid
        else: hi = mid
    return lo

def free_rotation(pin, slider, sign, limit=4.0):
    """Degrees the pinion can turn (direction sign) before touching the rack."""
    if ov(rot_pin(pin, sign * limit), slider) < EPS and \
       ov(rot_pin(pin, sign * limit / 2), slider) < EPS:
        return math.inf                        # never touches -> NOT meshed
    lo, hi = 0.0, limit
    for _ in range(12):
        mid = (lo + hi) / 2
        if ov(rot_pin(pin, sign * mid), slider) < EPS: lo = mid
        else: hi = mid
    return lo

# ------------------------------------------------------------------ sweep
def sweep(step=1.0):
    rows = []
    thetas = np.arange(0, P['travel_deg'] + 1e-9, step)
    y0 = SLIDER0.bounding_box()[1]; y1 = SLIDER0.bounding_box()[4]
    hy0, hy1 = HEAD.bounding_box()[1], HEAD.bounding_box()[4]
    for th in thetas:
        parts, s = pose(th)
        names = list(parts)
        pairs = {f'{a}-{b}': ov(parts[a], parts[b]) for i, a in enumerate(names) for b in names[i + 1:]}
        ccw = free_rotation(parts['pinion'], parts['slider'], +1)
        cw = free_rotation(parts['pinion'], parts['slider'], -1)
        sl = parts['slider']
        up = play(sl, (0, 0, 1))
        side = play(sl, (1, 0, 0))
        rows.append(dict(
            theta_deg=th, gap_mm=P['gap_min'] + s,
            max_overlap_mm3=max(pairs.values()), worst_pair=max(pairs, key=pairs.get),
            free_ccw_deg=ccw, free_cw_deg=cw,
            backlash_mm=R * math.radians(ccw + cw) if math.isfinite(ccw + cw) else math.inf,
            rack_margin_lo_mm=CY - (y0 + s), rack_margin_hi_mm=(y1 + s) - CY,
            slider_in_channel_mm=min(y1 + s, hy1) - max(y0 + s, hy0),
            lift_play_mm=up, side_play_mm=side))
    return rows

# ------------------------------------------------------------------ statics
def section_modulus(m, z, ylim=None):
    """Bending section modulus about the x axis (load along y) of slice z."""
    polys = [Polygon(q) for q in m.slice(z).to_polygons()]
    poly = unary_union([p for p in polys if p.is_valid])
    if ylim:
        poly = poly.intersection(Polygon([(-99, ylim[0]), (99, ylim[0]), (99, ylim[1]), (-99, ylim[1])]))
    # second moment about the centroidal x axis by triangulated shoelace
    from shapely.geometry import MultiPolygon
    geoms = poly.geoms if isinstance(poly, MultiPolygon) else [poly]
    A = Iy = Sy = 0.0
    for g in geoms:
        rings = [(np.asarray(g.exterior.coords), 1)] + [(np.asarray(i.coords), -1) for i in g.interiors]
        for c, sgn in rings:
            x, y = c[:-1, 0], c[:-1, 1]; x2, y2 = c[1:, 0], c[1:, 1]
            cr = x * y2 - x2 * y
            if (cr.sum() < 0) == (sgn > 0): cr = -cr   # orient: exterior +, holes -
            A += cr.sum() / 2; Sy += ((y + y2) * cr).sum() / 6
            Iy += ((y * y + y * y2 + y2 * y2) * cr).sum() / 12
    yc = Sy / A; I = Iy - A * yc * yc
    b = poly.bounds; c = max(yc - b[1], b[3] - yc)
    return A, I, I / c

def statics():
    T_stall = 0.177                 # N m, SG90 at 4.8 V (datasheet 1.8 kgf cm)
    mu = 0.3                        # PLA on PLA, dry
    alpha = math.radians(P['pressure'])
    out = {}
    for label, T in (('stall', T_stall), ('nominal (5 N squeeze)', 0.005 * R)):
        Ft = T / (R / 1000)                         # tangential force at pitch line
        Fr = Ft * math.tan(alpha)                   # separating force -> dovetail wall
        Ffric = mu * Fr
        F = Ft - Ffric                              # grip force at the jaw
        z_c = (P['jaw_tip_z'] + P['jaw_bottom_z'] + 3) / 2   # mid-finger contact height
        # Lewis tooth bending (rack tooth is stronger than pinion tooth; check pinion)
        Y = 0.295                                    # Lewis form factor z=16
        s_tooth = Ft / (P['pinion_b'] * P['module'] * Y)
        # moving jaw root (z just above the foot), and fixed jaw root (z just above channel)
        zj = P['slider_top'] + 3.2
        _, _, Zj = section_modulus(JAW0, zj)
        s_jaw = F * (z_c - zj) / Zj
        zf = 9.5
        yf = P['fixed_face_y']
        _, _, Zf = section_modulus(HEAD, zf, (yf - P['jaw_t'] - 4.5, yf + 0.01))
        s_fix = F * (z_c - zf) / Zf
        # jaw-to-slider joint: moment about the back edge of the foot is reacted by
        # the tenon bearing on the mortise wall over ~ the tenon depth
        M = F * (z_c - P['slider_top'])
        depth = P['slider_top'] - 3.65
        Fb = M / (depth * 2 / 3)
        s_bear = Fb / (2.7 * depth / 2)
        out[label] = dict(T_Nm=T, tangential_N=Ft, separating_N=Fr, friction_N=Ffric, grip_N=F,
                          tooth_MPa=s_tooth, moving_jaw_root_MPa=s_jaw, fixed_jaw_root_MPa=s_fix,
                          tenon_bearing_MPa=s_bear, contact_z=z_c)
    # transverse contact ratio, pinion with rack
    m = P['module']; ra = D['ra']; rb = R * math.cos(alpha)
    eps = (math.sqrt(ra ** 2 - rb ** 2) - R * math.sin(alpha) + m / math.sin(alpha)) / (math.pi * m * math.cos(alpha))
    return out, eps

# ------------------------------------------------------------------ animation
def slice_polys(m, plane, val):
    if plane == 'z':
        return [(q[:, 0], q[:, 1]) for q in m.slice(val).to_polygons()]
    # plane x = val, plotted as (y, z)
    mm = m.rotate((0, 90, 0))                          # Ry(90): (x,y,z)->(z,y,-x)
    return [(q[:, 1], q[:, 0]) for q in mm.slice(-val).to_polygons()]

COL = dict(head='#3f6fb5', slider='#e0892b', jaw='#c46a12', pinion='#4aa36c')

def animate(rows, n=36):
    th_seq = list(np.linspace(0, P['travel_deg'], n)) + list(np.linspace(P['travel_deg'], 0, n))
    head_plan = slice_polys(HEAD, 'z', 5.0); head_side = slice_polys(HEAD, 'x', -10.0)
    fig, (a1, a2) = plt.subplots(1, 2, figsize=(12, 6.2))
    def frame(i):
        th = th_seq[i]; parts, s = pose(th)
        for ax in (a1, a2): ax.clear(); ax.set_aspect('equal')
        for q in head_plan: a1.fill(*q, color=COL['head'], alpha=.5, ec='k', lw=.6)
        for k in ('slider', 'pinion'):
            for q in slice_polys(parts[k], 'z', 5.0): a1.fill(*q, color=COL[k], alpha=.7, ec='k', lw=.6)
        a1.plot([CX, CX + 7 * math.cos(math.radians(th))], [CY, CY + 7 * math.sin(math.radians(th))], 'k-', lw=2)
        a1.set_xlim(-22, 11); a1.set_ylim(-4, 62); a1.set_title('Plan section z = 5 mm (pinion + rack + dovetail)', fontsize=10)
        for q in head_side: a2.fill(*q, color=COL['head'], alpha=.5, ec='k', lw=.6)
        for k in ('pinion', 'jaw'):
            for q in slice_polys(parts[k], 'x', -10.0): a2.fill(*q, color=COL[k], alpha=.7, ec='k', lw=.6)
        a2.set_xlim(-4, 62); a2.set_ylim(-2, 40); a2.set_xlabel('y (mm)'); a2.set_ylabel('z (mm)')
        a2.set_title('Side section x = -10 mm (jaws over the pinion)', fontsize=10)
        g = P['gap_min'] + s
        fig.suptitle(f'servo {th:5.1f} deg   gap {g:4.1f} mm   ' + ('opening' if i < n else 'closing'), fontsize=12)
    anim = FuncAnimation(fig, frame, frames=len(th_seq))
    fn = os.path.join(OUT, 'spanner_open_close.gif'); anim.save(fn, writer=PillowWriter(fps=12), dpi=70)
    plt.close(fig); return fn

# ------------------------------------------------------------------ main
def main():
    rows = sweep(1.0)
    with open(os.path.join(OUT, 'simulation_log.csv'), 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0])); w.writeheader()
        for r in rows: w.writerow({k: (round(v, 4) if isinstance(v, float) else v) for k, v in r.items()})
    st, eps = statics()

    th = np.array([r['theta_deg'] for r in rows]); gap = np.array([r['gap_mm'] for r in rows])
    bl = np.array([r['backlash_mm'] for r in rows]); ovl = np.array([r['max_overlap_mm3'] for r in rows])
    mlo = np.array([r['rack_margin_lo_mm'] for r in rows]); mhi = np.array([r['rack_margin_hi_mm'] for r in rows])
    fig, ax = plt.subplots(2, 2, figsize=(12, 8))
    ax[0, 0].plot(th, gap); ax[0, 0].set_title('Jaw gap vs servo angle'); ax[0, 0].set_ylabel('mm')
    ax[0, 1].plot(th, bl, 'o-', ms=2); ax[0, 1].set_title('Backlash at the jaw (pinion free play x R)'); ax[0, 1].set_ylabel('mm'); ax[0, 1].set_ylim(0, 0.5)
    ax[1, 0].plot(th, ovl, 'r'); ax[1, 0].set_title('Worst overlap between any two parts (limit 0.001 mm³)'); ax[1, 0].set_ylabel('mm³')
    ax[1, 0].set_ylim(-0.0002, 0.002); ax[1, 0].axhline(EPS, color='grey', ls='--')
    ax[1, 1].plot(th, mlo, label='rack end behind pinion'); ax[1, 1].plot(th, mhi, label='rack end ahead of pinion')
    ax[1, 1].axhline(D['ra'] * 0.5, color='grey', ls='--', label='min needed (~4.5 mm)'); ax[1, 1].legend(fontsize=8)
    ax[1, 1].set_title('Rack engagement margin'); ax[1, 1].set_ylabel('mm')
    for a in ax.flat: a.set_xlabel('servo angle from closed (deg)'); a.grid(alpha=.3)
    plt.tight_layout(); plt.savefig(os.path.join(OUT, 'simulation_plots.png'), dpi=100); plt.close()
    gif = animate(rows)

    ok = lambda b: 'PASS' if b else 'FAIL'
    checks = [
        ('No part collides with another at any step', ovl.max() < EPS, f'max overlap {ovl.max():.4f} mm³'),
        ('Pinion stays meshed (finite free play both ways) at every step', np.isfinite(bl).all(),
         f'{np.isfinite(bl).sum()}/{len(bl)} steps meshed'),
        ('Backlash at the jaw <= 0.5 mm', np.nanmax(bl[np.isfinite(bl)]) <= 0.5,
         f'{bl[np.isfinite(bl)].min():.3f} – {bl[np.isfinite(bl)].max():.3f} mm'),
        ('Rack extends >= 4.5 mm past the pinion both sides', min(mlo.min(), mhi.min()) >= 4.5,
         f'min {min(mlo.min(), mhi.min()):.2f} mm'),
        ('Slider lift play (jaw rattle) <= 0.4 mm at every step', max(r['lift_play_mm'] for r in rows) <= 0.4,
         f"{min(r['lift_play_mm'] for r in rows):.2f} – {max(r['lift_play_mm'] for r in rows):.2f} mm"),
        ('Slider side play (away from pinion) <= 0.3 mm', max(r['side_play_mm'] for r in rows) <= 0.3,
         f"{max(r['side_play_mm'] for r in rows):.2f} mm"),
        ('Slider stays >= 20 mm inside the channel', min(r['slider_in_channel_mm'] for r in rows) >= 20,
         f"min {min(r['slider_in_channel_mm'] for r in rows):.1f} mm"),
        ('Contact ratio >= 1.2 (smooth tooth hand-over)', eps >= 1.2, f'{eps:.2f}'),
    ]
    allow_in, allow_across = 30.0, 15.0     # MPa, conservative printed PLA (in-layer / across layers)
    s = st['stall']; n = st['nominal (5 N squeeze)']
    checks += [
        ('Pinion tooth bending at stall < 30 MPa', s['tooth_MPa'] < allow_in, f"{s['tooth_MPa']:.1f} MPa"),
        ('Moving jaw root at stall < 30 MPa (printed face-down, stress in-layer)', s['moving_jaw_root_MPa'] < allow_in,
         f"{s['moving_jaw_root_MPa']:.1f} MPa"),
        ('Fixed jaw root at stall < 15 MPa (printed upright, stress across layers)', s['fixed_jaw_root_MPa'] < allow_across,
         f"{s['fixed_jaw_root_MPa']:.1f} MPa"),
        ('Tenon bearing at stall < 30 MPa', s['tenon_bearing_MPa'] < allow_in, f"{s['tenon_bearing_MPa']:.1f} MPa"),
    ]
    lines = ['# Spanner gripper – simulation report', '',
             f'Rigid-body sweep, {len(rows)} steps of 1° from 0 to {P["travel_deg"]:.0f}° servo travel '
             f'(gap {gap.min():.1f} → {gap.max():.1f} mm). Generated by `simulate.py`.', '',
             '| Check | Result | Value |', '| --- | --- | --- |']
    lines += [f'| {c} | **{ok(b)}** | {v} |' for c, b, v in checks]
    lines += ['', f"Overall: **{'ALL CHECKS PASS' if all(b for _, b, _ in checks) else 'SOME CHECKS FAIL'}**"]
    lines += ['', '## Loads', '', '| Quantity | Stall (0.177 N·m) | Nominal 5 N squeeze |', '| --- | --- | --- |']
    for k in ('tangential_N', 'separating_N', 'friction_N', 'grip_N', 'tooth_MPa', 'moving_jaw_root_MPa',
              'fixed_jaw_root_MPa', 'tenon_bearing_MPa'):
        lines.append(f'| {k} | {s[k]:.1f} | {n[k]:.2f} |')
    lines += ['', f'Contact point assumed at mid-finger, z = {s["contact_z"]:.1f} mm. Friction μ = 0.3 on the dovetail. '
              'Allowables are conservative for printed PLA, not measured.', '',
              '## Limits of this simulation', '',
              '- Rigid bodies with nominal CAD geometry. Print tolerance, elephant\'s-foot, part deflection and servo deadband are not modelled.',
              '- Statics use hand formulas (Lewis, beam bending, simple bearing). This is not FEA.',
              '- SG90 dimensions, shaft position and torque come from the datasheet, not from your servo.',
              '- A pass here means the design is geometrically sound. It still needs physical trials (GRIP-01..05).']
    open(os.path.join(OUT, 'simulation_report.md'), 'w').write('\n'.join(lines) + '\n')
    print('\n'.join(lines)); print('\n' + gif)

if __name__ == '__main__':
    main()
