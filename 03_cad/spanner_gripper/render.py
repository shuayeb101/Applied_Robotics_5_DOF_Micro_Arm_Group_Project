#!/usr/bin/env python3
"""Render the spanner gripper assembly (closed / open / exploded) to PNG.
Usage: python3 render.py  -> renders/*.png   (needs matplotlib)"""
import os, math, numpy as np, trimesh, matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import spanner_gripper as sg

HERE = os.path.dirname(os.path.abspath(__file__))
PLATE = os.path.join(HERE, '..', '..', '00_baseline', '3d_models', 'FrontPlate.stl')
COL = dict(plate='#b9bec7', head='#3f6fb5', slider='#e0892b', jaw='#e0892b', pinion='#4aa36c')

def draw(ax, meshes, elev, azim, title):
    lo, hi = np.full(3, 1e9), np.full(3, -1e9)
    light = np.array([0.3, -0.5, 0.8]); light /= np.linalg.norm(light)
    for name, tm in meshes:
        shade = 0.45 + 0.55 * np.clip(tm.face_normals @ light, 0, 1)
        base = np.array(matplotlib.colors.to_rgb(COL[name]))
        pc = Poly3DCollection(tm.triangles, facecolors=np.clip(base[None] * shade[:, None], 0, 1),
                              edgecolor='none')
        ax.add_collection3d(pc)
        lo = np.minimum(lo, tm.bounds[0]); hi = np.maximum(hi, tm.bounds[1])
    c = (lo + hi) / 2; r = (hi - lo).max() / 2
    ax.set_xlim(c[0] - r, c[0] + r); ax.set_ylim(c[1] - r, c[1] + r); ax.set_zlim(c[2] - r, c[2] + r)
    ax.set_box_aspect((1, 1, 1)); ax.view_init(elev, azim); ax.set_axis_off(); ax.set_title(title, fontsize=11)

def scene(frac, explode=0.0):
    p = dict(sg.P); d, parts = sg.build(p, frac)
    out = []
    plate = trimesh.load(PLATE); plate.apply_translation((0, 0, -2 - 2 * explode))
    out.append(('plate', plate))
    off = dict(head=0, slider=1, jaw=2.2, pinion=0.5)
    for k in ('head', 'pinion', 'slider', 'jaw'):
        tm = sg.to_trimesh(parts[k]); tm.apply_translation((0, 0, off[k] * explode * 12))
        out.append((k, tm))
    return p, d, out

def main():
    os.makedirs(os.path.join(HERE, 'renders'), exist_ok=True)
    p, d, _ = scene(0)
    fig = plt.figure(figsize=(15, 10))
    views = [(0, 'closed', 25, -125), (1, 'fully open', 25, -125), (0.45, 'view from the slider side', 22, 35),
             (0.45, 'exploded', 18, 40)]
    for i, (frac, name, el, az) in enumerate(views):
        ax = fig.add_subplot(2, 2, i + 1, projection='3d')
        _, _, ms = scene(frac, explode=1.0 if name == 'exploded' else 0)
        g = p['gap_min'] + frac * d['travel']
        draw(ax, ms, el, az, f"{name}  (gap {g:.1f} mm)" if name != 'exploded' else name)
    fig.suptitle('Spanner-inspired gripper: fixed datum jaw (blue) + sliding jaw (orange) on pinion (green)', fontsize=13)
    plt.tight_layout(); fn = os.path.join(HERE, 'renders', 'spanner_gripper_views.png'); plt.savefig(fn, dpi=110)
    print(fn)
    sections(p, d)

def sections(p, d):
    fig, axs = plt.subplots(1, 2, figsize=(15, 7.5))
    for ax, frac in ((axs[0], 0.45),):
        _, parts = sg.build(p, frac)
        for k in ('head', 'slider', 'pinion'):
            for i, q in enumerate(parts[k].slice(5.0).to_polygons()):
                q = np.vstack([q, q[:1]])
                ax.fill(q[:, 0], q[:, 1], color=COL[k], alpha=0.55, ec='k', lw=0.8, label=k if i == 0 else None)
        ax.add_patch(plt.Rectangle((-20, 0), 20, 43, fill=False, ls='--', ec='grey', label='original FrontPlate outline'))
        ax.axhline(p['axis_y'], color='grey', lw=0.6, ls=':'); ax.text(-19.5, p['axis_y'] + 0.5, 'arm axis', fontsize=8, color='grey')
        ax.set_title('Plan section at z = 5 mm: pinion drives the rack;\ndovetail wall (right) holds the slider down and square', fontsize=11)
        ax.set_aspect('equal'); ax.set_xlabel('x (mm)'); ax.set_ylabel('y (mm) - opening direction'); ax.legend(loc='lower left', fontsize=8)
    # end section through slider/jaw, plane y = jaw foot
    ax = axs[1]; _, parts = sg.build(p, 0.45)
    face = p['fixed_face_y'] + p['gap_min'] + 0.45 * d['travel']
    for k in ('head', 'slider', 'jaw', 'pinion'):
        polys = parts[k].rotate((90, 0, 0)).slice(face + 6.0).to_polygons()   # Rx(90): (x,y,z)->(x,-z,y)
        for i, q in enumerate(polys):
            q = np.vstack([q, q[:1]])
            ax.fill(q[:, 0], -q[:, 1], color=COL[k], alpha=0.55, ec='k', lw=0.8, label=k if i == 0 else None)
    ax.set_title(f'Cross-section across the slide at y = {face + 6:.1f} mm:\njaw finger bridges over the pinion; dovetail on the right', fontsize=11)
    ax.set_aspect('equal'); ax.set_xlabel('x (mm)'); ax.set_ylabel('z (mm) - reach away from arm'); ax.legend(fontsize=8)
    plt.tight_layout(); fn = os.path.join(HERE, 'renders', 'spanner_gripper_sections.png'); plt.savefig(fn, dpi=110); print(fn)

if __name__ == '__main__':
    main()
