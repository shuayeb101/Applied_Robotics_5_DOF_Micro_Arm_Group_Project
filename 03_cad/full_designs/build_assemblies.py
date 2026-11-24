"""Write each full design as an assembled model and a parts list.

For every design D0-D5 in arm_model.py:
  assemblies/<key>/<key>_assembly_home.stl  - all parts placed at the HOME pose
                                             (servos as SG90 envelopes)
  assemblies/<key>/<key>_assembly_home.3mf  - same, one named object per part
  assemblies/<key>/<key>_bom.csv            - every part: file, status, qty, mass

The printable part files are NOT copied: the BOM points at the single source of
each STL (00_baseline/3d_models, 03_cad/stl, 03_cad/mini_claw/stl).

Run after simulate_designs.py (uses its HOME pose):  python3 build_assemblies.py
"""
import csv
import json
import os
import sys

import trimesh

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import arm_model as am  # noqa: E402

OUT = os.path.join(am.HERE, "assemblies")


def main():
    for d in am.DESIGNS:
        d.build()
        res_file = os.path.join(am.HERE, "results", f"{d.key}_results.json")
        home = json.load(open(res_file))["home"] if os.path.exists(res_file) else None
        q = tuple(home) if home else (0, 90, -90, -45)
        T = am.link_transforms(q)
        folder = os.path.join(OUT, d.key)
        os.makedirs(folder, exist_ok=True)
        scene = trimesh.Scene()
        meshes = []
        for p in d.parts:
            m = p.mesh.copy()
            m.apply_transform(T[p.link])
            meshes.append(m)
            scene.add_geometry(m, node_name=p.name, geom_name=p.name)
        trimesh.util.concatenate(meshes).export(os.path.join(folder, f"{d.key}_assembly_home.stl"))
        scene.export(os.path.join(folder, f"{d.key}_assembly_home.3mf"))
        with open(os.path.join(folder, f"{d.key}_bom.csv"), "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["Part", "Moves with", "Status", "Qty", "Source file / item",
                        "Est. mass incl. fitted hardware (g)", "Print (Y/N)"])
            for p in d.parts:
                src = os.path.relpath(p.source, am.REPO) if os.path.exists(p.source) else p.source
                w.writerow([p.name, p.link, p.status, 1, src, f"{p.mass:.1f}",
                            "Y" if p.printed else "N"])
        print(d.key, d.title, "HOME", q, "->", folder)


if __name__ == "__main__":
    main()
