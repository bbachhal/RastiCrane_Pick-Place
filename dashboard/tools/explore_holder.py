"""Exploratory: what does the stamp holder's top face look like around its arc?

Prints a radius-vs-angle profile so we can see whether the stamp pockets show up as
notches we can locate automatically.
"""
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from read_cad import load_glb  # noqa: E402
from fit_holder_arc import collect, fit_circle  # noqa: E402

PATH = sys.argv[1] if len(sys.argv) > 1 else r"C:\Users\bachh\Downloads\Part Studio 1.glb"
gltf, binary = load_glb(PATH)
parts = collect(gltf, binary, {"Stamp holder"})
holder = parts["Stamp holder"]

z = holder[:, 2]
print(f"holder vertices: {len(holder)}   z range {z.min():.1f} .. {z.max():.1f} mm")
levels, counts = np.unique(np.round(z, 1), return_counts=True)
busy = sorted(zip(counts, levels), reverse=True)[:8]
print("busiest z levels (count, z):", [(int(c), float(l)) for c, l in busy])

top = holder[z > z.max() - 1.0]
cx, cy, radius, rms = fit_circle(top[:, :2])
print(f"top face: {len(top)} verts | circle fit centre ({cx:.1f},{cy:.1f}) r={radius:.1f} rms={rms:.2f}")

angle = np.degrees(np.arctan2(top[:, 1] - cy, top[:, 0] - cx))
rad = np.hypot(top[:, 0] - cx, top[:, 1] - cy)
order = np.argsort(angle)
angle, rad = angle[order], rad[order]

print("\nangle(deg) -> radius span of vertices in that 2-degree bin")
for start in range(int(np.floor(angle.min())), int(np.ceil(angle.max())), 2):
    mask = (angle >= start) & (angle < start + 2)
    if not mask.any():
        continue
    bin_r = rad[mask]
    bar = "#" * int(np.clip((bin_r.max() - bin_r.min()) / 4, 0, 40))
    print(f"{start:6d}  n={mask.sum():4d}  r {bin_r.min():6.1f}..{bin_r.max():6.1f}  {bar}")
