"""Find the stamp pockets in the holder and write them into cad_geometry.json.

The pockets show up as clusters of vertices on the shelf level inside the arc. Each cluster
becomes one slot with a centre, a radius and an azimuth measured from the robot base, which
is the CAD origin (base_table_bottom).
"""
import json
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from read_cad import load_glb  # noqa: E402
from fit_holder_arc import collect, fit_circle  # noqa: E402

PATH = sys.argv[1] if len(sys.argv) > 1 else r"C:\Users\bachh\Downloads\Part Studio 1.glb"
SHELF_Z = float(sys.argv[2]) if len(sys.argv) > 2 else 22.5
GAP_DEG = float(sys.argv[3]) if len(sys.argv) > 3 else 4.0   # gap that separates pockets

gltf, binary = load_glb(PATH)
holder = collect(gltf, binary, {"Stamp holder"})["Stamp holder"]

top = holder[holder[:, 2] > holder[:, 2].max() - 1.0]
cx, cy, arc_radius, rms = fit_circle(top[:, :2])

shelf = holder[np.abs(holder[:, 2] - SHELF_Z) < 0.6]
print(f"shelf level z={SHELF_Z}: {len(shelf)} vertices")
if len(shelf) < 20:
    print("Not enough vertices at that level; try another z from the busiest list.")
    sys.exit(1)

angle = np.degrees(np.arctan2(shelf[:, 1] - cy, shelf[:, 0] - cx))
radius = np.hypot(shelf[:, 0] - cx, shelf[:, 1] - cy)
order = np.argsort(angle)
angle, radius, shelf = angle[order], radius[order], shelf[order]

# Split into clusters wherever the angle jumps by more than GAP_DEG.
breaks = np.where(np.diff(angle) > GAP_DEG)[0]
groups = np.split(np.arange(len(angle)), breaks + 1)
groups = [g for g in groups if len(g) >= 6]

print(f"arc centre ({cx:.1f},{cy:.1f}) r={arc_radius:.1f}; found {len(groups)} clusters\n")
slots = []
for i, group in enumerate(groups):
    pts = shelf[group]
    centre = pts.mean(axis=0)
    span = (float(angle[group].min()), float(angle[group].max()))
    from_base = np.hypot(centre[0], centre[1])
    azimuth = np.degrees(np.arctan2(centre[1], centre[0]))
    arc_angle = float(np.mean(angle[group]))
    slots.append({
        "index": i,
        "center_mm": [round(float(centre[0]), 2), round(float(centre[1]), 2), round(float(centre[2]), 2)],
        "radiusFromBase_mm": round(float(from_base), 2),
        "azimuthFromBase_deg": round(float(azimuth), 2),
        "angleOnArc_deg": round(arc_angle, 2),
        "arcSpan_deg": [round(span[0], 2), round(span[1], 2)],
        "vertices": int(len(group)),
    })
    print(f"slot {i}: centre ({centre[0]:7.1f},{centre[1]:7.1f},{centre[2]:5.1f})  "
          f"from base r={from_base:6.1f} az={azimuth:7.2f}deg  "
          f"arc {span[0]:6.1f}..{span[1]:6.1f}  n={len(group)}")

if len(slots) > 1:
    spacing = np.diff([s["angleOnArc_deg"] for s in slots])
    print(f"\nangular spacing between clusters: {np.round(spacing,2).tolist()}")

target = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "cad_geometry.json"))
geometry = json.load(open(target, encoding="utf-8")) if os.path.isfile(target) else {}
geometry["holderSlots"] = {
    "shelfZ_mm": SHELF_Z,
    "arcCenter_mm": [round(cx, 2), round(cy, 2)],
    "arcRadius_mm": round(arc_radius, 2),
    "slots": slots,
    "note": ("Clusters of holder-shelf vertices, taken as stamp pockets. Positions are in the CAD "
             "frame whose origin is the robot base plate. Confirm against a photo before trusting."),
}
json.dump(geometry, open(target, "w", encoding="utf-8"), indent=2)
print(f"\nwrote {target}")
