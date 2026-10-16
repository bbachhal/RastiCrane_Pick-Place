"""Fit the stamp holder's arc from the CAD mesh, in the robot-base frame.

The CAD origin is base_table_bottom, i.e. the robot's own base plate, so every number
printed here is already measured from the robot. Writes cad_geometry.json for the dashboard.
"""
import json
import os
import struct
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from read_cad import load_glb, node_matrix  # noqa: E402

COMPONENT = {5126: "f", 5123: "H", 5125: "I", 5121: "B", 5120: "b", 5122: "h"}
COUNT = {"SCALAR": 1, "VEC2": 2, "VEC3": 3, "VEC4": 4}


def read_accessor(gltf, binary, index):
    accessor = gltf["accessors"][index]
    view = gltf["bufferViews"][accessor["bufferView"]]
    start = view.get("byteOffset", 0) + accessor.get("byteOffset", 0)
    items = COUNT[accessor["type"]]
    fmt = COMPONENT[accessor["componentType"]]
    size = struct.calcsize("<" + fmt)
    stride = view.get("byteStride") or size * items
    out = np.empty((accessor["count"], items), dtype=float)
    for i in range(accessor["count"]):
        offset = start + i * stride
        out[i] = struct.unpack_from("<" + fmt * items, binary, offset)
    return out


def collect(gltf, binary, wanted):
    """World-space vertices (mm) for every mesh whose node name matches `wanted`."""
    found = {name: [] for name in wanted}

    def walk(index, parent):
        node = gltf["nodes"][index]
        world = parent @ node_matrix(node)
        name = node.get("name", "")
        if "mesh" in node and name in found:
            for primitive in gltf["meshes"][node["mesh"]].get("primitives", []):
                position = primitive["attributes"].get("POSITION")
                if position is None:
                    continue
                local = read_accessor(gltf, binary, position)
                homogeneous = np.hstack([local, np.ones((len(local), 1))])
                found[name].append((world @ homogeneous.T).T[:, :3] * 1000.0)
        for child in node.get("children", []):
            walk(child, world)

    scene = gltf.get("scenes", [{}])[gltf.get("scene", 0)]
    for root in scene.get("nodes", []):
        walk(root, np.eye(4))
    return {k: (np.vstack(v) if v else None) for k, v in found.items()}


def fit_circle(points_xy):
    """Algebraic circle fit: solves for centre and radius in one least-squares step."""
    x, y = points_xy[:, 0], points_xy[:, 1]
    design = np.column_stack([x, y, np.ones(len(x))])
    solution, *_ = np.linalg.lstsq(design, x ** 2 + y ** 2, rcond=None)
    cx, cy = solution[0] / 2, solution[1] / 2
    radius = float(np.sqrt(solution[2] + cx ** 2 + cy ** 2))
    residual = np.hypot(x - cx, y - cy) - radius
    return float(cx), float(cy), radius, float(np.sqrt(np.mean(residual ** 2)))


def main(path):
    gltf, binary = load_glb(path)
    parts = collect(gltf, binary, {"Stamp holder", "base_table_bottom", "node5", "8 1/2 x 11\" paper"})

    holder = parts["Stamp holder"]
    if holder is None:
        print("No 'Stamp holder' mesh found.")
        return
    top_z = holder[:, 2].max()
    rim = holder[holder[:, 2] > top_z - 2.0][:, :2]        # vertices on the top face
    cx, cy, radius, rms = fit_circle(rim)

    angles = np.degrees(np.arctan2(rim[:, 1] - cy, rim[:, 0] - cx))
    angles = np.sort(angles)
    centre_azimuth = float(np.degrees(np.arctan2(cy, cx)))
    centre_distance = float(np.hypot(cx, cy))

    print(f"Stamp holder top face: {len(rim)} vertices at z ~ {top_z:.1f} mm")
    print(f"  arc centre (robot frame): ({cx:.1f}, {cy:.1f}) mm   radius {radius:.1f} mm   fit RMS {rms:.2f} mm")
    print(f"  arc centre is {centre_distance:.1f} mm from the robot base, azimuth {centre_azimuth:.2f} deg")
    print(f"  holder spans azimuth {angles.min():.1f} to {angles.max():.1f} deg about its own arc centre")

    bbox = {}
    for name, cloud in parts.items():
        if cloud is None:
            continue
        lo, hi = cloud.min(axis=0), cloud.max(axis=0)
        bbox[name] = {"min": lo.round(2).tolist(), "max": hi.round(2).tolist(),
                      "center": ((lo + hi) / 2).round(2).tolist()}
        centre = (lo + hi) / 2
        print(f"  {name:28s} centre ({centre[0]:7.1f},{centre[1]:7.1f},{centre[2]:6.1f})  "
              f"radius {np.hypot(centre[0], centre[1]):6.1f}  azimuth {np.degrees(np.arctan2(centre[1], centre[0])):7.2f} deg")

    out = {
        "source": os.path.basename(path),
        "frame": "CAD origin = base_table_bottom centre = robot base; +Z up; mm",
        "holderArc": {"center_mm": [round(cx, 2), round(cy, 2)], "radius_mm": round(radius, 2),
                      "fitRms_mm": round(rms, 3), "topZ_mm": round(float(top_z), 2),
                      "centerAzimuth_deg": round(centre_azimuth, 3),
                      "centerDistance_mm": round(centre_distance, 2)},
        "parts": bbox,
    }
    target = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "cad_geometry.json")
    with open(os.path.normpath(target), "w", encoding="utf-8") as handle:
        json.dump(out, handle, indent=2)
    print(f"\nwrote {os.path.normpath(target)}")


if __name__ == "__main__":
    main(sys.argv[1])
