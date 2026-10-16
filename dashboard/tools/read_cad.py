"""Walk a .glb scene graph and report each part's world-space bounding box in mm.

glTF stores mesh vertex extents in mesh-local space, so a bbox taken straight from the
accessors is meaningless for layout questions. This applies every node transform down the
tree, which is what makes "where is the stamp holder relative to the base" answerable.
"""
import json
import struct
import sys

import numpy as np


def load_glb(path):
    data = open(path, "rb").read()
    _, _, _ = struct.unpack("<III", data[:12])
    length, _ = struct.unpack("<II", data[12:20])
    gltf = json.loads(data[20:20 + length].decode("utf-8"))
    rest = data[20 + length:]
    buffers = []
    offset = 0
    while offset + 8 <= len(rest):
        chunk_len, chunk_type = struct.unpack("<II", rest[offset:offset + 8])
        payload = rest[offset + 8:offset + 8 + chunk_len]
        if chunk_type == 0x004E4942:  # BIN
            buffers.append(payload)
        offset += 8 + chunk_len
    return gltf, (buffers[0] if buffers else b"")


def node_matrix(node):
    if "matrix" in node:
        return np.array(node["matrix"], dtype=float).reshape(4, 4).T
    matrix = np.eye(4)
    if "scale" in node:
        matrix = matrix @ np.diag([*node["scale"], 1.0])
    if "rotation" in node:
        x, y, z, w = node["rotation"]
        rot = np.array([
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w), 0],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w), 0],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y), 0],
            [0, 0, 0, 1]])
        matrix = rot @ matrix
    if "translation" in node:
        trans = np.eye(4)
        trans[:3, 3] = node["translation"]
        matrix = trans @ matrix
    return matrix


def mesh_corners(gltf, mesh_index):
    """Eight corners of every primitive's local bbox, from accessor min/max."""
    corners = []
    for primitive in gltf["meshes"][mesh_index].get("primitives", []):
        index = primitive["attributes"].get("POSITION")
        if index is None:
            continue
        accessor = gltf["accessors"][index]
        if "min" not in accessor or "max" not in accessor:
            continue
        lo, hi = accessor["min"], accessor["max"]
        for bit in range(8):
            corners.append([
                hi[0] if bit & 1 else lo[0],
                hi[1] if bit & 2 else lo[1],
                hi[2] if bit & 4 else lo[2]])
    return np.array(corners, dtype=float) if corners else None


def walk(gltf, node_index, parent, out, path=""):
    node = gltf["nodes"][node_index]
    world = parent @ node_matrix(node)
    name = node.get("name", f"node{node_index}")
    full = f"{path}/{name}" if path else name
    if "mesh" in node:
        local = mesh_corners(gltf, node["mesh"])
        if local is not None:
            homogeneous = np.hstack([local, np.ones((len(local), 1))])
            world_points = (world @ homogeneous.T).T[:, :3]
            out.append({
                "name": name, "path": full,
                "min": world_points.min(axis=0), "max": world_points.max(axis=0),
                "center": (world_points.min(axis=0) + world_points.max(axis=0)) / 2})
    for child in node.get("children", []):
        walk(gltf, child, world, out, full)


def report(path, scale_to_mm=1000.0):
    gltf, _ = load_glb(path)
    parts = []
    scene = gltf.get("scenes", [{}])[gltf.get("scene", 0)]
    for root in scene.get("nodes", range(len(gltf.get("nodes", [])))):
        walk(gltf, root, np.eye(4), parts)

    print(f"\n=== {path.split(chr(92))[-1]} — {len(parts)} meshes placed ===")
    allmin = np.min([p["min"] for p in parts], axis=0) * scale_to_mm
    allmax = np.max([p["max"] for p in parts], axis=0) * scale_to_mm
    print(f"scene bbox mm: min {np.round(allmin,1)}  max {np.round(allmax,1)}  size {np.round(allmax-allmin,1)}")

    rows = {}
    for part in parts:
        key = part["name"]
        lo, hi = part["min"] * scale_to_mm, part["max"] * scale_to_mm
        if key in rows:
            rows[key]["min"] = np.minimum(rows[key]["min"], lo)
            rows[key]["max"] = np.maximum(rows[key]["max"], hi)
            rows[key]["count"] += 1
        else:
            rows[key] = {"min": lo, "max": hi, "count": 1}

    print(f"{'part':38s} {'count':>5s}  {'center (x,y,z) mm':>28s}  {'size mm':>24s}")
    for name, row in sorted(rows.items(), key=lambda kv: -float(np.prod(kv[1]['max'] - kv[1]['min'] + 1e-9))):
        center = (row["min"] + row["max"]) / 2
        size = row["max"] - row["min"]
        print(f"{name[:38]:38s} {row['count']:5d}  "
              f"({center[0]:8.1f},{center[1]:8.1f},{center[2]:7.1f})  "
              f"({size[0]:7.1f},{size[1]:7.1f},{size[2]:6.1f})")


if __name__ == "__main__":
    for path in sys.argv[1:]:
        report(path)
