"""Read an overhead photo of the setup and turn AprilTags into robot coordinates.

Frame definition (the one the user fixed, so it survives the base being removed):

    origin   = centre of tag 5, which sits under the robot's base plate
    0 degrees = the direction from the origin to the MIDDLE of the stamp holder arc
    +Z        = up, out of the table

The arc runs from the top-left corner of tag 0 to the top-right corner of tag 1, and the
CAD gives its radius, so three knowns fix the arc and therefore the home direction.

Everything here is a planar (table-top) estimate from one photo. It is not a calibrated
camera pose: tags that sit well above the table, or a steeply tilted camera, will be off.
"""
import base64
import json
import math

import cv2
import numpy as np

TAG_FAMILY = cv2.aruco.DICT_APRILTAG_36h11


def detect_tags(image_bgr):
    """Return {tag_id: 4x2 corner array} in the detector's order: TL, TR, BR, BL."""
    dictionary = cv2.aruco.getPredefinedDictionary(TAG_FAMILY)
    params = cv2.aruco.DetectorParameters()
    params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
    detector = cv2.aruco.ArucoDetector(dictionary, params)
    gray = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2GRAY)
    corners, ids, _ = detector.detectMarkers(gray)
    found = {}
    if ids is None:
        return found
    for marker, tag_id in zip(corners, ids.flatten()):
        found[int(tag_id)] = marker.reshape(4, 2).astype(float)
    return found


def plane_homography(tag_corners, tag_size_mm):
    """Map image pixels to a table-plane frame centred on this tag, +X along its top edge.

    Built from one tag's four corners, so it is only valid for things lying on that tag's
    plane. Good enough for table-top layout; not a camera calibration.
    """
    half = tag_size_mm / 2.0
    local = np.array([[-half, half], [half, half], [half, -half], [-half, -half]], dtype=np.float32)
    return cv2.getPerspectiveTransform(tag_corners.astype(np.float32), local)


def to_plane(homography, points_px):
    pts = np.asarray(points_px, dtype=np.float32).reshape(-1, 1, 2)
    return cv2.perspectiveTransform(pts, homography).reshape(-1, 2)


def arc_from_endpoints(p0, p1, radius_mm, away_from):
    """Circle centre for an arc of known radius through p0 and p1, bulging away from `away_from`.

    Returns (centre, arc midpoint). With two endpoints and a radius there are two possible
    centres; pick the one that puts the arc's midpoint further from the robot, which is how
    the holder actually sits.
    """
    p0, p1 = np.asarray(p0, float), np.asarray(p1, float)
    chord = p1 - p0
    chord_len = float(np.linalg.norm(chord))
    if chord_len < 1e-6 or chord_len > 2 * radius_mm:
        # Geometry impossible (tags too far apart for this radius): fall back to the chord.
        midpoint = (p0 + p1) / 2
        return None, midpoint
    mid = (p0 + p1) / 2
    height = math.sqrt(max(radius_mm ** 2 - (chord_len / 2) ** 2, 0.0))
    normal = np.array([-chord[1], chord[0]]) / chord_len
    options = [mid + normal * height, mid - normal * height]
    best = None
    for centre in options:
        arc_mid = centre + (mid - centre) / (np.linalg.norm(mid - centre) + 1e-9) * radius_mm
        score = np.linalg.norm(arc_mid - np.asarray(away_from, float))
        if best is None or score > best[0]:
            best = (score, centre, arc_mid)
    return best[1], best[2]


def analyse(image_bytes, config, cad):
    """Full pass: detect, build the robot frame, place ink and paper, annotate the photo."""
    data = np.frombuffer(image_bytes, dtype=np.uint8)
    image = cv2.imdecode(data, cv2.IMREAD_COLOR)
    if image is None:
        raise ValueError("Could not decode that image. Use JPG or PNG.")

    tag_size = float(config.get("vision", {}).get("tagSize_mm", 50.0))
    tags = detect_tags(image)
    result = {"imageSize": [int(image.shape[1]), int(image.shape[0])],
              "tagsFound": sorted(tags.keys()), "notes": []}

    if 5 not in tags:
        result["notes"].append("Tag 5 (under the robot base) was not found, so no robot frame "
                               "could be built. Everything below is detection only.")
        result["preview"], result["previewScale"] = annotate(image, tags, None)
        return result

    # --- raw plane frame, centred on tag 5, +X along tag 5's own top edge ---
    homography = plane_homography(tags[5], tag_size)
    centres_raw = {tid: to_plane(homography, corners.mean(axis=0))[0] for tid, corners in tags.items()}

    # --- home direction: the middle of the holder arc ---
    arc_radius = float(cad.get("holderArc", {}).get("radius_mm", 124.6))
    home_vector, arc_raw = None, None
    if 0 in tags and 1 in tags:
        # Tag CENTRES, not corners. The holder-end tags are rotated differently from photo to
        # photo, so a tag-local corner index does not reliably mean the same physical corner.
        # The two centres are symmetric about the arc, which is all the home direction needs.
        p_left = centres_raw[0]
        p_right = centres_raw[1]
        centre, arc_mid = arc_from_endpoints(p_left, p_right, arc_radius, away_from=[0, 0])
        home_vector = arc_mid
        # Stored below in the ROBOT frame, after home_angle is known, so that the arc,
        # the tags and the clicked points all live in one frame.
        arc_raw = {"leftEnd": p_left, "rightEnd": p_right, "arcMid": arc_mid, "center": centre}
        result["notes"].append("Home direction taken from the midpoint of tags 0 and 1, which assumes "
                               "they are mounted symmetrically on the holder.")
        if centre is None:
            result["notes"].append("Tag 0 and tag 1 are further apart than the CAD arc radius "
                                   "allows, so the chord midpoint was used instead of a true arc.")
    else:
        result["notes"].append("Tags 0 and 1 are needed to define the home direction "
                               "(holder arc). Falling back to tag 5's own +X axis.")
        home_vector = np.array([1.0, 0.0])

    home_angle = math.degrees(math.atan2(home_vector[1], home_vector[0]))

    def to_robot(point):
        """Rotate the raw plane frame so that 0 degrees points at the holder arc midpoint."""
        vector = np.asarray(point, float)
        angle = math.radians(-home_angle)
        rotation = np.array([[math.cos(angle), -math.sin(angle)],
                             [math.sin(angle), math.cos(angle)]])
        return rotation @ vector

    arc_info = {}
    if arc_raw:
        arc_info = {
            "leftEnd_mm": np.asarray(to_robot(arc_raw["leftEnd"]), float).round(2).tolist(),
            "rightEnd_mm": np.asarray(to_robot(arc_raw["rightEnd"]), float).round(2).tolist(),
            "arcMid_mm": np.asarray(to_robot(arc_raw["arcMid"]), float).round(2).tolist(),
            "radius_mm": arc_radius,
            "center_mm": (None if arc_raw["center"] is None
                          else np.asarray(to_robot(arc_raw["center"]), float).round(2).tolist()),
            "chordOnly": arc_raw["center"] is None,
        }

    objects = {}
    for tid, centre in centres_raw.items():
        robot_xy = to_robot(centre)
        corners_robot = np.array([to_robot(c) for c in to_plane(homography, tags[tid])])
        edge = corners_robot[1] - corners_robot[0]
        objects[str(tid)] = {
            "xy_mm": robot_xy.round(2).tolist(),
            "radius_mm": round(float(np.hypot(*robot_xy)), 2),
            "azimuth_deg": round(math.degrees(math.atan2(robot_xy[1], robot_xy[0])), 2),
            "yaw_deg": round(math.degrees(math.atan2(edge[1], edge[0])), 2),
            "sideLength_mm": round(float(np.mean([np.linalg.norm(corners_robot[(i + 1) % 4] - corners_robot[i])
                                                  for i in range(4)])), 2),
        }

    # --- ink and paper, from their tags plus the offsets measured in MATLAB ---
    for name, spec in (config.get("vision", {}).get("taggedTargets", {}) or {}).items():
        tid = str(spec["tagId"])
        if tid not in objects:
            result["notes"].append(f"{name}: tag {tid} not visible, position unknown.")
            continue
        base = np.array(objects[tid]["xy_mm"], float)
        yaw = math.radians(objects[tid]["yaw_deg"])
        rotation = np.array([[math.cos(yaw), -math.sin(yaw)], [math.sin(yaw), math.cos(yaw)]])
        point = base + rotation @ np.array(spec["offsetTag_mm"][:2], float)
        objects[name] = {
            "xy_mm": point.round(2).tolist(),
            "radius_mm": round(float(np.hypot(*point)), 2),
            "azimuth_deg": round(math.degrees(math.atan2(point[1], point[0])), 2),
            "height_mm": spec.get("height_mm"),
            "fromTag": int(tid),
        }

    # Tag 5's own side length is the sanity check: it should come back at tag_size.
    if "5" in objects:
        error = abs(objects["5"]["sideLength_mm"] - tag_size)
        if error > 2.0:
            result["notes"].append(f"Tag 5 measures {objects['5']['sideLength_mm']} mm instead of "
                                   f"{tag_size} mm. Check the printed tag size and the camera angle.")

    result.update({
        "homography": homography.tolist(),       # image px -> tag-5 plane mm
        "previewScale": 1.0,                     # filled in by annotate()
        "homeAngleInTagFrame_deg": round(home_angle, 2),
        "arc": arc_info,
        "objects": objects,
    })
    preview, scale = annotate(image, tags, {0: "holder end", 1: "holder end", 5: "ROBOT ORIGIN"})
    result["preview"] = preview
    result["previewScale"] = scale              # preview px * (1/scale) = original px
    return result


def pixel_to_robot(result, x_preview, y_preview):
    """Map a click on the preview image into robot-frame mm."""
    scale = result.get("previewScale") or 1.0
    homography = np.array(result["homography"], dtype=np.float32)
    point = to_plane(homography, [[x_preview / scale, y_preview / scale]])[0]
    angle = math.radians(-result["homeAngleInTagFrame_deg"])
    rotation = np.array([[math.cos(angle), -math.sin(angle)],
                         [math.sin(angle), math.cos(angle)]])
    return (rotation @ point).tolist()


def offset_in_tag_frame(result, tag_id, robot_xy):
    """Express a robot-frame point in one tag's own frame, so it follows that tag."""
    tag = result["objects"].get(str(tag_id))
    if not tag:
        return None
    base = np.array(tag["xy_mm"], dtype=float)
    yaw = math.radians(tag["yaw_deg"])
    rotation = np.array([[math.cos(yaw), -math.sin(yaw)],
                         [math.sin(yaw), math.cos(yaw)]])
    return (rotation.T @ (np.array(robot_xy, dtype=float) - base)).tolist()


def annotate(image, tags, labels, max_width=1100):
    """Draw detections and return a base64 JPEG small enough to hand to a browser."""
    canvas = image.copy()
    for tag_id, corners in tags.items():
        pts = corners.astype(int).reshape(-1, 1, 2)
        cv2.polylines(canvas, [pts], True, (0, 220, 120), 6)
        centre = corners.mean(axis=0).astype(int)
        text = f"{tag_id}"
        if labels and tag_id in labels:
            text += f" {labels[tag_id]}"
        cv2.putText(canvas, text, tuple(centre), cv2.FONT_HERSHEY_SIMPLEX, 2.0, (0, 0, 0), 8)
        cv2.putText(canvas, text, tuple(centre), cv2.FONT_HERSHEY_SIMPLEX, 2.0, (0, 230, 255), 3)
        corner0 = corners[0].astype(int)
        cv2.circle(canvas, tuple(corner0), 14, (255, 120, 0), -1)   # corner 0 marker
    scale = 1.0
    if canvas.shape[1] > max_width:
        scale = max_width / canvas.shape[1]
        canvas = cv2.resize(canvas, (int(canvas.shape[1] * scale), int(canvas.shape[0] * scale)))
    ok, buffer = cv2.imencode(".jpg", canvas, [int(cv2.IMWRITE_JPEG_QUALITY), 82])
    if not ok:
        return None, scale
    return "data:image/jpeg;base64," + base64.b64encode(buffer.tobytes()).decode("ascii"), scale
