"""Arm model fitting and letter-sequence planning.

Two jobs:

1. fit_arm()  - take a handful of "the magnet was touching THIS known point at THESE pulses"
   samples and solve the M1/M2 angle-vs-pulse lines. The known points come from the photo
   (tag centres), so no protractor is needed. This replaces the inherited photo estimates.

2. plan_letters() - build the whole pick / ink / press / return program from the model, so
   positions come from geometry instead of from nudging sliders at every slot.

Nothing here talks to the robot. Every pose is checked against the servo limits and the
calibrated pulse span before it is offered.
"""
import math

import numpy as np

ELBOW_UP = -1.0      # the branch the firmware enforces


# ---------------------------------------------------------------- pulse <-> angle
def angle_from_pulse(cal, pulse):
    pulses, angles = cal["pulses"], cal["angles_deg"]
    return float(np.interp(pulse, pulses, angles,
                           left=angles[0] + (pulse - pulses[0]) * (angles[1] - angles[0]) / (pulses[1] - pulses[0]),
                           right=angles[-1] + (pulse - pulses[-1]) * (angles[-1] - angles[-2]) / (pulses[-1] - pulses[-2])))


def pulse_from_angle(cal, angle):
    """Pulse for an angle, or None if that angle is outside what was calibrated.

    np.interp clamps by default, which would hand back the end-of-table pulse for an angle
    the arm cannot actually reach. Returning None keeps that out of the plan.
    """
    pulses, angles = np.array(cal["pulses"], float), np.array(cal["angles_deg"], float)
    order = np.argsort(angles)
    sorted_angles = angles[order]
    if angle < sorted_angles[0] - 1e-9 or angle > sorted_angles[-1] + 1e-9:
        return None
    return float(np.interp(angle, sorted_angles, pulses[order]))


def forward(cfg, m1_pulse, m2_pulse):
    """Pulses -> (radius, height) of the magnet, in the plane of the arm."""
    g = cfg["geometry"]
    t1 = math.radians(angle_from_pulse(cfg["calibration"]["M1"], m1_pulse))
    t2 = math.radians(angle_from_pulse(cfg["calibration"]["M2"], m2_pulse))
    radius = g["shoulderForward_mm"] + g["a1_mm"] * math.cos(t1) + g["a2_mm"] * math.cos(t2) + g["toolForward_mm"]
    height = g["shoulderHeight_mm"] + g["a1_mm"] * math.sin(t1) + g["a2_mm"] * math.sin(t2) + g["toolVertical_mm"]
    return radius, height


def inverse(cfg, radius_mm, height_mm):
    """(radius, height) -> shoulder and forearm angles, elbow-up. None if unreachable."""
    g = cfg["geometry"]
    horizontal = radius_mm - g["shoulderForward_mm"] - g["toolForward_mm"]
    vertical = height_mm - g["shoulderHeight_mm"] - g["toolVertical_mm"]
    if horizontal <= 0:
        return None
    reach = math.hypot(horizontal, vertical)
    if reach > g["a1_mm"] + g["a2_mm"] or reach < 1e-6:
        return None
    cos_elbow = (horizontal ** 2 + vertical ** 2 - g["a1_mm"] ** 2 - g["a2_mm"] ** 2) / (2 * g["a1_mm"] * g["a2_mm"])
    if abs(cos_elbow) > 1:
        return None
    relative = ELBOW_UP * math.degrees(math.acos(max(-1.0, min(1.0, cos_elbow))))
    shoulder = math.degrees(math.atan2(vertical, horizontal)) - math.degrees(
        math.atan2(g["a2_mm"] * math.sin(math.radians(relative)),
                   g["a1_mm"] + g["a2_mm"] * math.cos(math.radians(relative))))
    return shoulder, shoulder + relative


# ---------------------------------------------------------------- fitting from touches
def fit_arm(cfg, touches, iterations=60):
    """Solve M1/M2 angle = slope*pulse + offset from touch samples.

    Each touch is {"pulses": [m1, m2, base], "target": [x, y, z]} where target is a point
    whose position is known from the photo. Residuals are radius and height errors.
    """
    if len(touches) < 2:
        raise ValueError("Need at least two touches; three or more is better.")

    g = cfg["geometry"]
    cal1, cal2 = cfg["calibration"]["M1"], cfg["calibration"]["M2"]
    # Start from the existing lines so a poor fit degrades gracefully.
    params = np.array([
        (cal1["angles_deg"][-1] - cal1["angles_deg"][0]) / (cal1["pulses"][-1] - cal1["pulses"][0]),
        cal1["angles_deg"][0],
        (cal2["angles_deg"][-1] - cal2["angles_deg"][0]) / (cal2["pulses"][-1] - cal2["pulses"][0]),
        cal2["angles_deg"][0],
    ], dtype=float)
    base1, base2 = cal1["pulses"][0], cal2["pulses"][0]

    samples = []
    for touch in touches:
        m1, m2 = float(touch["pulses"][0]), float(touch["pulses"][1])
        x, y, z = (float(v) for v in touch["target"])
        samples.append((m1, m2, math.hypot(x, y), z))

    def residuals(p):
        out = []
        for m1, m2, radius, height in samples:
            t1 = math.radians(p[0] * (m1 - base1) + p[1])
            t2 = math.radians(p[2] * (m2 - base2) + p[3])
            r_model = g["shoulderForward_mm"] + g["a1_mm"] * math.cos(t1) + g["a2_mm"] * math.cos(t2) + g["toolForward_mm"]
            z_model = g["shoulderHeight_mm"] + g["a1_mm"] * math.sin(t1) + g["a2_mm"] * math.sin(t2) + g["toolVertical_mm"]
            out.extend([r_model - radius, z_model - height])
        return np.array(out)

    for _ in range(iterations):                      # Gauss-Newton with numeric Jacobian
        r0 = residuals(params)
        jacobian = np.zeros((len(r0), 4))
        for k in range(4):
            step = np.zeros(4)
            step[k] = 1e-4 if k in (0, 2) else 1e-2
            jacobian[:, k] = (residuals(params + step) - r0) / step[k]
        try:
            delta, *_ = np.linalg.lstsq(jacobian, -r0, rcond=None)
        except np.linalg.LinAlgError:
            break
        params = params + delta
        if np.linalg.norm(delta) < 1e-9:
            break

    final = residuals(params)
    rms = float(np.sqrt(np.mean(final ** 2)))

    def table(slope, offset, base, span):
        pulses = [span[0], (span[0] + span[1]) // 2, span[1]]
        return {"pulses": pulses,
                "angles_deg": [round(slope * (p - base) + offset, 3) for p in pulses],
                "source": f"fitted from {len(touches)} magnet touches, residual RMS {rms:.2f} mm"}

    return {
        "M1": table(params[0], params[1], base1, (1500, 2500)),
        "M2": table(params[2], params[3], base2, (1500, 2500)),
        "residualRms_mm": round(rms, 3),
        "perTouch_mm": [round(float(math.hypot(final[2 * i], final[2 * i + 1])), 2) for i in range(len(samples))],
    }


# ---------------------------------------------------------------- planning
def pose_for(cfg, home, x, y, z):
    """Full pulse triple for a robot-frame point, or a reason it cannot be reached."""
    radius = math.hypot(x, y)
    azimuth = math.degrees(math.atan2(y, x))
    solution = inverse(cfg, radius, z)
    if solution is None:
        return {"ok": False, "reason": f"unreachable at r={radius:.0f} z={z:.0f}"}
    shoulder, forearm = solution
    m1 = pulse_from_angle(cfg["calibration"]["M1"], shoulder)
    m2 = pulse_from_angle(cfg["calibration"]["M2"], forearm)
    for value, name, angle, cal in ((m1, "M1", shoulder, cfg["calibration"]["M1"]),
                                    (m2, "M2", forearm, cfg["calibration"]["M2"])):
        if value is None:
            span = sorted(cal["angles_deg"])
            return {"ok": False,
                    "reason": f"{name} needs {angle:.1f} deg, outside the calibrated "
                              f"{span[0]:.1f} to {span[-1]:.1f} deg"}
    us_per_deg = ((cfg["calibration"]["BASE"]["pulses"][-1] - cfg["calibration"]["BASE"]["pulses"][0]) /
                  (cfg["calibration"]["BASE"]["angles_deg"][-1] - cfg["calibration"]["BASE"]["angles_deg"][0]))
    base = home["homePulse_us"] + azimuth * us_per_deg
    pulses = [int(round(m1)), int(round(m2)), int(round(base))]

    lo, hi = cfg["limits"]["manualMin"], cfg["limits"]["manualMax"]
    for i, name in enumerate(("M1", "M2", "base")):
        if not (lo[i] <= pulses[i] <= hi[i]):
            return {"ok": False, "pulses": pulses,
                    "reason": f"{name} wants {pulses[i]} us, outside {lo[i]}-{hi[i]}"}

    for key, index in (("M1", 0), ("M2", 1)):
        span = cfg["calibration"][key]["pulses"]
        if not (span[0] <= pulses[index] <= span[-1]):
            return {"ok": False, "pulses": pulses,
                    "reason": f"{key} {pulses[index]} us is outside its calibrated span {span[0]}-{span[-1]}"}

    radius_back, height_back = forward(cfg, pulses[0], pulses[1])
    return {"ok": True, "pulses": pulses,
            "xyz": [round(x, 1), round(y, 1), round(z, 1)],
            "angles_deg": [round(shoulder, 2), round(forearm, 2), round(azimuth, 2)],
            "roundTrip_mm": round(math.hypot(radius_back - radius, height_back - z), 2)}


def plan_letters(cfg, vision, home, letters, slots, heights):
    """Build the full program: for each letter, grip from its slot, ink it, press it, return it."""
    steps = []

    def add(name, x, y, z, note=""):
        pose = pose_for(cfg, home, x, y, z)
        pose.update({"name": name, "note": note})
        steps.append(pose)

    hover = heights["hoverClearance_mm"]
    for letter, slot in zip(letters, slots):
        sx, sy = slot["xy"]
        add(f"{letter}_slot_hover", sx, sy, heights["gripZ_mm"] + hover, f"above slot {slot['index']}")
        add(f"{letter}_slot_grip", sx, sy, heights["gripZ_mm"], "magnet on the steel disc")
        add(f"{letter}_slot_lift", sx, sy, heights["gripZ_mm"] + hover, "lift the stamp clear")

        ink = vision["objects"].get("ink")
        if ink:
            ix, iy = ink["xy_mm"]
            add(f"{letter}_ink_hover", ix, iy, heights["inkPressZ_mm"] + hover, "above the ink pad")
            add(f"{letter}_ink_press", ix, iy, heights["inkPressZ_mm"], "stamp face on the ink")
            add(f"{letter}_ink_lift", ix, iy, heights["inkPressZ_mm"] + hover, "")

        paper = vision["objects"].get("paper")
        if paper:
            px, py = paper["xy_mm"]
            spacing = heights["letterSpacing_mm"] * (letters.index(letter) - (len(letters) - 1) / 2)
            angle = math.atan2(py, px)
            # Letters march outward along the same azimuth, so each one keeps the same rotation.
            lx = px + spacing * math.cos(angle)
            ly = py + spacing * math.sin(angle)
            add(f"{letter}_paper_hover", lx, ly, heights["paperPressZ_mm"] + hover, f"over the {letter} mark")
            add(f"{letter}_paper_press", lx, ly, heights["paperPressZ_mm"], "stamp face on the paper")
            add(f"{letter}_paper_lift", lx, ly, heights["paperPressZ_mm"] + hover, "")

        add(f"{letter}_return_hover", sx, sy, heights["gripZ_mm"] + hover, "back over its slot")
        add(f"{letter}_return_seat", sx, sy, heights["gripZ_mm"], "stamp back in the slot")
        add(f"{letter}_release", sx * heights["releaseScale"], sy * heights["releaseScale"],
            heights["gripZ_mm"], "slide outward so the holder strips the magnet")

    ok = all(step["ok"] for step in steps)
    return {"steps": steps, "allReachable": ok,
            "blocked": [s["name"] + ": " + s.get("reason", "") for s in steps if not s["ok"]]}
