# RASTICrane — state of the build, for the next AI (CAD side)

Written 2026-10-14. This is a factual handoff: what is measured, what is assumed, what is
still unknown. Items marked **UNVERIFIED** must not be treated as data. If a request
conflicts with a rule in section 6, stop and ask the human.

The human is wiring a phone-photo AprilTag vision pipeline (MATLAB) to a 3-DOF arm
(ESP32-S3, USB serial) so it can pick up a letter stamp, press it on an ink pad, and
stamp paper. Vision and base rotation now agree. Shoulder/elbow calibration and the
stamp geometry are the open work, and the stamp geometry is why CAD is needed.

---

## 1. Hardware and firmware facts

| Item | Value |
|---|---|
| Controller | ESP32-S3 Supermini, Arduino IDE board "ESP32S3 Dev Module", core 3.3.11 |
| Serial | COM3, 115200 baud, newline terminated |
| Servo PWM | LEDC only (ESP32Servo is broken on this S3), 50 Hz, 12-bit, `duty = us*4095/20000` |
| M1 shoulder | GPIO 8, 180° servo |
| M2 elbow/forearm | GPIO 9, 180° servo |
| M3 base yaw | GPIO 10, DS-S006L, 300° servo |
| Sketch | `firmware/Rasticrane_XYZ_Test.ino` (the copy here is current and flashed) |

Current firmware constants:

```cpp
home_us           = {1500,1500,1900};   // staggered startup, 750 ms apart
ready_us          = {1950,1950,1950};
safeMin_us        = {1500,1500,1250};   // hard clamp on ALL commands
safeMax_us        = {2500,2500,2500};
cartesianMin_us   = {1500,1500,1300};   // xyz/jog/ik only
cartesianMax_us   = {2000,2000,2500};
minTestHeight_mm  = 40.0;               // blocks all contact moves today
maxCartesianStep_mm = 10.0;             // xyz rejects larger jumps from current MODEL pose
```

Serial commands: `help status/where cal hwcheck planar r z ready ik x y z xyz x y z
jog dx dy dz <motor> <us|ms> <motor> zero zero stop off/0`.
`stop` holds pulses. `off`/`0` removes pulses: **the arm goes limp and drops.**
There is no encoder feedback anywhere; every "MODEL XYZ" is an estimate.

---

## 2. Kinematic model (as implemented, `matlab/rasticraneFk.m`)

Angles are `[shoulder, forearmABSOLUTE, yaw]`. The linkage is a **parallelogram**, so the
forearm angle is measured from horizontal, NOT relative to the upper arm.

```
radial = shoulderForward + a1*cos(shoulder) + a2*cos(forearmAbs) + toolForward
z      = shoulderHeight  + a1*sin(shoulder) + a2*sin(forearmAbs) + toolVertical
x,y    = radial*[cos(yaw), sin(yaw)]        (toolSideways is 0)
```

| Parameter | Value | Source |
|---|---|---|
| a1 upper arm | 80.0 mm | CAD |
| a2 forearm | 80.0 mm | CAD |
| shoulderForward | 12.0 mm | old CAD, **UNVERIFIED** against current build |
| shoulderHeight | 60.0 mm | user measurement, **UNVERIFIED** |
| toolForward (magnet) | 35.0 mm | user measurement, **UNVERIFIED** |
| toolVertical (magnet) | −6.5 mm | user measurement, **UNVERIFIED** |

Robot frame: origin on the tabletop directly below the base yaw axis, +Z up,
+X and +Y fixed by the base tag registration in section 3.

---

## 3. Vision pipeline (MATLAB R2024b, Computer Vision Toolbox)

One phone photo → `readAprilTag` (tag36h11, 50 mm black square) → fit the fixed base tag's
plane → recover camera pose from the calibrated intrinsics → build a separate homography
per tag height → tag centers and yaw in robot mm. Entry point `matlab/analyzeSetup.m`.

Settings that are measured and must not be "cleaned up":

```matlab
cfg.baseTag.id = 5;
cfg.baseTag.centerRobot_mm = [-130 0 4];   % physically measured by the human
cfg.baseTag.yawRobot_deg   = 180;          % physically measured
cfg.cornerOrder = [1 2 3 4];
cfg.maxPoseFit_px = 8;   % raised from 3 deliberately; see below
```

`maxPoseFit_px` was raised because the current photo's base-tag pose fit is 7.32 px, while
tags 2 and 3 — which are NOT used in that fit — recover their 50 mm sides to within
**0.62 mm and 0.72 mm**. The per-tag geometry check (3 mm) is the real quality gate.

**Tags 0 and 1 sit on the stamp holder and are tilted.** This pipeline maps every tag onto a
horizontal plane, so they cannot be located by it. Tag 1 returns size RMS 8.36 mm and shape
RMS 6.12 mm and is rejected. Either the rack needs a flat horizontal tag mount, or the
pipeline needs a tilt-capable pose step. Treat tags 0/1 as detection-only for now.

### Measured targets (run `data/`, photo IMG_3233, 2026-10-13)

| Target | Robot XYZ (mm) | Offset from its tag, in tag frame (mm) | Surface height |
|---|---|---|---|
| Ink surface center (tag 2) | **−14.517, −106.39, 12.3** | 27.607, 98.158, 8.3 | 12.3 mm, measured |
| Paper stamp point (tag 3) | **22.963, 134.43, 0.1** | −137.43, −71.097, −3.9 | 0.1 mm, single sheet |

Tag 2 and tag 3 are both mounted at 4 mm above the table (measured). Those tag-frame offsets
are what make the targets follow the objects when they are moved and re-photographed.

---

## 4. Base calibration — MEASURED 2026-10-14, trustworthy

The human sighted the extended arm (`ready`, 1950/1950) against three known directions whose
azimuths come from vision:

| Direction | Azimuth | Base pulse |
|---|---|---|
| Ink pad | −97.8° | 1400 |
| Rack tag 1 | −51.0° | 1676 |
| Paper point | +80.3° | 2450 |

Linear fit: **5.897 µs per degree, azimuth 0° at 1976.7 µs.** The three points agree to
better than a degree (the rack point predicted 1676, the paper point 2450.2).

```cpp
calibrationPulse_us base row:  {1300,1900,2500}
calibrationAngle_deg base row: {-114.75,-13.01,88.74}
```

Verified on hardware afterwards: at base 2500 the firmware reports magnet XYZ
`3.50 / 159.11`, which is azimuth 88.74°; at 1400 it reports `−21.58 / −157.67`, which is
−97.8°, the azimuth vision measured for the ink pad.

**Correction to earlier documentation:** the old claim that "base 1900 µs points along +X"
is wrong by about 13°. The old base table `{1900,1950,2000} ↔ {0,8,17}` was a photo estimate
and has been replaced. Any analysis built on the old table is suspect.

---

## 5. What is NOT calibrated, and the geometry problem

**M1 and M2 tables are still photo estimates, not measurements:**

```cpp
M1: {1500,1900,2000} -> {89,61,53} deg
M2: {1500,1900,2000} -> {-68,-35,-27} deg
```

A measurement sweep (1500…2500 µs, inclinometer or side-photo per link) is the immediate
next task. Also unverified: whether the forearm's absolute angle stays constant when the
shoulder moves, which the parallelogram assumption requires.

### Reach analysis with the current (unverified) tables

| Pose | M1 | M2 | Base | Status |
|---|---|---|---|---|
| 60 mm above ink | 1717 | 1578 | 1400 | inside limits, **executed successfully** |
| 60 mm above paper | 1911 | 1697 | 2450 | inside limits |
| Ink contact at 12.3 mm | needs 28.4° | needs **−97.2°** | 1400 | **impossible**: −97° extrapolates to ~1146 µs, below the 1500 µs floor |

The ink pad sits at radius 107 mm, which is close in, forcing a very steep forearm. The same
contact at **radius 160 mm** needs only 21.3° shoulder and −61.2° forearm, which is inside
the existing range. **Relocating the ink pad outward may be the cheapest fix** — a new photo
and target pick takes minutes. CAD input on where the pad can physically go is welcome.

Motion proven so far: five 8 mm `xyz` steps from `ready` toward the ink approach point,
which moved exactly as the model predicted. Nothing has touched a surface yet.

---

## 6. Rules any new code must follow

1. **LEDC only.** Never ESP32Servo / `Servo` / `ESP32PWM` on this board.
2. **Never exceed `safeMin_us`/`safeMax_us`**, and never command a pulse outside 500–2500 µs.
3. **Never extrapolate** beyond a servo's calibration points. NaN is the correct answer;
   inventing a value is not. This is how the base error above was caught.
4. **Keep firmware and `visionConfig.m` identical** for `pulseFit_us` / `angleFit_deg` /
   `cartMin_us` / `cartMax_us`. They are mirrored by hand today.
5. **Ramp, never jump**: ≤5 µs per 20 ms automatic, ≤15 µs per 20 ms manual, ≥1 s per move.
6. **`xyz` moves are ≤10 mm from the current MODEL pose**, so long travels are many small
   absolute commands, not one big one.
7. **Limp is not a safe stop.** `off`/`0` drops the arm. `stop` holds position.
8. Targets are **surface points**, not magnet commands. The magnet command for a held stamp
   is surface height + stamp face depth, and that depth is still unknown (section 7).
9. No motion command is ever approved by geometry alone: no collision model, no path check
   against the rack, no feedback that the arm arrived.

---

## 7. What is needed from CAD (the actual ask)

1. **Stamp assembly**: tag-center → steel-disc-center offset, disc height above table, and
   magnet-contact-plane → letter-face depth. The rack bounding-box height (36.5 mm) is NOT
   the disc height. Without these, pickup and press heights cannot be computed.
2. **Rack / stamp holder**: the mounting faces of tags 0 and 1, their tilt angles and heights.
   Either give the geometry so tilt can be modeled, or identify a flat horizontal face where
   a tag can be mounted instead.
3. **Ink pad**: offset from tag 2's mounting tab to the foam center, foam surface height, and
   how far outward the pad can be relocated (see the radius problem in section 5).
4. **Arm**: confirm a1, a2, shoulder pivot height above the table, shoulder forward offset,
   and the magnet mount's forward/vertical offsets. Four of those six are user measurements.
5. **Base**: the yaw axis location relative to tag 5's mounting face, to confirm
   `centerRobot_mm = [-130 0 4]` and `yawRobot_deg = 180`.
6. **Paper**: nominal placement and whether a stack vs single sheet is expected (current
   surface height is 0.1 mm, a single CAD sheet).

STEP exports are fine; the relevant parts need to be identifiable as assemblies.

---

## 8. Files here

```
firmware/Rasticrane_XYZ_Test.ino   flashed firmware, all constants above
matlab/analyzeSetup.m              photo -> tags -> robot coordinates
matlab/visionConfig.m              the one file holding registration + IK model
matlab/selectTargets.m             click ink/paper points on the UNDISTORTED photo
matlab/calibrateBaseAzimuth.m      fit base pulse -> azimuth from a photo sweep (unused; done by sighting)
matlab/planStampMoves.m            vision targets -> checked list of xyz waypoints
matlab/rasticraneFk.m, rasticraneIkPreview.m, heightMap.m, mapPlane.m,
matlab/fitPlaneMap.m, recoverCamera.m, runPhoto.m
data/analysis.json, analysis.mat, targets.csv, corners.csv   the 2026-10-13 run
data/targetSelection.mat           the clicked ink/paper targets and tag-frame offsets
START_HERE.md, MATH_AND_CAD.md     the original project write-ups (base-tag claims in these
                                   predate section 4; section 4 wins where they disagree)
```

Reproduce the current state:

```matlab
cfg = visionConfig();
result = analyzeSetup('IMG_3233.JPG', cfg);   % status CALIBRATED_ANALYSIS_PREVIEW_ONLY
out = planStampMoves('...\targetSelection.mat', 60, 8);   % approach-height path only
```

---

## 9. Open questions

- Do the measured M1/M2 angle tables cover what contact needs, or does the ink pad have to
  move outward?
- Does the forearm hold its absolute angle when the shoulder moves (parallelogram check)?
- What is the stamp face depth below the magnet contact plane?
- Can the rack carry a flat horizontal tag, or does the pipeline need tilted-tag support?
- Power: the servos' supply has never been confirmed; stalling under load was seen earlier in
  the build. Worth checking before longer motion sequences.
