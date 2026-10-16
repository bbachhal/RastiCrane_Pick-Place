# Math, CAD datums, and engineering scope

## Source dimensions

Dimensions below were extracted from the uploaded Onshape STEP exports, not estimated from photographs. `verification/cadBoundingBoxes.csv` retains the setup coordinates; `verification/armCadDatums.json` retains the arm datums. These two independent exports do not establish an assembly transform between their saved coordinate frames.

| Feature | Nominal value | Meaning / restriction |
|---|---:|---|
| Upper and forearm main links | 80 / 80 mm | Pivot-center nominal lengths; HW3 80/60 mm remains a separate math check |
| Shoulder radial offset | 11.992 mm | Nominal offset ahead of base yaw axis; firmware rounds to 12 mm |
| Shoulder above arm base underside | 54.983 mm | Old v0.1 arm export; not automatically height above current table |
| Setup mounting plate | 80 × 80 × 3 mm | A different part/export from the old arm bottom plate |
| Paper | 215.9 × 279.4 mm | Single sheet, top at setup Z=0.1 mm |
| Ink foam | 110 × 70 mm | Top at setup Z=5 mm |
| Rack composite highest point | Z=36.5 mm | Bounding extent only; not a stamp disc/contact datum |

If the old arm underside sits directly on the setup's 3 mm mounting plate, the stack predicts a shoulder height of 57.983 mm. The assembly/export version difference prevents treating that as a verified mounting relationship. The supplied old arm export also does not define the user's current Lego-mounted magnet contact face.

The installed firmware uses shoulder height 60 mm and magnet offsets forward 35 mm, vertical -6.5 mm, lateral 0 from earlier user measurements. This draft explicitly mirrors those existing values for USB/IK consistency and records CAD values separately. They must be changed together in MATLAB and firmware after selecting the actual mounting stack. Camera code cannot repair inconsistent IK geometry. No additional ruler measurements are required for the first detector test.

The exported world positions of the ink and paper are reference assembly positions only. The analysis NEVER uses them as the real run's object positions. Their dimensions can be reused; their placement is recovered from the current photo and the rigid tag attachment offsets.

## HW3 and HW4 camera math

All transforms here premultiply column vectors. Pixels are `[u;v]` = column/row; world is `[X;Y;Z]` in mm. Intrinsics K correspond to the undistorted image returned by MATLAB, including any updated principal point.

For a world point p, `pCamera = R*p + t`, with `t = -R*cameraCenter`. Project with K and divide by the third homogeneous coordinate. For the horizontal plane at height h:

```
Hh = K * [R(:,1), R(:,2), t + h*R(:,3)]
pixelHomogeneous = Hh * [X;Y;1]
worldHomogeneous = Hh \ [u;v;1]
```

The fixed tag may be above the table. Its H is fitted at its own height hRef, then decomposed using K. The translation recovered from that H is `t + hRef*r3`; subtract `hRef*r3` to recover table-based t. Using the base tag plane as the table by accident would bias every height correction.

The DLT solve normalizes both point sets for numerical conditioning and uses full SVD for the four-corner 8×9 system. The nearest proper rotation is obtained by SVD. Reprojection after imposing rigid-camera geometry is checked, as are positive camera depths and camera height above the reference plane. Four reference corners alone cannot supply independent world accuracy: a fitted H round trip is only an algebra check.

The tag's projected center is found by projecting the center of its square through a local homography, rather than averaging image corners under perspective. After applying Hh inverse, world corners determine center and yaw. Side-length and rigid-square residuals screen incompatible tag height, tilt, distortion, or detection geometry. These residuals are diagnostics, not a certified error bound.

For a straight-down camera, treating a point at height h as though it were on the table produces radial error `h*r/(cameraHeight-h)`. HW4's h=30, r=200, cameraHeight=400 mm example gives 16.216 mm. This is why top-view alone does not remove the need for height correction. Only objects lying parallel to the table are supported by this draft; a tilted moving-arm tag needs a separate rigid 3D tag-pose model.

## HW2 rigid attachment offsets

For tag-local planar offset d and recovered yaw psi:

```
pointXY = tagCenterXY + [cosd(psi) -sind(psi); sind(psi) cosd(psi)] * dXY
pointZ  = tagHeight + dZ
```

Each offset is an attachment property. It stays fixed when the object moves. In contrast, tag center and yaw are recomputed from each setup image. The base-tag-to-robot transform is also a fixed attachment property and must reference the yaw axis, not an arbitrary plate corner.

## HW3 parallelogram IK

The controlled point is the center of the magnet's bottom contact face. With a centered magnet:

```
yaw = atan2d(Y,X)
r = hypot(X,Y) - shoulderForward - toolForward
z = Z - shoulderHeight - toolVertical
c = (r*r + z*z - a1*a1 - a2*a2)/(2*a1*a2)
relativeElbow = -acosd(c)
shoulder = atan2d(z,r) - atan2d(a2*sind(relativeElbow), a1+a2*cosd(relativeElbow))
forearmAbsolute = shoulder + relativeElbow
```

M2 controls an absolute forearm angle through the parallelogram; a generic serial-arm relative-elbow servo model would be wrong. The draft rejects unreachable/non-outward targets and calibration extrapolation and checks the solution with FK. Piecewise linear servo fits exactly match the current sketch.

Integer-pulse model residual is NOT physical placement error. The existing 50 Hz, 12-bit PWM has approximately 4.88 microseconds per duty count, so a one-microsecond software pulse change may not change the actual waveform. Servo backlash, load, horn alignment, flex, camera error, and geometry offsets remain. Your earlier 5 mm height discrepancy is evidence that successful jogging establishes relative motion, not complete absolute calibration.

## What is deferred

The full lab needs the actual stamp/disc/magnet geometry, extended calibrated joint range, collision-aware approach and transport paths, attached-stamp height compensation, arc letter layout, radial release, and per-letter logs. A single `xyz` joint interpolation cannot substitute for a proven Cartesian descent or wipe. Unknown letter pickup rotation and possible magnetic slip also matter.

Final stamping should acquire the photo after any connection/reset/home operation, with the scene then held fixed. A new photo is required after moving paper or ink. Camera motion after the photo does not change stored coordinates, but a single photo offers no ongoing feedback about the robot or scene.

## API references

- MATLAB readAprilTag: https://www.mathworks.com/help/vision/ref/readapriltag.html
- undistortImage (second output is new intrinsics from R2024a): https://www.mathworks.com/help/vision/ref/undistortimage.html
- Camera Calibrator workflow: https://www.mathworks.com/help/vision/ug/using-the-single-camera-calibrator-app.html
- Printable patterns: https://www.mathworks.com/help/vision/ug/calibration-patterns.html
- serialport: https://www.mathworks.com/help/matlab/ref/serialport.html

Source PDFs: ME416 HW2 kinematics, HW3 IK/loops/camera, HW4 fiducials/vision solutions, Lab 2 Stamp Machine, RASTICrane assembly manual, and supplied AprilTag code/report. The report's assumed tag-zero reconstruction and pad-center assumption are not reused as measured run data.
