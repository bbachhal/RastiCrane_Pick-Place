# RASTICrane — phone photo to MATLAB vision draft

This checkpoint analyzes ONE photograph, locates AprilTags, converts configured tag locations into robot coordinates, and previews inverse kinematics. USB is the only robot connection. There is no MQTT, live camera requirement, automatic homing, or stamping sequence in this draft.

Start with detection now. Camera calibration and the fixed tag's attachment pose unlock metric coordinates. The missing attachment information is deliberately visible rather than replaced with guesses from your old homework photograph.

## 1. What to have ready

- Your existing RASTICrane, ESP32-S3 Dev Module, working firmware, and USB data cable.
- MATLAB R2024a or newer with Computer Vision Toolbox. Run `version` if unsure. No Arduino support package is required for ordinary USB serial.
- Your phone's rear main camera, ideally supported above the setup. Use one lens and one resolution throughout calibration and runs. Avoid ultrawide, portrait effects, digital zoom, screenshots, and messaging-app resizing.
- Unique tag36h11 tags. The lab's nominal black square is 50 mm per side, excluding the white border. Retain at least one cell of white margin. Confirm print scale once; this is calibration, not runtime measuring.
- A checkerboard print attached to something flat if you do not already have calibration from THIS phone/lens/image resolution.
- Paper, ink pad, and rack. Fix the base reference tag to a stationary, visible part of the setup, not a moving arm link. Keep a separate arm tag if required by the lab.

For the first detection test, the arm can remain disconnected. Keep fingers and the arm out of the tags' borders. Diffuse lighting helps avoid glare from tape.

## 2. Run the first photograph immediately

1. Unzip the project. Open its `RasticraneVisionDraft` folder as MATLAB's Current Folder.
2. Take one sharp overhead photo containing the whole setup and all tags. A modest camera tilt is acceptable.
3. Transfer the original JPG or PNG to the computer, for example with a USB photo transfer or an original-quality file transfer. For HEIC, export to JPG without resizing and use that same conversion process for calibration photos. Keep all images in the same orientation and pixel dimensions.
4. In MATLAB run:

```matlab
runPhoto
```

5. Select your photograph. MATLAB displays the detections and saves a dated folder inside `results`.
6. Send me `annotatedDetection.png` and `corners.csv` from that folder, plus the original photo.

Labels are `tagID:MATLABcorner`, with corners numbered 1 through 4. A result of `DETECTION_ONLY` is expected before setup registration. It means the detector ran; it does not mean calibration failed. Missing or hidden tags are not reconstructed from your old image.

The old Python PDF hardcodes edge-search coordinates for a particular 2048×1536 photo and reconstructs hidden tag 0. It is not a reusable detector for new views. Your other AprilTag PDF is an analysis report with assumptions, rather than a second executable MATLAB file. This draft reuses the homework mathematics, not its old image coordinates or assumed pad position.

## 3. Calibrate the phone once

Camera calibration and base-tag registration solve different problems:

- Camera calibration determines focal lengths, optical center, and lens distortion.
- The fixed base tag determines where that camera is relative to the robot for EACH setup photograph.

If your HW3 already produced `cameraParams` or `intrinsics` for this exact phone setup, use that file. Numerical values from a worked example or a different camera are not a calibration of this phone.

Otherwise:

1. In MATLAB, run `open checkerboardPattern.pdf`. Print the pattern without scaling and mount it flat. Measure one printed square once, in mm.
2. Take roughly 15–20 sharp photos of the complete checkerboard at different tilts and positions across the image, at distances around your working distance. Do not take 20 identical overhead views. Keep the same camera lens, resolution, orientation, and focus setting that you will use during the run; lock focus/exposure where your phone allows. If focus cannot be locked, keep the working distance consistent and review the calibration quality.
3. Transfer the original files into one folder. Run:

```matlab
cameraCalibrator
```

4. Add the images, select Checkerboard, and enter the actual square size in millimeters. Calibrate. Inspect the corner overlays and reprojection errors; remove visibly bad detections or blurry images and recalibrate.
5. Export the camera parameters to the workspace as `cameraParams`, then run from the project folder:

```matlab
save(fullfile('calibration','phoneCalibration.mat'),'cameraParams')
```

If your existing calibration variable is named `intrinsics`, save that instead. The script supports either variable name. Keep the calibration app's error summary for the eventual lab report.

One run photograph can register an already calibrated camera; this draft does not claim to estimate all intrinsic/distortion parameters from that single image. Changing camera position between runs is fine: detect the base tag again. Changing lens, resolution, crop, or focus substantially can invalidate the calibration.

## 4. Register the fixed base tag once

The robot frame is the frame already used by your working IK:

- Origin: tabletop directly below the base rotation axis.
- +X: outward along the arm at base pulse 1900 microseconds.
- +Y: left when looking outward along +X.
- +Z: upward.

A tag's center is not automatically the robot origin. Its printed orientation is not automatically the robot's +X direction.

Edit `visionConfig.m` after we review your photo/tag placement:

```matlab
cfg.cornerOrder = [ ... ];
cfg.baseTag.id = ...;
cfg.baseTag.centerRobot_mm = [x y z];
cfg.baseTag.yawRobot_deg = ...;
```

`cornerOrder` is the raw MATLAB indices corresponding to the physical decoded tag's bottom-left, bottom-right, top-right, top-left corners, in that order. Use the numbered overlay and the upright source print to identify them. “Bottom-left” means in the tag's printed orientation, not necessarily the bottom-left of the photograph. We must not reuse OpenCV's ordering blindly. This common ordering also defines tag-local +X along bottom-left to bottom-right and +Y toward top-left.

Tag center/yaw/height are a FIXED mounting relationship. We can get them from a tag placement added to the CAD or a dimensioned mounting drawing. The supplied STEP models do not contain an identified printed-tag mount that determines that relationship, so it remains unset. Do not enter all zeros simply to make the program run.

If you have the original upright image used to print a single tag, the optional helper can identify the order for you:

```matlab
cornerOrder = inspectTagReference('uprightTagWithWhiteBorder.png')
```

Copy its displayed order into the configuration and confirm the numbered overlay. Use the actual print source, not an oblique setup photograph.

The base tag must lie horizontally for this implementation. An arm-mounted tag can later check arm pose if its rigid attachment transform is known. It cannot alone establish unknown servo zero angles, and a stationary base tag cannot measure the joint angles.

## 5. Register the movable objects' tag attachments

The default IDs follow HW4: base 0, check tag 1, ink 2, paper 3. These are editable conventions, not new detections. Each object's configuration contains:

```matlab
cfg.objects(k).tagHeight_mm = ...;
cfg.objects(k).targetOffsetTag_mm = [dx dy dz];
```

The offset points FROM the tag CENTER TO the desired physical point, in the tag's own coordinates. The code rotates this offset when the object rotates. Do not enter a fixed robot/world position for the paper or ink.

For the ink pad, the desired point is the center of the exposed foam. Put its tag on a dry, rigid mounting tab with a known relationship to that foam. A freely hinged open lid is not a reliable rigid reference for the ink surface. Do not cover the usable foam with the tag. A tag on a vertical side is unsupported by this horizontal-plane draft.

For the paper, the desired point is its center for mapping. A tag can sit on an unused flat margin; its offset to the paper center comes from the placement drawing and CAD paper dimensions. The final letter positions will be chosen inside the paper AND reachable workspace; the paper center itself need not be reachable. A paper stack has a different surface height from the single CAD sheet.

For the check tag, set its actual attachment height and use `[0 0 0]` as offset if you want its center. Its recovered side lengths provide a useful check because it was not used to fit the reference homography. This is not an independent ruler measurement of robot accuracy.

For stamps, the later version needs tag-center-to-disc-center offsets, disc height, and magnet-contact-to-letter-face depth. The rack bounding-box height is NOT the disc height. The supplied workspace STEP does not identify complete stamp/tag/disc assemblies from which those offsets can be recovered reliably. A further STEP export of those parts, or the relevant Onshape part drawing, is sufficient; no new format is necessary.

## 6. Run calibrated analysis

After the configuration is filled:

```matlab
cfg = visionConfig();
result = analyzeSetup(fullfile('data','setup.jpg'),cfg);
```

Or use `runPhoto` again and select any original photo. The program:

1. Checks image size against calibration and corrects lens distortion.
2. Detects actual tag IDs/corners at full image resolution.
3. Fits the fixed reference plane and recovers camera pose using K.
4. Builds a separate Hh for each configured tag height.
5. Maps corners into robot coordinates and estimates each horizontal tag's center and yaw.
6. Rotates/translates the configured attachment offset to obtain an object target.
7. Computes an EMPTY-MAGNET IK preview of that point using the installed firmware's model.

`targets.csv`, `analysis.mat`, and `analysis.json` hold the results, statuses, configuration, pixel corners, camera fit, and IK diagnostics. Check individual object statuses; a completed analysis does not mean every object is located or reachable. `TARGET_LOCATED_PREVIEW_ONLY` is never motion approval. NaN in MATLAB, or null in JSON, means unavailable, not zero.

The ink/paper target is a physical surface point. It is NOT yet the magnet command for an attached stamp. If the stamp face lies d mm below the magnet contact plane, the later nominal press height is surface height + d, with any controlled compression handled separately. Pickup puts the magnet on the steel disc instead. Those heights cannot be exchanged.

## 7. Check the IK and optional USB connection

First, without the robot:

```matlab
demoMath
cfg = visionConfig();
ikPreview = rasticraneIkPreview([160 25 80],cfg.ik)
```

Expected proposed M1/M2/base pulses: `[1969 1968 1955]`, matching your earlier serial output. These are model commands, not measured positions.

Keep your currently working `Rasticrane_XYZ_Test.ino`. No re-upload is required for this checkpoint. A reference copy is included for comparison. It uses pins 8/9/10 from your working build and direct LEDC; the generic assembly manual has a different pin listing. Do not rewire a working build to match that generic listing.

Before USB: close Arduino Serial Monitor. Opening a serial port MAY reset the ESP32; your existing sketch then engages its startup home of 1500/1500/1900. Support/clear the arm as for an ordinary reset. The helper does not redefine home or request a move, but it cannot prevent that firmware startup behavior.

```matlab
serialportlist('available')
robotPort = openRobotUsb('COM3'); % Replace COM3 with your actual listed port.
statusText = queryRobotUsb(robotPort,'status');
mathText = queryRobotUsb(robotPort,'hwcheck');
comparison = compareIkUsb(robotPort,[160 25 80],visionConfig());
clear robotPort
```

The comparison queries `cal`, checks geometry and servo fits, and sends `ik`, which only previews. The wrapper refuses all motion commands. A complete reply has a timeout; it is never interpreted as physical arrival.

## 8. What the current arm can and cannot yet do

The current Cartesian controller restricts ALL motors to 1900–2000 microseconds, moves to at most 10 mm from the current model pose per command, and requires at least 40 mm nominal clearance. Its base calibration covers only 0–17 degrees. This is sufficient for your successful jog tests, but not a full semicircular rack or tabletop stamping.

We will later extend the measured servo range, check reachability of every approach/transfer/contact/release pose, and generate paths. Current `xyz` commands interpolate in joint pulses, so one large command is not a guaranteed vertical pickup or straight radial wipe. Release must move outward at constant azimuth, as required by the lab. A held stamp also enlarges the collision envelope. The arm has no independent wrist-yaw joint: the carried letter rotates by press azimuth minus pickup azimuth.

One initial photo is appropriate while the base, rack, ink, and paper stay fixed. Retake the photo after rearranging them. This open-loop draft cannot detect a failed magnetic pickup, slipping stamp, or a person moving the paper after capture. Tracking commanded state is not sensor confirmation.

## 9. Send these next

1. The original setup photo and the generated annotated image/corners CSV.
2. Your MATLAB release and phone model/lens; your existing phone calibration MAT file if available.
3. Which tag is the fixed base tag, and where it will attach relative to the base axis. A CAD placement or marked-up photo is enough to decide the remaining registration details; no repeat ruler survey is requested now.
4. For the later stamping stage: CAD containing the actual stamp, steel disc, tag mounting face, and current magnet mount. Keep using STEP.

After the first photo/calibration check, we can configure those fixed offsets together, then validate a raised point before building the complete pickup/ink/press/park/wipe cycle.
