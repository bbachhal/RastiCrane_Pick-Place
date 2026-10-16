# RASTICrane stamp machine

Everything built for the ME416 stamp machine, apart from the lab report itself.

## dashboard/
The live control console. `server.py` owns the USB serial link and serves `web/index.html`;
`vision.py` does AprilTag detection and the table homography; `plan.py` holds the kinematics,
the touch-based arm fit and the letter planner.

    cd dashboard && python server.py        # opens http://127.0.0.1:8787
    # needs: pip install pyserial opencv-contrib-python-headless numpy

State lives in JSON beside the server: `config.json` (geometry, calibration, limits, tag
offsets), `points.json` (taught poses and recorded paths), `sequences.json` (programs),
`home.json` (base pulse pointing at the middle of the holder arc), `vision.json` (last photo
analysis), `cad_geometry.json` (holder arc fitted from CAD).

`tools/` reads the CAD: `read_cad.py` walks a .glb applying node transforms, `fit_holder_arc.py`
fits the holder arc, `fit_holder_slots.py` locates the pockets, `build_archive.py` made this zip.

Docs: `README.md`, `STAMPING_GUIDE.md`, `FIT_ARM_GUIDE.md`.

## matlab_vision/
The earlier MATLAB pipeline: `analyzeSetup.m` (photo to robot coordinates), `visionConfig.m`
(the one file holding registration and the IK model), `selectTargets.m`, `calibrateBaseAzimuth.m`,
`planStampMoves.m`, plus the kinematics helpers and the run outputs. Annotated detection PNGs
over 2 MB were dropped; every analysis.json, corners.csv, targets.csv and .mat is here.

## firmware/
`Rasticrane_XYZ_Test.ino` as flashed, plus the backups taken before each change. Servos are on
GPIO 8 (M1 shoulder), 9 (M2 forearm) and 10 (M3 base, DS-S006L 300 deg). LEDC only, never
ESP32Servo.

## early_arduino_sketches/
The sketches from the start of the project: the first LEDC sweep, the Wokwi diagram, the
three-servo tests, and the servo-zero and dance sketches.

## cad/ and photos/
The OnShape exports, and the setup photos taken through the build.

## Key numbers

| | |
|---|---|
| Base calibration | 5.897 us/deg, zero at 1976.7 us, measured three ways |
| Base table | 1300/1900/2500 us to -114.75/-13.01/88.74 deg |
| Home anchor | 2204 us points at the middle of the holder arc |
| Rack | slot 2 at 2487 us, slot 13 at 1940 us, 13 slots, 49.7 us pitch |
| Holder arc (CAD) | centre (6.5, 24.2) mm from the base, radius 124.6 mm, span 13.6-136.4 deg |
| Shoulder height (CAD) | 60.7 mm above the table |
| Manual limits | M1/M2 1500-2500 us, base 1250-2500 us |

M1 and M2 angle-vs-pulse were never properly measured; the values in `config.json` are either
inherited photo estimates or a rough touch fit. Anything computed from them is approximate,
which is why the working programs are taught poses in pulse space rather than solved positions.
