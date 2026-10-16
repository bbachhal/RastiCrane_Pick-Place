"""Collect the whole RASTICrane project into one folder and zip it.

Deliberately excluded: the lab report PDFs and the script that builds them, Python caches,
and the 12 MB annotated detection images from every MATLAB run (the newest two are kept,
along with every .json/.csv/.mat result).
"""
import os
import shutil
import zipfile

HOME = os.path.expanduser("~")
DL = os.path.join(HOME, "Downloads")
STAGE = os.path.join(DL, "RASTICrane_Archive")
TEMP_IMAGES = os.path.join(
    HOME, "AppData", "Local", "Temp", "claude",
    "C--Users-bachh-AppData-Roaming-Claude-scratch-workspaces-6ff03199-a399-4f2a-863b-809779ed732f-"
    "b585d2a0-791c-45f8-9093-2a9eac627cf9-scratch-2026-09-15-20966a",
    "54960340-83f1-441f-a8d3-79878e165fce", "images")
SCRATCH = os.path.join(
    HOME, "AppData", "Roaming", "Claude", "scratch-workspaces",
    "6ff03199-a399-4f2a-863b-809779ed732f", "b585d2a0-791c-45f8-9093-2a9eac627cf9",
    "scratch-2026-09-15-20966a")

SKIP_NAMES = {"__pycache__", ".git", "ME416_Lab2_Report.pdf", "ME416_Lab2_Report_DRAFT.pdf",
              "make_report.py", "pdftext.py"}
SKIP_EXT = {".pyc"}


def copy_tree(src, dst, skip_heavy=False):
    if not os.path.isdir(src):
        print("  missing:", src)
        return 0
    count = 0
    for root, dirs, files in os.walk(src):
        dirs[:] = [d for d in dirs if d not in SKIP_NAMES]
        for name in files:
            if name in SKIP_NAMES or os.path.splitext(name)[1] in SKIP_EXT:
                continue
            full = os.path.join(root, name)
            if skip_heavy and os.path.getsize(full) > 2_000_000:
                continue
            rel = os.path.relpath(full, src)
            out = os.path.join(dst, rel)
            os.makedirs(os.path.dirname(out), exist_ok=True)
            shutil.copy2(full, out)
            count += 1
    return count


if os.path.isdir(STAGE):
    shutil.rmtree(STAGE)
os.makedirs(STAGE)

print("dashboard      :", copy_tree(os.path.join(DL, "RasticraneDashboard"),
                                    os.path.join(STAGE, "dashboard")), "files")
print("matlab vision  :", copy_tree(os.path.join(DL, "RasticraneVisionDraft"),
                                    os.path.join(STAGE, "matlab_vision"), skip_heavy=True), "files")
print("handoff        :", copy_tree(os.path.join(DL, "RasticraneHandoff"),
                                    os.path.join(STAGE, "handoff")), "files")
print("firmware       :", copy_tree(os.path.join(HOME, "OneDrive", "Documents", "Arduino",
                                                 "Rasticrane_XYZ_Test"),
                                    os.path.join(STAGE, "firmware", "Rasticrane_XYZ_Test")), "files")
print("early sketches :", copy_tree(SCRATCH, os.path.join(STAGE, "early_arduino_sketches")), "files")
print("setup photos   :", copy_tree(TEMP_IMAGES, os.path.join(STAGE, "photos")), "files")

cad_dir = os.path.join(STAGE, "cad")
os.makedirs(cad_dir, exist_ok=True)
for name in ("RASTICrane Assm, v0.1.glb", "Part Studio 1.glb"):
    src = os.path.join(DL, name)
    if os.path.isfile(src):
        shutil.copy2(src, os.path.join(cad_dir, name))
        print("cad            :", name)

readme = """# RASTICrane stamp machine

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
"""
with open(os.path.join(STAGE, "README.md"), "w", encoding="utf-8") as handle:
    handle.write(readme)

zip_path = os.path.join(DL, "RASTICrane_Archive.zip")
if os.path.exists(zip_path):
    os.remove(zip_path)
with zipfile.ZipFile(zip_path, "w", zipfile.ZIP_DEFLATED) as zf:
    for root, _, files in os.walk(STAGE):
        for name in files:
            full = os.path.join(root, name)
            zf.write(full, os.path.relpath(full, STAGE))

total = sum(len(files) for _, _, files in os.walk(STAGE))
print(f"\n{total} files -> {zip_path} ({os.path.getsize(zip_path)/1e6:.1f} MB)")
