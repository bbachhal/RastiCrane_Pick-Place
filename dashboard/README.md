# RASTICrane Console — live dashboard

A local web UI for driving the arm with sliders, teaching points by eye, and replaying them
as a sequence. No MATLAB needed for day-to-day work.

A browser cannot open COM3, so `server.py` owns the serial port and the page talks to it on
localhost. The server never sends a command on its own — only what you click.

## Run it

```
cd C:\Users\bachh\Downloads\RasticraneDashboard
python server.py
```

It opens http://127.0.0.1:8787. Ctrl+C in that window stops it.
Requires `pyserial` (already installed: `python -m pip install --user pyserial`).

**Close the Arduino Serial Monitor and release the port in MATLAB (`clear robotPort`) first** —
only one program can hold COM3.

## Using it

1. **Connect.** Pick the port, click Connect. Opening the port can reset the ESP32, which then
   runs its staggered startup home (1500/1500/1900), so support the arm. You get asked first.
2. **Jog.** Three sliders, one per motor, bounded by the firmware's own manual limits
   (M1/M2 1500–2500, base 1250–2500). ±5 and ±25 µs buttons for fine work. "Send while
   dragging" is on by default and debounced; turn it off to send only on release.
3. **Watch the model.** Live X/Y/Z, radius and azimuth, computed with the same kinematics as
   `rasticraneFk.m` and the same calibration tables as the firmware. The side view shows the
   linkage; the top view shows azimuth with your vision targets and taught points marked.
4. **Teach a step.** Two kinds:
   - **Save pose** — stores the final position only. Fine for open moves.
   - **Record moves** — press Record, drive the arm the way it has to be driven (including the
     fine-tuning nudges that keep you clear of neighbouring stamps), press Stop, name it, and
     **Save recording as step**. It stores every command in order with the real pauses between
     them, and replays the maneuver move for move.
   With **add to sequence** ticked (default), saving appends the step to the current sequence,
   so you can teach the whole job and then just press Run.
5. **Replay.** Each saved step has **replay**/**go**, **+seq**, and **load** (fills the sliders
   without sending). Build a **Sequence** and run it with a dwell between steps. Sequences save
   to `sequences.json` by name.
6. **Console.** Any firmware command can be typed raw: `ik x y z`, `status`, `cal`, `planar r z`.

## Why points are stored as pulses

M1 and M2 are still photo estimates, so any XYZ outside their calibrated span is extrapolation.
Teaching in **pulse space** sidesteps that entirely: what you saw is what gets replayed, and no
IK solve is needed. The XYZ saved alongside is a label, not the command. Whenever any motor is
outside its calibration samples, the page says so in amber and the saved point records
`extrapolated: true`.

`points.json` next to `server.py` holds them — plain JSON, easy to back up or hand to MATLAB.

## Safety

- **STOP (hold)** is always in the header; it sends `stop`, which holds the current pulses.
- **off — limp** is separate and asks first, because the arm drops.
- Moving to a saved point and running a sequence both ask for confirmation.
- The firmware remains the authority: it clamps every pulse, ramps all motion, and rejects
  anything outside its own bounds. The dashboard cannot talk it past that.

## What this does not do

- It does not solve IK for an arbitrary XYZ. Use the firmware's `ik x y z` preview, or MATLAB's
  `planStampMoves`, for that. The dashboard is a teaching pendant, not a planner.
- It has no collision model and no feedback that the arm arrived.
- Vision targets are drawn from `config.json`; they are not re-read from a new photo. After you
  re-photograph the setup, update `visionTargets` there (values come from `targetSelection.mat`).

## Setup vision — overhead photo

Upload an overhead photo of the setup and the server reads the AprilTags with OpenCV
(`DICT_APRILTAG_36h11`), then puts everything in one frame:

- **Origin** = the centre of **tag 5**, which sits under the robot's base plate.
- **0 degrees** = the direction from the origin to the **middle of the stamp holder arc**,
  built from tag 0's top-left corner, tag 1's top-right corner, and the arc radius from CAD
  (124.6 mm, fitted from the holder mesh in `Part Studio 1.glb`).
- **Ink and paper** come from tags 2 and 3 plus the offsets measured in MATLAB. Those offsets
  are stored in each tag's own frame, so they stay correct when the objects are moved and
  re-photographed.

What this is not: a calibrated camera pose. One tag's four corners give a table-plane
homography, so anything sitting above the table — tags 0 and 1 on the holder — is projected
down onto that plane and reads further out than it is. Tag 5's recovered side length is
printed as a sanity check; if it is not ~50 mm, something is off.

## What the CAD gave us

`tools/read_cad.py` and `tools/fit_holder_arc.py` walk the glTF scene graph applying every node
transform, so parts land in real world positions. Results are in `cad_geometry.json`.

From `Part Studio 1.glb` (origin = `base_table_bottom` = the robot's own base plate):

| Feature | Value |
|---|---|
| Holder arc centre | (6.5, 24.2) mm from the robot base |
| Holder arc radius | 124.6 mm |
| Holder arc span | 13.6° to 136.4°, symmetric about **75°** |
| Holder shelf / top | z = 22.5 mm / 36.5 mm |
| Pocket pitch | ~10.5° along the arc (count not readable from the mesh) |
| Ink foam | (152.1, −10.7, 4.5), 110 × 70 mm |
| Paper | (−198.9, −28.9, 0), 215.9 × 279.4 mm |

The robot base, the arc centre and the arc midpoint are collinear at 75°, which is why
"point at the middle of the arc" is a well-defined home.

From `RASTICrane Assm, v0.1.glb`: the shoulder servo horns sit at z = 77.1 mm with the table at
z = 16.4 mm, so the **shoulder is 60.7 mm above the table** — independent corroboration of the
60 mm in the model.

What the CAD does **not** give, and still needs measuring:

- **a1 and a2.** The CAD rods are diagonal members; their bounding boxes are not pivot-to-pivot
  distances. The 80/80 mm in the model stays a user number.
- **Tool offsets.** The effector in this CAD is a claw, not the magnet you run now.
- **M1/M2 angle vs pulse.** Geometry cannot supply this; it needs the inclinometer sweep.
- **Slot count.** Pocket pitch is visible, the number of pockets is not.

Also note the CAD is design intent: it places the ink at radius 152 mm, azimuth −4°, while the
photo measured radius 107 mm, azimuth −98°. The photo wins.

## Home anchor

The base plate comes off, so "where is 0 degrees" cannot live in the firmware. Instead:

1. Point the arm at the middle of the holder arc.
2. Click **Set home = current base pulse**.

That writes `home.json`, and every azimuth from vision converts to a base pulse with
`pulse = home + azimuth * 5.897`, where 5.897 µs/deg comes from the measured base calibration.
**Redo this one click after any reassembly** and every stored azimuth stays valid.

**Go to ink / Go to paper** swing the base to the vision azimuth and nothing else. Height and
reach are not computed, because the M1/M2 angle tables are still photo estimates — teach those
with recorded steps.

## Files

```
server.py            serial bridge + static files + points/sequences/vision/home storage
vision.py            AprilTag detection and the table-plane frame
web/index.html       the whole UI (no build step, no external libraries)
config.json          geometry, calibration tables, limits, vision targets and tag offsets
cad_geometry.json    holder arc and part layout fitted from the CAD
points.json          taught poses and recorded paths
sequences.json       saved programs
vision.json          last photo analysis
home.json            the base pulse that points at the holder arc middle
tools/read_cad.py    walks a .glb scene graph, prints world-space part boxes in mm
tools/fit_holder_arc.py  fits the holder arc from CAD vertices, writes cad_geometry.json
```

Calibration in `config.json` mirrors the firmware as of 2026-10-14: base measured
(1300/1900/2500 µs ↔ −114.75/−13.01/88.74°), M1 and M2 still photo estimates.
If you change the firmware tables, change them here too.
