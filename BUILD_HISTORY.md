# Build history

A generalized record of how the stamp machine was built, in the order it happened. Covers the
build only, not the lab report.

## 1. First motion
Started with a bare ESP32-S3 and three servos, driven with the LEDC peripheral directly rather
than ESP32Servo, which does not work reliably on the S3. Sketches were checked in Wokwi before
touching hardware. Early versions swept the servos slowly between limits and accepted pulse
widths over serial, with everything clamped and ramped. The arm was treated as fragile from the
start: no automatic motion at power-up beyond a staggered move to a home pose, and a `stop` that
holds position rather than going limp, because a limp arm falls.

## 2. Onto the real arm
Servos were wired to GPIO 8 (shoulder), 9 (forearm) and 10 (base, a 300-degree DS-S006L). An
existing Cartesian sketch was adopted and reduced to the pins and limits of this build. Setup
friction took a while: the ESP32 board package was not installed, the port had to be identified
as COM3 by its USB ID, and the IDE needed restarting before the board list populated.

## 3. The base calibration, and a wrong assumption
The inherited model assumed base 1900 us pointed along +X. Sighting the arm at three known
directions and fitting a line gave **5.897 us/deg with zero at 1976.7 us** — the old assumption
was off by about 13 degrees. Every earlier Cartesian move had therefore been pointing somewhere
other than where the model believed. Firmware limits were widened from 1650-2150 to 1250-2500 on
the base so the arm could actually reach the work area, and the Cartesian envelope was opened to
the calibrated span.

## 4. The dashboard
Driving the arm by typing serial commands did not scale, so the project moved to a browser
console backed by a small Python server that owns the USB port. It gives live jog sliders, a
side and top view of the arm computed from the same kinematics as the firmware, taught points,
and sequences. Several bugs surfaced and were fixed along the way: a stale serial handle that
reported "connected" after the board was unplugged, pose dropdowns that never populated, and a
sequence runner that reported success after a failed send.

Later the console gained **move recording**: every command sent is captured in order with the
real pauses between them, so an approach that threads past neighbouring stamps can be replayed
move for move rather than reduced to a single pose.

## 5. Vision
One overhead photo, phone camera calibrated in MATLAB, undistorted before measurement. AprilTags
(tag36h11, 50 mm) are detected with OpenCV; the four corners of the tag under the arm give a
table homography, which fixes an origin and a frame. Ink pad and paper targets are picked on the
photo and stored as offsets in their own tags' frames, so they follow the objects when moved.

Two real limitations were found and documented rather than papered over. Tags sitting above the
table, like the two on the holder, get projected onto the table plane and read 3-7% long. And
the frame built from a single 50 mm tag disagreed with the arm's own driven frame by roughly
8-23 degrees depending on how it was measured — enough that the working programs ended up using
driven and taught geometry for the rack rather than tag-derived azimuths.

## 6. CAD
The OnShape exports were mined with a scene-graph walker that applies node transforms, since
raw mesh extents are meaningless for layout. Fitting a circle to 501 vertices of the holder gave
an **arc radius of 124.6 mm** centred 25 mm from the base, with the shelf at 22.5 mm, the top at
36.5 mm and pockets repeating every ~10.5 degrees. The arm export independently confirmed the
**60.7 mm shoulder height**. Link lengths, tool offsets and the stamp-to-disc geometry were not
recoverable from the exports and stayed as hand measurements.

## 7. Home, and a reachability problem
Because the base plate comes off, home was redefined as *the bearing to the middle of the holder
arc*, stored as a single anchor pulse that takes one click to re-establish after reassembly.
Checking reach against that anchor showed the paper sat at +93 degrees, needing a base pulse
past the servo's limit. Rather than re-seat the horn, the paper was moved beside the ink pad,
which put every target inside the available travel.

## 8. From computed poses to taught poses
The original plan was to compute every pose from vision and inverse kinematics. That stalled on
the shoulder and elbow calibration, which was never measured and whose inherited tables covered
only part of the servo range. A planner bug made this worse by silently clamping out-of-range
angles to the end of the table and reporting them as reachable; once that was fixed, 34 of 36
planned steps were correctly refused.

The approach that worked: **teach one arm pose per action and let the base do the turning.** Slot
radius varies only ~6 mm across the rack, so a single hover pose covers every slot. The rack was
measured by driving to the first and last slot and interpolating — 13 slots, 49.7 us apart. Ink
and paper got taught hover and contact poses.

## 9. Tuning the cycle
The pick, ink, press, return and release cycle came together through repeated runs:

- **Vertical approach only.** All sideways motion happens at a transit height above the stamps;
  all descent happens with the base locked.
- **Grab depth** tuned in 3 mm steps until the magnet lifted the stamp reliably.
- **Seat depth** was found to be 7.5 mm *above* the depth the stamp was picked from, which is why
  stamps sat proud and would not release. Correcting it mattered more than any wipe change.
- **The wipe** went through several versions. Retracting the shoulder alone swings the tool up an
  arc, lifting the magnet over the stamp and re-attaching it; dropping the forearm as the
  shoulder retracts holds the height flat and strips the stamp against the holder lip. Slots at
  different angles needed slightly different wipe directions.
- **Ink and paper contact** got multiple impressions with a lift between them, after a single
  press left partial letters. A firmware floor on the forearm channel was found to be clamping
  the ink press, which explained why "deeper" was not deeper.
- **Pacing** was made adaptive: wait as long as each move actually needs, hold longer on contact
  steps. The full three-letter run went from 7.8 minutes to about 2.

## 10. Where it ended
A single sequence picks each stamp in turn, inks it, presses it on the page, returns it to its
slot and wipes the magnet free, with every position either measured against the arm's own frame
or taught by hand. The geometry that is well measured — the base calibration and the rack — is
trusted; the geometry that is not — the shoulder and elbow angle tables — is bypassed rather
than relied on.
