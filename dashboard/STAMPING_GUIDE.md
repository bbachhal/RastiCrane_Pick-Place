# Stamping "B I R" — what to teach, in what order

Everything here is taught by eye in the dashboard and stored as servo pulses, so none of it
depends on the uncalibrated M1/M2 angle tables. Nothing is computed; what you saw is what replays.

## Before you start

- Board plugged in, dashboard connected (red banner gone), `ready` pressed once.
- Paper taped down so it cannot shift. Ink pad in its final position.
- The three stamps in their rack slots.
- Pencil a small mark on the paper where each letter should land.

### Lay the name out along a radial line, not across

The arm has no wrist rotation. A carried letter rotates by **press azimuth minus pickup azimuth**,
so a letter stamped at a different azimuth from where it was picked up comes out rotated.

If B, I and R are all pressed at the **same azimuth** and differ only in **radius**, all three
come out rotated by the same amount — consistent, and you can just orient the paper to match.
Spreading them across different azimuths gives three differently-rotated letters.

So: mark the three spots along a line pointing away from the robot, roughly 25–30 mm apart.

---

## 1. The points to teach

Shared (teach once):

| Name | What it is |
|---|---|
| `park` | Arm high and clear of everything. Press `ready`, then save this pose. |
| `ink_hover` | Stamp held, hovering ~20 mm above the ink pad. |
| `ink_press` | Stamp face resting on the ink. |

Per letter, with `L` = `B`, `I`, `R`:

| Name | What it is |
|---|---|
| `L_slot_hover` | ~25 mm above that stamp's slot in the rack. |
| `L_slot_grip` | Magnet touching the stamp's steel disc, stamp still seated in the rack. |
| `L_paper_hover` | Stamp held, ~20 mm above that letter's pencil mark. |
| `L_paper_press` | Stamp face resting on the paper at the mark. |
| `L_slot_release` | Same as `L_slot_grip` but slid **radially outward**, which strips the stamp off the magnet while the rack holds it. |

That's 3 shared + 5 per letter = **18 points** for three letters.

---

## 2. Teaching procedure

Work one zone at a time. For every point: jog the **base first** (watch the azimuth in the top
view), then shoulder and elbow for radius and height. Use ±25 µs to get close and ±5 µs to finish.

### Record the approach where the path matters

A saved **pose** jumps all three motors at once toward the final numbers. Where the magnet has to
thread past neighbouring stamps, that straight-line-in-pulse-space move is exactly what you don't
want. For those steps use **Record** instead:

1. Press **● Record moves**.
2. Drive the arm the way it actually has to go in — approach high, come across, drop in, nudge.
   Every command is captured in order, with the real pauses between them.
3. Press **■ Stop recording**, name it, and click **Save recording as step**.

Replay repeats that maneuver move for move, pauses included. Use recordings for
`L_slot_grip` and `L_slot_release`, where clearance is tight, and plain poses for the open
moves like `park` and the hovers.

Keep **add to sequence** ticked and each step lands in the sequence as you teach it, in order —
teach the whole letter, then press Run.

### 2a. Rack points, magnet empty

1. From `park`, jog over slot B. Raise/lower until the magnet is ~25 mm above the disc.
   Name it `B_slot_hover`, kind **hover**, Save.
2. Lower in ±5 steps until the magnet just touches the steel disc. **Stop as soon as it contacts** —
   do not keep driving into it. Name it `B_slot_grip`, kind **stamp**, Save.
3. Repeat for `I` and `R`.

### 2b. Check the magnet actually holds

With the magnet on B's disc, go to `B_slot_hover`. The stamp should come up with it.
If it doesn't, the grip point is slightly too high — lower `B_slot_grip` by 5–10 µs and re-save.

### 2c. Ink points, stamp held

With B attached:
4. Jog over the ink pad, ~20 mm above the foam. Save as `ink_hover`, kind **ink**.
5. Lower in ±5 steps until the stamp face sits on the foam. Save as `ink_press`, kind **ink**.

A held stamp sits well below the magnet, so the magnet stays much higher than the 12.3 mm ink
surface. That is why this is reachable even though a bare-magnet move to 12.3 mm is not.

### 2d. Paper points, stamp held

6. Jog over B's pencil mark, ~20 mm up. Save as `B_paper_hover`, kind **paper**.
7. Lower until the stamp face touches the paper. Save as `B_paper_press`, kind **paper**.
8. Go back to `B_slot_hover`, then `B_slot_grip` so the stamp reseats in the rack, then jog
   **outward in radius only** until the magnet lets go. Save that as `B_slot_release`, kind **stamp**.
9. Repeat 6–8 for `I` and `R`, each over its own mark.

---

## 3. Building a sequence per letter

In the Sequence panel, add these steps in order (B shown):

```
park
B_slot_hover
B_slot_grip
B_slot_hover          <- lifts the stamp clear
ink_hover
ink_press
ink_hover
B_paper_hover
B_paper_press
B_paper_hover
B_slot_hover
B_slot_grip           <- reseats the stamp
B_slot_release        <- magnet slides outward, stamp stays
B_slot_hover
park
```

Name it `letter B`, set **dwell to 2500 ms**, and click **Save**. Repeat for `letter I` and
`letter R`. They persist in `sequences.json`, so they survive reloads and restarts.

**Every move between zones goes through a hover point.** That is the only thing keeping the arm
from dragging across the rack or the paper, since there is no collision checking anywhere.

Dwell matters: a big base swing is about 1050 µs of travel, roughly 1.4 s of ramp. 2500 ms gives
each step time to finish before the next one starts.

---

## 4. First run

1. **Dry run first.** Build a sequence of hover points only — `park`, `B_slot_hover`,
   `ink_hover`, `B_paper_hover`, `park` — and run it. Nothing touches anything.
2. Then run `letter B` with ink. Watch each step; keep a hand near the USB cable.
3. **STOP (hold)** freezes the arm wherever it is. Do not use `off` with the arm up — it drops.
4. After B works end to end, run `letter I`, then `letter R`.

The system cannot tell whether the stamp was actually picked up, inked, or released. Watch the
pickup and the release every time until you trust them.

---

## 5. When something is off

| Symptom | Fix |
|---|---|
| Stamp doesn't lift | `L_slot_grip` is too high. Lower 5–10 µs, re-save. |
| Stamp won't release | Slide further outward in `L_slot_release`, or make sure the rack is retaining it. |
| Letter prints faint | Longer dwell at `ink_press`, or lower it 5 µs. |
| Letter smudges | Raise `L_paper_press` by 5 µs; you are pressing too hard. |
| Arm clips the rack in transit | Raise the hover points, or add an extra mid-air point between zones. |
| Letters come out rotated differently | The three paper points are at different azimuths. See the radial-line note above. |
| A servo buzzes and stops | It is against a stop or stalling. **STOP** immediately, back off 10 µs. |

Back up `points.json` and `sequences.json` once the letters work — they are the whole program.
