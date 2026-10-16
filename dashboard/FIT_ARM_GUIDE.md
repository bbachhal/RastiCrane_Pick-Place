# Fitting the M1/M2 arm model by touching known points

This replaces the inherited photo estimates with numbers measured on your own arm. It takes
about 15 minutes and you only do it again if the arm is rebuilt or the magnet mount moves.

The idea: the photo already knows where things are. If you put the magnet exactly on one of
those things and record the pulses, that is one equation about your arm. Do it four times at
different reaches and heights and both servos are solved.

---

## Before you start

1. **Read the photo.** Vision panel → Read tags. The table should list ink, paper and tags 0,
   1, 2, 3, 5 with sensible radii.
2. **Set the home anchor.** Point the arm at the middle of the holder arc, click
   *Set home = current base pulse*. Without this the base cannot aim at anything.
3. **Nothing on the table moves from now until you finish.** If you bump the ink pad or the
   holder, retake the photo and start over.
4. Keep a hand near the USB cable. `stop` holds position; never `off` with the arm up.

---

## Choosing your four touch points

You need **spread in both reach and height**. Four points all at the same radius will fit
badly even if each touch is perfect.

| # | Target | Height to type | Why |
|---|---|---|---|
| 1 | **tag 3 centre** | 4 mm | near, low |
| 2 | **tag 2 centre** | 4 mm | near, low, other side |
| 3 | **paper stamp point** | 0.1 mm | lowest point you have |
| 4 | **holder top at a middle slot** | 36.5 mm | far and high — this one does the heavy lifting |

For #4, aim at a slot with the slot buttons, then touch the **flat top surface of the holder**
beside that slot, not the bottom of a pocket.

**Do not use tag 5.** It is directly under the base and the arm cannot reach its own centre.

**Avoid tags 0 and 1** as touch targets. They sit high on the holder and are tilted, so the
photo's XY for them is a projection, not a true position.

If you want a fifth, the **ink pad centre at 12.3 mm** is a good one.

---

## The touch procedure, one point at a time

1. **Aim the base.** Use *Go to ink*, *Go to paper*, or a slot button. For a tag centre, jog
   the base until the magnet is directly over it, sighting from above.
2. **Get close.** Lower with ±25 µs steps on M1 and M2 until the magnet is roughly 10 mm up.
3. **Sight from the side.** Put your eye level with the table. This is the step people skip and
   it is where most of the error comes from.
4. **Creep down in ±5 µs steps** until the magnet's **underside just kisses the surface**.
   - It should touch, not press. If the arm visibly flexes or a servo buzzes, you have gone
     too far — back off 5 µs.
   - It must be the flat bottom of the magnet, not its edge.
5. **Check the aim again from above.** The magnet should still be centred over the target. If
   it drifted, fix the base and re-do step 4.
6. **Pick that target in the dropdown**, type its **surface height** in the height box, and
   click **Capture touch at current pulses**.
7. Raise 20–30 mm before moving to the next point.

Repeat for all four. The list under the buttons shows what you captured; the ✕ removes a bad one.

---

## Fit it

Click **Fit arm model**. You get:

```
Fitted from 4 touches. Residual RMS 1.8 mm (per touch: 1.2, 2.4, 1.6, 1.9 mm)
M1 1500/2000/2500 µs → 88.4/54.1/19.8°
M2 1500/2000/2500 µs → -71.2/-38.0/-4.8°
```

**Reading the residual:**

| RMS | What it means |
|---|---|
| under 2 mm | good, carry on |
| 2–5 mm | usable, but one touch is probably sloppy — look at the per-touch numbers and redo the worst |
| over 5 mm | something is wrong: a height typed incorrectly, a touch that pressed into the surface, or the setup moved since the photo |

The per-touch list tells you which one disagrees. Remove it with ✕, redo that touch, fit again.

---

## Verify before you trust it

1. Raise the arm, then command the magnet to **hover 20 mm above tag 2's centre**: plan a
   single point, or jog until the readout says that position.
2. **Measure the real gap with a ruler.** If the model is right it will be close to 20 mm. If it
   says 20 and the gap is 35, the fit or the tool offsets are off.
3. Then run **Simulate program** with every height raised 20 mm and watch one dry pass.

---

## Things that quietly ruin the fit

- **Typing the wrong surface height.** The number you type is the Z the solver believes.
- **Pressing instead of touching.** The arm flexes, so the recorded pulses describe a pose the
  arm was never actually in.
- **Touching with the magnet's edge** rather than its flat underside.
- **All four points at similar reach.** Spread them.
- **Moving the setup** between the photo and the touches.
- **Capturing after you have already jogged away.** Capture while the arm is still touching.

---

## What this does and does not fix

It fits **M1 and M2 angle versus pulse**. It cannot fix a wrong link length, a wrong shoulder
height, or wrong magnet offsets — those are the numbers in `config.json` under `geometry`, and
errors there get partly absorbed into the fitted angles, which is why the residual matters.

If you later measure a1, a2, the shoulder height or the magnet offsets properly, update
`config.json` and re-run the fit.
