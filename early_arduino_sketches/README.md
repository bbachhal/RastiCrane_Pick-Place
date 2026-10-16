# Robotic Arm — Build Constraints and Code Rules

Hand this file to any AI model before it writes code for this arm.
Everything below is a hard constraint unless marked UNVERIFIED.
If a request conflicts with a rule here, stop and ask the human instead of guessing.

---

## 1. Hardware

| Item | Value |
|---|---|
| Board | ESP32-S3 Supermini |
| Arduino IDE board setting | **ESP32S3 Dev Module** (NOT "ESP32-S3 Dev Board") |
| ESP32 Arduino core installed | **3.3.11** (core 3.x API) |
| Serial port | COM3 (USB ID 303A:1001, native USB CDC) |
| Required IDE setting | **USB CDC On Boot = Enabled** (otherwise Serial prints nothing) |
| Serial baud | 115200 |
| Serial Monitor line ending | Newline |

### Servos and pins

| Name | GPIO | Type | Notes |
|---|---|---|---|
| Servo 1 | **8** | 180 degree positional | Arm joint, lifts against gravity |
| Servo 2 | **9** | 180 degree positional | Arm joint, lifts against gravity |
| Servo 3 (base) | **10** | "360" — treated as POSITIONAL | Rotating base, light load, works reliably |

Servo wire colors: **yellow = signal** (to the GPIO), **red/orange = +5V**, **brown = GND**.
Each servo's signal wire goes to its own GPIO. Power and ground are shared.

---

## 2. Absolute code rules

### 2.1 PWM generation

1. **NEVER use the ESP32Servo library.** It is broken on this ESP32-S3 setup. Do not use
   `Servo`, `ESP32PWM`, `servo.attach()`, `servo.write()`, or `servo.writeMicroseconds()`.
2. **Use the LEDC peripheral directly.** Core 3.x API:
   ```cpp
   ledcAttach(pin, 50, 12);   // pin, 50 Hz, 12-bit resolution
   ledcWrite(pin, duty);      // core 3.x writes BY PIN, not by channel
   ```
   Core 2.x used `ledcSetup(channel, ...)` + `ledcAttachPin()` + `ledcWrite(channel, ...)`.
   Keeping a `#if ESP_ARDUINO_VERSION >= ESP_ARDUINO_VERSION_VAL(3,0,0)` fallback is fine,
   but the installed core is 3.3.11, so the 3.x branch is the one that runs.
3. **Do NOT use `analogWrite()`** for servos.
4. **PWM frequency: 50 Hz** (20 ms period). Do not change.
5. **Resolution: 12 bits** in the current working sketch. Duty conversion:
   ```cpp
   duty = (us * 4095) / 20000;   // us = pulse width in microseconds
   ```
   14-bit also works (`duty = us * 16384 / 20000`). **The maximum on the ESP32-S3 is 14 bits.**
   16-bit is invalid here and will fail.
6. **Work in microseconds, not degrees.** The horns were re-seated by hand, so the relationship
   between servo angle and joint angle is not calibrated. Code that talks in degrees will be
   wrong. All positions, limits and commands are in microseconds of pulse width.

### 2.2 Travel limits — the safety-critical part

| Servo | Home (start) | Allowed range | Notes |
|---|---|---|---|
| Servo 1 (GPIO 8) | **1500 us** | **1500 – 2500 us** | 1500 is home AND the lower end of travel. **NEVER command below 1500.** |
| Servo 2 (GPIO 9) | **1500 us** | **1500 – 2500 us** | Same: never below 1500. |
| Servo 3 (base, GPIO 10) | **1900 us** | **1650 – 2150 us** — UNVERIFIED | Placeholder only. Ask the human for the real base limits before widening. |

Rules:

7. **Clamp every value before it reaches the hardware**, whether it came from a typed command,
   a routine, or a calculation: `constrain(us, SAFE_MIN_US[i], SAFE_MAX_US[i])`.
8. **Never write a pulse outside 500–2500 us** under any circumstance (hardware bound), and
   never outside the per-servo window in the table above (build bound).
9. **Do not add test or sweep routines that ignore the limits.** A full 500 to 2500 sweep on
   servo 1 or 2 drives the joint into a hard stop.
10. A servo pushed against a hard stop buzzes, stalls, overheats and can burn out. Treat
    "buzzing but not moving" as an emergency: stop the motion and de-energize.

### 2.3 Motion

11. **Always ramp; never jump.** Step toward the target in small increments on a timer.
    Current working value: **25 us per 20 ms step** (about 1250 us/s). 15 us per 20 ms is the
    slower setting also used. Do not exceed about 40 us per 20 ms.
12. **Non-blocking only.** Use `millis()` timing. Do not use `delay()` inside movement loops;
    it blocks the Serial stop command, which is the emergency stop.
13. **At startup, drive the servos to the home positions above.** Staggering them (one joint at
    a time, 0.5–1 s apart) is preferred; three simultaneous jumps cause a current spike.
14. Every automatic routine must be **interruptible from Serial** and must **stop itself** after
    a bounded number of cycles rather than running forever unattended.

### 2.4 De-energizing

15. Writing duty `0` stops the pulses and the servo goes **limp**. On this arm, joints 1 and 2
    **drop under gravity** when limp. Limp is NOT a safe stop for a raised arm.
16. A safe stop = **stop the motion but keep sending the current pulse**, so the joint holds.
17. Any code that de-energizes must say so in its Serial output and in its help text.

---

## 3. Serial command protocol (current working sketch)

Commands are newline-terminated, read with `Serial.readStringUntil('\n')` then `trim()`.

| Command | Action |
|---|---|
| `<motor> <value>` | Move one servo. Motor = 1, 2, or 3 (optional `M` prefix). Value = 0.50–2.50 (ms) or 500–2500 (us). Clamped to the safe window. |
| `zero` | All servos to home (1500 / 1500 / 1900) |
| `<motor> zero` | One servo to its home |
| `0` or `off` | ALL servos de-energized (limp — the arm drops) |
| `<motor> off` or `<motor> 0` | One servo de-energized (limp) |
| `dance` | Run the scripted routine |
| `stop` | End the routine, hold current position |

Keep these names and meanings stable. In particular: **`zero` moves to home, `0`/`off` goes
limp.** Do not swap those two meanings; muscle memory here is a safety issue.

---

## 4. Known problems to design around

18. **Power is the main unresolved issue.** Servos 1 and 2 have stalled and buzzed while lifting
    the arm. If they are powered from the ESP32's USB (about 0.5–0.9 A), that is not enough for
    two loaded servos. Recommended: a separate **5–6 V supply rated 3 A or more**, with its
    ground tied to the ESP32 GND. Symptoms of insufficient power: buzzing, creeping, board
    resets, Serial disconnects mid-move. UNVERIFIED: what the arm is powered from right now.
19. Faster motion and longer reach both increase current draw. If the board resets during a
    routine, slow the ramp or reduce the reach.
20. The base servo is treated as **positional** and responds correctly to position commands.
    UNVERIFIED: whether it is a 300 or 360 degree unit, and its model number.
21. UNVERIFIED: servo model numbers, stall currents, and the arm's mechanical joint ranges
    beyond the microsecond limits in section 2.2.

---

## 5. Checklist before handing code back

- [ ] No ESP32Servo, `Servo`, or `ESP32PWM` anywhere
- [ ] Uses `ledcAttach(pin, 50, 12)` and `ledcWrite(pin, duty)` (core 3.x, by pin)
- [ ] Pins are 8, 9, 10 for servo 1, servo 2, base
- [ ] Every write clamped to the per-servo window; servos 1 and 2 never below 1500 us
- [ ] Base stays within 1650–2150 us unless the human has given new limits
- [ ] Motion ramps at 40 us per 20 ms or slower; no `delay()` in the movement path
- [ ] Startup sends the home positions (1500 / 1500 / 1900)
- [ ] A stop command exists, works mid-move, and HOLDS position rather than going limp
- [ ] Any limp or de-energize path is clearly labeled as "the arm will drop"
- [ ] Automatic routines are bounded and interruptible
- [ ] Nothing in the code can command a pulse outside 500–2500 us

If any constraint above is missing from the request, ask the human rather than inventing a value.
