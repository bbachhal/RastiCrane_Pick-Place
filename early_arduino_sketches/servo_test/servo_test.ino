// Three-servo SAFE hardware test for ESP32-S3 Supermini using raw LEDC (NOT ESP32Servo).
// Arduino IDE board: "ESP32S3 Dev Module", USB CDC On Boot: Enabled
// Serial Monitor: 115200 baud, line ending "Newline"
//
// On boot the servos go to CENTER (1500 us = 1.5 ms), one at a time, and then WAIT.
// Nothing else moves until you type a command:
//   1.5      -> all servos slowly go to 1.5 ms, one at a time
//   2 1.2    -> servo 2 only slowly goes to 1.2 ms
//   c        -> slowly center all servos (1.5 ms)
//   t        -> safety check: each servo on its own, 1.5 -> 1.0 -> 2.0 -> 1.5 ms
//   s        -> STOP: pulses off, servos go limp (works even mid-move)
//   h        -> show this help

// Servo signal wires: servo 1 -> GPIO 8, servo 2 -> GPIO 9, servo 3 -> GPIO 10
const int SERVO_PINS[] = {8, 9, 10};
const int NUM_SERVOS   = sizeof(SERVO_PINS) / sizeof(SERVO_PINS[0]);

const int FREQ_HZ   = 50;                  // standard servo frame rate
const int RES_BITS  = 14;                  // ESP32-S3 LEDC max
const int PERIOD_US = 1000000 / FREQ_HZ;   // 20000 us

const int CENTER_US = 1500;   // 1.5 ms, where every servo starts

// Limits for typed commands (what you asked for: 0.5 to 2.5 ms).
// Many real servos can't physically reach these ends and will buzz/strain.
// If that happens, tighten these, e.g. 800 and 2200.
const int MIN_US = 500;
const int MAX_US = 2500;

// Range used by the automatic safety check ('t'). Deliberately narrow: safe for almost any servo.
const int TEST_MIN_US = 1000;
const int TEST_MAX_US = 2000;

// Speed: change the pulse by STEP_US every STEP_MS.
// 5 us / 20 ms = 250 us per second (roughly 20 deg/s). Very slow.
// Smaller STEP_US or bigger STEP_MS = even slower.
const int STEP_US = 5;
const int STEP_MS = 20;

int  currentUs[NUM_SERVOS];
bool pulsesOn = false;

// Output a pulse width (microseconds) on servo i, immediately.
void writePulse(int i, int us) {
  uint32_t duty = (uint32_t)us * (1UL << RES_BITS) / PERIOD_US;
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcWrite(SERVO_PINS[i], duty);   // core 3.x: by pin
#else
  ledcWrite(i, duty);               // core 2.x: by channel
#endif
  currentUs[i] = us;
}

// Stop sending pulses. Most servos go limp.
void stopAll() {
  for (int i = 0; i < NUM_SERVOS; i++) {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
    ledcWrite(SERVO_PINS[i], 0);
#else
    ledcWrite(i, 0);
#endif
  }
  pulsesOn = false;
  Serial.println("STOPPED: pulses off, servos limp. Type c to re-center.");
}

// Restart pulses at each servo's last commanded position.
void resumeIfStopped() {
  if (pulsesOn) return;
  for (int i = 0; i < NUM_SERVOS; i++) writePulse(i, currentUs[i]);
  pulsesOn = true;
}

// True if the user typed 's' (checked during moves so you can abort).
bool stopRequested() {
  if (Serial.available() && (Serial.peek() == 's' || Serial.peek() == 'S')) {
    Serial.readStringUntil('\n');   // consume the command
    return true;
  }
  return false;
}

// Ramp servo i from its current pulse to targetUs, slowly. Returns false if aborted.
bool moveSlowly(int i, int targetUs) {
  targetUs = constrain(targetUs, MIN_US, MAX_US);
  resumeIfStopped();
  Serial.printf("Servo %d (GPIO %d): %.2f ms -> %.2f ms\n",
                i + 1, SERVO_PINS[i], currentUs[i] / 1000.0, targetUs / 1000.0);
  while (currentUs[i] != targetUs) {
    if (stopRequested()) { stopAll(); return false; }
    int step = constrain(targetUs - currentUs[i], -STEP_US, STEP_US);
    writePulse(i, currentUs[i] + step);
    delay(STEP_MS);
  }
  return true;
}

// Each servo on its own through a safe range, back to center.
void safetyCheck() {
  Serial.println("\n=== Safety check: one servo at a time (type s to stop) ===");
  for (int i = 0; i < NUM_SERVOS; i++) {
    Serial.printf("-- Servo %d (GPIO %d) --\n", i + 1, SERVO_PINS[i]);
    if (!moveSlowly(i, TEST_MIN_US)) return;
    if (!moveSlowly(i, TEST_MAX_US)) return;
    if (!moveSlowly(i, CENTER_US))   return;
    delay(1000);
  }
  Serial.println("=== Safety check done ===");
}

void printHelp() {
  Serial.println("\nCommands (press Enter after each):");
  Serial.println("  1.5     -> all servos to 1.5 ms, one at a time (0.5 to 2.5 allowed)");
  Serial.println("  2 1.2   -> servo 2 only to 1.2 ms");
  Serial.println("  c       -> center all (1.5 ms)");
  Serial.println("  t       -> safety check (1.0 to 2.0 ms, one servo at a time)");
  Serial.println("  s       -> STOP (servos go limp)");
  Serial.println("  h       -> help");
}

void setup() {
  Serial.begin(115200);
  unsigned long t0 = millis();
  while (!Serial && millis() - t0 < 3000) delay(10);   // give USB serial a moment

  // Attach and center each servo, one at a time with a pause,
  // so they don't all jump (and draw current) at the same moment.
  for (int i = 0; i < NUM_SERVOS; i++) {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
    ledcAttach(SERVO_PINS[i], FREQ_HZ, RES_BITS);
#else
    ledcSetup(i, FREQ_HZ, RES_BITS);
    ledcAttachPin(SERVO_PINS[i], i);
#endif
    writePulse(i, CENTER_US);
    delay(700);
  }
  pulsesOn = true;

  Serial.println("\nAll servos centered at 1.5 ms (1500 us). Waiting for commands.");
  printHelp();
}

void loop() {
  if (!Serial.available()) return;

  String line = Serial.readStringUntil('\n');
  line.trim();
  if (line.length() == 0) return;

  if (line.equalsIgnoreCase("s")) { stopAll(); return; }
  if (line.equalsIgnoreCase("h")) { printHelp(); return; }
  if (line.equalsIgnoreCase("t")) { safetyCheck(); return; }
  if (line.equalsIgnoreCase("c")) {
    for (int i = 0; i < NUM_SERVOS; i++) if (!moveSlowly(i, CENTER_US)) return;
    Serial.println("All centered.");
    return;
  }

  // Either "<ms>" (all servos) or "<servo#> <ms>" (one servo).
  int servoNum = 0;   // 0 = all
  float ms;
  int space = line.indexOf(' ');
  if (space > 0) {
    servoNum = line.substring(0, space).toInt();
    ms = line.substring(space + 1).toFloat();
    if (servoNum < 1 || servoNum > NUM_SERVOS) {
      Serial.printf("Servo number must be 1 to %d\n", NUM_SERVOS);
      return;
    }
  } else {
    ms = line.toFloat();
  }

  if (ms < MIN_US / 1000.0 || ms > MAX_US / 1000.0) {
    Serial.printf("\"%s\" rejected: pulse must be %.1f to %.1f ms. Type h for help.\n",
                  line.c_str(), MIN_US / 1000.0, MAX_US / 1000.0);
    return;
  }

  int us = lround(ms * 1000);
  if (servoNum == 0) {
    for (int i = 0; i < NUM_SERVOS; i++) if (!moveSlowly(i, us)) return;
  } else {
    if (!moveSlowly(servoNum - 1, us)) return;
  }
  Serial.println("Done. Enter next command.");
}
