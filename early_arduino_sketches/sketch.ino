// Three-servo SLOW test for ESP32-S3 using raw LEDC (NOT ESP32Servo).
// Board: "ESP32S3 Dev Module"
//
// On boot: each servo moves on its own, slowly, through 0.5 ms -> 2.5 ms -> 1.5 ms.
// Then type commands in the Serial Monitor (115200 baud) and press Enter:
//   1.5      -> all servos go to a 1.5 ms pulse, one servo at a time
//   2 0.8    -> servo 2 only goes to a 0.8 ms pulse
//   c        -> center all servos (1.5 ms)
//   t        -> run the safety check again

const int SERVO_PINS[] = {9, 10, 11};
const int NUM_SERVOS   = sizeof(SERVO_PINS) / sizeof(SERVO_PINS[0]);

const int FREQ_HZ   = 50;                  // standard servo frame rate
const int RES_BITS  = 14;                  // ESP32-S3 LEDC max
const int PERIOD_US = 1000000 / FREQ_HZ;   // 20000 us

const int MIN_US    = 500;    // 0.5 ms
const int MAX_US    = 2500;   // 2.5 ms
const int CENTER_US = 1500;   // 1.5 ms

// Speed: change the pulse by STEP_US every STEP_MS.
// 5 us / 20 ms = 250 us per second, so the full 0.5 -> 2.5 ms range takes 8 s.
// Make STEP_US smaller or STEP_MS bigger to go even slower.
const int STEP_US = 5;
const int STEP_MS = 20;

int currentUs[NUM_SERVOS];

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

// Ramp servo i from its current pulse to targetUs, slowly.
void moveSlowly(int i, int targetUs) {
  targetUs = constrain(targetUs, MIN_US, MAX_US);
  Serial.printf("Servo %d (pin %d): %.2f ms -> %.2f ms\n",
                i + 1, SERVO_PINS[i], currentUs[i] / 1000.0, targetUs / 1000.0);
  while (currentUs[i] != targetUs) {
    int step = constrain(targetUs - currentUs[i], -STEP_US, STEP_US);
    writePulse(i, currentUs[i] + step);
    delay(STEP_MS);
  }
}

// Move each servo on its own through its full range, then back to center.
void safetyCheck() {
  Serial.println("\n=== Safety check: one servo at a time ===");
  for (int i = 0; i < NUM_SERVOS; i++) {
    Serial.printf("-- Servo %d (pin %d) --\n", i + 1, SERVO_PINS[i]);
    moveSlowly(i, MIN_US);
    moveSlowly(i, MAX_US);
    moveSlowly(i, CENTER_US);
    delay(1000);
  }
  Serial.println("=== Safety check done ===");
}

void printHelp() {
  Serial.println("\nType a pulse width in ms (0.5 to 2.5) and press Enter:");
  Serial.println("  1.5     -> all servos, one at a time");
  Serial.println("  2 0.8   -> servo 2 only");
  Serial.println("  c       -> center all");
  Serial.println("  t       -> rerun safety check");
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  // Attach and center each servo, one at a time with a pause,
  // so they don't all jump at once on power-up.
  for (int i = 0; i < NUM_SERVOS; i++) {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
    ledcAttach(SERVO_PINS[i], FREQ_HZ, RES_BITS);
#else
    ledcSetup(i, FREQ_HZ, RES_BITS);
    ledcAttachPin(SERVO_PINS[i], i);
#endif
    writePulse(i, CENTER_US);
    delay(500);
  }

  safetyCheck();
  printHelp();
}

void loop() {
  if (!Serial.available()) return;

  String line = Serial.readStringUntil('\n');
  line.trim();
  if (line.length() == 0) return;

  if (line.equalsIgnoreCase("t")) { safetyCheck(); printHelp(); return; }
  if (line.equalsIgnoreCase("c")) {
    for (int i = 0; i < NUM_SERVOS; i++) moveSlowly(i, CENTER_US);
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

  if (ms < 0.5 || ms > 2.5) {
    Serial.printf("\"%s\" rejected: pulse must be 0.5 to 2.5 ms\n", line.c_str());
    return;
  }

  int us = lround(ms * 1000);
  if (servoNum == 0) {
    for (int i = 0; i < NUM_SERVOS; i++) moveSlowly(i, us);
  } else {
    moveSlowly(servoNum - 1, us);
  }
  Serial.println("Done. Enter next command.");
}
