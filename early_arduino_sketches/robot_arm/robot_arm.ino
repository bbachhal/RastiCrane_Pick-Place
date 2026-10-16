// Robot arm limit finder for ESP32-S3 Supermini.
// Based on your friend's ESP32Servo sketch, ported to raw LEDC (ESP32Servo is buggy on the S3).
// Same servo settings, same commands, plus jog commands and safety checks.
//
// Arduino IDE:    Board "ESP32S3 Dev Module", USB CDC On Boot: Enabled
// Serial Monitor: 115200 baud, line ending "Newline"

// ==================== PINS ====================
const int PIN_A = 8;    // Joint A, 180° servo            (friend used GPIO 11)
const int PIN_B = 9;    // Joint B, 180° servo            (friend used GPIO 12)
const int PIN_C = 10;   // Joint C / base, 300° positional (friend used GPIO 13)

// Same pulse range as friend's code: 500-2400 us.
// A/B: angle 0-180 maps onto it. C: logical angle 0-300 maps onto it.
const int PULSE_MIN_US = 500;
const int PULSE_MAX_US = 2400;

const int FREQ_HZ   = 50;
const int RES_BITS  = 14;                  // ESP32-S3 LEDC max
const int PERIOD_US = 1000000 / FREQ_HZ;   // 20000 us

// ==================== JOINTS & STARTING LIMITS ====================
// Friend's limits were A 75-160, B 20-160, C 0-300.
// These start 10° INSIDE those, because your build will deviate a bit.
// Widen them with serial commands while watching the arm, then type 'p'
// to print lines you can paste back into this table.
struct Joint {
  const char* name;
  int  pin;
  int  absMax;     // 180 for A/B, 300 for the base. Limits can never go past 0..absMax.
  int  minAngle;
  int  maxAngle;
  int  pos;        // current live position
  int  target;     // where it's commanded to go
  bool active;     // currently moving?
};

Joint joints[] = {
  // name        pin    absMax  min  max
  {"A",        PIN_A,  180,    85, 150, 0, 0, false},
  {"B",        PIN_B,  180,    30, 150, 0, 0, false},
  {"C (base)", PIN_C,  300,    10, 290, 0, 0, false},
};
const int NUM_JOINTS = sizeof(joints) / sizeof(joints[0]);

// ==================== MOVEMENT TIMING ====================
// 1 degree per step. 40 ms/step = 25°/s (friend used 20 ms = 50°/s).
// Slower on purpose while you find limits. 'd' changes it.
int step_delay_ms = 40;
const int STEP_DELAY_MIN_MS = 10;
const int STEP_DELAY_MAX_MS = 500;

String inputString = "";

// ==================== LOW LEVEL ====================

int angleToPulse(int i, int angle) {
  return map(angle, 0, joints[i].absMax, PULSE_MIN_US, PULSE_MAX_US);
}

void writeJoint(int i, int angle) {
  uint32_t duty = (uint32_t)angleToPulse(i, angle) * (1UL << RES_BITS) / PERIOD_US;
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcWrite(joints[i].pin, duty);   // core 3.x: by pin
#else
  ledcWrite(i, duty);               // core 2.x: by channel
#endif
}

void attachJoint(int i) {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcAttach(joints[i].pin, FREQ_HZ, RES_BITS);
#else
  ledcSetup(i, FREQ_HZ, RES_BITS);
  ledcAttachPin(joints[i].pin, i);
#endif
}

// Accepts only an optional +/- followed by digits. Rejects typos like "1abc".
bool parseNumber(const String& s, int& out) {
  if (s.length() == 0) return false;
  int start = (s[0] == '+' || s[0] == '-') ? 1 : 0;
  if (start >= (int)s.length()) return false;
  for (int k = start; k < (int)s.length(); k++) {
    if (!isDigit(s[k])) return false;
  }
  out = s.toInt();
  return true;
}

// ==================== MOVEMENT ====================

// Command a joint to smoothly move to an angle, clamped to its limits.
void moveJointTo(int i, int angle) {
  Joint& j = joints[i];
  int clamped = constrain(angle, j.minAngle, j.maxAngle);
  j.target = clamped;
  j.active = true;
  Serial.printf("Joint %s moving to: %d", j.name, clamped);
  if (clamped != angle) Serial.printf("  (asked %d, clamped to limit)", angle);
  Serial.println();
}

// ==================== SETUP / LOOP ====================

void setup() {
  Serial.begin(115200);
  unsigned long t0 = millis();
  while (!Serial && millis() - t0 < 3000) delay(10);   // give USB serial a moment

  // Start each joint at the middle of its range (same start pose as friend's code:
  // A=117, B=90, C=150), one joint at a time so the arm doesn't jump all at once.
  for (int i = 0; i < NUM_JOINTS; i++) {
    Joint& j = joints[i];
    j.pos = j.target = (j.minAngle + j.maxAngle) / 2;
    attachJoint(i);
    writeJoint(i, j.pos);
    Serial.printf("Joint %s holding at %d\n", j.name, j.pos);
    delay(1000);
  }

  printStatus();
  printHelp();
}

void loop() {
  // Read serial without blocking. Accepts Newline, CR, or both.
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\n' || c == '\r') {
      if (inputString.length() > 0) {
        processCommand(inputString);
        inputString = "";
      }
    } else if (inputString.length() < 32) {
      inputString += c;
    }
  }

  // Step every active joint 1 degree toward its target.
  for (int i = 0; i < NUM_JOINTS; i++) {
    Joint& j = joints[i];
    if (!j.active) continue;
    if (j.pos < j.target) j.pos++;
    else if (j.pos > j.target) j.pos--;
    writeJoint(i, j.pos);
    if (j.pos == j.target) {
      j.active = false;
      Serial.printf("Joint %s reached %d (%d us)\n", j.name, j.pos, angleToPulse(i, j.pos));
    }
  }

  delay(step_delay_ms);
}

// ==================== SERIAL COMMANDS ====================

void setLimit(int i, bool isMax, const String& value) {
  Joint& j = joints[i];
  int v;
  if (!parseNumber(value, v)) {
    Serial.println("Bad number. Example: A155");
    return;
  }
  if (v < 0 || v > j.absMax) {
    Serial.printf("Rejected: joint %s limits must be 0 to %d\n", j.name, j.absMax);
    return;
  }
  if (!isMax && v > j.maxAngle) {
    Serial.printf("Rejected: MIN can't be above MAX (%d)\n", j.maxAngle);
    return;
  }
  if (isMax && v < j.minAngle) {
    Serial.printf("Rejected: MAX can't be below MIN (%d)\n", j.minAngle);
    return;
  }

  if (isMax) j.maxAngle = v; else j.minAngle = v;
  Serial.printf("Joint %s %s set to: %d  (range now %d-%d)\n",
                j.name, isMax ? "MAX" : "MIN", v, j.minAngle, j.maxAngle);

  // Keep any move in progress inside the new limits.
  if (j.active) j.target = constrain(j.target, j.minAngle, j.maxAngle);
  if (j.pos < j.minAngle || j.pos > j.maxAngle) {
    Serial.println("Note: joint is currently outside the new limits. It won't move until you command it.");
  }
}

void processCommand(String cmd) {
  cmd.trim();   // case matters: a vs A, b vs B, e vs E
  if (cmd.length() == 0) return;

  char command = cmd.charAt(0);
  String value = cmd.substring(1);
  value.trim();
  int n;

  switch (command) {
    // Move: 1120 = absolute, 1+5 / 1-5 = jog relative
    case '1': case '2': case '3': {
      int i = command - '1';
      if (!parseNumber(value, n)) {
        Serial.println("Bad number. Examples: 1120 (go to 120), 1+5 (jog +5), 1-5 (jog -5)");
        break;
      }
      bool relative = (value[0] == '+' || value[0] == '-');
      moveJointTo(i, relative ? joints[i].target + n : n);
      break;
    }

    // Limits
    case 'a': setLimit(0, false, value); break;
    case 'A': setLimit(0, true,  value); break;
    case 'b': setLimit(1, false, value); break;
    case 'B': setLimit(1, true,  value); break;
    case 'e': setLimit(2, false, value); break;
    case 'E': setLimit(2, true,  value); break;

    // Speed
    case 'd':
      if (!parseNumber(value, n)) { Serial.println("Bad number. Example: d40"); break; }
      step_delay_ms = constrain(n, STEP_DELAY_MIN_MS, STEP_DELAY_MAX_MS);
      Serial.printf("Step delay set to: %d ms\n", step_delay_ms);
      break;
    case '+':
      step_delay_ms = constrain(step_delay_ms + 5, STEP_DELAY_MIN_MS, STEP_DELAY_MAX_MS);
      Serial.printf("Step delay: %d ms (slower)\n", step_delay_ms);
      break;
    case '-':
      step_delay_ms = constrain(step_delay_ms - 5, STEP_DELAY_MIN_MS, STEP_DELAY_MAX_MS);
      Serial.printf("Step delay: %d ms (faster)\n", step_delay_ms);
      break;

    // Emergency stop: halt movement, joints HOLD current position
    case 'x':
      for (int i = 0; i < NUM_JOINTS; i++) {
        joints[i].active = false;
        joints[i].target = joints[i].pos;
      }
      Serial.println("STOPPED. Joints holding current position.");
      break;

    case 'c':
      for (int i = 0; i < NUM_JOINTS; i++) {
        moveJointTo(i, (joints[i].minAngle + joints[i].maxAngle) / 2);
      }
      Serial.println("Centering all joints...");
      break;

    case 's': printStatus(); break;
    case 'p': printLimitsAsCode(); break;
    case 'h': case '?': printHelp(); break;

    default:
      Serial.println("Unknown command. Type 'h' for help.");
  }
}

// ==================== OUTPUT ====================

void printStatus() {
  Serial.println("\n=== CURRENT STATUS ===");
  for (int i = 0; i < NUM_JOINTS; i++) {
    Joint& j = joints[i];
    Serial.printf("Joint %-8s GPIO %2d  pos=%3d (%4d us)  target=%3d  range=%d-%d%s\n",
                  j.name, j.pin, j.pos, angleToPulse(i, j.pos), j.target,
                  j.minAngle, j.maxAngle, j.active ? "  [moving]" : "");
  }
  Serial.printf("Step delay: %d ms\n", step_delay_ms);
  Serial.println("======================");
}

void printLimitsAsCode() {
  Serial.println("\n// Paste over the joints[] table in the code:");
  Serial.println("Joint joints[] = {");
  Serial.println("  // name        pin    absMax  min  max");
  const char* pinNames[] = {"PIN_A", "PIN_B", "PIN_C"};
  for (int i = 0; i < NUM_JOINTS; i++) {
    Joint& j = joints[i];
    Serial.printf("  {\"%s\",%*s%s,  %d,    %d, %d, 0, 0, false},\n",
                  j.name, 10 - (int)strlen(j.name), "", pinNames[i],
                  j.absMax, j.minAngle, j.maxAngle);
  }
  Serial.println("};");
}

void printHelp() {
  Serial.println("\n=== SERIAL COMMANDS ===");
  Serial.println("--- Movement (smooth, always clamped to limits) ---");
  Serial.println("1<num>  - Move Joint A to angle      (e.g. 1120)");
  Serial.println("2<num>  - Move Joint B to angle      (e.g. 290)");
  Serial.println("3<num>  - Move base C to angle 0-300 (e.g. 3200)");
  Serial.println("1+<n> / 1-<n> - Jog Joint A by n degrees (e.g. 1+5, 2-3, 3+10)");
  Serial.println("c       - Center all joints");
  Serial.println("x       - STOP all movement (joints hold position)");
  Serial.println("--- Limits ---");
  Serial.println("a<num> / A<num>  - Set A MIN / MAX (0-180)  (e.g. a80, A155)");
  Serial.println("b<num> / B<num>  - Set B MIN / MAX (0-180)  (e.g. b25, B155)");
  Serial.println("e<num> / E<num>  - Set base MIN / MAX (0-300) (e.g. e5, E295)");
  Serial.println("p       - Print current limits as code to paste back in");
  Serial.println("--- Speed / info ---");
  Serial.println("d<num>  - Step delay in ms, 10-500 (bigger = slower, e.g. d60)");
  Serial.println("+ / -   - Slower / faster by 5 ms");
  Serial.println("s       - Status");
  Serial.println("h or ?  - This help");
  Serial.println("=======================");
}
