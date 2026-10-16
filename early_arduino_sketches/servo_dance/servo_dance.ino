#include <Arduino.h>
#include <esp_arduino_version.h>

// ESP32-S3 Servo Pin Assignments
const int NUM_SERVOS = 3;
const int SERVO_PINS[NUM_SERVOS] = {8,9,10}; // Physical GPIOs (Servo 1, 2, 3)

// LEDC configuration constants
const int LEDC_FREQ = 50;     // 50 Hz standard frequency for servos
const int LEDC_RES = 12;      // 12-bit resolution (0 to 4095 steps)

// Legacy LEDC channels (Only used if compiling on ESP32 Arduino Core v2.x)
const int SERVO_CHANNELS[NUM_SERVOS] = {0, 1, 2}; // Channels for Servo 1, 2, 3

// Safe standard servo pulse limits (limits physical over-travel)
const int MIN_US = 500;  // 0.5 ms (usually 0 degrees)
const int MAX_US = 2500; // 2.5 ms (usually 180 degrees)

// --- Zero / home positions ---
// Servo 1 and Servo 2 (180 deg) home at 1500 us, which is also the END of their travel.
const int ZERO_US = 1500;
// Servo 3 (360 deg base) home.
const int BASE_ZERO_US = 1900;
const int ZERO_US_PER_SERVO[NUM_SERVOS] = {ZERO_US, ZERO_US, BASE_ZERO_US};

// --- SAFE TRAVEL WINDOW (the most important safety setting) ---
// No command, typed or danced, is ever sent outside these.
// Servo 1 and 2: 1500 (home) up to 2500, per your measured arm limits.
// Base: home 1900 +/- 250 us. EDIT THIS once you know the base's real limits.
const int SAFE_MIN_US[NUM_SERVOS] = {1500, 1500, 1650};
const int SAFE_MAX_US[NUM_SERVOS] = {2500, 2500, 2150};

// --- Speed & Sweep Configuration ---
const unsigned long UPDATE_INTERVAL_MS = 20; // Step update interval (20ms matches 50Hz servo frequency)
const int SERVO_SPEED_STEP = 25;             // Speed control: Us change per 20ms step.
                                             // LOWER = Slower/Smoother (e.g., 5)
                                             // HIGHER = Faster (e.g., 40)

// --- DANCE ROUTINE ---
// Absolute pulse targets in us for each servo, plus how long to pause on arrival.
// Every value here is inside the safe window above, and is clamped again before it is sent.
struct DanceStep {
  int us[NUM_SERVOS];
  uint16_t holdMs;
};

const DanceStep DANCE[] = {
  // { Servo1, Servo2, Base },  hold
  {{ 1700, 1700, 2150},  300},   // lift slightly, base swings one way
  {{ 2100, 1600, 2150},  300},   // arm reaches out
  {{ 2100, 1600, 1650},  400},   // sweep across
  {{ 1600, 2100, 1650},  300},   // arm tucks in
  {{ 1600, 2100, 2150},  400},   // sweep back
  {{ 2300, 2300, 1900},  250},   // big stretch at center
  {{ 1700, 1700, 1900},  250},   // and back down
  {{ 1500, 1500, 1900},  600},   // return home
};
const int DANCE_STEPS = sizeof(DANCE) / sizeof(DANCE[0]);
const int DANCE_MAX_CYCLES = 3;  // stops on its own so nothing runs hot unattended

bool danceActive = false;
int danceStepIdx = 0;
int danceCycle = 0;
bool danceHolding = false;
unsigned long danceHoldStart = 0;

// --- INDEPENDENT MOTOR STATE VARIABLES ---
// Index 0 = Servo 1, Index 1 = Servo 2, Index 2 = Servo 3
int targetUs[NUM_SERVOS] = {ZERO_US, ZERO_US, BASE_ZERO_US};  // The final target positions (all start at home)
int currentUs[NUM_SERVOS] = {ZERO_US, ZERO_US, BASE_ZERO_US}; // The current intermediate positions of the servos

unsigned long lastUpdateMs = 0; // Timer tracking for smooth updates

void writeServoUs(int pin, int channel, int us);
void printInstructions();

// --- Safe window helpers ---
int safeMinUs(int i) { return constrain(SAFE_MIN_US[i], MIN_US, MAX_US); }
int safeMaxUs(int i) { return constrain(SAFE_MAX_US[i], MIN_US, MAX_US); }

void setup() {
  Serial.begin(115200);

  // ESP32-S3 USB CDC Boot Guard:
  // Waits up to 4 seconds for USB Serial Monitor to connect so you don't miss boot messages.
  while (!Serial && millis() < 4000) {
    delay(10);
  }

  Serial.println("\n=======================================================");
  Serial.println("ESP32-S3 Hardware PWM (LEDC) Servo Driver Active");
  Serial.printf("GPIO Pins Assigned: Servo1 -> GPIO %d, Servo2 -> GPIO %d, Servo3 -> GPIO %d\n",
                SERVO_PINS[0], SERVO_PINS[1], SERVO_PINS[2]);

  // Initialize S3 Hardware PWM
  #if ESP_ARDUINO_VERSION >= ESP_ARDUINO_VERSION_VAL(3, 0, 0)
    Serial.println("Configuring S3 LEDC using Core v3.x API...");
    for (int i = 0; i < NUM_SERVOS; i++) {
      ledcAttach(SERVO_PINS[i], LEDC_FREQ, LEDC_RES);
    }
  #else
    Serial.println("Configuring S3 LEDC using Core v2.x Legacy API...");
    for (int i = 0; i < NUM_SERVOS; i++) {
      ledcSetup(SERVO_CHANNELS[i], LEDC_FREQ, LEDC_RES);
      ledcAttachPin(SERVO_PINS[i], SERVO_CHANNELS[i]);
    }
  #endif

  // Home all servos: Servo 1 & 2 at 1500 us, base at BASE_ZERO_US
  for (int i = 0; i < NUM_SERVOS; i++) {
    writeServoUs(SERVO_PINS[i], SERVO_CHANNELS[i], currentUs[i]);
  }
  Serial.printf("Servo 1 & 2 homed at %d us. Servo 3 (base) homed at %d us.\n",
                ZERO_US, BASE_ZERO_US);
  for (int i = 0; i < NUM_SERVOS; i++) {
    Serial.printf("Servo %d safe window: %d - %d us\n", i + 1, safeMinUs(i), safeMaxUs(i));
  }

  printInstructions();
}

void loop() {
  // 1. Listen for user input on the ESP32-S3 Serial bus
  if (Serial.available() > 0) {
    String inputStr = Serial.readStringUntil('\n');
    inputStr.trim();

    if (inputStr.length() > 0) {
      // "dance" = run the routine; "stop" = end it and hold position
      if (inputStr.equalsIgnoreCase("dance")) {
        startDance();
        return;
      }
      if (inputStr.equalsIgnoreCase("stop")) {
        stopDance("stop command");
        for (int i = 0; i < NUM_SERVOS; i++) targetUs[i] = currentUs[i]; // freeze where it is
        return;
      }

      // "zero" = move ALL servos back to their home positions
      if (inputStr.equalsIgnoreCase("zero")) {
        stopDance("zero command");
        Serial.printf(">> Homing Servo 1 & 2 to %d us and Servo 3 (base) to %d us.\n", ZERO_US, BASE_ZERO_US);
        for (int i = 0; i < NUM_SERVOS; i++) {
          targetUs[i] = ZERO_US_PER_SERVO[i];
        }
        return;
      }

      // Check if user requested to shut off ALL motors globally
      if (inputStr.equalsIgnoreCase("0") || inputStr.equalsIgnoreCase("off")) {
        stopDance("off command");
        Serial.println(">> ESP32-S3: De-energizing ALL Servos immediately (they go limp).");
        for (int i = 0; i < NUM_SERVOS; i++) {
          targetUs[i] = 0;
          currentUs[i] = 0; // Turn off instantly
          writeServoUs(SERVO_PINS[i], SERVO_CHANNELS[i], 0);
        }
        return;
      }

      // Expected format: "<motor> <value>" (e.g. "1 1500" or "M3 1.8")
      int spaceIndex = inputStr.indexOf(' ');
      if (spaceIndex == -1) {
        Serial.printf(">> Invalid command format: '%s'. Enter '<motor> <value>'\n", inputStr.c_str());
        printInstructions();
        return;
      }

      String motorStr = inputStr.substring(0, spaceIndex);
      String valueStr = inputStr.substring(spaceIndex + 1);
      motorStr.trim();
      valueStr.trim();

      // Standardize motor index (strip "M" or "m" prefix if present)
      if (motorStr.startsWith("M") || motorStr.startsWith("m")) {
        motorStr = motorStr.substring(1);
      }

      int motorIdx = motorStr.toInt() - 1; // Convert human input (1-3) to array index (0-2)

      if (motorIdx < 0 || motorIdx >= NUM_SERVOS) {
        Serial.printf(">> Invalid motor selection: '%s'. Choose 1, 2, or 3.\n", motorStr.c_str());
        return;
      }

      // Any manual move ends the dance so the two don't fight each other
      stopDance("manual command");

      // "<motor> zero" = move one servo back to its home position
      if (valueStr.equalsIgnoreCase("zero")) {
        targetUs[motorIdx] = ZERO_US_PER_SERVO[motorIdx];
        Serial.printf(">> Homing Servo %d to %d us.\n", motorIdx + 1, targetUs[motorIdx]);
        return;
      }

      float valueInput = 0;
      bool motorZeroCommand = false;

      // Check if user requested to shut off this specific motor
      if (valueStr.equalsIgnoreCase("0") || valueStr.equalsIgnoreCase("off")) {
        motorZeroCommand = true;
      } else {
        valueInput = valueStr.toFloat();
        if (valueInput == 0.0f && valueStr != "0.0") {
          Serial.printf(">> Invalid motor value: '%s'. Enter a float/int, 'zero', or 'off'.\n", valueStr.c_str());
          return;
        }
      }

      // Process and update "targetUs" for the specific motor
      if (motorZeroCommand) {
        targetUs[motorIdx] = 0;
        currentUs[motorIdx] = 0; // Turn off instantly
        writeServoUs(SERVO_PINS[motorIdx], SERVO_CHANNELS[motorIdx], 0);
        Serial.printf(">> Servo %d de-energized immediately.\n", motorIdx + 1);
      }
      else if (valueInput >= 0.5f && valueInput <= 2.501f) {
        setTargetClamped(motorIdx, (int)(valueInput * 1000.0f));
      }
      else if (valueInput >= 500.0f && valueInput <= 2500.0f) {
        setTargetClamped(motorIdx, (int)valueInput);
      }
      else {
        Serial.printf(">> Input value '%s' is out of safe range (0.5ms - 2.5ms / 500us - 2500us)\n", valueStr.c_str());
      }
    }
  }

  // 2. Non-blocking Sweep Engine (Ramps positions incrementally and independently)
  unsigned long currentMs = millis();
  if (currentMs - lastUpdateMs >= UPDATE_INTERVAL_MS) {
    lastUpdateMs = currentMs;

    for (int i = 0; i < NUM_SERVOS; i++) {
      // Skip updates if the servo has been commanded to turn OFF/0
      if (targetUs[i] == 0) {
        continue;
      }

      if (currentUs[i] != targetUs[i]) {
        // If returning from a shut-off state (0 us), jump instantly to the target
        // to prevent ramping through invalid PWM ranges (below 500us).
        if (currentUs[i] == 0) {
          currentUs[i] = targetUs[i];
        }
        // Otherwise, incrementally step closer to the target
        else if (currentUs[i] < targetUs[i]) {
          currentUs[i] += SERVO_SPEED_STEP;
          if (currentUs[i] > targetUs[i]) currentUs[i] = targetUs[i]; // Prevent overshoot
        }
        else if (currentUs[i] > targetUs[i]) {
          currentUs[i] -= SERVO_SPEED_STEP;
          if (currentUs[i] < targetUs[i]) currentUs[i] = targetUs[i]; // Prevent undershoot
        }

        // Write current intermediate step directly to hardware pin
        writeServoUs(SERVO_PINS[i], SERVO_CHANNELS[i], currentUs[i]);
      }
    }
  }

  // 3. Dance sequencer (only advances once every servo has arrived)
  updateDance();
}

// Set a target, but never outside that servo's safe window
void setTargetClamped(int motorIdx, int requestedUs) {
  int clamped = constrain(requestedUs, safeMinUs(motorIdx), safeMaxUs(motorIdx));
  targetUs[motorIdx] = clamped;
  if (clamped != requestedUs) {
    Serial.printf(">> %d us is outside Servo %d's safe window (%d - %d us). Clamped to %d us.\n",
                  requestedUs, motorIdx + 1, safeMinUs(motorIdx), safeMaxUs(motorIdx), clamped);
  }
  Serial.printf(">> Sweeping Servo %d to: %d us (%.2f ms)\n", motorIdx + 1, clamped, (float)clamped / 1000.0f);
}

// ==================== DANCE ====================

void applyDanceStep() {
  for (int i = 0; i < NUM_SERVOS; i++) {
    targetUs[i] = constrain(DANCE[danceStepIdx].us[i], safeMinUs(i), safeMaxUs(i)); // clamped again, always
  }
  Serial.printf(">> Dance cycle %d/%d, step %d/%d -> %d / %d / %d us\n",
                danceCycle + 1, DANCE_MAX_CYCLES, danceStepIdx + 1, DANCE_STEPS,
                targetUs[0], targetUs[1], targetUs[2]);
}

void startDance() {
  danceActive = true;
  danceStepIdx = 0;
  danceCycle = 0;
  danceHolding = false;
  Serial.printf(">> Dance starting: %d steps, %d cycles. Type 'stop' to end it, '0' to go limp.\n",
                DANCE_STEPS, DANCE_MAX_CYCLES);
  applyDanceStep();
}

void stopDance(const char* why) {
  if (!danceActive) return;
  danceActive = false;
  danceHolding = false;
  Serial.printf(">> Dance stopped (%s).\n", why);
}

void updateDance() {
  if (!danceActive) return;

  // Wait until every servo has reached its target for this step
  for (int i = 0; i < NUM_SERVOS; i++) {
    if (currentUs[i] != targetUs[i]) return;
  }

  // Hold the pose for the step's hold time
  if (!danceHolding) {
    danceHolding = true;
    danceHoldStart = millis();
    return;
  }
  if (millis() - danceHoldStart < DANCE[danceStepIdx].holdMs) return;

  danceHolding = false;
  danceStepIdx++;

  if (danceStepIdx >= DANCE_STEPS) {
    danceStepIdx = 0;
    danceCycle++;
    if (danceCycle >= DANCE_MAX_CYCLES) {
      danceActive = false;
      for (int i = 0; i < NUM_SERVOS; i++) targetUs[i] = ZERO_US_PER_SERVO[i];
      Serial.println(">> Dance finished. Returning home and holding.");
      return;
    }
  }

  applyDanceStep();
}

// Low-level helper translating target microsecond timings directly to the hardware timer
void writeServoUs(int pin, int channel, int us) {
  uint32_t duty = 0;

  if (us == 0) {
    duty = 0; // De-energizes the motor
  } else {
    us = constrain(us, MIN_US, MAX_US);
    // Math: (target_us / 20000us period duration) * 4095 total 12-bit steps
    duty = (us * 4095) / 20000;
  }

  #if ESP_ARDUINO_VERSION >= ESP_ARDUINO_VERSION_VAL(3, 0, 0)
    ledcWrite(pin, duty); // ESP32-S3 Core v3 uses pin directly
  #else
    ledcWrite(channel, duty); // ESP32-S3 Core v2 maps via logical channel
  #endif
}

void printInstructions() {
  Serial.println("------------------------------------------------------------------");
  Serial.println("Enter your command in the format: <motor_number> <value>");
  Serial.println("Motors: 1, 2, or 3 (e.g. M1, M2, M3)");
  Serial.println("Values:");
  Serial.println("  - In Milliseconds: 0.50 to 2.50 (e.g. '1 1.75' to set motor 1)");
  Serial.println("  - In Microseconds: 500 to 2500  (e.g. '2 1200' to set motor 2)");
  Serial.println("  - All values are clamped to each servo's safe window (see startup print)");
  Serial.println("  - To DANCE: 'dance'   |   To end the dance and hold: 'stop'");
  Serial.println("  - To HOME: 'zero' (all 3) or '<motor> zero' (e.g. '3 zero' for the base)");
  Serial.println("  - To De-energize one motor (limp): <motor> off (e.g. '3 off' or '3 0')");
  Serial.println("  - To De-energize ALL motors (limp): Enter '0' or 'off'");
  Serial.println("------------------------------------------------------------------");
}
