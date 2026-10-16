// ==================== CONFIGURATION ====================

// --- Joint A (GPIO 8, 180° servo) ---
int servoA_min = 75;
int servoA_max = 160;

// --- Joint B (GPIO 9, 180° servo) ---
int servoB_min = 20;
int servoB_max = 170;

// --- Joint C / Base (GPIO 10, 300° servo, positional) ---
int servoC_min = 0;      // logical angle, 0-300 range
int servoC_max = 300;
const int SERVOC_PULSE_MIN_US = 500;   // pulse width at logical angle 0
const int SERVOC_PULSE_MAX_US = 2400;  // pulse width at logical angle 300

// --- Movement timing ---
int moveTime_ms = 1000;    // Time for a move to complete (ms)

// ==================== HARDWARE SETUP ====================

const int SERVO1_PIN = 8;
const int SERVO2_PIN = 9;
const int SERVO3_PIN = 10;

const int PWM_FREQ = 50;        // 50 Hz for servos
const int PWM_RESOLUTION = 14;  // 14-bit resolution (16-bit has issues in 3.3.11)

// Store which pin is assigned (ledcAttach returns void in 3.3.11)
int pinA = -1, pinB = -1, pinC = -1;

// Current positions (in pulse microseconds)
float currentPulseA_us = 1500, currentPulseB_us = 1500, currentPulseC_us = 1450;
float startPulseA_us, startPulseB_us, startPulseC_us;

// Move state
bool isMoving = false;
unsigned long moveStartTime = 0;
uint16_t moveDuration = 0;

// Targets
int targetA_angle, targetB_angle, targetC_angle;
float targetPulseA_us, targetPulseB_us, targetPulseC_us;

// Command buffer
String inputString = "";
bool stringComplete = false;

// ==================== PWM HELPERS ====================

// Calculate duty for given microseconds (14-bit resolution)
uint32_t usToDuty(int microseconds) {
    // At 50Hz: 20ms period
    // duty = (pulse_us / 20000) * 16383 (for 14-bit)
    return (uint32_t)((((uint64_t)microseconds) * 16383ULL) / 20000ULL);
}

int angle180ToPulse(int angle) {
    long clamped = constrain(angle, 0, 180);
    return (int)map(clamped, 0L, 180L, 500L, 2400L);
}

int angle300ToPulse(int angle) {
    long clamped = constrain(angle, 0, 300);
    return (int)map(clamped, 0L, 300L, (long)SERVOC_PULSE_MIN_US, (long)SERVOC_PULSE_MAX_US);
}

float easeInOutSine(float t) {
    if (t <= 0) return 0;
    if (t >= 1) return 1;
    return -0.5f * (cosf(PI * t) - 1.0f);
}

// ESP32 core 3.3.11: ledcWrite uses PIN directly, not channel
void writeServoUs(int pin, int microseconds) {
    uint32_t duty = usToDuty(constrain(microseconds, 500, 2500));
    ledcWrite(pin, duty);  // v3.3.11: write to PIN
}

// ==================== SETUP ====================

void setup() {
    Serial.begin(115200);
    delay(1000);
    Serial.println("\n\n=== ROBOT ARM v3.3.11 ===");

    // ESP32 Core 3.3.11 API: ledcAttach(pin, freq, resolution)
    // This replaces ledcSetup + ledcAttachPin
    // Returns bool for success

    bool okA = ledcAttach(SERVO1_PIN, PWM_FREQ, PWM_RESOLUTION);
    bool okB = ledcAttach(SERVO2_PIN, PWM_FREQ, PWM_RESOLUTION);
    bool okC = ledcAttach(SERVO3_PIN, PWM_FREQ, PWM_RESOLUTION);

    Serial.print("PWM A:"); Serial.print(okA ? "OK" : "FAIL");
    Serial.print(" B:"); Serial.print(okB ? "OK" : "FAIL");
    Serial.print(" C:"); Serial.println(okC ? "OK" : "FAIL");

    pinA = SERVO1_PIN;
    pinB = SERVO2_PIN;
    pinC = SERVO3_PIN;

    // Start at center positions
    int startA = (servoA_min + servoA_max) / 2;
    int startB = (servoB_min + servoB_max) / 2;
    int startC = (servoC_min + servoC_max) / 2;

    currentPulseA_us = angle180ToPulse(startA);
    currentPulseB_us = angle180ToPulse(startB);
    currentPulseC_us = angle300ToPulse(startC);
    startPulseA_us = currentPulseA_us;
    startPulseB_us = currentPulseB_us;
    startPulseC_us = currentPulseC_us;

    targetA_angle = startA;
    targetB_angle = startB;
    targetC_angle = startC;
    targetPulseA_us = currentPulseA_us;
    targetPulseB_us = currentPulseB_us;
    targetPulseC_us = currentPulseC_us;

    // Send initial position repeatedly
    Serial.println("Initializing...");
    for (int i = 0; i < 50; i++) {
        writeServoUs(pinA, (int)currentPulseA_us);
        writeServoUs(pinB, (int)currentPulseB_us);
        writeServoUs(pinC, (int)currentPulseC_us);
        delay(20);
    }

    Serial.print("A:"); Serial.print(startA); Serial.print("deg="); Serial.println((int)currentPulseA_us);
    Serial.print("B:"); Serial.print(startB); Serial.print("deg="); Serial.println((int)currentPulseB_us);
    Serial.print("C:"); Serial.print(startC); Serial.print("deg="); Serial.println((int)currentPulseC_us);

    printHelp();
}

void loop() {
    while (Serial.available()) {
        char inChar = (char)Serial.read();
        if (inChar == '\n') {
            stringComplete = true;
        } else if (inChar != '\r') {
            inputString += inChar;
        }
    }

    if (stringComplete) {
        processCommand(inputString);
        inputString = "";
        stringComplete = false;
    }

    if (isMoving) {
        updateMovement();
    }

    delay(1);
}

// ==================== MOVEMENT ====================

void moveAllJoints(int angleA, int angleB, int angleC) {
    angleA = constrain(angleA, servoA_min, servoA_max);
    angleB = constrain(angleB, servoB_min, servoB_max);
    angleC = constrain(angleC, servoC_min, servoC_max);

    startPulseA_us = currentPulseA_us;
    startPulseB_us = currentPulseB_us;
    startPulseC_us = currentPulseC_us;

    targetA_angle = angleA;
    targetB_angle = angleB;
    targetC_angle = angleC;
    targetPulseA_us = angle180ToPulse(angleA);
    targetPulseB_us = angle180ToPulse(angleB);
    targetPulseC_us = angle300ToPulse(angleC);

    float distA = fabsf(targetPulseA_us - startPulseA_us);
    float distB = fabsf(targetPulseB_us - startPulseB_us);
    float distC = fabsf(targetPulseC_us - startPulseC_us);
    float maxDist = fmaxf(distA, fmaxf(distB, distC));

    moveDuration = (uint16_t)(moveTime_ms * (maxDist / 1900.0f));
    if (moveDuration < 200) moveDuration = 200;
    if (moveDuration > 3000) moveDuration = 3000;

    isMoving = true;
    moveStartTime = millis();

    Serial.print("MOVE: A="); Serial.print(angleA);
    Serial.print(" B="); Serial.print(angleB);
    Serial.print(" C="); Serial.print(angleC);
    Serial.print(" in "); Serial.print(moveDuration);
    Serial.println("ms");
}

void updateMovement() {
    unsigned long now = millis();
    unsigned long elapsed = now - moveStartTime;

    if (elapsed >= moveDuration) {
        currentPulseA_us = targetPulseA_us;
        currentPulseB_us = targetPulseB_us;
        currentPulseC_us = targetPulseC_us;
        isMoving = false;

        writeServoUs(pinA, (int)currentPulseA_us);
        writeServoUs(pinB, (int)currentPulseB_us);
        writeServoUs(pinC, (int)currentPulseC_us);

        Serial.println("DONE");
    } else {
        float progress = (float)elapsed / (float)moveDuration;
        float eased = easeInOutSine(progress);

        currentPulseA_us = startPulseA_us + (targetPulseA_us - startPulseA_us) * eased;
        currentPulseB_us = startPulseB_us + (targetPulseB_us - startPulseB_us) * eased;
        currentPulseC_us = startPulseC_us + (targetPulseC_us - startPulseC_us) * eased;

        writeServoUs(pinA, (int)currentPulseA_us);
        writeServoUs(pinB, (int)currentPulseB_us);
        writeServoUs(pinC, (int)currentPulseC_us);
    }
}

void moveJointTo(char joint, int angle) {
    switch (joint) {
        case 'A': moveAllJoints(angle, targetB_angle, targetC_angle); break;
        case 'B': moveAllJoints(targetA_angle, angle, targetC_angle); break;
        case 'C': moveAllJoints(targetA_angle, targetB_angle, angle); break;
    }
}

void stopAll() {
    isMoving = false;
    Serial.println("STOP");
}

// ==================== COMMANDS ====================

void processCommand(String cmd) {
    cmd.trim();
    if (cmd.length() == 0) return;

    char command = cmd.charAt(0);
    String value = cmd.substring(1);
    value.trim();

    // DIRECT PWM to pin 8
    if (command == 'p') {
        int pulse = constrain(value.toInt(), 500, 2500);
        Serial.print("DIRECT pin "); Serial.print(SERVO1_PIN);
        Serial.print(" = "); Serial.print(pulse); Serial.println("us");
        for (int i = 0; i < 50; i++) {
            writeServoUs(SERVO1_PIN, pulse);
            delay(20);
        }
        return;
    }

    // Test all pins
    if (command == 't') {
        testServos();
        return;
    }

    // Multi-joint
    if (command == 'm') {
        int c1 = value.indexOf(',');
        int c2 = value.indexOf(',', c1 + 1);
        if (c1 > 0 && c2 > c1) {
            int a = value.substring(0, c1).toInt();
            int b = value.substring(c1 + 1, c2).toInt();
            int c = value.substring(c2 + 1).toInt();
            moveAllJoints(a, b, c);
        } else {
            Serial.println("Use: mA,B,C");
        }
        return;
    }

    switch (command) {
        case 'a': servoA_min = value.toInt(); break;
        case 'A': servoA_max = value.toInt(); break;
        case 'b': servoB_min = value.toInt(); break;
        case 'B': servoB_max = value.toInt(); break;
        case 'e': servoC_min = value.toInt(); break;
        case 'E': servoC_max = value.toInt(); break;

        case '1': moveJointTo('A', value.toInt()); break;
        case '2': moveJointTo('B', value.toInt()); break;
        case '3': moveJointTo('C', value.toInt()); break;

        case 'c':
            moveAllJoints((servoA_min + servoA_max) / 2,
                         (servoB_min + servoB_max) / 2,
                         (servoC_min + servoC_max) / 2);
            break;

        case 'f':
            Serial.println("Sweep A");
            for (int p = 500; p <= 2500; p += 50) {
                writeServoUs(pinA, p);
                Serial.println(p);
                delay(50);
            }
            break;

        case 'd':
            if (value.length() > 0) {
                moveTime_ms = constrain(value.toInt(), 100, 5000);
                Serial.print("Time: "); Serial.println(moveTime_ms);
            } else {
                moveAllJoints(servoA_min, servoB_max, targetC_angle);
            }
            break;

        case 's':
            if (value.length() > 0) {
                int speed = constrain(value.toInt(), 1, 100);
                moveTime_ms = (int)map((long)speed, 1L, 100L, 3000L, 200L);
            } else {
                printStatus();
            }
            break;

        case 'x': stopAll(); break;
        case 'h':
        case '?': printHelp(); break;
        default: Serial.println("Unknown. '?' for help");
    }
}

void testServos() {
    int pins[3] = {pinA, pinB, pinC};
    const char* names[3] = {"A(8)", "B(9)", "C(10)"};

    for (int i = 0; i < 3; i++) {
        Serial.print("Testing "); Serial.println(names[i]);
        writeServoUs(pins[i], 1500);
        delay(400);
        writeServoUs(pins[i], 1000);
        delay(400);
        writeServoUs(pins[i], 2000);
        delay(400);
        writeServoUs(pins[i], 1500);
        delay(400);
    }
    Serial.println("Done");
}

void printStatus() {
    Serial.println("\n--- STATUS ---");
    Serial.print("A: "); Serial.print(targetA_angle); Serial.print(" = "); Serial.println((int)currentPulseA_us);
    Serial.print("B: "); Serial.print(targetB_angle); Serial.print(" = "); Serial.println((int)currentPulseB_us);
    Serial.print("C: "); Serial.print(targetC_angle); Serial.print(" = "); Serial.println((int)currentPulseC_us);
    Serial.print("Time: "); Serial.print(moveTime_ms);
    Serial.print("ms  Moving: "); Serial.println(isMoving ? "YES" : "NO");
}

void printHelp() {
    Serial.println("\n=== COMMANDS ===");
    Serial.println("t       - Test all servos");
    Serial.println("p<us>   - Direct PWM to pin 8 (500-2500)");
    Serial.println("f       - Sweep A");
    Serial.println("mA,B,C  - Coordinated move");
    Serial.println("1/2/3<N> - Single joint");
    Serial.println("c       - Center");
    Serial.println("d<N>    - Set time ms");
    Serial.println("s       - Status");
    Serial.println("x       - Stop");
    Serial.println("?,h     - Help");
}
