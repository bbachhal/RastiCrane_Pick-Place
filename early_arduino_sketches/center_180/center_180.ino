// Moves the two 180 degree servos (GPIO 8 and 9) to 1500 us (90 degrees) and holds them there.
// Servo 3 (GPIO 10) is not touched.
// Board: ESP32S3 Dev Module (ESP32 core 3.x), USB CDC On Boot: Enabled

const int SERVO1_PIN = 8;
const int SERVO2_PIN = 9;

const int FREQ_HZ   = 50;     // 20 ms servo period
const int RES_BITS  = 14;     // ESP32-S3 LEDC max
const int CENTER_US = 1500;   // 1.5 ms = 90 degrees

void setup() {
  Serial.begin(115200);

  uint32_t duty = (uint32_t)CENTER_US * (1UL << RES_BITS) / 20000;

  ledcAttach(SERVO1_PIN, FREQ_HZ, RES_BITS);
  ledcWrite(SERVO1_PIN, duty);

  ledcAttach(SERVO2_PIN, FREQ_HZ, RES_BITS);
  ledcWrite(SERVO2_PIN, duty);
}

void loop() {
  // The PWM keeps running on its own; just report once a second.
  Serial.println("Servo 1 (GPIO 8) and Servo 2 (GPIO 9) holding at 1500 us (90 deg)");
  delay(1000);
}
