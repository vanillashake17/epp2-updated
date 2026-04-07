#include <Servo.h>

// ── Enable / disable individual servos ──────────────────────────────────────
const bool TEST_BASE     = false;
const bool TEST_SHOULDER = false;
const bool TEST_ELBOW    = true;
const bool TEST_GRIPPER  = false;

// ── Pin assignments ──────────────────────────────────────────────────────────
const int PIN_BASE     = 37;
const int PIN_SHOULDER = 36;
const int PIN_ELBOW    = 33;
const int PIN_GRIPPER  = 35;

// ── Angle constraints (must match sensor_miniproject_template.ino) ───────────
const int BASE_MIN = 0,   BASE_MAX = 175;
const int SHLD_MIN = 140, SHLD_MAX = 180; 
const int ELBW_MIN = 0, ELBW_MAX = 60;
const int GRIP_MIN = 5,   GRIP_MAX = 35;  // 0 is open, 35 is fully closed

// ── Pulse range (µs) ────────────────────────────────────────────────────────
const int MIN_PULSE_US = 600;
const int MAX_PULSE_US = 2400;

// ── Timing ───────────────────────────────────────────────────────────────────
const int STEP_DELAY_MS = 25;

Servo servoBase;
Servo servoShoulder;
Servo servoElbow;
Servo servoGripper;

void setup() {
  if (TEST_BASE)     { servoBase.attach(PIN_BASE,         MIN_PULSE_US, MAX_PULSE_US); servoBase.write(BASE_MIN);     delay(1000); }
  if (TEST_SHOULDER) { servoShoulder.attach(PIN_SHOULDER, MIN_PULSE_US, MAX_PULSE_US); servoShoulder.write(SHLD_MIN); delay(1000); }
  if (TEST_ELBOW)    { servoElbow.attach(PIN_ELBOW,       MIN_PULSE_US, MAX_PULSE_US); servoElbow.write(ELBW_MIN);    delay(1000); }
  if (TEST_GRIPPER)  { servoGripper.attach(PIN_GRIPPER,   MIN_PULSE_US, MAX_PULSE_US); servoGripper.write(GRIP_MIN);  delay(1000); }
}

void loop() {
  if (TEST_BASE) {
    for (int i = BASE_MIN; i <= BASE_MAX; i++) { servoBase.write(i); delay(STEP_DELAY_MS); }
    for (int i = BASE_MAX; i >= BASE_MIN; i--) { servoBase.write(i); delay(STEP_DELAY_MS); }
  }

  if (TEST_SHOULDER) {
    for (int i = SHLD_MIN; i <= SHLD_MAX; i++) { servoShoulder.write(i); delay(STEP_DELAY_MS); }
    for (int i = SHLD_MAX; i >= SHLD_MIN; i--) { servoShoulder.write(i); delay(STEP_DELAY_MS); }
  }

  if (TEST_ELBOW) {
    for (int i = ELBW_MIN; i <= ELBW_MAX; i++) { servoElbow.write(i); delay(STEP_DELAY_MS); }
    for (int i = ELBW_MAX; i >= ELBW_MIN; i--) { servoElbow.write(i); delay(STEP_DELAY_MS); }
  }

  if (TEST_GRIPPER) {
    for (int i = GRIP_MIN; i <= GRIP_MAX; i++) { servoGripper.write(i); delay(STEP_DELAY_MS); }
    for (int i = GRIP_MAX; i >= GRIP_MIN; i--) { servoGripper.write(i); delay(STEP_DELAY_MS); }
  }
}
