#define BLYNK_TEMPLATE_ID "TMPL6Ilkl1m4s"
#define BLYNK_TEMPLATE_NAME "Logistics Bot"
#define BLYNK_AUTH_TOKEN "3vNmhfSmpQcfvnxmFQ7jxZ1E2L3NjTW_"
#include <WiFi.h>
#include <BlynkSimpleEsp32.h>
#include <math.h>
#include <stdlib.h>
#include <Preferences.h>
const char WIFI_SSID[] = "Abe's iPhone";
const char WIFI_PASSWORD[] = "nintendo";

// Keep remote movement locked until safety tests pass.
constexpr bool ENABLE_BLYNK_MOVEMENT = false;
constexpr unsigned long BLYNK_RECONNECT_MS = 5000UL;

BlynkTimer blynkTimer;

bool remoteStopLatched = false;
bool remoteMotionActive = false;
void publishBlynkTelemetry();

// LOGISTICS BOT - ESP32 / L298N / 3x HC-SR04 / E-STOP
// SERIAL: 115200 BAUD
/* ============================================================
   1. CONFIGURATION - PIN ASSIGNMENTS / SENSOR LIMITS
   ============================================================ */
const int senseLeft_PIN_TRIG = 22;
const int senseLeft_PIN_ECHO = 33;
const int senseFront_PIN_TRIG  = 23;
const int senseFront_PIN_ECHO  = 34;
const int senseRight_PIN_TRIG  = 21;
const int senseRight_PIN_ECHO  = 32;
const int ESTOP_PIN = 25;
const int leftMotor_PWM  = 18;      // L298N ENA
const int rightMotor_PWM  = 19;     // L298N ENB
const int leftMotor_IN1  = 26;      // L298N IN1
const int leftMotor_IN2  = 27;      // L298N IN2
const int rightMotor_IN3  = 14;     // L298N IN3
const int rightMotor_IN4  = 13;     // L298N IN4
const unsigned long SENSOR_GAP_MS = 65;          // Wait at least 65 milliseconds between ultrasonic trigger pulses
const unsigned long ECHO_TIMEOUT_US = 25000UL;   // Stop waiting for an ECHO after 25,000 microseconds (25 ms)
const float SENSOR_MIN_CM = 2.0f;        // Minimum accepted measurement: 2 cm
const float SENSOR_MAX_CM = 400.0f;      // Maximum accepted measurement: 400 cm
const float FRONT_OBSTACLE_CM = 15.0f;
constexpr float AUTO_CLEARANCE_CM = 15.0f;
constexpr uint8_t CLOSE_CONFIRMATIONS = 2;
/* ============================================================
   2. MOTOR CONTROL AND EMERGENCY STOP
   ============================================================ */
volatile bool estopLatched = false;
bool motorFaultLatched = false;
bool pwmModeEnabled = false;
const uint8_t MOTOR_TEST_DUTY = 180; // Base PWM duty

// Straight-line motor balance. Tune using measured lateral drift.
constexpr float RIGHT_DRIVE_RATIO = 1.00f; // Diagnostic baseline: equal PWM duty

uint8_t rightDriveDuty() {
  return (uint8_t)(MOTOR_TEST_DUTY * RIGHT_DRIVE_RATIO + 0.5f);
}
constexpr unsigned long MOTOR_TEST_MIN_MS = 100UL;
constexpr unsigned long MOTOR_TEST_MAX_MS = 1000UL;
constexpr unsigned long MOTOR_TEST_STEP_MS = 100UL;
constexpr unsigned long MOTOR_TEST_INITIAL_MS = 500UL;

static_assert(
  MOTOR_TEST_INITIAL_MS >= MOTOR_TEST_MIN_MS &&
  MOTOR_TEST_INITIAL_MS <= MOTOR_TEST_MAX_MS,
  "Initial motor test duration is outside permitted limits"
);

unsigned long motorTestMs = MOTOR_TEST_INITIAL_MS;

void ARDUINO_ISR_ATTR onEmergencyStop() {
  estopLatched = true;
}

bool emergencyStopActive() {
  if (digitalRead(ESTOP_PIN) == LOW) estopLatched = true;
  return estopLatched;
}

void stopLeftMotor() {
  if (pwmModeEnabled) ledcWrite(leftMotor_PWM, 0);
  else digitalWrite(leftMotor_PWM, LOW);
  digitalWrite(leftMotor_IN1, LOW);
  digitalWrite(leftMotor_IN2, LOW);
}

void stopRightMotor() {
  if (pwmModeEnabled) ledcWrite(rightMotor_PWM, 0);
  else digitalWrite(rightMotor_PWM, LOW);
  digitalWrite(rightMotor_IN3, LOW);
  digitalWrite(rightMotor_IN4, LOW);
}

void stopMotors() {
  stopLeftMotor();
  stopRightMotor();
}

bool movementPermitted() {
  return !emergencyStopActive() &&
         !motorFaultLatched &&
         !remoteStopLatched;
}

bool enforceMotorInterlock() {
  if (movementPermitted()) return true;
  stopMotors();
  return false;
}

void latchMotorFault() {
  motorFaultLatched = true;
  stopMotors();
  Serial.println("MOTOR FAULT: BOTH MOTORS OFF; RESET REQUIRED");
}

// direction: +1 forward, -1 reverse, 0 stop.
void setLeftMotor(int direction) {
   stopLeftMotor();
   if (!enforceMotorInterlock()) return;
   if (direction != 1 && direction != -1) return;
   
   digitalWrite(leftMotor_IN1, direction == -1 ? HIGH : LOW);
   digitalWrite(leftMotor_IN2, direction == 1 ? HIGH : LOW);

   if (!enforceMotorInterlock()) return;

   
  if (pwmModeEnabled) {
    if (!ledcWrite(leftMotor_PWM, MOTOR_TEST_DUTY)) {
      latchMotorFault();
      return;
    }

    delay(2);
    if (!enforceMotorInterlock()) return;
     
    const uint32_t actualDuty = ledcRead(leftMotor_PWM);

    Serial.printf(
      "LEFT LEDC duty: requested=%u readback=%lu | %s\n",
      (unsigned)MOTOR_TEST_DUTY,
      (unsigned long)actualDuty,
      actualDuty == MOTOR_TEST_DUTY ? "MATCH" : "MISMATCH"
    );

    if (actualDuty != MOTOR_TEST_DUTY) {
      latchMotorFault();
      return;
    }

  } else {
    digitalWrite(leftMotor_PWM, HIGH);
  }
   
   enforceMotorInterlock();
}

void setRightMotor(int direction, uint8_t requestedDuty) {
  stopRightMotor();

  if (!enforceMotorInterlock()) return;
  if (direction != 1 && direction != -1) return;

  digitalWrite(rightMotor_IN3, direction == 1 ? HIGH : LOW);
  digitalWrite(rightMotor_IN4, direction == -1 ? HIGH : LOW);

  if (!enforceMotorInterlock()) return;

  
  if (pwmModeEnabled) {
    if (!ledcWrite(rightMotor_PWM, requestedDuty)) {
      latchMotorFault();
      return;
    }

    delay(2);
    if (!enforceMotorInterlock()) return;

    const uint32_t actualDuty = ledcRead(rightMotor_PWM);

     Serial.printf(
      "RIGHT LEDC duty: requested=%u readback=%lu | %s\n",
      (unsigned)requestedDuty,
      (unsigned long)actualDuty,
      actualDuty == requestedDuty ? "MATCH" : "MISMATCH"
    );

    if (actualDuty != requestedDuty) {
      latchMotorFault();
      return;
    }

  } else {
    digitalWrite(rightMotor_PWM, HIGH);
  }

  enforceMotorInterlock();
}

/* ============================================================
   3. TIMED MOVEMENT CALIBRATION
   ============================================================ */
// Provisional timings (calibrated through C1, C2, CQ, CE).
constexpr unsigned long MOTION_MIN_MS = 50UL;
constexpr unsigned long MOTION_MAX_MS = 4000UL;

unsigned long forward10Ms = 455UL;
unsigned long forward15Ms = 666UL;
unsigned long turnLeft90Ms = 1159UL;
unsigned long turnRight90Ms = 728UL;

bool calibrated10 = false;
bool calibrated15 = false;
bool calibratedLeft = false;
bool calibratedRight = false;


/* ============================================================
   VERIFIED CALIBRATION STORAGE - ESP32 NVS FLASH
   ============================================================ */

Preferences calibrationStore;
bool calibrationStorageReady = false;

// Increment this if motor hardware or timing behaviour changes.
constexpr uint32_t CAL_PROFILE_VERSION = 1;

// Changing the base PWM or right-wheel trim invalidates old timings.
uint32_t calibrationProfile() {
  return (CAL_PROFILE_VERSION << 16) |
         ((uint32_t)MOTOR_TEST_DUTY << 8) |
         (uint32_t)rightDriveDuty();
}

void restoreCalibration(const char* key,
                        unsigned long& duration,
                        bool& verified) {
  const unsigned long saved = calibrationStore.getULong(key, 0);

  if (saved >= MOTION_MIN_MS && saved <= MOTION_MAX_MS) {
    duration = saved;
    verified = true;
  }
}

void loadSavedCalibrations() {
  calibrationStorageReady = calibrationStore.begin("logbotcal", false);

  if (!calibrationStorageReady) {
    Serial.println("CAL FLASH ERROR: defaults retained");
    return;
  }

  const uint32_t profile = calibrationProfile();

  if (calibrationStore.getULong("profile", 0) != profile) {
    // Clear only this project's calibration namespace.
    if (!calibrationStore.clear() ||
        calibrationStore.putULong("profile", profile) != 4) {
      Serial.println("CAL FLASH ERROR: cannot initialise storage");
      calibrationStore.end();
      calibrationStorageReady = false;
      return;
    }

    Serial.println("CAL FLASH: new motor profile; old timings cleared");
  }

  restoreCalibration("c1", forward10Ms, calibrated10);
  restoreCalibration("c2", forward15Ms, calibrated15);
  restoreCalibration("cq", turnLeft90Ms, calibratedLeft);
  restoreCalibration("ce", turnRight90Ms, calibratedRight);

  Serial.printf(
    "CAL FLASH: C1=%s C2=%s CQ=%s CE=%s\n",
    calibrated10 ? "RESTORED" : "UNVERIFIED",
    calibrated15 ? "RESTORED" : "UNVERIFIED",
    calibratedLeft ? "RESTORED" : "UNVERIFIED",
    calibratedRight ? "RESTORED" : "UNVERIFIED"
  );
}


/* ============================================================
   4. SETUP - INITIALISE HARDWARE AND SAFETY INPUTS
   ============================================================ */

void setup() {
  Serial.begin(115200);

  // Establish LOW output levels before enabling GPIO outputs.
  digitalWrite(leftMotor_PWM, LOW);
  digitalWrite(rightMotor_PWM, LOW);
  digitalWrite(leftMotor_IN1, LOW);
  digitalWrite(leftMotor_IN2, LOW);
  digitalWrite(rightMotor_IN3, LOW);
  digitalWrite(rightMotor_IN4, LOW);

  pinMode(leftMotor_PWM, OUTPUT);
  pinMode(rightMotor_PWM, OUTPUT);
  pinMode(leftMotor_IN1, OUTPUT);
  pinMode(leftMotor_IN2, OUTPUT);
  pinMode(rightMotor_IN3, OUTPUT);
  pinMode(rightMotor_IN4, OUTPUT);

  stopMotors();

  pinMode(ESTOP_PIN, INPUT_PULLUP);
  emergencyStopActive();
  attachInterrupt(digitalPinToInterrupt(ESTOP_PIN),
                  onEmergencyStop, FALLING);

  pinMode(senseRight_PIN_TRIG, OUTPUT);
  pinMode(senseLeft_PIN_TRIG, OUTPUT);
  pinMode(senseFront_PIN_TRIG, OUTPUT);
  pinMode(senseRight_PIN_ECHO, INPUT);
  pinMode(senseLeft_PIN_ECHO, INPUT);
  pinMode(senseFront_PIN_ECHO, INPUT);

  
  loadSavedCalibrations();

  Serial.println("LOGISTICS BOT - COMMISSIONING MODE");

  Serial.println("BENCH TESTS: raise wheels before testing motors.");
  Serial.println("E-STOP: software latch; hardware cutoff separate.");
  if (emergencyStopActive()) {
    Serial.println("E-STOP ACTIVE: commands locked until reset.");
    return;
  }
 
  // Stage B only: ESP32 Arduino core v3.x LEDC API.
  const bool leftPwmReady = ledcAttach(leftMotor_PWM, 1000, 8);
  const bool rightPwmReady = ledcAttach(rightMotor_PWM, 1000, 8);
  if (!leftPwmReady || !rightPwmReady) {
    if (leftPwmReady) ledcDetach(leftMotor_PWM);
    if (rightPwmReady) ledcDetach(rightMotor_PWM);
    motorFaultLatched = true;  // Fail closed; reset required.
    stopMotors();
    Serial.println("PWM INIT FAILED: TEST LOCKED");
    return;
  }
  pwmModeEnabled = true;
  stopMotors();
   Serial.printf(
    "PWM: 1000 Hz | 8-bit | duty=%u/255\n",
    (unsigned)MOTOR_TEST_DUTY
  );

  Serial.println("TURNS: Q/E = left/right 90 degrees");
  Serial.println("DISTANCE: 1/2 = forward 10cm/15cm");
  Serial.println("CALIBRATE: C1=10cm, C2=15cm, CQ, CE");
  Serial.println("BENCH: A/l left, S/r right, F/B both");
  Serial.println("CONTROL: +/- duration, X stop, ? status");
  Serial.println("SERIAL: 115200 baud, Newline ending");
  Serial.println("AUTO: N = one action | Z = continuous ON/OFF (30s / 10 actions)");
  Serial.println("ESCAPE: G arms one unsensed 15cm probe for the next local Z session (TEST ONLY)");

  Serial.printf(
    "Bench duration: %lu ms | Limits: %lu-%lu ms\n",
    motorTestMs,
    MOTOR_TEST_MIN_MS,
    MOTOR_TEST_MAX_MS
  );

  Serial.printf(
    "Maximum movement duration: %lu ms\n",
    MOTION_MAX_MS
  );
 
  WiFi.mode(WIFI_STA);
  WiFi.setAutoReconnect(true);

  Serial.printf("WIFI: attempting connection to %s\n", WIFI_SSID);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  Blynk.config(BLYNK_AUTH_TOKEN);

  Blynk.disconnect();

  blynkTimer.setInterval(2000L, publishBlynkTelemetry);

  Serial.println(

    "BLYNK: Wi-Fi starting; motors remain under local interlock"

  );
}

/* ============================================================
   5. ULTRASONIC SENSOR FUNCTIONS
   ============================================================ */
// Returns a distance in cm.
// -2.0f = TIMEOUT; -1.0f = out of range.

float readDistanceCm(int trigPin, int echoPin) {
  // Send a 10 microsecond trigger pulse
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);

  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  // Measure how long ECHO remains HIGH
  unsigned long duration =
      pulseIn(echoPin, HIGH, ECHO_TIMEOUT_US);

  // No echo received within the timeout period.
  if (duration == 0) {
    Serial.printf("ECHO GPIO %d | TIMEOUT | duration=%lu us\n",
                  echoPin, duration);
    return -2.0f;
  }
  // Convert round-trip travel time to distance
  float distanceCm = duration * 0.0343f / 2.0f;
  if (distanceCm < SENSOR_MIN_CM ||
      distanceCm > SENSOR_MAX_CM) {
    Serial.printf("ECHO GPIO %d | OUT_OF_RANGE | duration=%lu us | distance=%.1f cm\n",
                  echoPin, duration, distanceCm);
    return -1.0f;
  }
  Serial.printf("ECHO GPIO %d | OK | duration=%lu us | distance=%.1f cm\n",
                echoPin, duration, distanceCm);
  return distanceCm;
}
// A missing echo is unknown, not a confirmed close obstacle
bool isFrontBlocked(float distanceCm) {
  return distanceCm >= SENSOR_MIN_CM &&
         distanceCm <= FRONT_OBSTACLE_CM;
}
// Median of five samples; require at least three valid echoes.

float readFilteredDistanceCm(int trigPin, int echoPin,
                             uint8_t* closeEchoCount = nullptr,
                             uint8_t* timeoutEchoCount = nullptr) {
  if (closeEchoCount != nullptr) *closeEchoCount = 0;
  if (timeoutEchoCount != nullptr) *timeoutEchoCount = 0;

  const int SAMPLE_COUNT = 5;
  const int MIN_VALID = 3;
  float validSamples[SAMPLE_COUNT];
  int validCount = 0;

  for (int i = 0; i < SAMPLE_COUNT; i++) {
    if (!enforceMotorInterlock()) return -1.0f;

    const float cm = readDistanceCm(trigPin, echoPin);
    if (cm == -2.0f && timeoutEchoCount != nullptr) {
      ++(*timeoutEchoCount);
    }

    if (cm >= 0.0f) {
      validSamples[validCount++] = cm;

      if (closeEchoCount != nullptr && isFrontBlocked(cm)) {
        ++(*closeEchoCount);
      }
    }

    if (i < SAMPLE_COUNT - 1) {
      delay(SENSOR_GAP_MS);
    }
  }

  Serial.printf(
    "ECHO GPIO %d | VALID ECHO READINGS: %d/5 (%d%%)\n",
    echoPin,
    validCount,
    validCount * 20
  );

  if (validCount < MIN_VALID) {
    return -1.0f;
  }

  // Sort valid measurements to calculate the median.
  for (int i = 0; i < validCount - 1; i++) {
    for (int j = i + 1; j < validCount; j++) {
      if (validSamples[i] > validSamples[j]) {
        float temp = validSamples[i];
        validSamples[i] = validSamples[j];
        validSamples[j] = temp;
      }
    }
  }

  return validSamples[(validCount - 1) / 2];
}


// True only if all 15 raw ECHO attempts timed out (not merely an invalid median).
bool lastFullScanAllTimeouts = false;

// Read and classify front, left and right sensor scans.
void readAllDistances(float* front, float* left, float* right,
                      bool* frontClose, bool* leftClose, bool* rightClose) {
  *front = *left = *right = -1.0f;
  *frontClose = *leftClose = *rightClose = false;
  lastFullScanAllTimeouts = false;

  uint8_t closeCount = 0;
  uint8_t frontTimeouts = 0, leftTimeouts = 0, rightTimeouts = 0;

  // Front sensor
  *front = readFilteredDistanceCm(
      senseFront_PIN_TRIG, senseFront_PIN_ECHO, &closeCount, &frontTimeouts);
  *frontClose = closeCount >= CLOSE_CONFIRMATIONS;

  if (!enforceMotorInterlock()) return;
  delay(SENSOR_GAP_MS);

  // Left sensor
  closeCount = 0;
  *left = readFilteredDistanceCm(
      senseLeft_PIN_TRIG, senseLeft_PIN_ECHO, &closeCount, &leftTimeouts);
  *leftClose = closeCount >= CLOSE_CONFIRMATIONS;

  if (!enforceMotorInterlock()) return;
  delay(SENSOR_GAP_MS);

  // Right sensor
  closeCount = 0;
  *right = readFilteredDistanceCm(
      senseRight_PIN_TRIG, senseRight_PIN_ECHO, &closeCount, &rightTimeouts);
  *rightClose = closeCount >= CLOSE_CONFIRMATIONS;
  lastFullScanAllTimeouts = (frontTimeouts == 5 &&
                             leftTimeouts == 5 && rightTimeouts == 5);
}



/* ============================================================
   6. SERIAL MOTION AND CALIBRATION
   ============================================================ */
enum MotionKind : uint8_t { DRIVE, TURN_LEFT, TURN_RIGHT, BENCH };
enum MotionPurpose : uint8_t { NONE, AUTO, MANUAL, CALIBRATE, BENCH_TEST };

// Explicit prototype to prevent Arduino auto-declaration errors.
void beginPlan(MotionPurpose purpose);

struct MotionStep {
  MotionKind kind;
  int leftDir, rightDir;
  unsigned long durationMs;
  float forwardCm;      // 0 = no forward clearance test
};

MotionStep steps[3];
uint8_t stepCount = 0;
uint8_t stepIndex = 0;
unsigned long stepStartMs = 0;
unsigned long lastFrontMs = 0;

uint8_t movingFrontCloseCount = 0;
uint8_t movingFrontInvalidCount = 0;

bool motionRunning = false;
bool waitingForMeasurement = false;
MotionPurpose motionPurpose = NONE;
char calibrationKind = 0;
unsigned long calibrationTestMs = 0;

// Continuous mode is opt-in, time-bounded and OFF after each reboot.
constexpr unsigned long AUTO_SCAN_PAUSE_MS = 500UL;
constexpr unsigned long AUTO_SESSION_LIMIT_MS = 30000UL;
constexpr uint8_t AUTO_ACTION_LIMIT = 10;
bool continuousAutoEnabled = false;
bool continuousAutoFromBlynk = false;
unsigned long autoStartedMs = 0;
unsigned long nextAutoScanMs = 0;
uint8_t autoActionsCompleted = 0;

// All-timeout recovery: one in-place 90-degree turn per session, then
// optionally one 15 cm unsensed probe, locally armed with G before Z.
// The probe is OFF after a reboot/session; only use it in a cleared,
// supervised test area. It cannot guarantee collision avoidance.
constexpr uint8_t ALL_TIMEOUT_CONFIRM_SCANS = 2;
enum EscapeStage : uint8_t { ESCAPE_READY, ESCAPE_TURN_ATTEMPTED, ESCAPE_PROBE_USED };
EscapeStage escapeStage = ESCAPE_READY;
uint8_t allTimeoutScans = 0;
bool escapeProbeArmed = false;
bool escapeProbeActive = false;

// One automatic REVERSE pulse per local Z session after repeated front
// timeout failures. No rear sensor: test only in a cleared, supervised area.
constexpr unsigned long AUTO_REVERSE_RECOVERY_MS = 500UL;
constexpr unsigned long AUTO_REVERSE_LATEST_START_MS = 25000UL;
static_assert(AUTO_REVERSE_RECOVERY_MS >= MOTION_MIN_MS &&
              AUTO_REVERSE_RECOVERY_MS <= MOTION_MAX_MS,
              "Reverse recovery duration exceeds movement limits");
uint8_t failedFrontRecoveryScans = 0;
bool autoReverseRecoveryUsed = false;

void cancelMotion(const char* message) {
  stopMotors();
  delay(5);  // Allow PWM shutdown to settle before readback.
  remoteMotionActive = false;
  const bool autoWasEnabled = continuousAutoEnabled;
  continuousAutoEnabled = false;
  continuousAutoFromBlynk = false;
  escapeProbeArmed = false;
  escapeProbeActive = false;
  if (autoWasEnabled && Blynk.connected()) Blynk.virtualWrite(V7, 0);

  // Check commanded PWM duty after shutdown.
  if (pwmModeEnabled &&
      (ledcRead(leftMotor_PWM) != 0 ||
       ledcRead(rightMotor_PWM) != 0)) {
    motorFaultLatched = true;
    Serial.println("STOP PWM READBACK FAILED: RESET REQUIRED");
  }

  motionRunning = false;
  waitingForMeasurement = false;
  motionPurpose = NONE;
  stepCount = 0;
  stepIndex = 0;
  calibrationKind = 0;

  Serial.println(message);
}

// Sensor uncertainty is recoverable during continuous autonomy, not a motor fault.
// Stop and verify motor PWM is zero before scheduling another stationary scan.
// Manual movement and single-action N still cancel on a failed front check.
void stopForFrontRecheck(const char* message) {
  if (motionPurpose != AUTO || !continuousAutoEnabled ||
      !movementPermitted()) {
    cancelMotion(message);
    return;
  }

  stopMotors();
  delay(5);
  if (pwmModeEnabled &&
      (ledcRead(leftMotor_PWM) != 0 ||
       ledcRead(rightMotor_PWM) != 0)) {
    latchMotorFault();
    cancelMotion("STOP PWM DUTY READBACK NONZERO: reset required");
    return;
  }

  motionRunning = false;
  remoteMotionActive = false;
  motionPurpose = NONE;
  stepCount = 0;
  stepIndex = 0;
  movingFrontCloseCount = 0;
  movingFrontInvalidCount = 0;
  escapeProbeActive = false;
  nextAutoScanMs = millis() + AUTO_SCAN_PAUSE_MS;

  Serial.println(message);
  Serial.println("AUTO: stopped for front recheck; retry scan in 500 ms");
}

void addStep(MotionKind kind, int left, int right,
             unsigned long duration, float forwardCm = 0.0f) {
  steps[stepCount++] = {kind, left, right, duration, forwardCm};
}

bool beginStep() {
  if (!enforceMotorInterlock()) {
    cancelMotion("INTERLOCK: motion cancelled");
    return false;
  }
  const MotionStep& s = steps[stepIndex];
  if (s.durationMs < MOTION_MIN_MS || s.durationMs > MOTION_MAX_MS) {
    cancelMotion("INVALID DURATION: motion cancelled");
    return false;
  }

  
  
  // Every forward travel step requires a valid, sufficiently clear scan.
  stopMotors();
  // Only the locally armed escape probe bypasses the front precheck.
  // Supervised calibration still scans and stops on confirmed close objects,
  // but can proceed when the sensor returns no valid median.
  if (s.forwardCm > 0.0f && !escapeProbeActive) {
    uint8_t closeEchoCount = 0;
    const float frontCm = readFilteredDistanceCm(
        senseFront_PIN_TRIG, senseFront_PIN_ECHO, &closeEchoCount);

    if (!enforceMotorInterlock()) {
      cancelMotion("INTERLOCK DURING FRONT CHECK");
      return false;
    }

    Serial.printf("FRONT PRECHECK: median=%.1f cm | close=%u/5\n",
                  frontCm, (unsigned)closeEchoCount);

    if (closeEchoCount >= CLOSE_CONFIRMATIONS) {
      stopForFrontRecheck("FRONT CLOSE OBSTACLE CONFIRMED: STOP");
      return false;
    }

    if (frontCm < AUTO_CLEARANCE_CM &&
        !(motionPurpose == CALIBRATE && frontCm < 0.0f)) {
      if (motionPurpose == AUTO && continuousAutoEnabled && frontCm < 0.0f &&
          failedFrontRecoveryScans < 255) {
        ++failedFrontRecoveryScans;
      }
      stopForFrontRecheck("FRONT STOP: clearance unknown or insufficient");
      return false;
    }
  }


   // Apply right-wheel trim only when both wheels drive forward.
  const bool straightForward =
      s.kind == DRIVE && s.leftDir == 1 && s.rightDir == 1;

  const uint8_t requestedRightDuty =
      straightForward ? rightDriveDuty() : MOTOR_TEST_DUTY;

  motionRunning = true;

  if (s.leftDir) setLeftMotor(s.leftDir);
  if (!enforceMotorInterlock()) {
    cancelMotion("INTERLOCK DURING MOTOR START");
    return false;
  }

  if (s.rightDir) setRightMotor(s.rightDir, requestedRightDuty);
  if (!enforceMotorInterlock()) {
    cancelMotion("INTERLOCK DURING MOTOR START");
    return false;
  }

  const uint32_t expectedLeft = s.leftDir ? MOTOR_TEST_DUTY : 0;
  const uint32_t expectedRight = s.rightDir ? requestedRightDuty : 0;
  if (ledcRead(leftMotor_PWM) != expectedLeft ||
      ledcRead(rightMotor_PWM) != expectedRight) {
    latchMotorFault();
    cancelMotion("PWM READBACK MISMATCH: reset required");
    return false;
  }




  stepStartMs = millis();
  lastFrontMs = stepStartMs;
  movingFrontCloseCount = 0;
  movingFrontInvalidCount = 0;
  Serial.printf("STEP %u/%u: %lu ms; LEFT=%d RIGHT=%d\n",
                stepIndex + 1, stepCount, s.durationMs,
                s.leftDir, s.rightDir);
  return true;
}

void beginPlan(MotionPurpose purpose) {
  if (motionRunning || waitingForMeasurement || stepCount == 0) return;
  motionPurpose = purpose;
  stepIndex = 0;
  beginStep();
}

void advanceMotion() {
  if (!motionRunning) return;
  if (!enforceMotorInterlock()) {
    cancelMotion("INTERLOCK: motors off; reset required");
    return;
  }
  const MotionStep s = steps[stepIndex];
  if (millis() - stepStartMs < s.durationMs) {
    if (s.forwardCm > 0.0f && millis() - lastFrontMs >= SENSOR_GAP_MS) {
      lastFrontMs = millis();
      const float cm = readDistanceCm(senseFront_PIN_TRIG,
                                      senseFront_PIN_ECHO);
      if (!enforceMotorInterlock()) {
        cancelMotion("INTERLOCK DURING FRONT CHECK");
        return;
      }
      if (isFrontBlocked(cm)) {
        ++movingFrontCloseCount;
        if (movingFrontCloseCount >= CLOSE_CONFIRMATIONS) {
          stopForFrontRecheck("CONFIRMED FRONT OBSTACLE <=15 cm: STOP");
          return;
        }
      } else if (cm >= 0.0f) {
        movingFrontCloseCount = 0;
      }
      // Escape probing and supervised calibration tolerate missing echoes.
      // Confirmed close obstacles still stop both kinds of motion.
      // Other forward movements retain the two-invalid-echo stop.
      if (cm < 0.0f && !escapeProbeActive &&
          motionPurpose != CALIBRATE) {
        if (++movingFrontInvalidCount >= 2) {
          if (motionPurpose == AUTO && continuousAutoEnabled &&
              failedFrontRecoveryScans < 255) {
            ++failedFrontRecoveryScans;
          }
          stopForFrontRecheck("FRONT STOP: repeated invalid front echoes");
          return;
        }
      } else if (cm >= 0.0f) {
        movingFrontInvalidCount = 0;
      }
    }
    return;
  }

  stopMotors();
  delay(5);  // Allow PWM shutdown to settle.

  const uint32_t leftStopDuty = ledcRead(leftMotor_PWM);
  const uint32_t rightStopDuty = ledcRead(rightMotor_PWM);

  Serial.printf(
    "STOP PWM READBACK: LEFT=%lu RIGHT=%lu\n",
    (unsigned long)leftStopDuty,
    (unsigned long)rightStopDuty
  );

  if (leftStopDuty != 0 || rightStopDuty != 0) {
    latchMotorFault();
    cancelMotion("STOP PWM DUTY READBACK NONZERO: reset required");
    return;
  }
  Serial.println(
    "STEP COMPLETE: BOTH LEDC DUTY READBACKS ZERO"
  );


  motionRunning = false;
  escapeProbeActive = false;
  ++stepIndex;
  if (stepIndex < stepCount) {
    beginStep();
    return;
  }

  if (motionPurpose == AUTO && continuousAutoEnabled) {
    if (s.kind == DRIVE && s.leftDir == 1 && s.rightDir == 1) {
      failedFrontRecoveryScans = 0; // Progress: a forward step completed.
    }
    ++autoActionsCompleted;
    nextAutoScanMs = millis() + AUTO_SCAN_PAUSE_MS;
  }
  if (motionPurpose == CALIBRATE) {
    waitingForMeasurement = true;
    Serial.printf("CALIBRATION %c: enter measured %s, then ENTER (X cancels):\n",
                  calibrationKind,
                  (calibrationKind == '1' || calibrationKind == '2')
                      ? "distance in cm" : "angle in degrees");
  }
  if (!waitingForMeasurement) motionPurpose = NONE;
  remoteMotionActive = false;
}

void recordMeasurement(const char* input) {
  char* end = nullptr;
  const float actual = strtof(input, &end);
  if (end == input || *end != '\0' || !isfinite(actual) || actual <= 0.0f) {
    Serial.println("INVALID: enter a positive number (e.g., 23.5), or X");
    return;
  }
  const float target = (calibrationKind == '1') ? 10.0f :
                       (calibrationKind == '2') ? 15.0f : 90.0f;
  const float tolerance = (calibrationKind == '1') ? 0.5f :
                          (calibrationKind == '2') ? 1.0f : 5.0f;
  const double proposed = (double)calibrationTestMs * target / actual;
  if (proposed < MOTION_MIN_MS || proposed > MOTION_MAX_MS) {
    Serial.println("CALIBRATION OUT OF BOUNDS: X, then use shorter test conditions");
    return;
  }
  const unsigned long corrected = (unsigned long)(proposed + 0.5);
  const bool verified = fabsf(target - actual) <= tolerance;
  // A timing is verified only if that exact timing was physically tested.
  const unsigned long nextMs = verified ? calibrationTestMs : corrected;
  switch (calibrationKind) {
    case '1': forward10Ms = nextMs; calibrated10 = verified; break;
    case '2': forward15Ms = nextMs; calibrated15 = verified; break;
    case 'Q': turnLeft90Ms = nextMs; calibratedLeft = verified; break;
    case 'E': turnRight90Ms = nextMs; calibratedRight = verified; break;
  }
  
  // Identify the flash key for this calibration.
  const char* calKey =
      calibrationKind == '1' ? "c1" :
      calibrationKind == '2' ? "c2" :
      calibrationKind == 'Q' ? "cq" :
      calibrationKind == 'E' ? "ce" : nullptr;

  if (calibrationStorageReady && calKey != nullptr) {
    if (verified) {
      // Save only the duration that was physically verified.
      if (calibrationStore.putULong(calKey, nextMs) == 4) {
        Serial.printf("CAL FLASH SAVED: %s = %lu ms\n",
                      calKey, nextMs);
      } else {
        Serial.println("CAL FLASH SAVE FAILED: RAM only");
      }
    } else {
      // A failed retest invalidates any previously saved calibration.
      if (calibrationStore.isKey(calKey) &&
          !calibrationStore.remove(calKey)) {
        Serial.println("CAL FLASH WARNING: old value not erased");
      }
    }
  }

  Serial.printf("MEASURED %.1f, TARGET %.1f => NEXT TEST %lu ms; %s\n",
                actual, target, nextMs,
                verified ? "VERIFIED (within test tolerance)" : "REPEAT CALIBRATION");
  waitingForMeasurement = false;
  motionPurpose = NONE;
  calibrationKind = 0;
}



// A timed reverse is a last-resort local recovery action, not a proof
// that the space behind the robot is clear. One pulse maximum per session.
bool startAutoReverseRecovery() {
  if (!continuousAutoEnabled || continuousAutoFromBlynk ||
      autoReverseRecoveryUsed || !movementPermitted() ||
      motionRunning || waitingForMeasurement || motionPurpose != NONE ||
      millis() - autoStartedMs > AUTO_REVERSE_LATEST_START_MS ||
      autoActionsCompleted >= AUTO_ACTION_LIMIT) {
    return false;
  }

  autoReverseRecoveryUsed = true;
  failedFrontRecoveryScans = 0;
  stepCount = 0;
  addStep(BENCH, -1, -1, AUTO_REVERSE_RECOVERY_MS);
  Serial.println("AUTO RECOVERY: one 500 ms reverse pulse, then rescan; NO rear sensing");
  beginPlan(AUTO);
  return motionRunning;
}

// Avoid starting a navigation plan too close to the 30-second hard cutoff.
bool autoPlanFitsSession(unsigned long planDurationMs) {
  if (!continuousAutoEnabled) return true;  // Single-action N is not timed.
  const unsigned long elapsed = millis() - autoStartedMs;
  const unsigned long reserveMs = 750UL; // Front precheck and loop overhead.
  return elapsed < AUTO_SESSION_LIMIT_MS &&
         planDurationMs <= AUTO_SESSION_LIMIT_MS - elapsed &&
         reserveMs <= AUTO_SESSION_LIMIT_MS - elapsed - planDurationMs;
}

// One bounded reactive decision; returns true only if motion begins.
bool startReactiveAction() {
  if (motionRunning || waitingForMeasurement ||
      motionPurpose != NONE || !movementPermitted()) return false;

  if (!calibrated15) {
    Serial.println("AUTO LOCKED: calibrate and verify C2 first");
    return false;
  }

  stopMotors();
  float frontCm = -1.0f, leftCm = -1.0f, rightCm = -1.0f;
  bool frontClose = false, leftClose = false, rightClose = false;

  readAllDistances(&frontCm, &leftCm, &rightCm,
                   &frontClose, &leftClose, &rightClose);

  if (!enforceMotorInterlock()) {
    Serial.println("AUTO STOP: interlock during scan");
    return false;
  }

  Serial.printf("AUTO SCAN: F=%.1f L=%.1f R=%.1f | close=%d,%d,%d\n",
                frontCm, leftCm, rightCm,
                (int)frontClose, (int)leftClose, (int)rightClose);

  if (lastFullScanAllTimeouts) {
    if (failedFrontRecoveryScans < 255) ++failedFrontRecoveryScans;
    if (allTimeoutScans < ALL_TIMEOUT_CONFIRM_SCANS) ++allTimeoutScans;
    if (continuousAutoEnabled &&
        allTimeoutScans >= ALL_TIMEOUT_CONFIRM_SCANS) {
      stepCount = 0;
      if (escapeStage == ESCAPE_READY && calibratedLeft &&
          autoPlanFitsSession(turnLeft90Ms)) {
        escapeStage = ESCAPE_TURN_ATTEMPTED;
        failedFrontRecoveryScans = 0; // Assess the new direction after turning.
        addStep(TURN_LEFT, -1, 1, turnLeft90Ms);
        Serial.println("ESCAPE: 15/15 timeouts twice; one left 90-degree turn, then rescan");
        beginPlan(AUTO);
        return motionRunning;
      }
      // After the escape turn, try a bounded automatic reverse before the
      // optional G-armed forward probe. A timeout does not latch a fault.
      if (escapeStage == ESCAPE_TURN_ATTEMPTED &&
          startAutoReverseRecovery()) {
        return true;
      }
      if (escapeStage == ESCAPE_TURN_ATTEMPTED && escapeProbeArmed &&
          !continuousAutoFromBlynk && autoPlanFitsSession(forward15Ms)) {
        escapeStage = ESCAPE_PROBE_USED;
        escapeProbeArmed = false;
        escapeProbeActive = true;
        addStep(DRIVE, 1, 1, forward15Ms, 15.0f);
        Serial.println("ESCAPE PROBE: one UNVERIFIED 15 cm step; supervise and use X/E-stop");
        beginPlan(AUTO);
        if (!motionRunning) escapeProbeActive = false;
        return motionRunning;
      }
    }
    Serial.println("AUTO: all 15 echoes timed out; waiting for another scan");
    return false;
  }
  allTimeoutScans = 0;

  const bool frontClear = !frontClose && frontCm >= AUTO_CLEARANCE_CM;
  const bool leftClear = !leftClose && leftCm >= AUTO_CLEARANCE_CM;
  const bool rightClear = !rightClose && rightCm >= AUTO_CLEARANCE_CM;

  // Repeated failed front prechecks can occur despite a previously valid
  // scan. Try one reverse pulse rather than endlessly repeating the plan.
  if (failedFrontRecoveryScans >= 2 && startAutoReverseRecovery()) {
    return true;
  }

  stepCount = 0;
  if (frontClear) {
    Serial.println("AUTO: forward selected");
  } else if (leftClear || rightClear) {
    // Prefer the side with more measured clearance, left on an exact tie.
    const bool chooseLeft = leftClear && (!rightClear || leftCm >= rightCm);
    if (chooseLeft) {
      if (!calibratedLeft) {
        Serial.println("AUTO LOCKED: verify CQ (left 90) first");
        return false;
      }
      addStep(TURN_LEFT, -1, 1, turnLeft90Ms);
      Serial.println("AUTO: turn left 90, then recheck front before driving");
    } else {
      if (!calibratedRight) {
        Serial.println("AUTO LOCKED: verify CE (right 90) first");
        return false;
      }
      addStep(TURN_RIGHT, 1, -1, turnRight90Ms);
      Serial.println("AUTO: turn right 90, then recheck front before driving");
    }
  } else {
    if (frontCm < 0.0f && failedFrontRecoveryScans < 255) {
      ++failedFrontRecoveryScans;
    }
    if (frontCm < 0.0f && failedFrontRecoveryScans >= 2 &&
        startAutoReverseRecovery()) {
      return true;
    }
    Serial.println("AUTO STOP: no confirmed direction with sufficient clearance");
    return false;
  }

  // The existing beginStep() front precheck runs AFTER a turn, too.
  addStep(DRIVE, 1, 1, forward15Ms, 15.0f);
  unsigned long planDurationMs = forward15Ms;
  if (stepCount > 1) planDurationMs += steps[0].durationMs;
  if (!autoPlanFitsSession(planDurationMs)) {
    stepCount = 0;
    Serial.println("AUTO: insufficient time remaining for a complete action");
    return false;
  }
  beginPlan(AUTO);
  return motionRunning;
}

// Z toggles continuous autonomy locally; Blynk V7 provides the same switch.
// Both paths require all three measured movement calibrations to be verified.
void setContinuousAuto(bool enable, bool fromBlynk) {
  if (!enable) {
    if (continuousAutoEnabled && motionPurpose == AUTO && motionRunning) {
      cancelMotion("AUTO OFF: motion cancelled");
    } else {
      continuousAutoEnabled = false;
      continuousAutoFromBlynk = false;
      escapeProbeArmed = false;
      escapeProbeActive = false;
      if (Blynk.connected()) Blynk.virtualWrite(V7, 0);
      Serial.println("AUTO OFF: stopped");
    }
    return;
  }

  if (continuousAutoEnabled) return;

  // Remote autonomy must obey the same commissioning lock as V2-V5.
  // This does not disable local serial N/Z testing or the remote V6 stop.
  if (fromBlynk && !ENABLE_BLYNK_MOVEMENT) {
    Serial.println("AUTO LOCKED: Blynk movement disabled for commissioning");
    if (Blynk.connected()) Blynk.virtualWrite(V7, 0);
    return;
  }

  if (!movementPermitted() || motionRunning || waitingForMeasurement ||
      motionPurpose != NONE || !calibrated15 || !calibratedLeft ||
      !calibratedRight ||
      (fromBlynk && (WiFi.status() != WL_CONNECTED || !Blynk.connected()))) {
    Serial.println("AUTO LOCKED: clear interlocks and verify C2, CQ and CE");
    if (Blynk.connected()) Blynk.virtualWrite(V7, 0);
    return;
  }

  continuousAutoEnabled = true;
  continuousAutoFromBlynk = fromBlynk;
  autoStartedMs = millis();
  nextAutoScanMs = autoStartedMs + AUTO_SCAN_PAUSE_MS;
  autoActionsCompleted = 0;
  escapeStage = ESCAPE_READY;
  allTimeoutScans = 0;
  escapeProbeActive = false;
  failedFrontRecoveryScans = 0;
  autoReverseRecoveryUsed = false;
  if (Blynk.connected()) Blynk.virtualWrite(V7, 1);
  Serial.println("AUTO ON: up to 10 actions or 30 seconds; X, Z or V7 OFF stops");
}

// Run only when no motion is active. Never queue a second motion plan.
void serviceContinuousAuto() {
  if (!continuousAutoEnabled) return;

  const unsigned long now = millis();
  if (!movementPermitted()) {
    cancelMotion("AUTO STOP: interlock active");
    return;
  }
  if (continuousAutoFromBlynk &&
      (WiFi.status() != WL_CONNECTED || !Blynk.connected())) {
    cancelMotion("AUTO STOP: Blynk disconnected");
    return;
  }
  if (now - autoStartedMs >= AUTO_SESSION_LIMIT_MS ||
      autoActionsCompleted >= AUTO_ACTION_LIMIT) {
    cancelMotion("AUTO STOP: supervised session limit reached");
    return;
  }
  if (motionRunning || waitingForMeasurement || motionPurpose != NONE) return;
  if ((int32_t)(now - nextAutoScanMs) < 0) return;

  if (!startReactiveAction() && continuousAutoEnabled) {
    if (!movementPermitted()) {
      cancelMotion("AUTO STOP: interlock during scan");
      return;
    }
    // A scan without a verified direction does not disable continuous autonomy.
    // Keep the existing 30-second / 10-action supervised limits in effect.
    nextAutoScanMs = millis() + AUTO_SCAN_PAUSE_MS;
    Serial.println("AUTO: no verified clear action; rescan in 500 ms");
  }
}

void processCommand(const char* command) {
  // Z can turn autonomy OFF even while a movement is underway.
  if (command[0] == 'Z' && command[1] == '\0') {
    setContinuousAuto(!continuousAutoEnabled, false);
    return;
  }
  if (waitingForMeasurement) {
    recordMeasurement(command);
    return;
  }
  if (motionRunning) {
    Serial.println("BUSY: only X is accepted while motors run");
    return;
  }
  if (!enforceMotorInterlock()) return;
  if (command[0] == 'C' && command[2] == '\0' &&
      (command[1] == '1' || command[1] == '2' ||
       command[1] == 'Q' || command[1] == 'E')) {
    calibrationKind = command[1];
    stepCount = 0;
    if (calibrationKind == '1') {
      calibrationTestMs = forward10Ms;
      addStep(DRIVE, 1, 1, calibrationTestMs, 10.0f);
    } else if (calibrationKind == '2') {
      calibrationTestMs = forward15Ms;
      addStep(DRIVE, 1, 1, calibrationTestMs, 15.0f);
    } else if (calibrationKind == 'Q') {
      calibrationTestMs = turnLeft90Ms;
      addStep(TURN_LEFT, -1, 1, calibrationTestMs);
    } else {
      calibrationTestMs = turnRight90Ms;
      addStep(TURN_RIGHT, 1, -1, calibrationTestMs);
    }
    beginPlan(CALIBRATE);
    return;
  }

  if (command[1] != '\0') {
    
    Serial.println("COMMANDS: ? status, N one action, Z auto ON/OFF, G probe arm, X stop");
    Serial.println("MOVE: 1/2, Q/E | CAL: C1/C2/CQ/CE");
    Serial.println("BENCH: A/l left, S/r right, F/B both, +/- duration");

    return;
  }
  const char c = command[0];
  if (c == '?') {
    Serial.printf("CAL: 10=%lu(%d) 15=%lu(%d) LEFT90=%lu(%d) RIGHT90=%lu(%d) ms\n",
                  forward10Ms, calibrated10, forward15Ms, calibrated15,
                  turnLeft90Ms, calibratedLeft, turnRight90Ms, calibratedRight);
    Serial.printf("NAV: reactive, no map | continuous=%s | actions=%u/%u\n",
                  continuousAutoEnabled ? "ON" : "OFF",
                  (unsigned)autoActionsCompleted, (unsigned)AUTO_ACTION_LIMIT);
    Serial.printf("ESCAPE: probe=%s | stage=%u | all-timeout scans=%u\n",
                  escapeProbeArmed ? "ARMED (TEST ONLY)" : "OFF",
                  (unsigned)escapeStage, (unsigned)allTimeoutScans);
    Serial.printf("REVERSE RECOVERY: used=%s | failed-front=%u | duration=%lu ms\n",
                  autoReverseRecoveryUsed ? "YES" : "NO",
                  (unsigned)failedFrontRecoveryScans,
                  AUTO_REVERSE_RECOVERY_MS);

    Serial.printf(
      "INTERLOCKS: E-stop=%s | Fault=%s | Remote=%s\n",
      estopLatched ? "LATCHED" : "CLEAR",
      motorFaultLatched ? "LATCHED" : "CLEAR",
      remoteStopLatched ? "LATCHED" : "CLEAR"
    );

    
    Serial.printf(
      "BLYNK: %s | Remote movement: %s\n",
      Blynk.connected() ? "ONLINE" : "OFFLINE",
      ENABLE_BLYNK_MOVEMENT ? "ENABLED" : "LOCKED"
    );

    const wl_status_t wifiStatus = WiFi.status();

    Serial.printf(
      "WIFI: %s | status=%d\n",
      wifiStatus == WL_CONNECTED ? "CONNECTED" : "NOT CONNECTED",
      (int)wifiStatus
    );

    if (wifiStatus == WL_CONNECTED) {
      Serial.print("WIFI IP: ");
      Serial.println(WiFi.localIP());
      Serial.printf("WIFI RSSI: %d dBm\n", WiFi.RSSI());
    }

    return;

  }
  if (c == '+' || c == '-') {
    if (c == '+') motorTestMs = min(motorTestMs + MOTOR_TEST_STEP_MS,
                                   MOTOR_TEST_MAX_MS);
    else motorTestMs = max(motorTestMs - MOTOR_TEST_STEP_MS,
                           MOTOR_TEST_MIN_MS);
    Serial.printf("BENCH DURATION=%lu ms\n", motorTestMs);
    return;
  }

  if (continuousAutoEnabled) {
    Serial.println("AUTO RUNNING: Z, X or Blynk V7 OFF stops it");
    return;
  }
  if (c == 'G') {
    escapeProbeArmed = !escapeProbeArmed;
    Serial.println(escapeProbeArmed
        ? "ESCAPE PROBE ARMED: supervised test only; one blind 15 cm step maximum"
        : "ESCAPE PROBE DISARMED");
    return;
  }
  if (c == 'N') {
    startReactiveAction();
    return;
  }

  stepCount = 0;
  if (c == '1' || c == '2') {
    addStep(DRIVE, 1, 1, c == '1' ? forward10Ms : forward15Ms,
            c == '1' ? 10.0f : 15.0f);
    beginPlan(MANUAL);
    return;
  }
  if (c == 'Q' || c == 'E') {
    if ((c == 'Q' && !calibratedLeft) ||
      (c == 'E' && !calibratedRight)) {
     Serial.println("TURN WARNING: timing not yet calibrated");
   }
     addStep(c == 'Q' ? TURN_LEFT : TURN_RIGHT,
            c == 'Q' ? -1 : 1, c == 'Q' ? 1 : -1,
            c == 'Q' ? turnLeft90Ms : turnRight90Ms);
    beginPlan(MANUAL);
    return;
  }
  if (c == 'A' || c == 'l' || c == 'S' || c == 'r' ||
      c == 'F' || c == 'B') {
    const int left = (c == 'A' || c == 'F') ? 1 :
                     (c == 'l' || c == 'B') ? -1 : 0;
    const int right = (c == 'S' || c == 'F') ? 1 :
                      (c == 'r' || c == 'B') ? -1 : 0;
    addStep(BENCH, left, right, motorTestMs);
    beginPlan(BENCH_TEST);
    return;
  }
  Serial.println("UNKNOWN COMMAND: send ? for calibration status");
}

/* ============================================================
   7. BLYNK TELEMETRY AND BOUNDED REMOTE COMMANDS
   ============================================================ */

void publishBlynkTelemetry() {
  if (!Blynk.connected()) return;

  float frontCm = -1.0f;

  // Avoid an additional ultrasonic measurement during movement.
  if (!motionRunning &&
      !waitingForMeasurement &&
      movementPermitted()) {
    frontCm = readDistanceCm(
      senseFront_PIN_TRIG,
      senseFront_PIN_ECHO
    );
  }

  const char* status =
      estopLatched ? "SOFTWARE E-STOP" :
      remoteStopLatched ? "REMOTE STOP LATCHED" :
      motorFaultLatched ? "MOTOR FAULT" :
      motionRunning ? (motionPurpose == AUTO ? "AUTO MOVING" : "MOVING") :
      continuousAutoEnabled ? "AUTO SCANNING" :
      "IDLE";

  Blynk.virtualWrite(V0, frontCm);
  Blynk.virtualWrite(V1, status);
}



void serviceBlynk() {
  static wl_status_t lastWifiStatus = WL_IDLE_STATUS;
  static unsigned long lastWifiReportMs = 0;
  static unsigned long lastConnectAttempt = 0;

  const wl_status_t wifiStatus = WiFi.status();
  const unsigned long now = millis();

  // A cloud-started autonomous session must stop on lost connectivity.
  if (continuousAutoEnabled && continuousAutoFromBlynk &&
      (wifiStatus != WL_CONNECTED || !Blynk.connected())) {
    cancelMotion("AUTO STOP: Blynk disconnected");
  }

  // Cancel individual remote movements if connectivity is lost.
  if (remoteMotionActive &&
      (wifiStatus != WL_CONNECTED || !Blynk.connected())) {
    cancelMotion("BLYNK DISCONNECTED: remote movement cancelled");
  }

  // Report changes immediately; report offline status every 10 seconds.
  if (wifiStatus != lastWifiStatus ||
      (wifiStatus != WL_CONNECTED &&
       now - lastWifiReportMs >= 10000UL)) {

    if (wifiStatus == WL_CONNECTED) {
      Serial.print("WIFI CONNECTED | IP=");
      Serial.print(WiFi.localIP());
      Serial.printf(" | RSSI=%d dBm\n", WiFi.RSSI());
    } else {
      Serial.printf("WIFI NOT CONNECTED | status=%d\n",
                    (int)wifiStatus);
    }

    lastWifiStatus = wifiStatus;
    lastWifiReportMs = now;
  }

  // Blynk cannot connect until Wi-Fi is established.
  if (wifiStatus != WL_CONNECTED) return;

  // Try Blynk every 5 seconds while motors are idle.
  if (!Blynk.connected() && !motionRunning &&
      (lastConnectAttempt == 0 ||
       now - lastConnectAttempt >= BLYNK_RECONNECT_MS)) {

    lastConnectAttempt = now;
    Serial.println("BLYNK: attempting cloud connection");

    const bool connected = Blynk.connect(3000);

    Serial.println(
      connected ? "BLYNK: connection successful"
                : "BLYNK: connection failed"
    );
  }

  if (Blynk.connected()) {
    Blynk.run();
    blynkTimer.run();
  }
}



void requestRemoteMovement(const char* command) {
  if (!ENABLE_BLYNK_MOVEMENT ||
      !Blynk.connected() ||
      !movementPermitted() ||
      continuousAutoEnabled ||
      motionRunning ||
      waitingForMeasurement ||
      motionPurpose != NONE) {
    Serial.println(
      "BLYNK MOVE BLOCKED: disabled, busy or interlocked"
    );
    return;
  }

  // Refuse uncalibrated forward or turning commands.
  if ((command[0] == '2' && !calibrated15) ||
      (command[0] == 'Q' && !calibratedLeft) ||
      (command[0] == 'E' && !calibratedRight)) {
    Serial.println(
      "BLYNK MOVE BLOCKED: calibrate C2/CQ/CE first"
    );
    return;
  }

  processCommand(command);

  if (motionRunning) {
    remoteMotionActive = true;
  }
}


// Synchronise the latched STOP only. Never replay ON or movement after reconnect.
BLYNK_CONNECTED() {
  Serial.println("BLYNK ONLINE");
  if (continuousAutoFromBlynk) {
    cancelMotion("AUTO STOP: cloud reconnected; re-enable explicitly");
  }
  Blynk.virtualWrite(V7, continuousAutoEnabled ? 1 : 0);
  Blynk.syncVirtual(V6);
}


// Forward: one calibrated 15 cm movement.
BLYNK_WRITE(V2) {
  if (param.asInt() == 1) {
    requestRemoteMovement("2");
  }
}


// Reverse: one bounded motor-test pulse.
BLYNK_WRITE(V3) {
  if (param.asInt() == 1) {
    requestRemoteMovement("B");
  }
}


// Left: one calibrated 90-degree turn.
BLYNK_WRITE(V4) {
  if (param.asInt() == 1) {
    requestRemoteMovement("Q");
  }
}


// Right: one calibrated 90-degree turn.
BLYNK_WRITE(V5) {
  if (param.asInt() == 1) {
    requestRemoteMovement("E");
  }
}


// Continuous autonomous mode: Blynk switch V7, 1=ON and 0=OFF.
BLYNK_WRITE(V7) {
  setContinuousAuto(param.asInt() == 1, true);
}

// Remote stop: latched until ESP32 reset.
BLYNK_WRITE(V6) {
  if (param.asInt() == 1) {
    remoteStopLatched = true;

    cancelMotion(
      "BLYNK REMOTE STOP: latched until ESP32 reset"
    );
  }
}

void loop() {
  static bool estopReported = false;
  static char line[24];
  static uint8_t length = 0;
  static bool discardingLine = false;

  if (!enforceMotorInterlock()) {
    if (motionRunning || waitingForMeasurement || continuousAutoEnabled)
      cancelMotion("E-STOP/MOTOR FAULT: STOP; reset required");
    if (!estopReported) {
      Serial.println("INTERLOCK LATCHED: reset required");
      estopReported = true;
    }
        while (Serial.available() > 0) Serial.read();
    serviceBlynk();
    return;
  }

  serviceBlynk();

  // A Blynk callback may have latched the remote stop.
  if (!enforceMotorInterlock()) return;

  while (Serial.available() > 0) {
  const char c = (char)Serial.read();

  if (c == 'X' || c == 'x') {
    length = 0;
    discardingLine = false;

    cancelMotion("MANUAL X: BOTH MOTORS OFF; operation cancelled");

    while (Serial.available() > 0) Serial.read();
    return;
  }

  // Ignore the remainder of an oversized command.
  if (discardingLine) {
    if (c == '\r' || c == '\n') discardingLine = false;
    continue;
  }

  if (c == '\r' || c == '\n') {
    if (length > 0) {
      line[length] = '\0';
      processCommand(line);
      length = 0;
    }
  } else if (length < sizeof(line) - 1) {
    line[length++] = c;
  } else {
    length = 0;
    discardingLine = true;
    Serial.println("SERIAL INPUT TOO LONG: discarded");
  }
}
   advanceMotion();
   serviceContinuousAuto();

}
