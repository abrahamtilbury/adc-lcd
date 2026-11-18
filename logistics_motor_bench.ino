#define BLYNK_TEMPLATE_ID "TMPL_REPLACE_ME"
#define BLYNK_TEMPLATE_NAME "Logistics Bot"
#define BLYNK_AUTH_TOKEN "REPLACE_WITH_DEVICE_TOKEN"

#include <WiFi.h>
#include <BlynkSimpleEsp32.h>
#include <math.h>
#include <stdlib.h>

const char WIFI_SSID[] = "REPLACE_WITH_WIFI_NAME";
const char WIFI_PASSWORD[] = "REPLACE_WITH_WIFI_PASSWORD";

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
const float FRONT_OBSTACLE_CM = 20.0f;

/* ============================================================
   2. MOTOR CONTROL AND EMERGENCY STOP
   ============================================================ */
volatile bool estopLatched = false;
bool motorFaultLatched = false;
bool pwmModeEnabled = false;
const uint8_t MOTOR_TEST_DUTY = 180; // Initial bench test: 180/255 (70.6% duty)
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

    const uint32_t actualDuty = ledcRead(leftMotor_PWM);

      Serial.printf(
      "LEFT PWM: requested=%u readback=%lu | %s\n",
      (unsigned)MOTOR_TEST_DUTY,
      (unsigned long)actualDuty,
      actualDuty == MOTOR_TEST_DUTY ? "OK" : "MISMATCH"
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

void setRightMotor(int direction) {
  stopRightMotor();

  if (!enforceMotorInterlock()) return;
  if (direction != 1 && direction != -1) return;

  digitalWrite(rightMotor_IN3, direction == 1 ? HIGH : LOW);
  digitalWrite(rightMotor_IN4, direction == -1 ? HIGH : LOW);

  if (!enforceMotorInterlock()) return;

  
  if (pwmModeEnabled) {
    if (!ledcWrite(rightMotor_PWM, MOTOR_TEST_DUTY)) {
      latchMotorFault();
      return;
    }

    const uint32_t actualDuty = ledcRead(rightMotor_PWM);

     Serial.printf(
      "RIGHT PWM: requested=%u readback=%lu | %s\n",
      (unsigned)MOTOR_TEST_DUTY,
      (unsigned long)actualDuty,
      actualDuty == MOTOR_TEST_DUTY ? "OK" : "MISMATCH"
    );

    if (actualDuty != MOTOR_TEST_DUTY) {
      latchMotorFault();
      return;
    }

  } else {
    digitalWrite(rightMotor_PWM, HIGH);
  }

  enforceMotorInterlock();
}

/* ============================================================
   3. GRID MAP - INITIALISATION AND SERIAL DISPLAY
   ============================================================ */
constexpr int MAP_SIZE = 9;

// -1 = unknown, 0 = clear, 1 = blocked.
int8_t tileMap[MAP_SIZE][MAP_SIZE];

// Record visits separately from obstacle status.
uint8_t tileVisits[MAP_SIZE][MAP_SIZE];

constexpr uint8_t AUTO_MAX_TILES = 81;

// Robot's provisional grid position
int robotRow = MAP_SIZE - 1;
int robotCol = 0;

// Provisional timings (calibrated through C1, C2, CQ, CE).
constexpr unsigned long MOTION_MIN_MS = 50UL;
constexpr unsigned long MOTION_MAX_MS = 4000UL;
constexpr float CALIBRATION_CLEARANCE_CM = 150.0f;

unsigned long forward10Ms = 1000UL;
unsigned long forward20Ms = 2000UL;
unsigned long turnLeft90Ms = 500UL;
unsigned long turnRight90Ms = 500UL;

bool calibrated10 = false;
bool calibrated20 = false;
bool calibratedLeft = false;
bool calibratedRight = false;

// 0=up, 1=right, 2=down, 3=left.
int robotHeading = 0;
bool poseKnown = false;

void initialiseMap() {
  for (int row = 0; row < MAP_SIZE; row++) {
    for (int col = 0; col < MAP_SIZE; col++) {
      tileMap[row][col] = -1;
      tileVisits[row][col] = 0;
    }
  }

  // Starting tile: bottom-left
  tileMap[MAP_SIZE - 1][0] = 0;
  tileVisits[MAP_SIZE - 1][0] = 1;
}

void printMap() {
  for (int row = 0; row < MAP_SIZE; row++) {
    for (int col = 0; col < MAP_SIZE; col++) {
      Serial.printf("%3d", tileMap[row][col]);
    }
    Serial.println();
  }
}

/* ============================================================
   3.1 MAP BOUNDS CHECK
   ============================================================ */

bool inMap(int row, int col) {
  return row >= 0 && row < MAP_SIZE &&
         col >= 0 && col < MAP_SIZE;
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

  initialiseMap();

  Serial.println("LOGISTICS BOT - MOTOR DRIVER BENCH TEST");
  Serial.println("WHEELS OFF THE GROUND. E-stop is software-latched.");
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

  Serial.println("GRID: U/D/L/R = one 20cm tile");
  Serial.println("TURNS: Q/E = left/right 90 degrees");
  Serial.println("DISTANCE: 1/2 = forward 10cm/20cm");
  Serial.println("CALIBRATE: C1, C2, CQ, CE");
  Serial.println("HOME: H = physical grid alignment");
  Serial.println("BENCH: A/l left, S/r right, F/B both");
  Serial.println("CONTROL: +/- duration, X stop, ? status");
  Serial.println("SERIAL: 115200 baud, Newline ending");
  Serial.println("AUTO: N = choose one tile | Z = keep exploring");
  Serial.println("MAP: M = show tile states (-1/0/1)");

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
// Returns a float distance in centimetres.
// Returns -1.0f when the reading is invalid.

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

  // No valid echo received
  if (duration == 0) {
    Serial.printf("ECHO GPIO %d | TIMEOUT | duration=%lu us\n",
                  echoPin, duration);
    return -1.0f;
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

// The filtered front distance can indicate an obstacle.
// A separate close-sample flag preserves conservative detection.
bool isFrontBlocked(float distanceCm) {
  return distanceCm < 0.0f ||
         distanceCm <= FRONT_OBSTACLE_CM;
}


// Five samples per sensor; at least three must be valid.
// Report the median of valid samples.
// With four valid samples, use the lower-middle value.
// Optional output provides nearest valid sample, independent
// of whether enough readings exist to calculate a median.
// Obstacle classification happens in readAllDistances().

float readFilteredDistanceCm(int trigPin, int echoPin,
                             float* nearestValidSampleCm = nullptr) {
  if (nearestValidSampleCm != nullptr) {
    *nearestValidSampleCm = -1.0f;
  }
  const int SAMPLE_COUNT = 5;
  const int MIN_VALID = 3;

  float validSamples[SAMPLE_COUNT];
  int validCount = 0;

  for (int i = 0; i < SAMPLE_COUNT; i++) {
    if (!enforceMotorInterlock()) return -1.0f;
    float cm = readDistanceCm(trigPin, echoPin);

  if (cm >= 0.0f) {
    validSamples[validCount++] = cm;

  if (nearestValidSampleCm != nullptr &&
    (*nearestValidSampleCm < 0.0f ||
       cm < *nearestValidSampleCm)) {
        *nearestValidSampleCm = cm;
      }
    }

    if (i < SAMPLE_COUNT - 1) {
      delay(SENSOR_GAP_MS);
    }
  }

  Serial.printf(
    "ECHO GPIO %d | QUALITY: %d/5 (%d%%)\n",
    echoPin, validCount, validCount * 20
  );

  if (validCount < MIN_VALID) {
    return -1.0f;
  }

  // Sort only the valid measurements.
  for (int i = 0; i < validCount - 1; i++) {
    for (int j = i + 1; j < validCount; j++) {
      if (validSamples[i] > validSamples[j]) {
        float temp = validSamples[i];
        validSamples[i] = validSamples[j];
        validSamples[j] = temp;
      }
    }
  }

  // Always return the median valid distance.
  return validSamples[(validCount - 1) / 2];
}

// Update the caller's front, left and right distances.
// Separately report whether a front sample was too close.
void readAllDistances(float* front, float* left, float* right,
                      bool* frontCloseSample) {

  *front = -1.0f;
  *left = -1.0f;
  *right = -1.0f;

  if (frontCloseSample != nullptr) {
    *frontCloseSample = false;
  }

  float nearestFrontSampleCm = -1.0f;


  *front = readFilteredDistanceCm(
      senseFront_PIN_TRIG,
      senseFront_PIN_ECHO,
      &nearestFrontSampleCm
  );

  if (frontCloseSample != nullptr) {
    *frontCloseSample =
        nearestFrontSampleCm >= 0.0f &&
        isFrontBlocked(nearestFrontSampleCm);
  }

  if (!enforceMotorInterlock()) return;
  delay(SENSOR_GAP_MS);

  *left = readFilteredDistanceCm(
      senseLeft_PIN_TRIG,
      senseLeft_PIN_ECHO
  );

  
  if (!enforceMotorInterlock()) return;
  delay(SENSOR_GAP_MS);

  *right = readFilteredDistanceCm(
      senseRight_PIN_TRIG,
      senseRight_PIN_ECHO
  );
}

/* ============================================================
   6. NON-BLOCKING SERIAL MOTION AND INTERACTIVE CALIBRATION
   ============================================================ */
enum MotionKind : uint8_t { DRIVE, TURN_LEFT, TURN_RIGHT, BENCH };
enum MotionPurpose : uint8_t { NONE, GRID, MANUAL, CALIBRATE, BENCH_TEST };

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
bool motionRunning = false;
bool waitingForMeasurement = false;
MotionPurpose motionPurpose = NONE;
char calibrationKind = 0;
unsigned long calibrationTestMs = 0;
int nextRow = 0, nextCol = 0;
uint8_t autonomousMode = 0; // 0=off, 1=one tile, 2=continuous
uint8_t autonomousMoves = 0;

void cancelMotion(const char* message) {
  stopMotors();
  remoteMotionActive = false;

  // Check commanded PWM duty after shutdown.
  if (pwmModeEnabled &&
      (ledcRead(leftMotor_PWM) != 0 ||
       ledcRead(rightMotor_PWM) != 0)) {
    motorFaultLatched = true;
    Serial.println("STOP PWM READBACK FAILED: RESET REQUIRED");
  }

  if (motionRunning ||
      (motionPurpose == GRID && stepIndex > 0)) {
    poseKnown = false;

    Serial.println(
      "POSE UNKNOWN: movement interrupted; physical alignment required"
    );
  }

  motionRunning = false;
  waitingForMeasurement = false;
  motionPurpose = NONE;
  stepCount = 0;
  stepIndex = 0;
  calibrationKind = 0;
  autonomousMode = 0;

  Serial.println(message);
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

  // Motors remain off during the filtered pre-movement clearance scan.
  stopMotors();
  if (s.forwardCm > 0.0f) {
    float nearest = -1.0f;
    float median = readFilteredDistanceCm(
        senseFront_PIN_TRIG, senseFront_PIN_ECHO, &nearest);
    const float needed = (motionPurpose == CALIBRATE)
        ? CALIBRATION_CLEARANCE_CM
        : s.forwardCm + FRONT_OBSTACLE_CM + 5.0f;
        if (!enforceMotorInterlock()) {
      cancelMotion("INTERLOCK DURING CLEARANCE CHECK");
      return false;
    }

    // Insufficient valid echoes cannot classify a tile.
    if (median < 0.0f || nearest < 0.0f) {
      cancelMotion("FRONT ECHO INVALID: motion cancelled");
      return false;
    }

    // A close return makes the target unsafe to enter.
    if (nearest <= needed) {
      if (motionPurpose == GRID && inMap(nextRow, nextCol)) {
        tileMap[nextRow][nextCol] = 1;
      }

      if (motionPurpose == GRID && autonomousMode != 0) {
        stopMotors();
        motionRunning = false;
        motionPurpose = NONE;
        stepCount = 0;
        stepIndex = 0;

        Serial.printf(
          "AUTO: tile (%d,%d) BLOCKED; selecting another\n",
          nextCol, nextRow
        );
        return false;
      }

      cancelMotion("FRONT BLOCKED: motion cancelled");
      return false;
    }

    // Sensor clearance confirmed, travel not yet confirmed.
    if (motionPurpose == GRID && inMap(nextRow, nextCol)) {
      tileMap[nextRow][nextCol] = 0;
    }
  }

  motionRunning = true;  // A partial motor start must invalidate pose on abort.
  if (s.leftDir) setLeftMotor(s.leftDir);
  if (!enforceMotorInterlock()) {
    cancelMotion("INTERLOCK DURING MOTOR START");
    return false;
  }
  if (s.rightDir) setRightMotor(s.rightDir);
  if (!enforceMotorInterlock()) {
    cancelMotion("INTERLOCK DURING MOTOR START");
    return false;
  }

  const uint32_t expectedLeft = s.leftDir ? MOTOR_TEST_DUTY : 0;
  const uint32_t expectedRight = s.rightDir ? MOTOR_TEST_DUTY : 0;
  if (ledcRead(leftMotor_PWM) != expectedLeft ||
      ledcRead(rightMotor_PWM) != expectedRight) {
    latchMotorFault();
    cancelMotion("PWM READBACK MISMATCH: reset required");
    return false;
  }

  stepStartMs = millis();
  lastFrontMs = stepStartMs;
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
      if (!enforceMotorInterlock() || cm < 0.0f ||
          cm <= FRONT_OBSTACLE_CM) {
        cancelMotion("FRONT OBSTACLE/INVALID ECHO: STOP; pose unknown");
      }
    }
    return;
  }

  stopMotors();
  if (ledcRead(leftMotor_PWM) != 0 || ledcRead(rightMotor_PWM) != 0) {
    latchMotorFault();
    cancelMotion("PWM REMAINED ON AFTER STOP: reset required");
    return;
  }
  Serial.println("STEP COMPLETE: BOTH PWM OUTPUTS ZERO");

  // Heading is only an estimate, not encoder/IMU-verified.
  if (poseKnown) {
    if (s.kind == TURN_LEFT) robotHeading = (robotHeading + 3) % 4;
    if (s.kind == TURN_RIGHT) robotHeading = (robotHeading + 1) % 4;
  }

  motionRunning = false;
  ++stepIndex;
  if (stepIndex < stepCount) {
    beginStep();
    return;
  }

   if (motionPurpose == GRID && poseKnown) {
    robotRow = nextRow;
    robotCol = nextCol;

    tileMap[robotRow][robotCol] = 0;

    if (tileVisits[robotRow][robotCol] < 255) {
      ++tileVisits[robotRow][robotCol];
    }

    if (autonomousMode != 0) {
      ++autonomousMoves;

      if (autonomousMode == 1 ||
          autonomousMoves >= AUTO_MAX_TILES) {
        autonomousMode = 0;

        Serial.println(
          "AUTO: tile complete; stopped (one-step/limit)"
        );
      }
    }

    Serial.printf(
      "ESTIMATED GRID POSE: X=%d,Y=%d, heading=%d\n",
      robotCol, robotRow, robotHeading
    );
  } else if (motionPurpose == CALIBRATE) {
    waitingForMeasurement = true;
    poseKnown = false;
    Serial.printf("CALIBRATION %c: enter measured %s, then ENTER (X cancels):\n",
                  calibrationKind,
                  (calibrationKind == '1' || calibrationKind == '2')
                      ? "distance in cm" : "angle in degrees");
  } else if (motionPurpose == MANUAL || motionPurpose == BENCH_TEST) {
    if (s.kind == DRIVE || s.kind == BENCH) poseKnown = false;
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
                       (calibrationKind == '2') ? 20.0f : 90.0f;
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
    case '2': forward20Ms = nextMs; calibrated20 = verified; break;
    case 'Q': turnLeft90Ms = nextMs; calibratedLeft = verified; break;
    case 'E': turnRight90Ms = nextMs; calibratedRight = verified; break;
  }
  Serial.printf("MEASURED %.1f, TARGET %.1f => NEXT TEST %lu ms; %s\n",
                actual, target, nextMs,
                verified ? "VERIFIED (within test tolerance)" : "REPEAT CALIBRATION");
  waitingForMeasurement = false;
  motionPurpose = NONE;
  calibrationKind = 0;
}

// Prefer unknown tiles, then least-visited clear tiles.
// Direction tie-break: front, right, left, back.
void startAutonomousTile() {
  if (autonomousMode == 0 || motionRunning ||
      waitingForMeasurement || motionPurpose != NONE) return;

  if (!poseKnown || !calibrated20 || !calibratedLeft ||
      !calibratedRight || !movementPermitted()) {
    autonomousMode = 0;
    Serial.println(
      "AUTO LOCKED: calibrate C2/CQ/CE, physically align, send H"
    );
    return;
  }

  const int dRow[4] = {-1, 0, 1, 0};
  const int dCol[4] = {0, 1, 0, -1};
  const int relative[4] = {0, 1, 3, 2};

  int bestDir = -1;
  int bestScore = 99999;

  for (int i = 0; i < 4; ++i) {
    const int direction = (robotHeading + relative[i]) % 4;
    const int row = robotRow + dRow[direction];
    const int col = robotCol + dCol[direction];

    if (!inMap(row, col) || tileMap[row][col] == 1) {
      continue;
    }

    const int score =
        (tileMap[row][col] == -1 ? 0 : 1000) +
        10 * tileVisits[row][col] + i;

    if (score < bestScore) {
      bestScore = score;
      bestDir = direction;
    }
  }

  if (bestDir < 0) {
    autonomousMode = 0;
    Serial.println("AUTO STOP: no adjacent unblocked tile");
    return;
  }

  nextRow = robotRow + dRow[bestDir];
  nextCol = robotCol + dCol[bestDir];

  const int delta = (bestDir - robotHeading + 4) % 4;

  stepCount = 0;

  if (delta == 1 || delta == 2) {
    addStep(TURN_RIGHT, 1, -1, turnRight90Ms);
  }

  if (delta == 2) {
    addStep(TURN_RIGHT, 1, -1, turnRight90Ms);
  }

  if (delta == 3) {
    addStep(TURN_LEFT, -1, 1, turnLeft90Ms);
  }

  addStep(DRIVE, 1, 1, forward20Ms, 20.0f);

  Serial.printf(
    "AUTO: target (%d,%d), state=%d, visits=%u\n",
    nextCol,
    nextRow,
    tileMap[nextRow][nextCol],
    (unsigned)tileVisits[nextRow][nextCol]
  );

  beginPlan(GRID);
}

void processCommand(const char* command) {
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
      calibrationTestMs = forward20Ms;
      addStep(DRIVE, 1, 1, calibrationTestMs, 20.0f);
    } else if (calibrationKind == 'Q') {
      calibrationTestMs = turnLeft90Ms;
      addStep(TURN_LEFT, -1, 1, calibrationTestMs);
    } else {
      calibrationTestMs = turnRight90Ms;
      addStep(TURN_RIGHT, 1, -1, calibrationTestMs);
    }
    poseKnown = false;
    beginPlan(CALIBRATE);
    return;
  }

  if (command[1] != '\0') {
    Serial.println("UNKNOWN COMMAND. Use ?, U/D/L/R, Q/E, 1/2, C1/C2/CQ/CE, H, X");
    return;
  }
  const char c = command[0];
  if (c == '?') {
    Serial.printf("CAL: 10=%lu(%d) 20=%lu(%d) LEFT90=%lu(%d) RIGHT90=%lu(%d) ms\n",
                  forward10Ms, calibrated10, forward20Ms, calibrated20,
                  turnLeft90Ms, calibratedLeft, turnRight90Ms, calibratedRight);
        Serial.printf(
      "POSE: %s X=%d,Y=%d heading=%d | auto=%u\n",
      poseKnown ? "ESTIMATED" : "UNKNOWN",
      robotCol,
      robotRow,
      robotHeading,
      (unsigned)autonomousMode
    );
    return;
  }
  if (c == 'M') {
    printMap();
    return;
  }
  if (c == 'H') {
    initialiseMap();
    robotRow = MAP_SIZE - 1;
    robotCol = 0;
    robotHeading = 0;
    poseKnown = true;
    Serial.println(
      "HOME SET: X=0,Y=8 facing U. Fresh map; align physically first."
    );

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

    if (c == 'N' || c == 'Z') {
    if (!poseKnown || !calibrated20 ||
        !calibratedLeft || !calibratedRight) {
      Serial.println(
        "AUTO LOCKED: verify C2/CQ/CE, align robot, send H"
      );
      return;
    }

    autonomousMoves = 0;
    autonomousMode = (c == 'N') ? 1 : 2;

    startAutonomousTile();
    return;
  }
     
  stepCount = 0;
  if (c == 'U' || c == 'D' || c == 'L' || c == 'R') {
    if (!poseKnown || !calibrated20) {
      Serial.println("GRID LOCKED: verify C2, then physically align and send H");
      return;
    }
    const int wanted = (c == 'U') ? 0 : (c == 'R') ? 1 :
                       (c == 'D') ? 2 : 3;
    const int dRow[4] = {-1, 0, 1, 0};
    const int dCol[4] = {0, 1, 0, -1};
    nextRow = robotRow + dRow[wanted];
    nextCol = robotCol + dCol[wanted];
    if (!inMap(nextRow, nextCol) || tileMap[nextRow][nextCol] == 1) {
      Serial.println("GRID BLOCKED: outside map or tile marked blocked");
      return;
    }
    const int delta = (wanted - robotHeading + 4) % 4;
    if (((delta == 1 || delta == 2) && !calibratedRight) ||
        (delta == 3 && !calibratedLeft)) {
      Serial.println("GRID LOCKED: required 90-degree turn is unverified");
      return;
    }
    if (delta == 1 || delta == 2)
      addStep(TURN_RIGHT, 1, -1, turnRight90Ms);
    if (delta == 2)
      addStep(TURN_RIGHT, 1, -1, turnRight90Ms);
    if (delta == 3)
      addStep(TURN_LEFT, -1, 1, turnLeft90Ms);
    addStep(DRIVE, 1, 1, forward20Ms, 20.0f);
    beginPlan(GRID);
    return;
  }
  if (c == '1' || c == '2') {
    addStep(DRIVE, 1, 1, c == '1' ? forward10Ms : forward20Ms,
            c == '1' ? 10.0f : 20.0f);
    beginPlan(MANUAL);
    return;
  }
  if (c == 'Q' || c == 'E') {
    if ((c == 'Q' && !calibratedLeft) ||
      (c == 'E' && !calibratedRight)) {
     poseKnown = false;
     Serial.println("POSE UNKNOWN: turn not calibrated");
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
      estopLatched ? "PHYSICAL E-STOP" :
      remoteStopLatched ? "REMOTE STOP LATCHED" :
      motorFaultLatched ? "MOTOR FAULT" :
      motionRunning ? "MOVING" :
      autonomousMode ? "AUTO READY" : "IDLE";

  Blynk.virtualWrite(V0, frontCm);
  Blynk.virtualWrite(V1, status);
}


void serviceBlynk() {
  // Cancel remotely initiated motion if the connection disappears.
  if (remoteMotionActive &&
      (WiFi.status() != WL_CONNECTED ||
       !Blynk.connected())) {
    cancelMotion(
      "BLYNK DISCONNECTED: remote movement cancelled"
    );
  }

  if (WiFi.status() != WL_CONNECTED) return;

  static unsigned long lastConnectAttempt = 0;

  // Do not attempt a potentially blocking cloud login
  // while the motors are executing a movement step.
  if (!Blynk.connected() && !motionRunning &&
      (lastConnectAttempt == 0 ||
       millis() - lastConnectAttempt >= BLYNK_RECONNECT_MS)) {
    lastConnectAttempt = millis();
    Blynk.connect(250);
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
      motionRunning ||
      waitingForMeasurement ||
      autonomousMode != 0 ||
      motionPurpose != NONE) {
    Serial.println(
      "BLYNK MOVE BLOCKED: disabled, busy or interlocked"
    );
    return;
  }

  // Refuse uncalibrated forward or turning commands.
  if ((command[0] == '2' && !calibrated20) ||
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


// Synchronise STOP only.
// Never replay movement buttons when reconnecting.
BLYNK_CONNECTED() {
  Serial.println("BLYNK ONLINE");
  Blynk.syncVirtual(V6);
}


// Forward: one calibrated 20 cm movement.
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

  if (!enforceMotorInterlock()) {
    if (motionRunning || waitingForMeasurement)
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
    char c = (char)Serial.read();
           if (c == 'X' || c == 'x') {
      length = 0;

      cancelMotion(
        "MANUAL X: BOTH MOTORS OFF; operation cancelled"
      );

      // Discard any commands already waiting in Serial.
      while (Serial.available() > 0) {
        Serial.read();
      }

      return;
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
      Serial.println("SERIAL INPUT TOO LONG: discarded");
    }
  }
  advanceMotion();
   // Decide on another tile only when the controller is idle.
  if (autonomousMode != 0 && !motionRunning &&
      !waitingForMeasurement && motionPurpose == NONE) {
    startAutonomousTile();
  }
}

