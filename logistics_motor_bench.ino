// LOGISTICS BOT - ESP32 / L298N / 3x HC-SR04
// PART 1: STANDALONE AUTONOMOUS ROBOT
// CURRENT STAGE: MOTOR DRIVER BENCH TESTING
// SERIAL MONITOR: 115200 BAUD
// Motors run only after an explicit Serial command.
// Emergency stop uses a software latch;
// it is not an independently verified hardware stop.
// Grid map is initialised but not updated by movement.
/* ============================================================
   1. CONFIGURATION - PIN ASSIGNMENTS AND SENSOR LIMITS
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
// Provisional obstacle-detection threshold, not a verified stopping distance.
// Reassess after measuring motor stopping performance.

// Minimum 1000 ms between scan starts.
// Not a guaranteed one-second reporting rate.
// Blocking sensor measurements can exceed this interval.
const unsigned long SENSOR_REPORT_MS = 1000UL;

/* ============================================================
   2. MOTOR CONTROL AND EMERGENCY STOP
   ============================================================ */

volatile bool estopLatched = false;

bool leftPwmAttached = false;
bool rightPwmAttached = false;

const uint8_t MOTOR_PWM_DUTY = 180;
const unsigned long MOTOR_TEST_MS = 500UL;

// Disable LEFT bridge before changing direction.
void disableLeftMotor() {
  if (leftPwmAttached) {
    ledcWrite(leftMotor_PWM, 0);
  } else {
    digitalWrite(leftMotor_PWM, LOW);
  }

  digitalWrite(leftMotor_IN1, LOW);
  digitalWrite(leftMotor_IN2, LOW);
}

// Disable RIGHT bridge before changing direction.
void disableRightMotor() {
  if (rightPwmAttached) {
    ledcWrite(rightMotor_PWM, 0);
  } else {
    digitalWrite(rightMotor_PWM, LOW);
  }

  digitalWrite(rightMotor_IN3, LOW);
  digitalWrite(rightMotor_IN4, LOW);
}

void stopMotors() {
  disableLeftMotor();
  disableRightMotor();
}

// An activated E-stop stays latched until reboot.
bool emergencyStopActive() {
  if (estopLatched ||
      digitalRead(ESTOP_PIN) == LOW) {
    estopLatched = true;
    return true;
  }

  return false;
}

void ARDUINO_ISR_ATTR onEmergencyStop() {
  estopLatched = true;
}

// Direction: 1 = forward, -1 = reverse, 0 = stop.
// Invalid directions also stop this motor.
void setLeftMotor(int direction) {
  disableLeftMotor();

  if (direction != 1 && direction != -1) {
    return;
  }

  if (emergencyStopActive()) {
    return;
  }

  digitalWrite(
    leftMotor_IN1,
    direction == 1 ? HIGH : LOW
  );

  digitalWrite(
    leftMotor_IN2,
    direction == -1 ? HIGH : LOW
  );

  // Recheck immediately before enabling.
  if (emergencyStopActive()) {
    disableLeftMotor();
    return;
  }

  if (leftPwmAttached) {
    if (!ledcWrite(leftMotor_PWM, MOTOR_PWM_DUTY)) {
      estopLatched = true;
      disableLeftMotor();
    }
  } else {
    digitalWrite(leftMotor_PWM, HIGH);
  }
}

void setRightMotor(int direction) {
  disableRightMotor();

  if (direction != 1 && direction != -1) {
    return;
  }

  if (emergencyStopActive()) {
    return;
  }

  digitalWrite(
    rightMotor_IN3,
    direction == 1 ? HIGH : LOW
  );

  digitalWrite(
    rightMotor_IN4,
    direction == -1 ? HIGH : LOW
  );

  // Recheck immediately before enabling.
  if (emergencyStopActive()) {
    disableRightMotor();
    return;
  }

  if (rightPwmAttached) {
    if (!ledcWrite(rightMotor_PWM, MOTOR_PWM_DUTY)) {
      estopLatched = true;
      disableRightMotor();
    }
  } else {
    digitalWrite(rightMotor_PWM, HIGH);
  }
}


volatile bool estopLatched = false;

void ARDUINO_ISR_ATTR onEmergencyStop() {
  estopLatched = true;
}


/* ============================================================
   3. GRID MAP - INITIALISATION AND SERIAL DISPLAY
   ============================================================ */
const int MAP_SIZE = 9;
int8_t tileMap[MAP_SIZE][MAP_SIZE];

// Navigation configuration
const int TILE_LENGTH_CM = 20;

// Robot's provisional grid position
int robotRow = MAP_SIZE - 1;
int robotCol = 0;

// Map evidence
// Clearance 0 means no measurement recorded.
float tileClearanceCm[MAP_SIZE][MAP_SIZE] = {};
uint8_t tileObstacleSamples[MAP_SIZE][MAP_SIZE] = {};

// Preliminary motor timing calibration
// Zero means not calibrated.
unsigned long turnRight90Ms = 0;
unsigned long turnLeft90Ms = 0;
unsigned long forward20Ms = 0;
unsigned long reverse10Ms = 0;

void initialiseMap() {
  for (int row = 0; row < MAP_SIZE; row++) {
    for (int col = 0; col < MAP_SIZE; col++) {
      tileMap[row][col] = -1;
    }
  }

  // Starting / sweep tile: bottom-left
  tileMap[MAP_SIZE - 1][0] = 0;
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
   3.1 TILE SELECTION AND MOVEMENT PLANNING
   ============================================================ */

bool inMap(int row, int col) {
  return row >= 0 && row < MAP_SIZE &&
         col >= 0 && col < MAP_SIZE;
}

// Select an adjacent candidate tile.
// Priority:
// 1. Unexplored tile
// 2. Greatest recorded obstacle clearance
// 3. Fewest obstacle observations
// tileMap: -1 unknown, 0 visited, 1 blocked.

bool chooseTile(int& targetRow, int& targetCol) {
  const int dRow[4] = {-1, 0, 1, 0};
  const int dCol[4] = {0, 1, 0, -1};

  bool found = false;
  int bestUnknown = -1;
  float bestClearance = -1.0f;
  int bestDataCount = 256;

  for (int d = 0; d < 4; d++) {
    int row = robotRow + dRow[d];
    int col = robotCol + dCol[d];

    if (!inMap(row, col) ||
        tileMap[row][col] == 1) {
      continue;
    }

    int unknown =
        (tileMap[row][col] == -1) ? 1 : 0;

    float clearance = tileClearanceCm[row][col];
    int dataCount = tileObstacleSamples[row][col];

    if (!found ||
        unknown > bestUnknown ||
        (unknown == bestUnknown &&
         clearance > bestClearance) ||
        (unknown == bestUnknown &&
         clearance == bestClearance &&
         dataCount < bestDataCount)) {

      found = true;
      bestUnknown = unknown;
      bestClearance = clearance;
      bestDataCount = dataCount;

      targetRow = row;
      targetCol = col;
    }
  }

  return found;
}

// Plan a single 20 cm tile movement.
// Motor actuation deliberately disabled.

void moveTile() {
  int targetRow;
  int targetCol;

  if (estopLatched ||
      digitalRead(ESTOP_PIN) == LOW) {
    estopLatched = true;
    stopMotors();
    Serial.println("MOVE TILE BLOCKED: E-STOP");
    return;
  }

  if (!chooseTile(targetRow, targetCol)) {
    Serial.println(
        "MOVE TILE: NO ADJACENT CANDIDATE"
    );
    return;
  }

  Serial.printf(
      "PLAN ONLY: 20 cm tile (%d,%d) -> (%d,%d)\n",
      robotRow, robotCol, targetRow, targetCol
  );

  Serial.println(
      "No motor command. Path safety unverified."
  );

  // Do not update the robot's coordinates
  // until actual movement is confirmed.
}

/* ============================================================
   3.2 MOVEMENT CALIBRATION
   ============================================================ */

// Calculate preliminary durations for 90-degree turns.
// Supply actual measurements from separate right and
// left turn tests. Does not operate the motors.

void calibrateAngle(
    float measuredRightDegrees,
    unsigned long rightTestMs,
    float measuredLeftDegrees,
    unsigned long leftTestMs) {

  if (measuredRightDegrees <= 0 ||
      measuredLeftDegrees <= 0 ||
      rightTestMs == 0 ||
      leftTestMs == 0) {

    Serial.println(
        "ANGLE CALIBRATION: INVALID MEASUREMENTS"
    );
    return;
  }

  turnRight90Ms = (unsigned long)(
      90.0f * rightTestMs /
      measuredRightDegrees + 0.5f
  );

  turnLeft90Ms = (unsigned long)(
      90.0f * leftTestMs /
      measuredLeftDegrees + 0.5f
  );

  Serial.printf(
      "INITIAL TURN ESTIMATES: RIGHT=%lu ms LEFT=%lu ms\n",
      turnRight90Ms, turnLeft90Ms
  );

  Serial.println(
      "Verify both turns before enabling movement."
  );
}

// Calculate preliminary 20 cm forward and
// 10 cm reverse travel durations.
// Supply physically measured test results.

void calibrateDistance(
    float measuredForwardCm,
    unsigned long forwardTestMs,
    float measuredReverseCm,
    unsigned long reverseTestMs) {

  if (measuredForwardCm <= 0 ||
      measuredReverseCm <= 0 ||
      forwardTestMs == 0 ||
      reverseTestMs == 0) {

    Serial.println(
        "DISTANCE CALIBRATION: INVALID MEASUREMENTS"
    );
    return;
  }

  forward20Ms = (unsigned long)(
      20.0f * forwardTestMs /
      measuredForwardCm + 0.5f
  );

  reverse10Ms = (unsigned long)(
      10.0f * reverseTestMs /
      measuredReverseCm + 0.5f
  );

  Serial.printf(
      "INITIAL DISTANCE ESTIMATES: FWD20=%lu ms REV10=%lu ms\n",
      forward20Ms, reverse10Ms
  );

  Serial.println(
      "Verify both distances before enabling movement."
  );
}

/* ============================================================
   3.3 FULL ROTATION SWEEP
   ============================================================ */

void sweep360() {
  stopMotors();

  Serial.println("SWEEP 360: PLANNING ONLY");

  Serial.println(
      "Step 1: Reverse 10 cm after checking rear clearance"
  );

  Serial.println(
      "Step 2: Scan at starting heading"
  );

  Serial.println(
      "Step 3: Rotate RIGHT 90 degrees and scan"
  );

  Serial.println(
      "Step 4: Rotate RIGHT 90 degrees and scan"
  );

  Serial.println(
      "Step 5: Rotate RIGHT 90 degrees and scan"
  );

  Serial.println(
      "Step 6: Rotate RIGHT 90 degrees to original heading"
  );

  if (reverse10Ms == 0 ||
      turnRight90Ms == 0) {

    Serial.println(
        "SWEEP NOT READY: Missing calibration"
    );
    return;
  }

  Serial.println(
      "Calibration estimates available."
  );

  Serial.println(
      "Motor execution and map updates not implemented."
  );
}

/* ============================================================
   3.4 SNAKE COVERAGE PATTERN
   ============================================================ */

// Generate row-by-row coverage order.
// Begin at bottom-left.
// Alternate travel direction on each row.
// Planning only; obstacles are not yet routed around.

void snake() {
  Serial.println(
      "SNAKE COVERAGE ORDER:"
  );

  for (int row = MAP_SIZE - 1; row >= 0; row--) {

    bool leftToRight =
        ((MAP_SIZE - 1 - row) % 2 == 0);

    for (int i = 0; i < MAP_SIZE; i++) {

      int col = leftToRight ?
          i : MAP_SIZE - 1 - i;

      Serial.printf(
          "(%d,%d) ", row, col
      );
    }

    Serial.println();
  }
}

/* ============================================================
   3.5 CENTRE SPIRAL COVERAGE
   ============================================================ */

// Generate outward spiral tile order.
// Assumes the robot begins at the centre.
// Planning only; does not execute movement.

void centreSpiral() {
  Serial.println(
      "CENTRE SPIRAL COVERAGE ORDER:"
  );

  int row = MAP_SIZE / 2;
  int col = MAP_SIZE / 2;

  Serial.printf("(%d,%d) ", row, col);

  const int dRow[4] = {0, 1, 0, -1};
  const int dCol[4] = {1, 0, -1, 0};

  int stepLength = 1;
  int printed = 1;

  for (int leg = 0;
       printed < MAP_SIZE * MAP_SIZE;
       leg++) {

    int direction = leg % 4;

    for (int step = 0;
         step < stepLength;
         step++) {

      row += dRow[direction];
      col += dCol[direction];

      if (inMap(row, col)) {
        Serial.printf(
            "(%d,%d) ", row, col
        );

        printed++;

        if (printed == MAP_SIZE * MAP_SIZE) {
          break;
        }
      }
    }

    if (leg % 2 == 1) {
      stepLength++;
    }
  }

  Serial.println();
}

/* ============================================================
   4. SETUP - INITIALISE HARDWARE AND SAFETY INPUTS
   ============================================================ */
void setup() {
  Serial.begin(115200);

  pinMode(senseRight_PIN_TRIG, OUTPUT);  
  pinMode(senseLeft_PIN_TRIG, OUTPUT);      
  pinMode(senseFront_PIN_TRIG, OUTPUT);     
  pinMode(senseRight_PIN_ECHO, INPUT);      
  pinMode(senseLeft_PIN_ECHO, INPUT);    
  pinMode(senseFront_PIN_ECHO, INPUT);        

  pinMode(leftMotor_IN1, OUTPUT);
  pinMode(leftMotor_IN2, OUTPUT);
  pinMode(rightMotor_IN3, OUTPUT);
  pinMode(rightMotor_IN4, OUTPUT);
  pinMode(leftMotor_PWM, OUTPUT);
  pinMode(rightMotor_PWM, OUTPUT);
  pinMode(ESTOP_PIN, INPUT_PULLUP);
  
  
  stopMotors();

  // Capture an E-stop already held at startup.
  if (digitalRead(ESTOP_PIN) == LOW) {
    estopLatched = true;
  }

  attachInterrupt(
    digitalPinToInterrupt(ESTOP_PIN),
    onEmergencyStop,
    FALLING
  );

   
  // Independent LEDC PWM setup.
  // Arduino-ESP32 3.x API.
  const uint32_t PWM_FREQUENCY_HZ = 1000;
  const uint8_t PWM_RESOLUTION_BITS = 8;

  leftPwmAttached = ledcAttach(
    leftMotor_PWM,
    PWM_FREQUENCY_HZ,
    PWM_RESOLUTION_BITS
  );

  rightPwmAttached = ledcAttach(
    rightMotor_PWM,
    PWM_FREQUENCY_HZ,
    PWM_RESOLUTION_BITS
  );

  // Start at zero duty on both channels.
  bool leftZeroOk =
      leftPwmAttached &&
      ledcWrite(leftMotor_PWM, 0);

  bool rightZeroOk =
      rightPwmAttached &&
      ledcWrite(rightMotor_PWM, 0);

  Serial.printf(
    "LEFT PWM: %s\n",
    leftZeroOk ? "OK" : "FAILED"
  );

  Serial.printf(
    "RIGHT PWM: %s\n",
    rightZeroOk ? "OK" : "FAILED"
  );

  // Lock out motor tests if either output fails.
  if (!leftZeroOk || !rightZeroOk) {
    estopLatched = true;
    stopMotors();

    Serial.println(
      "PWM INITIALISATION FAILED - TEST LOCKED OUT"
    );
  }

   
  initialiseMap();
  printMap();

  Serial.println("LOGISTICS BOT - MOTOR BENCH TEST");
  Serial.println("Software E-stop latch enabled.");
  Serial.println("Wheels must be raised.");

  Serial.println("COMMANDS:");
  Serial.println("1 = LEFT forward");
  Serial.println("2 = LEFT reverse");
  Serial.println("3 = RIGHT forward");
  Serial.println("4 = RIGHT reverse");
  Serial.println("0 = STOP");
}


  // Navigation dry-run test
  moveTile();

  // Enable one at a time for route printing:
  // snake();
  // centreSpiral();
  // sweep360();
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
  return distanceCm >= 0.0f && distanceCm <= FRONT_OBSTACLE_CM;
}

// Five samples per sensor; at least three must be valid.
// Report the median of valid samples.
// With four valid samples, use the lower-middle value.
// Conservative safety override: a nearest valid sample
// <= 20 cm triggers a separate close-obstacle warning.
// The warning does not alter the reported median.

float readFilteredDistanceCm(int trigPin, int echoPin,
                             bool* closeSampleDetected = nullptr) {
  if (closeSampleDetected != nullptr) {
    *closeSampleDetected = false;
  }
  const int SAMPLE_COUNT = 5;
  const int MIN_VALID = 3;

  float validSamples[SAMPLE_COUNT];
  int validCount = 0;

  for (int i = 0; i < SAMPLE_COUNT; i++) {
    if (estopLatched || digitalRead(ESTOP_PIN) == LOW) {
      estopLatched = true;
      return -1.0f;
    }

    float cm = readDistanceCm(trigPin, echoPin);

    if (cm >= 0.0f) {
      validSamples[validCount] = cm;
      validCount++;
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

  // Conservative close-obstacle warning.
  // The nearest valid sample determines the warning,
  // without changing the reported median.
  if (closeSampleDetected != nullptr) {
    *closeSampleDetected =
        (validSamples[0] <= FRONT_OBSTACLE_CM);
  }

  // Always return the median valid distance.
  return validSamples[(validCount - 1) / 2];
}

// Update the caller's front, left and right distances.
// Separately report whether a front sample was too close.
void readAllDistances(float* front, float* left, float* right,
                      bool* frontCloseSample) {

  *front = readFilteredDistanceCm(
      senseFront_PIN_TRIG,
      senseFront_PIN_ECHO,
      frontCloseSample
  );

  if (estopLatched) return;
  delay(SENSOR_GAP_MS);

  *left = readFilteredDistanceCm(
      senseLeft_PIN_TRIG,
      senseLeft_PIN_ECHO
  );

  if (estopLatched) return;
  delay(SENSOR_GAP_MS);

  *right = readFilteredDistanceCm(
      senseRight_PIN_TRIG,
      senseRight_PIN_ECHO
  );
}


/* ============================================================
   6. MAIN LOOP - GUARDED MOTOR BENCH TEST
   ============================================================ */

void loop() {
  static bool testRunning = false;
  static unsigned long testStartedMs = 0;
  static bool estopReported = false;

  // 6.1 Highest-priority software safety check.
  if (emergencyStopActive()) {
    stopMotors();
    testRunning = false;

    if (!estopReported) {
      Serial.println("E-STOP LATCHED - MOTORS DISABLED");
      estopReported = true;
    }

    while (Serial.available() > 0) {
      Serial.read();
    }

    return;
  }

  // 6.2 Non-blocking automatic timeout.
  if (testRunning &&
      millis() - testStartedMs >= MOTOR_TEST_MS) {
    stopMotors();
    testRunning = false;

    Serial.println("AUTO STOP - 500 ms elapsed");
  }

  // 6.3 Wait for an explicit Serial command.
  if (Serial.available() == 0) {
    return;
  }

  char command = (char)Serial.read();

  // Ignore Serial Monitor line endings.
  if (command == '\n' || command == '\r') {
    return;
  }

  // Manual stop is always accepted.
  if (command == '0') {
    stopMotors();
    testRunning = false;
    Serial.println("MANUAL STOP");
    return;
  }

  // Do not restart or extend a running test.
  if (testRunning) {
    Serial.println("BUSY - WAIT FOR AUTO STOP");
    return;
  }

  stopMotors();

  // 6.4 Select exactly one motor and direction.
  switch (command) {
    case '1':
      setLeftMotor(1);
      break;

    case '2':
      setLeftMotor(-1);
      break;

    case '3':
      setRightMotor(1);
      break;

    case '4':
      setRightMotor(-1);
      break;

    default:
      Serial.println("INVALID COMMAND: USE 0-4");
      return;
  }

  // Check again after the motor function.
  if (emergencyStopActive()) {
    stopMotors();
    testRunning = false;
    return;
  }

  testStartedMs = millis();
  testRunning = true;

  Serial.println("MOTOR TEST STARTED - MAX 500 ms");
}

