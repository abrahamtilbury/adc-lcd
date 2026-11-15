// LOGISTICS BOT - ESP32 / L298N / 3x HC-SR04
// PART 1: STANDALONE AUTONOMOUS ROBOT
// CURRENT STAGE: STATIONARY SENSOR TESTING
// SERIAL MONITOR: 115200 BAUD
// Motors intentionally disabled during testing.
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

const unsigned long SENSOR_REPORT_MS = 1000UL;

/* ============================================================
   2. MOTOR CONTROL AND EMERGENCY STOP
   ============================================================ */
void stopMotors() {
  digitalWrite(leftMotor_PWM, LOW);
  digitalWrite(rightMotor_PWM, LOW);

  digitalWrite(leftMotor_IN1, LOW);
  digitalWrite(leftMotor_IN2, LOW);
  digitalWrite(rightMotor_IN3, LOW);
  digitalWrite(rightMotor_IN4, LOW);
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

void initialiseMap() {
  for (int row = 0; row < MAP_SIZE; row++) {
    for (int col = 0; col < MAP_SIZE; col++) {
      tileMap[row][col] = -1;
    }
  }

   tileMap[MAP_SIZE / 2][MAP_SIZE / 2] = 0;
   // Integer division locates the centre of an odd-sized square map.

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
  
  stopMotors();     // void function: sets motor outputs LOW, returns nothing.
   Serial.println("LOGISTICS BOT - STATIONARY SENSOR TEST");
   Serial.println("Motor outputs disabled; E-stop is software-latched.");
  initialiseMap();  // void function: changes the global tileMap array.
  printMap();       // void function: prints the map to Serial.

  if (digitalRead(ESTOP_PIN) == LOW) {
    estopLatched = true;
  }
  
  attachInterrupt(
      digitalPinToInterrupt(ESTOP_PIN),
      onEmergencyStop,
      FALLING
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


 // Only a valid front reading can confirm an obstruction.
bool isFrontBlocked(float distanceCm) {
  return distanceCm >= 0.0f && distanceCm <= FRONT_OBSTACLE_CM;
}

// Three-sample median; two valid samples use the nearer value.
float readFilteredDistanceCm(int trigPin, int echoPin) {
  float samples[3];
  int validCount = 0;

  for (int i = 0; i < 3; i++) {
    if (estopLatched || digitalRead(ESTOP_PIN) == LOW) {
      estopLatched = true;
      return -1.0f;
    }

    float cm = readDistanceCm(trigPin, echoPin);
    if (cm >= 0.0f) samples[validCount++] = cm;
    if (i < 2) delay(SENSOR_GAP_MS);
  }

  if (validCount < 2) return -1.0f;
  if (validCount == 2) {
    return samples[0] < samples[1] ? samples[0] : samples[1];
  }

  for (int i = 0; i < 2; i++) {
    for (int j = i + 1; j < 3; j++) {
      if (samples[i] > samples[j]) {
        float temp = samples[i];
        samples[i] = samples[j];
        samples[j] = temp;
      }
    }
  }
  return samples[1];
}

// Update the caller's front, left and right distances.
void readAllDistances(float* front, float* left, float* right) {
  *front = readFilteredDistanceCm(senseFront_PIN_TRIG, senseFront_PIN_ECHO);
  if (estopLatched) return;
  delay(SENSOR_GAP_MS);

  *left = readFilteredDistanceCm(senseLeft_PIN_TRIG, senseLeft_PIN_ECHO);
  if (estopLatched) return;
  delay(SENSOR_GAP_MS);

  *right = readFilteredDistanceCm(senseRight_PIN_TRIG, senseRight_PIN_ECHO);
}

/* ============================================================
   6. MAIN LOOP - STATIONARY SENSOR TESTING
   ============================================================ */
void loop() {
  stopMotors();  // Stationary testing only.

  // 6.1 Physical E-stop input and software latch
  if (estopLatched || digitalRead(ESTOP_PIN) == LOW) {
    estopLatched = true;
    static bool reported = false;
    if (!reported) {
      Serial.println("E-STOP LATCHED");
      reported = true;
    }
    return;
  }

  // 6.2 Schedule reports; ultrasonic reads are still blocking.
  static unsigned long lastReportMs = 0;
  if (millis() - lastReportMs < SENSOR_REPORT_MS) return;
  lastReportMs = millis();

  float frontDistance = -1.0f;
  float leftDistance = -1.0f;
  float rightDistance = -1.0f;

  readAllDistances(&frontDistance, &leftDistance, &rightDistance);

  if (estopLatched || digitalRead(ESTOP_PIN) == LOW) {
    estopLatched = true;
    return;
  }

  Serial.printf("Front: %.1f cm | Left: %.1f cm | Right: %.1f cm\n",
                frontDistance, leftDistance, rightDistance);

  // 6.3 Distinguish obstruction from unknown measurements.
  if (frontDistance < 0.0f) {
    Serial.println("Decision: STOP - FRONT UNKNOWN");
  } else if (isFrontBlocked(frontDistance)) {
    Serial.println("Decision: STOP - FRONT OBSTACLE");
  } else if (leftDistance < 0.0f || rightDistance < 0.0f) {
    Serial.println("Decision: HOLD - SIDE UNKNOWN");
  } else if (leftDistance > rightDistance) {
    Serial.println("Clearer side: LEFT");
  } else if (rightDistance > leftDistance) {
    Serial.println("Clearer side: RIGHT");
  } else {
    Serial.println("Clearer side: EQUAL");
  }
}
