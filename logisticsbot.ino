// 30127103 Logistics Bot, L298N Driver, 3x HC-SR04 sensors, ESP32
// 115200 baud in arduino software

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


const int MAP_SIZE = 7;
int8_t tileMap[MAP_SIZE][MAP_SIZE];

void initialiseMap() {
  for (int row = 0; row < MAP_SIZE; row++) {
    for (int col = 0; col < MAP_SIZE; col++) {
      tileMap[row][col] = -1;
    }
  }

  tileMap[3][3] = 0;
}

void printMap() {
  for (int row = 0; row < MAP_SIZE; row++) {
    for (int col = 0; col < MAP_SIZE; col++) {
      Serial.printf("%3d", tileMap[row][col]);
    }
    Serial.println();
  }
}


void setup() {
  Serial.begin(115200);
  
  initialiseMap();
  printMap();

  pinMode(senseRight_PIN_TRIG, OUTPUT);         // senseRight_PIN_TRIG
  pinMode(senseLeft_PIN_TRIG, OUTPUT);         // senseLeft_PIN_TRIG
  pinMode(senseFront_PIN_TRIG, OUTPUT);         // senseFront_PIN_TRIG
  pinMode(senseRight_PIN_ECHO, INPUT);          // senseRight_PIN_ECHO
  pinMode(senseLeft_PIN_ECHO, INPUT);          // senseLeft_PIN_ECHO
  pinMode(senseFront_PIN_ECHO, INPUT);          // senseFront_PIN_ECHO

  pinMode(leftMotor_IN1, OUTPUT);
  pinMode(leftMotor_IN2, OUTPUT);
  pinMode(rightMotor_IN3, OUTPUT);
  pinMode(rightMotor_IN4, OUTPUT);
  pinMode(leftMotor_PWM, OUTPUT);
  pinMode(rightMotor_PWM, OUTPUT);
  pinMode(ESTOP_PIN, INPUT_PULLUP);
  if (digitalRead(ESTOP_PIN) == LOW) {
    estopLatched = true;
  }
  
  attachInterrupt(
      digitalPinToInterrupt(ESTOP_PIN),
      onEmergencyStop,
      FALLING
  );

  digitalWrite(leftMotor_PWM, LOW);
  digitalWrite(rightMotor_PWM, LOW);
  digitalWrite(leftMotor_IN1, LOW);
  digitalWrite(leftMotor_IN2, LOW);
  digitalWrite(rightMotor_IN3, LOW);
  digitalWrite(rightMotor_IN4, LOW);
}


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
    return -1.0f;
  }

  // Convert round-trip travel time to distance
  float distanceCm = duration * 0.0343f / 2.0f;

  if (distanceCm < SENSOR_MIN_CM ||
      distanceCm > SENSOR_MAX_CM) {
    return -1.0f;
  }

  return distanceCm;
}

void loop() {
  // Keep both motors disabled during testing
  digitalWrite(leftMotor_PWM, LOW);
  digitalWrite(rightMotor_PWM, LOW);

  // Check emergency-stop button
  
  if (estopLatched || digitalRead(ESTOP_PIN) == LOW) {
    estopLatched = true;
    stopMotors();
    Serial.println("E-STOP LATCHED");
    delay(100);
    return;
  }


  // Read sensors one at a time
  float frontDistance = readDistanceCm(
      senseFront_PIN_TRIG, senseFront_PIN_ECHO);

  delay(SENSOR_GAP_MS);

  float leftDistance = readDistanceCm(
      senseLeft_PIN_TRIG, senseLeft_PIN_ECHO);

  delay(SENSOR_GAP_MS);

  float rightDistance = readDistanceCm(
      senseRight_PIN_TRIG, senseRight_PIN_ECHO);

  // Display readings
  Serial.printf(
      "Front: %.1f cm | Left: %.1f cm | Right: %.1f cm\n",
      frontDistance,
      leftDistance,
      rightDistance
  );

  // -1 indicates an invalid measurement
  if (frontDistance < 0 ||
      leftDistance < 0 ||
      rightDistance < 0) {
    Serial.println("WARNING: Invalid sensor reading");
  }

  
  if (frontDistance < 0 ||
      leftDistance < 0 ||
      rightDistance < 0) {
    Serial.println("Decision: STOP - invalid reading");
  } else if (leftDistance > rightDistance) {
    Serial.println("Clearer side: LEFT");
  } else if (rightDistance > leftDistance) {
    Serial.println("Clearer side: RIGHT");
  } else {
    Serial.println("Clearer side: EQUAL");
  }
  
      
  delay(200);
}
