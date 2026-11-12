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

void setup() {
  Serial.begin(115200);
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
  digitalWrite(leftMotor_PWM, LOW);
  digitalWrite(rightMotor_PWM, LOW);
  digitalWrite(leftMotor_IN1, LOW);
  digitalWrite(leftMotor_IN2, LOW);
  digitalWrite(rightMotor_IN3, LOW);
  digitalWrite(rightMotor_IN4, LOW);
}

loop()
  // TODO: Check emergency stop
  // TODO: Read front, left and right sensors
  // TODO: Rotate robot approximately 45 degrees
  // TODO: Read all three sensors again
  // TODO: Compare six readings and select movement
  read front sensor
    write list[1]
  read left sensor
    write list[2]
  read right sensor
    write list[3]
  rotate wheels 45 degrees (maybe write a function that does this?
  read front sensor
    write list[4]
  read left sensor
    write list[5]
  read right sensor
    write list[6]
  wait 10uS
  
