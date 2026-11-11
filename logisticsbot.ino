// 30127103 Logistics Bot, L298N Driver, 3x HC-SR04 sensors, ESP32
// 115200 baud in arduino software

const int senseLeft_PIN_TRIG = 22;
const int senseLeft_PIN_ECHO = 33;
const int senseFront_PIN_TRIG  = 23;
const int senseFront_PIN_ECHO  = 34;
const int senseRight_PIN_TRIG  = 21;
const int senseRight_PIN_ECHO  = 32;
const int ESTOP_PIN = 25;

const int leftMotor_IN  = 26;
const int rightMotor_IN  = 27;

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
}

loop()
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
  
