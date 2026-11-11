// 30127103 Logistics Bot, L298N Driver, 3x HC-SR09 sensors, ESP32

#include <WiFi.h>
#include <PubSubClient.h>
#include <DHT.h>
#include <Arduino.h>
#include <math.h>

const int senseLeft_PIN   = 4;
const int senseFront_PIN  = 34;
const int senseRight_PIN  = 34;
const int ESTOP_PIN = 25;

const int SERVO_IN1  = 26;
const int SERVO_IN2  = 27;

const char* WIFI_SSID     = "Abe's iPhone";
const char* WIFI_PASSWORD = "nintendo";
