#include <Preferences.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>


#define TS_ENABLE_SSL
#define WiFiClientSecure_h
#include <ThingSpeak.h>



// ESP32 standalone automatic watering system
// GPIO25 -> L298N ENA (jumper removed)
// GPIO26 -> L298N IN1
// L298N IN2 -> GND
// GPIO34 -> Soil moisture divider output
// All grounds connected together
// BOOT button (GPIO0) resets reservoir counter

const int pumpPin = 26;
const int enablePin = 25;
const int moisturePin = 34;
const int refillButtonPin = 0;

const int pumpDuty = 204;  // Fixed 80% PWM

// CALIBRATION
const int dryThreshold = 2100;  // PROVISIONAL ONLY
const bool automaticWatering = true;  // Enable after calibration

// TIMING
const unsigned long moistureInterval = 2000;
const unsigned long pumpDuration = 20000;
const unsigned long cooldown = 1800000;
const unsigned long startupDelay = 60000;

// RESERVOIR
const unsigned int maxDoses = 20;
const float mlPerDose = 15.0;

Preferences storage;

// WIFI
const char* wifiSSID = "Abe's iPhone";
const char* wifiPassword = "nintendo";

// THINGSPEAK
const unsigned long channelID = 3576823;  // Replace with Channel ID
const char* writeAPIKey = "QHVPOPTHQMCUE4PW";

WiFiClientSecure thingSpeakClient;

const unsigned long uploadInterval = 30000;
const unsigned long wifiRetryInterval = 30000;

unsigned long lastUpload = 0;
unsigned long lastWifiAttempt = 0;

int latestSoilADC = -1;
int latestSoilVoltage = -1;


unsigned int dosesUsed = 0;
unsigned long pumpStart = 0;
unsigned long lastPumpStop = 0;
unsigned long lastMoistureRead = 0;
unsigned long buttonStart = 0;

bool pumpRunning = false;
bool pumpHasRun = false;
bool storageReady = false;
bool buttonHandled = false;

int dryCount = 0;

// STOP PUMP
void stopPump() {
  ledcWrite(enablePin, 0);
  digitalWrite(pumpPin, LOW);

  pumpRunning = false;
  pumpHasRun = true;
  lastPumpStop = millis();

  Serial.println("Pump OFF");
}

// START PUMP
void startPump() {
  if (!storageReady || dosesUsed >= maxDoses) return;

  // Save dose BEFORE starting, including in case power fails.
  if (storage.putUInt("doses", dosesUsed + 1) != 4) {
    storageReady = false;
    Serial.println("STORAGE ERROR - Pump disabled");
    return;
  }

  dosesUsed++;

  digitalWrite(pumpPin, HIGH);
  ledcWrite(enablePin, pumpDuty);

  pumpStart = millis();
  pumpRunning = true;
  dryCount = 0;

  Serial.println("Pump ON - 20 second dose");
  Serial.print("Doses used: ");
  Serial.println(dosesUsed);
}

// SETUP
void setup() {
  digitalWrite(pumpPin, LOW);
  pinMode(pumpPin, OUTPUT);
  pinMode(refillButtonPin, INPUT_PULLUP);

  Serial.begin(115200);

  // Start Wi-Fi without waiting for connection
  WiFi.mode(WIFI_STA);
  WiFi.setAutoReconnect(true);
  WiFi.begin(wifiSSID, wifiPassword);

  // HTTPS for initial testing
  thingSpeakClient.setInsecure();
  ThingSpeak.begin(thingSpeakClient);

  Serial.println("Wi-Fi connection started");


  if (!ledcAttach(enablePin, 1000, 8)) {
    Serial.println("PWM ERROR");
    while (true) delay(1000);
  }

  ledcWrite(enablePin, 0);

  pinMode(moisturePin, INPUT);
  analogReadResolution(12);
  analogSetPinAttenuation(moisturePin, ADC_11db);

  storageReady = storage.begin("soilpump", false);

  if (storageReady) {
    dosesUsed = storage.getUInt("doses", 0);
  } else {
    Serial.println("STORAGE ERROR - Pump disabled");
  }

  Serial.println("Standalone watering controller started");

  if (!automaticWatering) {
    Serial.println("CALIBRATION MODE - Pump disabled");
  }
}

// MAIN LOOP
void loop() {
  unsigned long now = millis();

  // Automatic pump timeout
  if (pumpRunning && now - pumpStart >= pumpDuration) {
    stopPump();
  }

  // BOOT button: stop pump and reset after 3-second hold
  if (digitalRead(refillButtonPin) == LOW) {

    if (pumpRunning) stopPump();

    if (buttonStart == 0) buttonStart = now;

    if (!buttonHandled && now - buttonStart >= 3000) {
      buttonHandled = true;

      if (storageReady && storage.putUInt("doses", 0) == 4) {
        dosesUsed = 0;
        dryCount = 0;
        Serial.println("Reservoir counter reset: 500 mL");
      } else {
        storageReady = false;
        Serial.println("RESET FAILED - Pump disabled");
      }
    }

  } else {
    buttonStart = 0;
    buttonHandled = false;
  }

  // Moisture reading every 2 seconds
  if (now - lastMoistureRead >= moistureInterval) {
    lastMoistureRead = now;

    int raw = analogRead(moisturePin);
    int millivolts = analogReadMilliVolts(moisturePin);

    latestSoilADC = raw;
    latestSoilVoltage = millivolts;


    Serial.print("Soil ADC: ");
    Serial.print(raw);
    Serial.print(" | Voltage: ");
    Serial.print(millivolts);
    Serial.println(" mV");

    // Reject very low or saturated readings
    if (raw < 300 || raw > 4000) {
      dryCount = 0;
      Serial.println("Sensor reading outside allowed range");
    }
    else if (raw < dryThreshold) {
      if (dryCount < 5) dryCount++;
    }
    else {
      dryCount = 0;
    }
  }

  // Automatic watering conditions
  bool cooldownFinished =
    !pumpHasRun || (now - lastPumpStop >= cooldown);

  if (
    automaticWatering &&
    storageReady &&
    !pumpRunning &&
    digitalRead(refillButtonPin) == HIGH &&
    now >= startupDelay &&
    cooldownFinished &&
    dosesUsed < maxDoses &&
    dryCount >= 5
  ) {

    startPump();
  }

  // Reconnect Wi-Fi only while pump is OFF
  if (!pumpRunning &&
      WiFi.status() != WL_CONNECTED &&
      now - lastWifiAttempt >= wifiRetryInterval) {

    lastWifiAttempt = now;
    WiFi.reconnect();
  }


  // WIFI AND DNS DIAGNOSTICS
  static unsigned long lastDiagnostic = 0;

  if (now - lastDiagnostic >= 30000) {
    lastDiagnostic = now;

    Serial.println("----- NETWORK DIAGNOSTICS -----");

    Serial.print("WiFi status: ");
    Serial.println(WiFi.status());

    if (WiFi.status() == WL_CONNECTED) {
      Serial.print("IP address: ");
      Serial.println(WiFi.localIP());

      Serial.print("Signal strength: ");
      Serial.print(WiFi.RSSI());
      Serial.println(" dBm");

      Serial.print("Gateway: ");
      Serial.println(WiFi.gatewayIP());

      IPAddress resolvedIP;
      int dnsResult = WiFi.hostByName(
        "api.thingspeak.com", resolvedIP
      );

      Serial.print("DNS result: ");
      Serial.println(dnsResult);


      // Test TCP connectivity to ThingSpeak
      WiFiClient tcpProbe;

      Serial.print("TCP port 443: ");

      if (tcpProbe.connect("api.thingspeak.com", 443)) {
        Serial.println("CONNECTED");
        tcpProbe.stop();
      } else {
        Serial.println("FAILED");
      }

      // Test HTTPS/TLS handshake
      WiFiClientSecure tlsProbe;
      tlsProbe.setInsecure();

      Serial.print("HTTPS/TLS: ");

      if (tlsProbe.connect("api.thingspeak.com", 443)) {
        Serial.println("CONNECTED");
        tlsProbe.stop();
      } else {
        Serial.println("FAILED");
      }


      Serial.print("ThingSpeak IP: ");
      Serial.println(resolvedIP);

    } else {
      Serial.println("WiFi disconnected");
    }

    Serial.println("-------------------------------");
  }


  // Upload to ThingSpeak every 30 seconds
  if (!pumpRunning &&
      WiFi.status() == WL_CONNECTED &&
      latestSoilADC >= 0 &&
      now - lastUpload >= uploadInterval) {

    lastUpload = now;

    // Soil status: -1 invalid, 0 wet, 1 dry
    int soilStatus = -1;

    if (latestSoilADC >= 300 && latestSoilADC <= 4000) {
      soilStatus = latestSoilADC < dryThreshold ? 1 : 0;
    }

    // Estimated reservoir remaining
    int waterRemaining = 500 - (dosesUsed * mlPerDose);
    if (waterRemaining < 0) waterRemaining = 0;

    ThingSpeak.setField(1, latestSoilADC);
    ThingSpeak.setField(2, latestSoilVoltage);
    ThingSpeak.setField(3, soilStatus);
    ThingSpeak.setField(4, (int)dosesUsed);
    ThingSpeak.setField(5, waterRemaining);

    int result = ThingSpeak.writeFields(channelID, writeAPIKey);

    if (result == 200) {
      Serial.println("ThingSpeak upload successful");
    } else {
      Serial.print("ThingSpeak upload failed: ");
      Serial.println(result);
    }
  }
}

