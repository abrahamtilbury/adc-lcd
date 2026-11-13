#include <Preferences.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <HTTPClient.h>


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
const int pumpLedPin = 16;
const int emptyLedPin = 17;
const int manualButtonPin = 18;

const int pumpDuty = 204;  // Fixed 80% PWM

// CALIBRATION
const int dryThreshold = 2200;
const int wetThreshold = 2400;

bool automaticWatering = false;  // OFF during commissioning
const bool allowRemoteDose = false;

// TIMING
const unsigned long moistureInterval = 2000;
const unsigned long pumpDuration = 20000;
const unsigned long soakDuration = 120000;
const unsigned long cooldown = 1800000;
const unsigned long startupDelay = 1800000;

const int maxPulsesPerSession = 3;

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

// TALKBACK
const unsigned long talkBackID = 0;
const char* talkBackKey = "WGE6LSWQHTYVT9IM";

const unsigned long talkBackInterval = 60000;
unsigned long lastTalkBack = 0;

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
int wetCount = 0;

bool sensorValid = false;
bool moistureKnown = false;
bool moistureDry = false;

bool manualRunning = false;
bool sessionActive = false;
bool soaking = false;
bool singleDoseSession = false;

int sessionPulses = 0;
unsigned long soakStart = 0;
unsigned long manualStartTime = 0;

// STOP PUMP
void stopPump() {
  ledcWrite(enablePin, 0);
  digitalWrite(pumpPin, LOW);
  digitalWrite(pumpLedPin, LOW);

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
  digitalWrite(pumpLedPin, HIGH);

  pumpStart = millis();
  pumpRunning = true;
  dryCount = 0;

  Serial.println("Pump ON - 20 second dose");
  Serial.print("Doses used: ");
  Serial.println(dosesUsed);
}

void finishSession() {
  sessionActive = false;
  soaking = false;
  singleDoseSession = false;
  sessionPulses = 0;

  pumpHasRun = true;
  lastPumpStop = millis();
  dryCount = 0;

  Serial.println("Watering session finished");
  Serial.println("30-minute cooldown started");
}

// SETUP
void setup() {
  digitalWrite(pumpPin, LOW);
  pinMode(pumpPin, OUTPUT);
  pinMode(refillButtonPin, INPUT_PULLUP);
  pinMode(manualButtonPin, INPUT_PULLUP);

  pinMode(pumpLedPin, OUTPUT);
  pinMode(emptyLedPin, OUTPUT);

  digitalWrite(pumpLedPin, LOW);
  digitalWrite(emptyLedPin, LOW);

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

  // Dose-limit indicator
  digitalWrite(
    emptyLedPin,
    dosesUsed >= maxDoses ? HIGH : LOW
  );

  // Physical manual hold-to-run control
  bool manualHeld = digitalRead(manualButtonPin) == LOW;

  if (manualHeld) {
    if (!manualRunning) {

      if (pumpRunning) {
        stopPump();
      }

      if (sessionActive) {
        finishSession();
      }

      digitalWrite(pumpPin, HIGH);
      ledcWrite(enablePin, pumpDuty);
      digitalWrite(pumpLedPin, HIGH);

      manualRunning = true;
      manualStartTime = millis();
      Serial.println("MANUAL PUMP ON");
    }

    // No software timer while physically held.
    // Skip Wi-Fi operations during manual pumping.
    return;
  }

  if (manualRunning) {
    ledcWrite(enablePin, 0);
    digitalWrite(pumpPin, LOW);
    digitalWrite(pumpLedPin, LOW);
    Serial.print("Manual pump runtime: ");
    Serial.print((millis() - manualStartTime) / 1000.0, 1);
    Serial.println(" seconds");
    manualRunning = false;
    finishSession();

    Serial.println("MANUAL PUMP OFF");
  }

  // Automatic/remote 20-second cutoff
  if (pumpRunning && now - pumpStart >= pumpDuration) {
    stopPump();

    if (sessionActive) {
      if (singleDoseSession ||
          sessionPulses >= maxPulsesPerSession) {
        finishSession();
      } else {
        soaking = true;
        soakStart = millis();
        Serial.println("Soaking for 2 minutes");
      }
    }
  }

  // BOOT button: stop pump and reset after 3-second hold
  if (digitalRead(refillButtonPin) == LOW) {

    if (pumpRunning) {
      stopPump();
    }

    if (sessionActive) {
      finishSession();
    }

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

        // Moisture validation and hysteresis
    if (raw < 300 || raw > 4000) {
      sensorValid = false;
      moistureKnown = false;
      moistureDry = false;

      dryCount = 0;
      wetCount = 0;

      Serial.println("Invalid moisture reading");
    } else {
      sensorValid = true;

      if (raw < dryThreshold) {
        if (dryCount < 5) dryCount++;
        wetCount = 0;

        if (dryCount >= 5) {
          moistureDry = true;
          moistureKnown = true;
        }
      }
      else if (raw >= wetThreshold) {
        if (wetCount < 5) wetCount++;
        dryCount = 0;

        if (wetCount >= 5) {
          moistureDry = false;
          moistureKnown = true;
        }
      }
      else {
        dryCount = 0;
        wetCount = 0;
      }
    }

    Serial.print("Moisture state: ");
    if (!sensorValid || !moistureKnown) {
      Serial.println("UNKNOWN");
    } else {
      Serial.println(moistureDry ? "DRY" : "WET");
    }
  }

    // Finish soak and decide whether another pulse is needed
  if (sessionActive &&
      soaking &&
      !pumpRunning &&
      now - soakStart >= soakDuration) {
      

    soaking = false;

    if (!automaticWatering ||
        !sensorValid ||
        !moistureKnown ||
        !moistureDry) {

      finishSession();

    } else {

      startPump();

      if (pumpRunning) {
        sessionPulses++;

        Serial.print("Session pulse: ");
        Serial.println(sessionPulses);
      } else {
        finishSession();
      }
    }
  }

  // Begin a new automatic watering session
  bool cooldownFinished =
    !pumpHasRun || (now - lastPumpStop >= cooldown);

  if (
    automaticWatering &&
    storageReady &&
    !pumpRunning &&
    !sessionActive &&
    digitalRead(refillButtonPin) == HIGH &&
    now >= startupDelay &&
    cooldownFinished &&
    dosesUsed < maxDoses &&
    sensorValid &&
    moistureKnown &&
    moistureDry &&
    dryCount >= 5
  ) {

    startPump();

    if (pumpRunning) {
      sessionActive = true;
      singleDoseSession = false;
      sessionPulses = 1;

      Serial.println("Automatic session started");
    }
  }

    // ThingSpeak TalkBack commands
  if (!pumpRunning &&
      !manualRunning &&
      WiFi.status() == WL_CONNECTED &&
      talkBackID != 0 &&
      now - lastTalkBack >= talkBackInterval) {

    lastTalkBack = now;

    HTTPClient http;
    http.setTimeout(3000);

    String url =
      String("https://api.thingspeak.com/talkbacks/") +
      String(talkBackID) +
      "/commands/execute";

    if (http.begin(thingSpeakClient, url)) {
      http.addHeader(
        "Content-Type",
        "application/x-www-form-urlencoded"
      );

      int response = http.POST(
        String("api_key=") + talkBackKey
      );

      if (response == 200) {
        String command = http.getString();
        command.trim();

        if (command == "AUTO_ON") {
          automaticWatering = true;
          Serial.println("Remote automatic watering ENABLED");
        }
        else if (command == "AUTO_OFF") {
          automaticWatering = false;

          if (sessionActive) {
            finishSession();
          }

          Serial.println("Remote automatic watering DISABLED");
        }
        else if (command == "DOSE") {
          bool remoteCooldownFinished =
            !pumpHasRun ||
            (millis() - lastPumpStop >= cooldown);

          if (allowRemoteDose &&
              storageReady &&
              !sessionActive &&
              !pumpRunning &&
              dosesUsed < maxDoses &&
              millis() >= startupDelay &&
              remoteCooldownFinished) {

            startPump();

            if (pumpRunning) {
              sessionActive = true;
              singleDoseSession = true;
              sessionPulses = 1;

              Serial.println("Remote single dose started");
            }
          } else {
            Serial.println("Remote dose rejected");
          }
        }
      } else {
        Serial.print("TalkBack HTTP status: ");
        Serial.println(response);
      }

      http.end();
    }
  }

  // Reconnect Wi-Fi only while pump is OFF
  if (!pumpRunning &&
      WiFi.status() != WL_CONNECTED &&
      now - lastWifiAttempt >= wifiRetryInterval) {

    lastWifiAttempt = now;
    WiFi.reconnect();
  }

  // Upload to ThingSpeak every 30 seconds
  if (!pumpRunning &&
      WiFi.status() == WL_CONNECTED &&
      latestSoilADC >= 0 &&
      now - lastUpload >= uploadInterval) {

    lastUpload = now;

    // Soil status: -1 invalid, 0 wet, 1 dry
    int soilStatus = -1;

    if (sensorValid && moistureKnown) {
      soilStatus = moistureDry ? 1 : 0;
    }

    // Estimated reservoir remaining
    int waterRemaining = 500 - (dosesUsed * mlPerDose);
    if (waterRemaining < 0) waterRemaining = 0;

    ThingSpeak.setField(1, latestSoilADC);
    ThingSpeak.setField(2, latestSoilVoltage);
    ThingSpeak.setField(3, soilStatus);
    ThingSpeak.setField(4, (int)dosesUsed);
    ThingSpeak.setField(5, waterRemaining);
    ThingSpeak.setField(6, automaticWatering ? 1 : 0);
    ThingSpeak.setField(7, soaking ? 2 : (sessionActive ? 1 : 0));

    int result = ThingSpeak.writeFields(channelID, writeAPIKey);
    

    if (result == 200) {
      Serial.println("ThingSpeak upload successful");
    } else {
      Serial.print("ThingSpeak upload failed: ");
      Serial.println(result);
    }
  }
}

