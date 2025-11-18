#include "Particle.h"
#include "HX711ADC.h"
#include <vector>

SYSTEM_MODE(AUTOMATIC);
SYSTEM_THREAD(ENABLED);

// --------------------------------------------------------
// Pins
// --------------------------------------------------------
#define HX711_DOUT D2
#define HX711_SCK  D3
#define BUTTON_PIN D5
#define LED_PIN    D6     // LED indicator

// --------------------------------------------------------
HX711ADC scale(HX711_DOUT, HX711_SCK);

// --------------------------------------------------------
// Calibration + thresholds
// --------------------------------------------------------
float calibrationFactor = -7050;
float zeroOffset = 0;

float stepThreshold = 10.0;     // Newtons needed to count a step
bool aboveThreshold = false;     // Track force state for step detection

// --------------------------------------------------------
// Data logging buffers
// --------------------------------------------------------
std::vector<float> forceData;
std::vector<unsigned long> timeData;

int stepCount = 0;

// --------------------------------------------------------
// Logging control
// --------------------------------------------------------
bool loggingActive = false;
bool prevButton = false;

unsigned long lastSample = 0;
unsigned long sampleInterval = 100;  // ms

// --------------------------------------------------------
void setup() {
    pinMode(BUTTON_PIN, INPUT_PULLDOWN);
    pinMode(LED_PIN, OUTPUT);

    digitalWrite(LED_PIN, LOW);

    scale.begin();
    delay(500);
    scale.start(2000);
    scale.setCalFactor(calibrationFactor);
    zeroOffset = scale.read();

    Particle.publish("status", "Boot Complete", PRIVATE);
}

// --------------------------------------------------------
float readForce() {
    float raw = scale.read();
    float f = (raw - zeroOffset) * (1.0 / calibrationFactor) * 100.0;
    if (f < 0) f = 0;
    return f;
}

// --------------------------------------------------------
// Step detection: rising-edge threshold crossing
// --------------------------------------------------------
void detectStep(float force) {
    if (force > stepThreshold && !aboveThreshold) {
        stepCount++;
        aboveThreshold = true;
    }
    if (force < stepThreshold) {
        aboveThreshold = false;
    }
}

// --------------------------------------------------------
// Build JSON upload
// --------------------------------------------------------
String buildJSON() {
    String json = "{ \"steps\": ";
    json += String(stepCount);
    json += ", \"force\": [";

    for (size_t i = 0; i < forceData.size(); i++) {
        json += String::format("{\"t\":%lu,\"f\":%.2f}", timeData[i], forceData[i]);
        if (i < forceData.size() - 1) json += ",";
    }
    json += "] }";

    return json;
}

// --------------------------------------------------------
void uploadSession() {

    // Blink LED during upload
    for (int i = 0; i < 3; i++) {
        digitalWrite(LED_PIN, HIGH);
        delay(150);
        digitalWrite(LED_PIN, LOW);
        delay(150);
    }

    String payload = buildJSON();
    Particle.publish("session_data", payload, PRIVATE);

    forceData.clear();
    timeData.clear();
    stepCount = 0;
    aboveThreshold = false;

    Particle.publish("status", "Upload Complete", PRIVATE);
}

// --------------------------------------------------------
// MAIN LOOP
// --------------------------------------------------------
void loop() {

    // Button press (rising edge)
    bool pressed = digitalRead(BUTTON_PIN);

    if (pressed && !prevButton) {
        loggingActive = !loggingActive;

        if (loggingActive) {
            // Start logging
            forceData.clear();
            timeData.clear();
            stepCount = 0;
            aboveThreshold = false;

            digitalWrite(LED_PIN, HIGH);  // LED ON = logging
            Particle.publish("status", "Logging Started", PRIVATE);
        }
        else {
            // Stop logging + upload
            digitalWrite(LED_PIN, LOW);   // LED OFF = idle
            Particle.publish("status", "Logging Stopped", PRIVATE);

            uploadSession();
        }
    }

    prevButton = pressed;

    // Logging loop
    if (loggingActive && millis() - lastSample >= sampleInterval) {
        lastSample = millis();

        float f = readForce();

        forceData.push_back(f);
        timeData.push_back(millis());

        detectStep(f);  // <-- Step detection here
    }
}
