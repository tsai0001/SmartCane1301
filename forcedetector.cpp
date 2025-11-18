// Force-Sensing Cane Firmware (Extended)
// Adds step counter + pressure/area calculations
// HX711 + EMA smoothing + start/stop detection + step counting
// Cloud variables: weight, force, pressure, area, steps

#include "Particle.h"
#include "HX711.h"

#define HX711_DOUT D2
#define HX711_CLK  D3

HX711 scale; 

// ----- Calibration -----
float calibrationFactor = -7050.0f;   // change after calibration
float zeroOffset = 0;

// ----- Filtering -----
float ema = 0.0;
float alpha = 0.15;

// ----- Force detection -----
float thresholdKg = 0.5;              // threshold to detect contact start (~5N)
bool inContact = false;

// ----- Step counting -----
int stepCount = 0;

// ----- Physics -----
const float g = 9.80665;              // m/s²
float forceN = 0.0;

// ----- Sensor geometry -----
// If using a single-round sensor or known FSR footprint:
float sensorArea_cm2 = 4.0;           // Example: 4 cm² pad (adjust to your sensor)
                                      // If you want dynamic area detection, see below.

// Calculated
float pressure_kPa = 0.0;
float est_contact_area_cm2 = 0.0;

// ----- Cloud variables -----
double weightForCloud = 0;
double forceForCloud = 0;
double pressureForCloud = 0;
double areaForCloud = 0;
int    stepsForCloud = 0;

void setup() {
    Serial.begin(9600);
    waitFor(Serial.isConnected, 3000);

    scale.begin(HX711_DOUT, HX711_CLK);
    delay(500);

    scale.set_scale(calibrationFactor);
    scale.tare();
    zeroOffset = scale.get_offset();

    Particle.variable("weight", weightForCloud);
    Particle.variable("force", forceForCloud);
    Particle.variable("pressure", pressureForCloud);
    Particle.variable("area", areaForCloud);
    Particle.variable("steps", stepsForCloud);

    Particle.publish("status", "Force cane started", PRIVATE);

    Serial.println("Force cane firmware loaded.");
}

void loop() {

    // -------- 1: Read HX711 --------
    float rawKg = scale.get_units(1);

    // -------- 2: EMA smoothing --------
    if (ema == 0) ema = rawKg;
    ema = alpha * rawKg + (1 - alpha) * ema;

    float kg = ema;
    weightForCloud = kg;

    // -------- 3: Convert to force --------
    forceN = kg * g;
    forceForCloud = forceN;

    // -------- 4: Pressure & Contact Area Calculation --------
    // Basic pressure calculation: P = F / A
    // Convert cm² → m² (1 cm² = 0.0001 m²)
    float area_m2 = sensorArea_cm2 * 0.0001;

    // Constant-area pressure
    pressure_kPa = (forceN / area_m2) / 1000.0;   // Pa → kPa
    pressureForCloud = pressure_kPa;

    // Dynamic area estimation:  
    // As force increases, FSRs "spread" and effectively increase contact patch.
    // This is a simple linear estimate: adjust slope based on your sensor tests.
    est_contact_area_cm2 = sensorArea_cm2 + (kg * 1.25);   // add area with more load
    areaForCloud = est_contact_area_cm2;

    // -------- 5: Step detection (start/stop) --------
    bool above = (kg >= thresholdKg);

    if (!inContact && above) {
        inContact = true;
        Particle.publish("cane_start", String(kg, 3), PRIVATE);
        Serial.println("START detected.");
    }

    if (inContact && !above) {
        inContact = false;
        stepCount++;
        stepsForCloud = stepCount;

        Particle.publish("cane_stop", String(kg, 3), PRIVATE);
        Particle.publish("cane_step", String(stepCount), PRIVATE);

        Serial.printf("STOP detected. Step %d\n", stepCount);
    }

    // -------- 6: Debug print --------
    Serial.printf("kg: %.3f   F: %.2f N   P: %.2f kPa   Area: %.2f cm^2   Steps: %d\n",
                  kg, forceN, pressure_kPa, est_contact_area_cm2, stepCount);

    delay(50);
}
