/**
 * ============================================================================
 *  gyroscope_calibration_test.cpp
 *  EE4360 Micromouse Group 4 — Gyroscope Calibration & Validation Test Suite
 * ----------------------------------------------------------------------------
 *  Purpose:
 *    Step-by-step automated test suite for the MPU-6050 gyroscope.
 *    Flash this sketch alone (comment out main.cpp) and open Serial Monitor
 *    at 115200 baud to run all calibration tests and record results.
 *
 *  Tests Included:
 *    TEST 1: Device Connection Check
 *    TEST 2: Pre-Calibration Raw Baseline Reading
 *    TEST 3: Static Zero-Rate Calibration (1000 samples)
 *    TEST 4: Post-Calibration Noise & Stability Check
 *    TEST 5: Static Drift Test (10 seconds stationary heading)
 *    TEST 6: Temperature Reading & Thermal Check
 *    TEST 7: Manual 90° Turn Accuracy Test (requires physical rotation)
 *    TEST 8: Manual 180° Turn Accuracy Test (requires physical rotation)
 *
 *  How to Use:
 *    1. Place robot on a FLAT, STILL surface (table, not your hand)
 *    2. Flash this file to the Arduino Mega
 *    3. Open Serial Monitor @ 115200 baud
 *    4. Watch tests 1–6 run automatically
 *    5. For tests 7–8, follow the on-screen instructions to rotate the robot
 *    6. Copy the output offset values into gyroscope_initial_data.h
 * ============================================================================
 */

#include <Arduino.h>
#include <Wire.h>
#include "Gyroscope.h"
#include "gyroscope_initial_data.h"

// ---------------------------------------------------------------------------
//  Sensor instance: 0x68 (AD0 LOW), ±250 °/s, ±2g, 44 Hz DLPF
// ---------------------------------------------------------------------------
Gyroscope gyro(MPU6050_I2C_ADDR,
               GYRO_RECOMMENDED_FS,
               ACCEL_RECOMMENDED_FS,
               GYRO_RECOMMENDED_DLPF);

// ---------------------------------------------------------------------------
//  Test result tracking
// ---------------------------------------------------------------------------
struct TestResult {
    const char* name;
    bool        passed;
    char        notes[64];
};

static TestResult results[8];
static uint8_t    resultCount = 0;

void recordResult(const char* name, bool pass, const char* note = "") {
    if (resultCount < 8) {
        results[resultCount].name   = name;
        results[resultCount].passed = pass;
        strncpy(results[resultCount].notes, note, 63);
        resultCount++;
    }
}

void printSeparator() {
    Serial.println(F("--------------------------------------------"));
}

void printHeader(const char* title) {
    Serial.println(F(""));
    Serial.println(F("============================================"));
    Serial.print(F("  "));
    Serial.println(title);
    Serial.println(F("============================================"));
}

// ===========================================================================
//  TEST 1: Device Connection Check
// ===========================================================================
void test1_connectionCheck() {
    printHeader("TEST 1: Device Connection Check");

    bool connected = gyro.isConnected();
    Serial.print(F("I2C address 0x68 respond: "));
    Serial.println(connected ? F("YES ✓") : F("NO ✗"));

    bool initOk = gyro.begin();
    Serial.print(F("MPU-6050 WHO_AM_I check : "));
    Serial.println(initOk ? F("PASSED (0x68) ✓") : F("FAILED ✗"));

    bool pass = connected && initOk;
    recordResult("Connection Check", pass,
                 pass ? "Device found on I2C" : "Device NOT found - check wiring");

    Serial.print(F("Result: ")); Serial.println(pass ? F("PASS") : F("FAIL"));
}

// ===========================================================================
//  TEST 2: Pre-Calibration Raw Baseline Reading
// ===========================================================================
void test2_baselineReading() {
    printHeader("TEST 2: Pre-Calibration Raw Baseline");
    Serial.println(F("Reading 100 samples with no calibration applied..."));
    Serial.println(F(">>> KEEP ROBOT COMPLETELY STILL <<<"));
    delay(2000);

    long sumGX=0, sumGY=0, sumGZ=0;
    long sumAX=0, sumAY=0, sumAZ=0;

    for (int i = 0; i < 100; i++) {
        GyroRawData r = gyro.readRaw();
        sumGX += r.gyroX; sumGY += r.gyroY; sumGZ += r.gyroZ;
        sumAX += r.accelX; sumAY += r.accelY; sumAZ += r.accelZ;
        delay(5);
    }

    Serial.println(F("Pre-calibration raw averages (copy to gyroscope_initial_data.h):"));
    Serial.print(F("  GYRO_X_RAW  = ")); Serial.println(sumGX / 100);
    Serial.print(F("  GYRO_Y_RAW  = ")); Serial.println(sumGY / 100);
    Serial.print(F("  GYRO_Z_RAW  = ")); Serial.println(sumGZ / 100);
    Serial.print(F("  ACCEL_X_RAW = ")); Serial.println(sumAX / 100);
    Serial.print(F("  ACCEL_Y_RAW = ")); Serial.println(sumAY / 100);
    Serial.print(F("  ACCEL_Z_RAW = ")); Serial.println(sumAZ / 100);

    // Sanity: accelZ should be close to 16384 (±2g range, flat)
    int16_t avgAZ = sumAZ / 100;
    bool accelOk  = (avgAZ > 14000 && avgAZ < 18000);
    Serial.print(F("  AccelZ 1g plausibility: "));
    Serial.println(accelOk ? F("OK (near 16384)") : F("WARNING - sensor may not be flat or wiring issue"));

    recordResult("Baseline Reading", accelOk,
                 accelOk ? "AccelZ near 16384" : "AccelZ out of expected range");
}

// ===========================================================================
//  TEST 3: Static Zero-Rate Calibration (1000 samples)
// ===========================================================================
void test3_staticCalibration() {
    printHeader("TEST 3: Static Calibration (1000 samples)");
    Serial.println(F(">>> KEEP ROBOT COMPLETELY STILL for ~5 seconds <<<"));
    delay(3000);
    Serial.println(F("Collecting samples..."));

    const GyroCalibration& cal = gyro.runStaticCalibration(1000, 2);

    Serial.println(F(""));
    gyro.printCalibration();

    Serial.println(F(">>> COPY THESE VALUES INTO gyroscope_initial_data.h <<<"));
    Serial.println(F("STORED_GYRO_CALIBRATION = {"));
    Serial.print(F("    .gyroOffsetX  = ")); Serial.print(cal.gyroOffsetX,  2); Serial.println(F("f,"));
    Serial.print(F("    .gyroOffsetY  = ")); Serial.print(cal.gyroOffsetY,  2); Serial.println(F("f,"));
    Serial.print(F("    .gyroOffsetZ  = ")); Serial.print(cal.gyroOffsetZ,  2); Serial.println(F("f,"));
    Serial.print(F("    .accelOffsetX = ")); Serial.print(cal.accelOffsetX, 2); Serial.println(F("f,"));
    Serial.print(F("    .accelOffsetY = ")); Serial.print(cal.accelOffsetY, 2); Serial.println(F("f,"));
    Serial.print(F("    .accelOffsetZ = ")); Serial.print(cal.accelOffsetZ, 2); Serial.println(F("f,"));
    Serial.print(F("    .sampleCount  = ")); Serial.print(cal.sampleCount);    Serial.println(F(","));
    Serial.print(F("    .gyroNoiseSigmaZ = ")); Serial.print(cal.gyroNoiseSigmaZ, 3); Serial.println(F("f,"));
    Serial.println(F("    .isValid      = true"));
    Serial.println(F("};"));

    // Check offset quality
    bool gyroZOffsetOk = (abs(cal.gyroOffsetZ) < CAL_MAX_GYRO_OFFSET_LSB * 5); // 5x margin pre-offset
    bool noiseOk       = (cal.gyroNoiseSigmaZ < CAL_MAX_GYRO_NOISE_SIGMA_LSB * 2);
    bool pass          = cal.isValid && noiseOk;

    Serial.print(F("GyroZ noise sigma (LSB): ")); Serial.print(cal.gyroNoiseSigmaZ, 3);
    Serial.print(F("  → ")); Serial.println(noiseOk ? F("OK") : F("HIGH NOISE - check vibration"));

    recordResult("Static Calibration", pass,
                 pass ? "Offsets computed" : "High noise or invalid result");
}

// ===========================================================================
//  TEST 4: Post-Calibration Noise & Stability
// ===========================================================================
void test4_postCalNoise() {
    printHeader("TEST 4: Post-Calibration Noise Check");
    Serial.println(F("Reading 200 calibrated samples to verify noise floor..."));
    Serial.println(F(">>> KEEP ROBOT STILL <<<"));
    delay(1000);

    float minZ = 1e6f, maxZ = -1e6f, sumZ = 0;
    for (int i = 0; i < 200; i++) {
        GyroData d = gyro.readScaled();
        float gz = d.gyroZ_dps;
        sumZ += gz;
        if (gz < minZ) minZ = gz;
        if (gz > maxZ) maxZ = gz;
        delay(5);
    }

    float meanZ  = sumZ / 200.0f;
    float rangeZ = maxZ - minZ;

    Serial.print(F("  GyroZ mean  (°/s): ")); Serial.println(meanZ, 4);
    Serial.print(F("  GyroZ min   (°/s): ")); Serial.println(minZ,  4);
    Serial.print(F("  GyroZ max   (°/s): ")); Serial.println(maxZ,  4);
    Serial.print(F("  GyroZ range (°/s): ")); Serial.println(rangeZ, 4);

    bool meanOk  = (abs(meanZ)  < 0.1f);   // <0.1 °/s residual bias
    bool rangeOk = (rangeZ      < 0.3f);   // <0.3 °/s peak-to-peak noise
    bool pass    = meanOk && rangeOk;

    Serial.print(F("  Mean bias check  : ")); Serial.println(meanOk  ? F("PASS") : F("FAIL"));
    Serial.print(F("  Noise range check: ")); Serial.println(rangeOk ? F("PASS") : F("FAIL"));

    recordResult("Post-Cal Noise", pass,
                 pass ? "Noise within spec" : "Excessive noise or bias remaining");
}

// ===========================================================================
//  TEST 5: Static Drift Test (10 seconds)
// ===========================================================================
void test5_staticDrift() {
    printHeader("TEST 5: Static Drift Test (10 seconds)");
    Serial.println(F(">>> DO NOT TOUCH THE ROBOT for 10 seconds <<<"));
    Serial.println(F("Integrating heading - should stay near 0°..."));

    gyro.resetHeading();
    unsigned long start = millis();
    float maxDrift = 0.0f;

    while (millis() - start < 10000UL) {
        float h = gyro.updateHeading();
        if (abs(h) > maxDrift) maxDrift = abs(h);
        delay(5);
    }

    float finalDrift = abs(gyro.getHeading());

    Serial.print(F("  Final heading after 10s (°): ")); Serial.println(gyro.getHeading(), 3);
    Serial.print(F("  Peak drift during test  (°): ")); Serial.println(maxDrift, 3);

    bool pass = (finalDrift < CAL_MAX_STATIC_DRIFT_10S_DEG);
    Serial.print(F("  Drift <")); Serial.print(CAL_MAX_STATIC_DRIFT_10S_DEG);
    Serial.print(F("°: ")); Serial.println(pass ? F("PASS") : F("FAIL - recalibrate"));

    recordResult("Static Drift 10s", pass,
                 pass ? "Drift within spec" : "Excessive drift - redo calibration");
}

// ===========================================================================
//  TEST 6: Temperature Reading
// ===========================================================================
void test6_temperature() {
    printHeader("TEST 6: Temperature Reading");

    float tempC = gyro.readTemperatureC();
    Serial.print(F("  Die temperature: ")); Serial.print(tempC, 1); Serial.println(F(" °C"));

    // MPU-6050 typical operating range: -40°C to +85°C; die normally 25–45°C
    bool inRange = (tempC > 10.0f && tempC < 70.0f);
    Serial.print(F("  Temperature range check: "));
    Serial.println(inRange ? F("PASS (10–70°C)") : F("FAIL - unusual reading"));

    Serial.println(F("  NOTE: Calibration drifts ~±1°/s per 10°C change."));
    Serial.println(F("  Recalibrate if operating temperature differs by >5°C from"));
    Serial.println(F("  the temperature at calibration time."));

    recordResult("Temperature Check", inRange,
                 inRange ? "Temp in normal range" : "Temp out of expected range");
}

// ===========================================================================
//  TEST 7: Manual 90° Turn Accuracy
// ===========================================================================
void test7_turn90() {
    printHeader("TEST 7: Manual 90° Turn Test");
    Serial.println(F(""));
    Serial.println(F("INSTRUCTIONS:"));
    Serial.println(F("  1. Place robot on flat surface"));
    Serial.println(F("  2. Mark the robot's start orientation with tape"));
    Serial.println(F("  3. When prompted, MANUALLY rotate the robot exactly 90°"));
    Serial.println(F("     (CW = positive heading)"));
    Serial.println(F("  4. Hold still at the 90° position"));
    Serial.println(F(""));
    Serial.println(F("  Sending 'G' in Serial Monitor starts measurement."));
    Serial.println(F("  Sending 'S' stops it."));
    Serial.println(F(""));
    Serial.println(F(">>> Waiting for 'G' to begin..."));

    // Wait for user input
    while (Serial.read() != 'G') { delay(50); }
    Serial.read(); // clear buffer

    gyro.resetHeading();
    Serial.println(F("MEASURING... rotate robot 90° CW then send 'S'"));

    while (true) {
        gyro.updateHeading();
        if (Serial.available() && Serial.read() == 'S') break;
        delay(5);
    }

    float measured = gyro.getHeading();
    float error    = measured - 90.0f;

    Serial.print(F("  Measured angle (°): ")); Serial.println(measured, 2);
    Serial.print(F("  Error from 90° (°): ")); Serial.println(error,    2);

    bool pass = (abs(error) < CAL_TURN_90_TOLERANCE_DEG);
    Serial.print(F("  Within ±")); Serial.print(CAL_TURN_90_TOLERANCE_DEG);
    Serial.print(F("°: ")); Serial.println(pass ? F("PASS") : F("FAIL"));

    char note[64];
    snprintf(note, 64, "Measured: %.1f deg, Error: %.1f deg", measured, error);
    recordResult("90° Turn Accuracy", pass, note);
}

// ===========================================================================
//  TEST 8: Manual 180° Turn Accuracy
// ===========================================================================
void test8_turn180() {
    printHeader("TEST 8: Manual 180° Turn Test");
    Serial.println(F("INSTRUCTIONS: Same as Test 7 but rotate 180°."));
    Serial.println(F("  Send 'G' to begin, 'S' to stop."));
    Serial.println(F(">>> Waiting for 'G'..."));

    while (Serial.read() != 'G') { delay(50); }
    Serial.read();

    gyro.resetHeading();
    Serial.println(F("MEASURING... rotate robot 180° CW then send 'S'"));

    while (true) {
        gyro.updateHeading();
        if (Serial.available() && Serial.read() == 'S') break;
        delay(5);
    }

    float measured = gyro.getHeading();
    float error    = measured - 180.0f;

    Serial.print(F("  Measured angle (°): ")); Serial.println(measured, 2);
    Serial.print(F("  Error from 180°(°): ")); Serial.println(error,    2);

    bool pass = (abs(error) < CAL_TURN_180_TOLERANCE_DEG);
    Serial.print(F("  Within ±")); Serial.print(CAL_TURN_180_TOLERANCE_DEG);
    Serial.print(F("°: ")); Serial.println(pass ? F("PASS") : F("FAIL"));

    char note[64];
    snprintf(note, 64, "Measured: %.1f deg, Error: %.1f deg", measured, error);
    recordResult("180° Turn Accuracy", pass, note);
}

// ===========================================================================
//  Print Final Summary
// ===========================================================================
void printSummary() {
    printHeader("CALIBRATION TEST SUMMARY");
    uint8_t passed = 0;

    for (uint8_t i = 0; i < resultCount; i++) {
        Serial.print(F("  TEST "));
        Serial.print(i + 1);
        Serial.print(F(": "));
        Serial.print(results[i].name);
        Serial.print(F(" → "));
        Serial.print(results[i].passed ? F("PASS") : F("FAIL"));
        Serial.print(F("  ["));
        Serial.print(results[i].notes);
        Serial.println(F("]"));
        if (results[i].passed) passed++;
    }

    printSeparator();
    Serial.print(F("  OVERALL: ")); Serial.print(passed);
    Serial.print(F("/")); Serial.print(resultCount);
    Serial.println(passed == resultCount ? F(" — ALL PASSED ✓") : F(" — SOME FAILED ✗"));
    printSeparator();

    Serial.println(F(""));
    Serial.println(F("NEXT STEPS:"));
    Serial.println(F("  1. Copy offset values from TEST 3 into gyroscope_initial_data.h"));
    Serial.println(F("  2. Set STORED_GYRO_CALIBRATION.isValid = true"));
    Serial.println(F("  3. Remove this test sketch from compilation (rename or move)"));
    Serial.println(F("  4. In production, call gyro.loadCalibration(STORED_GYRO_CALIBRATION)"));
    Serial.println(F("     instead of runStaticCalibration() to save boot time"));
}

// ===========================================================================
//  setup() / loop()
// ===========================================================================
void setup() {
    Serial.begin(115200);
    while (!Serial) { ; }

    Serial.println(F(""));
    Serial.println(F("╔══════════════════════════════════════════╗"));
    Serial.println(F("║  EE4360 Micromouse - Gyroscope Test     ║"));
    Serial.println(F("║  Group 4  |  MPU-6050 Calibration Suite ║"));
    Serial.println(F("╚══════════════════════════════════════════╝"));
    Serial.println(F(""));
    Serial.println(F("Place robot on FLAT, STILL surface and keep it stable."));
    Serial.println(F("Starting in 5 seconds..."));
    delay(5000);

    test1_connectionCheck();
    delay(500);

    test2_baselineReading();
    delay(500);

    test3_staticCalibration();
    delay(500);

    test4_postCalNoise();
    delay(500);

    test5_staticDrift();
    delay(500);

    test6_temperature();
    delay(500);

    // Manual tests — user-interactive
    test7_turn90();
    delay(1000);

    test8_turn180();
    delay(500);

    printSummary();
}

void loop() {
    // Idle — all tests done in setup()
    delay(1000);
}
