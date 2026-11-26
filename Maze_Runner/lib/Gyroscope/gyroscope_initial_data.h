#ifndef GYROSCOPE_INITIAL_DATA_H
#define GYROSCOPE_INITIAL_DATA_H

/**
 * ============================================================================
 *  gyroscope_initial_data.h
 *  EE4360 Micromouse Group 4 — MPU-6050 Factory / Pre-Calibration Data
 * ----------------------------------------------------------------------------
 *  This file captures:
 *    1. Factory-specified datasheet constants (do NOT change)
 *    2. First-boot measured offsets from Group 4's physical sensor unit
 *       (update these after running gyroscope_calibration_test)
 *    3. Recommended configuration constants for Micromouse use
 * ============================================================================
 */

#include "Gyroscope.h"

// ============================================================================
//  SECTION 1: MPU-6050 Datasheet Constants (read-only reference)
// ============================================================================

// Gyroscope sensitivity (LSB per °/s) for each full-scale range
static const float MPU6050_GYRO_SENS_250DPS  = 131.0f;
static const float MPU6050_GYRO_SENS_500DPS  =  65.5f;
static const float MPU6050_GYRO_SENS_1000DPS =  32.8f;
static const float MPU6050_GYRO_SENS_2000DPS =  16.4f;

// Accelerometer sensitivity (LSB per g) for each full-scale range
static const float MPU6050_ACCEL_SENS_2G  = 16384.0f;
static const float MPU6050_ACCEL_SENS_4G  =  8192.0f;
static const float MPU6050_ACCEL_SENS_8G  =  4096.0f;
static const float MPU6050_ACCEL_SENS_16G =  2048.0f;

// Temperature sensor:  Temp (°C) = rawTemp / 340 + 36.53
static const float MPU6050_TEMP_SCALE  = 340.0f;
static const float MPU6050_TEMP_OFFSET = 36.53f;

// Gyroscope zero-rate output (ZRO) drift spec from datasheet (typical)
//   ±20 LSB (±250 °/s range) at 25 °C, can vary ±1°/s across temperature
static const float MPU6050_GYRO_ZRO_TYPICAL_LSB = 20.0f;

// Gyroscope total RMS noise (from datasheet, ±250°/s range, DLPF 42 Hz)
static const float MPU6050_GYRO_NOISE_RMS_DPS = 0.05f;

// ============================================================================
//  SECTION 2: Recommended Configuration for Micromouse Turn Control
// ============================================================================
// Chosen for a good balance of sensitivity and noise for 90° turns at
// moderate speeds. Adjust if the robot spins very fast (use 500 DPS).

#define GYRO_RECOMMENDED_FS    GYRO_FS_250DPS    // ±250 °/s, 131 LSB/(°/s)
#define ACCEL_RECOMMENDED_FS   ACCEL_FS_2G       // ±2 g,   16384 LSB/g
#define GYRO_RECOMMENDED_DLPF  DLPF_BW_44HZ      // 44 Hz bandwidth

// Sample rate configuration
//   Gyro Output Rate = 1000 Hz (DLPF active)
//   Sample Rate = 1000 / (1 + SMPLRT_DIV)
//   SMPLRT_DIV = 4  → 200 Hz (5 ms per sample)
#define GYRO_SMPLRT_DIV  4

// Dead-band threshold: ignore gyroZ drift below this value
static const float GYRO_DEADBAND_DPS = 0.05f;

// Integration timeout: abort turn if not complete in this time
static const uint16_t GYRO_TURN_TIMEOUT_MS = 2000;

// ============================================================================
//  SECTION 3: First-Boot / Pre-Calibration Baseline Readings
// ----------------------------------------------------------------------------
//  Fill these in after the FIRST physical power-on reading test.
//  These are raw 16-bit ADC values read with the robot on flat ground,
//  completely still, before any calibration is applied.
//
//  HOW TO OBTAIN: Run gyroscope_calibration_test.cpp sketch, copy the
//  "Pre-calibration raw averages" section output here.
// ============================================================================

// --- Group 4 Unit Baseline (update after first physical test run) ---
// Status: auto-updated by gyro_watch.py on 2026-11-19 15:11
static const int16_t GYRO_BASELINE_GYRO_X_RAW  =  -498;
static const int16_t GYRO_BASELINE_GYRO_Y_RAW  =  -157;
static const int16_t GYRO_BASELINE_GYRO_Z_RAW  =  94;
static const int16_t GYRO_BASELINE_ACCEL_X_RAW =  -1276;
static const int16_t GYRO_BASELINE_ACCEL_Y_RAW =  -225;
static const int16_t GYRO_BASELINE_ACCEL_Z_RAW =  12183; // Expected: ~16384 (1g flat)

// ============================================================================
//  SECTION 4: Stored Calibration Offsets (Load Into Gyroscope::loadCalibration)
// ----------------------------------------------------------------------------
//  After running runStaticCalibration(), copy the printed offset values here.
//  These will be used as the default hard-coded calibration in production runs.
//
//  Saves ~2 seconds of recalibration on every boot.
// ============================================================================

// Status: auto-updated by gyro_watch.py on 2026-11-19 15:11
static const GyroCalibration STORED_GYRO_CALIBRATION = {
    /* gyroOffsetX  */  -499.02f,
    /* gyroOffsetY  */  -157.38f,
    /* gyroOffsetZ  */  93.11f,
    /* accelOffsetX */  -1272.80f,
    /* accelOffsetY */  -227.24f,
    /* accelOffsetZ */  -4198.71f,
    /* sampleCount  */  1000,
    /* gyroNoiseSigmaZ */ 4.896f,
    /* isValid      */  true,     // Set to true after filling real values
    /* headingScaleFactor */ 1.0229f,  // From TEST 7 & 8: sum(targets)/sum(measured)
    /* calTemperatureC    */ 28.26f,   // Die temperature at calibration time (TEST 6)
    /* gyroDriftPerDegC   */ 1.3f      // MPU-6050 datasheet typical (LSB/°C)
};

// ============================================================================
//  SECTION 5: Calibration Quality Thresholds
// ----------------------------------------------------------------------------
//  Used by calibration test to auto-PASS or FAIL each test.
// ============================================================================

// Maximum acceptable gyroZ offset after calibration (in LSB)
// At 131 LSB/(°/s), 10 LSB = 0.076 °/s bias — acceptable for Micromouse
static const float CAL_MAX_GYRO_OFFSET_LSB = 10.0f;

// Maximum acceptable gyroZ noise sigma (1-sigma, in LSB)
// 5 LSB = 0.038 °/s — typical noise for MPU-6050 with DLPF 44 Hz
static const float CAL_MAX_GYRO_NOISE_SIGMA_LSB = 8.0f;

// Maximum acceptable Z-axis gyro drift during 10-second static test (degrees)
static const float CAL_MAX_STATIC_DRIFT_10S_DEG = 0.5f;

// 90° turn accuracy tolerance (degrees)
static const float CAL_TURN_90_TOLERANCE_DEG = 3.0f;

// 180° turn accuracy tolerance (degrees)
static const float CAL_TURN_180_TOLERANCE_DEG = 5.0f;

// Temperature window: calibration is valid within ±5°C of calibration temperature
static const float CAL_TEMP_WINDOW_C = 5.0f;

// ============================================================================
//  SECTION 6: Turn Accuracy Results & Heading Scale Factor
// ----------------------------------------------------------------------------
//  Written automatically by Arduino_Test/gyroscope_test/gyro_watch.py from
//  TEST 6 (temperature) and the repeated 90° / 180° manual turn trials
//  (TEST 7 & 8).
//
//  GYRO_HEADING_SCALE_FACTOR = sum(target angles) / sum(measured angles)
//  over all turn trials. Multiply getHeading() by it to correct the gyro's
//  scale error (1.0 = no correction).
// ============================================================================
static const float GYRO_CAL_TEMPERATURE_C    = 28.26f;
static const float GYRO_TURN90_MEAN_DEG      = 86.667f;
static const float GYRO_TURN90_STDDEV_DEG    = 1.980f;
static const float GYRO_TURN180_MEAN_DEG     = 177.297f;
static const float GYRO_TURN180_STDDEV_DEG   = 2.188f;
static const float GYRO_HEADING_SCALE_FACTOR = 1.0229f;

#endif // GYROSCOPE_INITIAL_DATA_H
