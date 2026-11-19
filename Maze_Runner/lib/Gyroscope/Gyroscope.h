#ifndef GYROSCOPE_H
#define GYROSCOPE_H

#include <Arduino.h>
#include <Wire.h>

// ============================================================================
//  MPU-6050 Register Map (I2C Address: 0x68 default, 0x69 if AD0 pin = HIGH)
// ============================================================================
#define MPU6050_I2C_ADDR        0x68

// Configuration Registers
#define MPU6050_REG_PWR_MGMT_1  0x6B
#define MPU6050_REG_SMPLRT_DIV  0x19
#define MPU6050_REG_CONFIG      0x1A
#define MPU6050_REG_GYRO_CONFIG 0x1B
#define MPU6050_REG_ACCEL_CONFIG 0x1C

// Data Output Registers (16-bit signed, Big-Endian)
#define MPU6050_REG_ACCEL_XOUT_H 0x3B
#define MPU6050_REG_GYRO_XOUT_H  0x43
#define MPU6050_REG_TEMP_OUT_H   0x41
#define MPU6050_REG_WHO_AM_I     0x75

// Gyroscope Full-Scale Range Options (FS_SEL bits)
// Affects the LSB sensitivity (LSB/°/s)
#define GYRO_FS_250DPS   0x00    // ±250  °/s  → 131.0 LSB/(°/s)
#define GYRO_FS_500DPS   0x08    // ±500  °/s  →  65.5 LSB/(°/s)
#define GYRO_FS_1000DPS  0x10    // ±1000 °/s  →  32.8 LSB/(°/s)
#define GYRO_FS_2000DPS  0x18    // ±2000 °/s  →  16.4 LSB/(°/s)

// Accelerometer Full-Scale Range Options (AFS_SEL bits)
#define ACCEL_FS_2G   0x00       //  ±2g  → 16384 LSB/g
#define ACCEL_FS_4G   0x08       //  ±4g  →  8192 LSB/g
#define ACCEL_FS_8G   0x10       //  ±8g  →  4096 LSB/g
#define ACCEL_FS_16G  0x18       //  ±16g →  2048 LSB/g

// DLPF (Digital Low-Pass Filter) Configurations
//   Bandwidth:   260Hz (OFF) | 184Hz | 94Hz | 44Hz | 21Hz | 10Hz | 5Hz
#define DLPF_BW_260HZ  0x00
#define DLPF_BW_184HZ  0x01
#define DLPF_BW_94HZ   0x02
#define DLPF_BW_44HZ   0x03
#define DLPF_BW_21HZ   0x04
#define DLPF_BW_10HZ   0x05
#define DLPF_BW_5HZ    0x06

// ============================================================================
//  Calibration Data Structure
//  Stores offset biases computed during static calibration
// ============================================================================
struct GyroCalibration {
    // Gyroscope zero-rate offsets (in raw LSB units)
    float gyroOffsetX;   // Roll axis  (X)
    float gyroOffsetY;   // Pitch axis (Y)
    float gyroOffsetZ;   // Yaw axis   (Z)  ← most critical for Micromouse

    // Accelerometer zero-g offsets (in raw LSB units)
    float accelOffsetX;
    float accelOffsetY;
    float accelOffsetZ;  // Should read +16384 (1g) when flat; offset = raw - 16384

    // Calibration metadata
    uint16_t sampleCount;      // Number of samples averaged during calibration
    float    gyroNoiseSigmaZ;  // 1-sigma noise on Z-axis gyro (LSB), test quality
    bool     isValid;          // Set true only after a successful calibration run
};

// ============================================================================
//  Raw Sensor Data Snapshot
// ============================================================================
struct GyroRawData {
    int16_t accelX, accelY, accelZ;
    int16_t gyroX,  gyroY,  gyroZ;
    int16_t tempRaw;
};

// ============================================================================
//  Processed / Scaled Sensor Data
// ============================================================================
struct GyroData {
    float accelX_g, accelY_g, accelZ_g;   // Acceleration in g
    float gyroX_dps, gyroY_dps, gyroZ_dps; // Angular velocity in °/s
    float tempC;                            // Temperature in °C
};

// ============================================================================
//  Gyroscope / IMU Driver Class  (MPU-6050 on Arduino Mega via I2C)
// ============================================================================
class Gyroscope {
public:
    // -------------------------------------------------------------------------
    //  Constructor
    //  @param i2cAddr  I2C slave address (0x68 default, 0x69 if AD0 is HIGH)
    //  @param gyroFs   Full-scale range for gyro  (GYRO_FS_*)
    //  @param accelFs  Full-scale range for accel (ACCEL_FS_*)
    //  @param dlpf     Digital Low-Pass Filter bandwidth (DLPF_BW_*)
    // -------------------------------------------------------------------------
    Gyroscope(uint8_t i2cAddr   = MPU6050_I2C_ADDR,
              uint8_t gyroFs    = GYRO_FS_250DPS,
              uint8_t accelFs   = ACCEL_FS_2G,
              uint8_t dlpf      = DLPF_BW_44HZ);

    // -------------------------------------------------------------------------
    //  Initialise I2C bus and MPU-6050 registers.
    //  Call once from setup().
    //  @return true if WHO_AM_I check passes (device found and responsive)
    // -------------------------------------------------------------------------
    bool begin();

    // -------------------------------------------------------------------------
    //  Read raw 16-bit ADC values from all 6 axes + temperature.
    // -------------------------------------------------------------------------
    GyroRawData readRaw();

    // -------------------------------------------------------------------------
    //  Read calibration-corrected and scaled sensor data.
    //  Applies stored offsets and LSB sensitivity automatically.
    // -------------------------------------------------------------------------
    GyroData readScaled();

    // =========================================================================
    //  CALIBRATION API
    // =========================================================================

    // -------------------------------------------------------------------------
    //  Static Zero-Rate / Zero-G Calibration
    //  Place robot on flat, still surface before calling.
    //  Collects `numSamples` readings, averages them, stores result in _cal.
    //  @param numSamples  Recommended: 500–2000 samples for good statistics
    //  @param delayMs     Delay between samples (ms). 2–5 ms typical.
    //  @return Reference to the populated GyroCalibration struct
    // -------------------------------------------------------------------------
    const GyroCalibration& runStaticCalibration(uint16_t numSamples = 1000,
                                                uint8_t  delayMs    = 2);

    // -------------------------------------------------------------------------
    //  Load previously computed calibration offsets into this instance.
    //  Call if you are loading from EEPROM or hard-coded constants.
    // -------------------------------------------------------------------------
    void loadCalibration(const GyroCalibration& cal);

    // -------------------------------------------------------------------------
    //  Access currently loaded calibration data.
    // -------------------------------------------------------------------------
    const GyroCalibration& getCalibration() const { return _cal; }

    // -------------------------------------------------------------------------
    //  Print calibration results to Serial in human-readable format.
    // -------------------------------------------------------------------------
    void printCalibration() const;

    // =========================================================================
    //  HEADING INTEGRATION (Yaw tracking for Micromouse turns)
    // =========================================================================

    // Reset integrated heading to zero (call before each movement segment)
    void resetHeading();

    // Integrate gyroZ reading over time → returns accumulated yaw in degrees.
    // Call repeatedly in a tight loop (every ~5–10 ms) while turning.
    float updateHeading();

    // Return current integrated heading (degrees, positive = CW viewed from above; sensor mounted Z-up)
    float getHeading() const { return _headingDeg; }

    // =========================================================================
    //  UTILITY
    // =========================================================================

    // Check if device is reachable on I2C bus
    bool isConnected();

    // Read chip temperature (useful for thermal drift monitoring)
    float readTemperatureC();

    // Get LSB sensitivity values based on configured full-scale ranges
    float getGyroSensitivity()  const { return _gyroSensitivity;  }
    float getAccelSensitivity() const { return _accelSensitivity; }

private:
    // I2C helpers
    void     writeRegister(uint8_t reg, uint8_t value);
    uint8_t  readRegister(uint8_t reg);
    void     readRegisters(uint8_t startReg, uint8_t count, uint8_t* buf);

    // Configuration
    uint8_t  _i2cAddr;
    uint8_t  _gyroFsConfig;
    uint8_t  _accelFsConfig;
    uint8_t  _dlpfConfig;

    // LSB sensitivities (set in begin() based on FS config)
    float    _gyroSensitivity;   // LSB per °/s
    float    _accelSensitivity;  // LSB per g

    // Calibration storage
    GyroCalibration _cal;

    // Heading integration state
    float          _headingDeg;
    unsigned long  _lastUpdateUs;
};

#endif // GYROSCOPE_H
