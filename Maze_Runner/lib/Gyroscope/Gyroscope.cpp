#include "Gyroscope.h"

// ============================================================================
//  Constructor
// ============================================================================
Gyroscope::Gyroscope(uint8_t i2cAddr, uint8_t gyroFs, uint8_t accelFs, uint8_t dlpf)
    : _i2cAddr(i2cAddr),
      _gyroFsConfig(gyroFs),
      _accelFsConfig(accelFs),
      _dlpfConfig(dlpf),
      _gyroSensitivity(131.0f),
      _accelSensitivity(16384.0f),
      _headingDeg(0.0f),
      _lastUpdateUs(0)
{
    // Zero-initialise calibration struct
    _cal = { 0.0f, 0.0f, 0.0f,   // gyro offsets
             0.0f, 0.0f, 0.0f,   // accel offsets
             0, 0.0f, false };
}

// ============================================================================
//  begin() — initialise MPU-6050
// ============================================================================
bool Gyroscope::begin() {
    Wire.begin();

    // --- Wake the device (clear SLEEP bit in PWR_MGMT_1) ---
    // Also select PLL with X-axis gyro reference (more stable than internal oscillator)
    writeRegister(MPU6050_REG_PWR_MGMT_1, 0x01);  // CLKSEL=001 (Gyro X PLL)
    delay(100);  // Allow oscillator to stabilise

    // --- Verify identity ---
    uint8_t whoAmI = readRegister(MPU6050_REG_WHO_AM_I);
    if (whoAmI != 0x68) {
        // WHO_AM_I should always return 0x68 for MPU-6050
        return false;
    }

    // --- Sample Rate Divider ---
    // Sample Rate = Gyroscope Output Rate / (1 + SMPLRT_DIV)
    // With DLPF active, Gyro Output Rate = 1 kHz
    // SMPLRT_DIV=4 → 1000 / (1+4) = 200 Hz
    writeRegister(MPU6050_REG_SMPLRT_DIV, 0x04);

    // --- DLPF Configuration ---
    writeRegister(MPU6050_REG_CONFIG, _dlpfConfig);

    // --- Gyroscope Full-Scale Range ---
    writeRegister(MPU6050_REG_GYRO_CONFIG, _gyroFsConfig);

    // --- Accelerometer Full-Scale Range ---
    writeRegister(MPU6050_REG_ACCEL_CONFIG, _accelFsConfig);

    // --- Resolve LSB sensitivities ---
    switch (_gyroFsConfig) {
        case GYRO_FS_250DPS:  _gyroSensitivity = 131.0f;  break;
        case GYRO_FS_500DPS:  _gyroSensitivity =  65.5f;  break;
        case GYRO_FS_1000DPS: _gyroSensitivity =  32.8f;  break;
        case GYRO_FS_2000DPS: _gyroSensitivity =  16.4f;  break;
        default:              _gyroSensitivity = 131.0f;  break;
    }

    switch (_accelFsConfig) {
        case ACCEL_FS_2G:  _accelSensitivity = 16384.0f; break;
        case ACCEL_FS_4G:  _accelSensitivity =  8192.0f; break;
        case ACCEL_FS_8G:  _accelSensitivity =  4096.0f; break;
        case ACCEL_FS_16G: _accelSensitivity =  2048.0f; break;
        default:           _accelSensitivity = 16384.0f; break;
    }

    return true;
}

// ============================================================================
//  Low-Level I2C helpers
// ============================================================================
void Gyroscope::writeRegister(uint8_t reg, uint8_t value) {
    Wire.beginTransmission(_i2cAddr);
    Wire.write(reg);
    Wire.write(value);
    Wire.endTransmission();
}

uint8_t Gyroscope::readRegister(uint8_t reg) {
    Wire.beginTransmission(_i2cAddr);
    Wire.write(reg);
    Wire.endTransmission(false);  // Repeated-start
    Wire.requestFrom(_i2cAddr, (uint8_t)1);
    return Wire.available() ? Wire.read() : 0xFF;
}

void Gyroscope::readRegisters(uint8_t startReg, uint8_t count, uint8_t* buf) {
    Wire.beginTransmission(_i2cAddr);
    Wire.write(startReg);
    Wire.endTransmission(false);
    Wire.requestFrom(_i2cAddr, count);
    for (uint8_t i = 0; i < count && Wire.available(); i++) {
        buf[i] = Wire.read();
    }
}

// ============================================================================
//  readRaw() — burst-read all 14 bytes (Accel + Temp + Gyro)
// ============================================================================
GyroRawData Gyroscope::readRaw() {
    uint8_t buf[14];
    readRegisters(MPU6050_REG_ACCEL_XOUT_H, 14, buf);

    GyroRawData raw;
    raw.accelX = (int16_t)((buf[0]  << 8) | buf[1]);
    raw.accelY = (int16_t)((buf[2]  << 8) | buf[3]);
    raw.accelZ = (int16_t)((buf[4]  << 8) | buf[5]);
    raw.tempRaw= (int16_t)((buf[6]  << 8) | buf[7]);
    raw.gyroX  = (int16_t)((buf[8]  << 8) | buf[9]);
    raw.gyroY  = (int16_t)((buf[10] << 8) | buf[11]);
    raw.gyroZ  = (int16_t)((buf[12] << 8) | buf[13]);

    return raw;
}

// ============================================================================
//  readScaled() — apply offsets and sensitivity scaling
// ============================================================================
GyroData Gyroscope::readScaled() {
    GyroRawData raw = readRaw();
    GyroData out;

    out.accelX_g   = (raw.accelX - _cal.accelOffsetX) / _accelSensitivity;
    out.accelY_g   = (raw.accelY - _cal.accelOffsetY) / _accelSensitivity;
    out.accelZ_g   = (raw.accelZ - _cal.accelOffsetZ) / _accelSensitivity;

    out.gyroX_dps  = (raw.gyroX  - _cal.gyroOffsetX)  / _gyroSensitivity;
    out.gyroY_dps  = (raw.gyroY  - _cal.gyroOffsetY)  / _gyroSensitivity;
    out.gyroZ_dps  = (raw.gyroZ  - _cal.gyroOffsetZ)  / _gyroSensitivity;

    // MPU-6050 temperature formula from datasheet:
    // Temperature(°C) = (TEMP_OUT / 340) + 36.53
    out.tempC = (raw.tempRaw / 340.0f) + 36.53f;

    return out;
}

// ============================================================================
//  runStaticCalibration()
// ============================================================================
const GyroCalibration& Gyroscope::runStaticCalibration(uint16_t numSamples, uint8_t delayMs) {
    _cal.isValid = false;

    // Discard first 50 samples (let sensor settle after power-on noise burst)
    for (int i = 0; i < 50; i++) {
        readRaw();
        delay(delayMs);
    }

    // Accumulate sums
    double sumGX = 0, sumGY = 0, sumGZ = 0;
    double sumAX = 0, sumAY = 0, sumAZ = 0;
    double sumSqGZ = 0;  // For sigma calculation

    for (uint16_t i = 0; i < numSamples; i++) {
        GyroRawData r = readRaw();
        sumGX += r.gyroX;
        sumGY += r.gyroY;
        sumGZ += r.gyroZ;
        sumAX += r.accelX;
        sumAY += r.accelY;
        sumAZ += r.accelZ;
        sumSqGZ += (double)r.gyroZ * r.gyroZ;
        delay(delayMs);
    }

    // Compute means
    _cal.gyroOffsetX  = (float)(sumGX / numSamples);
    _cal.gyroOffsetY  = (float)(sumGY / numSamples);
    _cal.gyroOffsetZ  = (float)(sumGZ / numSamples);
    _cal.accelOffsetX = (float)(sumAX / numSamples);
    _cal.accelOffsetY = (float)(sumAY / numSamples);

    // AccelZ should read +_accelSensitivity (1g) when flat:
    // We store the deviation from the expected 1g reading
    _cal.accelOffsetZ = (float)(sumAZ / numSamples) - _accelSensitivity;

    // Noise sigma for gyroZ (variance = E[X²] - E[X]²)
    double varGZ = (sumSqGZ / numSamples) - (_cal.gyroOffsetZ * _cal.gyroOffsetZ);
    _cal.gyroNoiseSigmaZ = (varGZ > 0) ? (float)sqrt(varGZ) : 0.0f;

    _cal.sampleCount = numSamples;
    _cal.isValid = true;

    return _cal;
}

// ============================================================================
//  loadCalibration()
// ============================================================================
void Gyroscope::loadCalibration(const GyroCalibration& cal) {
    _cal = cal;
}

// ============================================================================
//  printCalibration()
// ============================================================================
void Gyroscope::printCalibration() const {
    Serial.println(F("--- Gyroscope Calibration Results ---"));
    Serial.print(F("  Valid     : ")); Serial.println(_cal.isValid ? F("YES") : F("NO"));
    Serial.print(F("  Samples   : ")); Serial.println(_cal.sampleCount);
    Serial.println(F("  Gyro Offsets (LSB):"));
    Serial.print(F("    X (Roll)  = ")); Serial.println(_cal.gyroOffsetX, 2);
    Serial.print(F("    Y (Pitch) = ")); Serial.println(_cal.gyroOffsetY, 2);
    Serial.print(F("    Z (Yaw)   = ")); Serial.println(_cal.gyroOffsetZ, 2);
    Serial.print(F("  Gyro Z Noise sigma (LSB) = ")); Serial.println(_cal.gyroNoiseSigmaZ, 3);
    Serial.println(F("  Accel Offsets (LSB):"));
    Serial.print(F("    X = ")); Serial.println(_cal.accelOffsetX, 2);
    Serial.print(F("    Y = ")); Serial.println(_cal.accelOffsetY, 2);
    Serial.print(F("    Z = ")); Serial.println(_cal.accelOffsetZ, 2);
    Serial.println(F("-------------------------------------"));
}

// ============================================================================
//  Heading Integration (Yaw / Z-axis)
// ============================================================================
void Gyroscope::resetHeading() {
    _headingDeg   = 0.0f;
    _lastUpdateUs = micros();
}

float Gyroscope::updateHeading() {
    unsigned long now = micros();
    float dtSeconds   = (now - _lastUpdateUs) * 1e-6f;
    _lastUpdateUs     = now;

    GyroRawData raw = readRaw();
    // Apply offset then scale to °/s
    float gyroZ_dps = (raw.gyroZ - _cal.gyroOffsetZ) / _gyroSensitivity;

    // Dead-band: ignore noise below 0.05 °/s
    if (gyroZ_dps > -0.05f && gyroZ_dps < 0.05f) {
        gyroZ_dps = 0.0f;
    }

    _headingDeg += gyroZ_dps * dtSeconds;
    return _headingDeg;
}

// ============================================================================
//  Utility
// ============================================================================
bool Gyroscope::isConnected() {
    Wire.beginTransmission(_i2cAddr);
    return (Wire.endTransmission() == 0);
}

float Gyroscope::readTemperatureC() {
    uint8_t buf[2];
    readRegisters(MPU6050_REG_TEMP_OUT_H, 2, buf);
    int16_t rawTemp = (int16_t)((buf[0] << 8) | buf[1]);
    return (rawTemp / 340.0f) + 36.53f;
}
