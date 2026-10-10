#include "Gyroscope.h"
#include "MotorDriver.h"
#include "Wire.h"
#include "Arduino.h"
#include "config.h"
// MPU6050 is mounted with +Z down, so Z gyro rate is positive for a
// clockwise (right) turn. Calibrate the stationary bias at every boot.
// Hardware & Config Definitions
const uint8_t MPU6050_ADDRESS = 0x68;
const uint8_t MPU6050_REG_PWR_MGMT_1 = 0x6B;
const uint8_t MPU6050_REG_GYRO_CONFIG = 0x1B;
const uint8_t MPU6050_REG_ACCEL_XOUT_H = 0x3B;
const float MPU6050_GYRO_LSB_PER_DPS = 131.0f;
const unsigned long GYRO_SAMPLE_INTERVAL_US = 10000UL;
const unsigned long GYRO_BIAS_CALIBRATION_MS = 3000UL;
const float TURN_STOP_TOLERANCE_DEG = 4.0f;

// Global State Variables
int16_t gyroOffsets[3] = {0, 0, 0};
bool gyroReady = false;
float gyroHeadingDeg = 0.0f;
float gyroHeadingTargetDeg = 0.0f;

// 1. Read raw Z-axis values over I2C
bool readGyro(int16_t gyroRaw[3]) {
    Wire.beginTransmission(MPU6050_ADDRESS);
    Wire.write(MPU6050_REG_ACCEL_XOUT_H + 8); // GYRO_XOUT_H
    if (Wire.endTransmission(false) != 0 || Wire.requestFrom(MPU6050_ADDRESS, (uint8_t)6) != 6) {
        while (Wire.available()) Wire.read();
        return false;
    }
    for (uint8_t axis = 0; axis < 3; ++axis) {
        gyroRaw[axis] = (int16_t)((Wire.read() << 8) | Wire.read());
    }
    return true;
}

// 2. Initialize hardware registers and calibrate zero-motion offset
bool initializeGyroscope() {
    Wire.begin();
    Wire.setClock(100000);

    Wire.beginTransmission(MPU6050_ADDRESS);
    Wire.write(MPU6050_REG_PWR_MGMT_1);
    Wire.write(0x00); // Wake the MPU6050 and use the internal clock.
    if (Wire.endTransmission() != 0) return false;
    delay(100);

    Wire.beginTransmission(MPU6050_ADDRESS);
    Wire.write(MPU6050_REG_GYRO_CONFIG);
    Wire.write(0x00); // +/-250 degrees/second, 131 LSB per degree/second.
    if (Wire.endTransmission() != 0) return false;

    int32_t sums[3] = {0, 0, 0};
    uint16_t samples = 0;
    int16_t raw[3];
    const unsigned long startMs = millis();
    unsigned long nextSampleUs = micros();
    Serial.println(F("Keep robot still: calibrating gyro bias for 3 seconds."));
    while (millis() - startMs < GYRO_BIAS_CALIBRATION_MS) {
        if ((long)(micros() - nextSampleUs) >= 0) {
            if (!readGyro(raw)) return false;
            for (uint8_t axis = 0; axis < 3; ++axis) sums[axis] += raw[axis];
            ++samples;
            nextSampleUs += GYRO_SAMPLE_INTERVAL_US;
        }
    }
    if (samples == 0) return false;
    for (uint8_t axis = 0; axis < 3; ++axis) gyroOffsets[axis] = (int16_t)(sums[axis] / samples);
    Serial.print(F("Gyro bias raw X/Y/Z: "));
    Serial.print(gyroOffsets[0]); Serial.print('/');
    Serial.print(gyroOffsets[1]); Serial.print('/');
    Serial.println(gyroOffsets[2]);
    return true;
}

// 3. Perform closed-loop turns to target angles
bool turnWithGyroscope(float targetDegrees) {
    if (!gyroReady) {
        stopMotors();
        Serial.println(F("Turn refused: MPU6050 is not ready."));
        return false;
    }

    const uint8_t yawAxis = MPU6050_YAW_AXIS_INDEX;
    const unsigned long timeoutMs = (fabs(targetDegrees) > 90.0f) ? 4500UL : 3000UL;
    const unsigned long startMs = millis();
    unsigned long nextSampleUs = micros();
    unsigned long lastSampleUs = nextSampleUs;
    float angle = 0.0f;
    int motorYawSign = 1;
    bool motorDirectionVerified = false;
    int16_t raw[3];

    Serial.print(F("Gyro turn target: ")); Serial.print(targetDegrees, 0); Serial.println(F(" deg"));
    while (fabs(targetDegrees - angle) > TURN_STOP_TOLERANCE_DEG && millis() - startMs < timeoutMs) {
        if ((long)(micros() - nextSampleUs) < 0) continue;
        if (!readGyro(raw)) {
            stopMotors();
            Serial.println(F("MPU6050 read failed during turn; motors stopped."));
            return false;
        }

        const unsigned long sampleUs = micros();
        const float dt = (sampleUs - lastSampleUs) / 1000000.0f;
        lastSampleUs = sampleUs;
        const float rateDps = (raw[yawAxis] - gyroOffsets[yawAxis]) / MPU6050_GYRO_LSB_PER_DPS;
        angle += rateDps * dt;
        gyroHeadingDeg += rateDps * dt;

        if (!motorDirectionVerified && millis() - startMs >= 200UL && fabs(angle) >= 2.0f) {
            motorYawSign = (angle * targetDegrees > 0.0f) ? 1 : -1;
            motorDirectionVerified = true;
        }

        const float error = targetDegrees - angle;
        const int direction = ((error > 0.0f) ? 1 : -1) * motorYawSign;
        const int duty = (fabs(error) < 20.0f) ? 18 : TURN_SPEED;
        setMotors(direction * duty, -direction * duty);
        nextSampleUs += GYRO_SAMPLE_INTERVAL_US;
    }

    stopMotors();
    delay(200); // Let the robot settle before reporting the final integrated angle.
    if (!readGyro(raw)) {
        Serial.println(F("MPU6050 read failed after turn."));
        return false;
    }
    const bool reached = fabs(targetDegrees - angle) <= TURN_STOP_TOLERANCE_DEG;
    if (reached) gyroHeadingTargetDeg += targetDegrees;
    Serial.print(F("Turn angle=")); Serial.print(angle, 1);
    Serial.print(F(" deg; target=")); Serial.print(targetDegrees, 0);
    Serial.println(reached ? F("; reached") : F("; timeout/under-turn"));
    return reached;
}