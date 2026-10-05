#include "GyroSensing.h"
#include "config.h"
#include <Wire.h>

bool GyroSensing::begin() {
    Wire.begin();

    Wire.beginTransmission(MPU6050_I2C_ADDR);
    uint8_t whoAmIStatus = Wire.endTransmission();
    if (whoAmIStatus != 0) return false; // device not responding

    writeRegister(REG_PWR_MGMT_1, 0x00);   // wake the device up
    writeRegister(REG_GYRO_CONFIG, 0x00);  // +/-250 deg/s full scale

    calibrate();
    return true;
}