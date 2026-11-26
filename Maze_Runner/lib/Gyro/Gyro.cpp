#include <Wire.h>
#include "config.h"
#include "Gyro.h"

static const uint8_t MPU_ADDR       = 0x68;
static const float   LSB_PER_DEG_S  = 131.0f;   // range +-250 deg/s

static float biasZ = 0.0f;   // deg/s
static float yaw   = 0.0f;   // deg
static float rate  = 0.0f;   // deg/s
static unsigned long lastUs = 0;
static bool ready = false;

static void writeReg(uint8_t reg, uint8_t val) {
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(reg);
    Wire.write(val);
    Wire.endTransmission();
}

static int16_t readGyroZRaw() {
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x47);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU_ADDR, (uint8_t)2);
    uint8_t hi = Wire.read();
    uint8_t lo = Wire.read();
    return (int16_t)((hi << 8) | lo);
}

bool gyroBegin() {
    Wire.begin();
    Wire.setClock(400000);

    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x75);                       // WHO_AM_I
    Wire.endTransmission(false);
    Wire.requestFrom(MPU_ADDR, (uint8_t)1);
    if (Wire.read() != 0x68) return false;

    writeReg(0x6B, 0x00);                   // wake up
    writeReg(0x1B, 0x00);                   // gyro range +-250 deg/s
    delay(100);
    ready = true;
    return true;
}

bool gyroReady() { return ready; }

bool gyroCalibrate(int samples) {
    long sum = 0;
    int16_t minV = 32767;
    int16_t maxV = -32768;

    for (int i = 0; i < samples; i++) {
        int16_t v = readGyroZRaw();
        sum += v;
        if (v < minV) minV = v;
        if (v > maxV) maxV = v;
        delay(2);
    }

    lastUs = micros();   // yaw is kept: only the bias is refreshed
    if ((maxV - minV) / LSB_PER_DEG_S >= 3.0f) return false;   // robot moved: keep old bias

    biasZ = (sum / (float)samples) / LSB_PER_DEG_S;
    return true;
}

void gyroUpdate() {
    unsigned long now = micros();
    float dt = (now - lastUs) / 1e6f;
    lastUs = now;

    rate = -(readGyroZRaw() / LSB_PER_DEG_S - biasZ) * GYRO_SCALE;   // sensor is mounted reversed
    yaw += rate * dt;
}

float gyroYaw()  { return yaw; }
float gyroRate() { return rate; }

void gyroResetYaw() {
    yaw    = 0.0f;
    lastUs = micros();
}
