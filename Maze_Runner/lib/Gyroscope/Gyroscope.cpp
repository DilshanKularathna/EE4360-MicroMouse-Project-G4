#include "Gyroscope.h"

Gyroscope::Gyroscope(TwoWire &wire, bool upsideDownMounting)
    : wire(wire), mpu(wire), ready(false),
      yawSign(upsideDownMounting ? -1.0f : 1.0f) {
    // MPU6050_light applies this flag to accelerometer Z, but not gyro Z.
    // Compensate yaw explicitly below for an upside-down board.
    mpu.upsideDownMounting = upsideDownMounting;
}

bool Gyroscope::begin(uint8_t gyroConfig, uint8_t accelConfig) {
    wire.begin();
    wire.setClock(100000);

    const byte status = mpu.begin(gyroConfig, accelConfig);
    ready = (status == 0);
    return ready;
}

bool Gyroscope::calibrate() {
    if (!ready) return false;

    // Yaw comes from integrating the Z gyro, so accelerometer offsets are not
    // needed for this application.
    mpu.calcOffsets(true, false);
    mpu.update();
    return true;
}

bool Gyroscope::calibrateAccelerometer() {
    if (!ready) return false;
    mpu.calcOffsets(false, true);
    mpu.update();
    return true;
}

void Gyroscope::update() {
    if (ready) mpu.update();
}

bool Gyroscope::isReady() const {
    return ready;
}

float Gyroscope::getYawAngleDeg() {
    return ready ? yawSign * mpu.getAngleZ() : 0.0f;
}

float Gyroscope::getYawRateDps() {
    return ready ? yawSign * mpu.getGyroZ() : 0.0f;
}

float Gyroscope::getAccelAngleXDeg() {
    return ready ? mpu.getAccAngleX() : 0.0f;
}

float Gyroscope::getAccelAngleYDeg() {
    return ready ? mpu.getAccAngleY() : 0.0f;
}
