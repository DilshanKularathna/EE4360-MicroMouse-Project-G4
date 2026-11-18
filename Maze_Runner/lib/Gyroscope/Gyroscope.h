#ifndef GYROSCOPE_H
#define GYROSCOPE_H

#include <Arduino.h>
#include <Wire.h>
#include <MPU6050_light.h>

// Navigation-level interface for the robot's yaw gyro. The current backend is
// MPU6050_light; maze code should depend only on this class's yaw operations.
class Gyroscope {
public:
    explicit Gyroscope(TwoWire &wire = Wire, bool upsideDownMounting = false);

    bool begin(uint8_t gyroConfig = 0, uint8_t accelConfig = 0);
    bool calibrate();
    bool calibrateAccelerometer();
    void update();

    bool isReady() const;
    float getYawAngleDeg();
    float getYawRateDps();
    float getAccelAngleXDeg();
    float getAccelAngleYDeg();

private:
    TwoWire &wire;
    MPU6050 mpu;
    bool ready;
    float yawSign;
};

#endif // GYROSCOPE_H
