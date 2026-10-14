#ifndef GYROSCOPE_H
#define GYROSCOPE_H

#include <Arduino.h>
#include <Wire.h>
<<<<<<< HEAD
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
=======

// Hardware & Config Constants
extern const uint8_t MPU6050_ADDRESS;
extern const uint8_t MPU6050_REG_PWR_MGMT_1;
extern const uint8_t MPU6050_REG_GYRO_CONFIG;
extern const uint8_t MPU6050_REG_ACCEL_XOUT_H;
extern const float MPU6050_GYRO_LSB_PER_DPS;
extern const unsigned long GYRO_SAMPLE_INTERVAL_US;
extern const unsigned long GYRO_BIAS_CALIBRATION_MS;
extern const float TURN_STOP_TOLERANCE_DEG;

// Target & Angle Variables
extern int16_t gyroOffsets[3];
extern bool gyroReady;
extern float gyroHeadingDeg;
extern float gyroHeadingTargetDeg;

// Function Declarations
bool initializeGyroscope();
bool readGyro(int16_t gyroRaw[3]);
bool turnWithGyroscope(float targetDegrees);

#endif // GYROSCOPE_H
>>>>>>> 52a4566e4cc79b9346b7953eabe55524b83b3ea4
