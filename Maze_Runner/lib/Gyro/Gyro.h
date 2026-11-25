#ifndef GYRO_H
#define GYRO_H

#include <Arduino.h>

// MPU-6050 yaw (Z axis). Right turn = positive yaw.
bool  gyroBegin();        // start I2C and wake the sensor; false if not found
bool  gyroReady();        // true after a successful gyroBegin()
bool  gyroCalibrate(int samples = 500);   // measure bias (robot must be still); yaw is kept.
                                          // false if it moved (old bias is kept)
void  gyroUpdate();       // call often: integrates yaw
float gyroYaw();          // degrees
float gyroRate();         // deg/s
void  gyroResetYaw();

#endif // GYRO_H
