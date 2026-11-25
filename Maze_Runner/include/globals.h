#ifndef GLOBALS_H
#define GLOBALS_H

#include <Arduino.h>

#define TURN_SPEED 30
const float MAX_DUTY = 28.0f;

extern volatile long leftEncoderCount; // LeftEncodeISR() in MotorDriver.cpp
extern volatile long rightEncoderCount; // RightEncodeISR() in MotorDriver.cpp

extern float frontDistance;
extern float leftDistance;
extern float rightDistance;
extern int currentHeading;
extern bool isMovingForward; // MovedistanceCM() in MotorDriver.cpp
extern bool  isTurning;
extern float lastError;
extern float integralError;

#endif