#ifndef GLOBALS_H
#define GLOBALS_H

#include <Arduino.h>

extern volatile long leftEncoderCount  = 0; // LeftEncodeISR() in MotorDriver.cpp
extern volatile long rightEncoderCount = 0;

extern float frontDistance = 0;
extern float leftDistance  = 0;
extern float rightDistance = 0;
extern bool isMovingForward   = false; // MovedistanceCM() in MotorDriver.cpp
extern bool  isTurning     = false;
extern float lastError        = 0.0f;
extern float integralError    = 0.0f;

#endif