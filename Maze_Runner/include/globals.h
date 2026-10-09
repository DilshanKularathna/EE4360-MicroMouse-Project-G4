#ifndef GLOBALS_H
#define GLOBALS_H

#include <Arduino.h>

extern volatile long leftEncoderCount;
extern volatile long rightEncoderCount;

extern float frontDistance;
extern float leftDistance;
extern float rightDistance;
extern bool isMovingForward; // MovedistanceCM() in MotorDriver.cpp
extern bool isTurning;
extern float lastError;
extern float integralError;

#endif
