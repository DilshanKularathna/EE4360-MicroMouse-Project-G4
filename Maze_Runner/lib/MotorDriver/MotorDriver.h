#ifndef MOTOR_DRIVER_H
#define MOTOR_DRIVER_H

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

// float readUltrasonic(int trig, int echo);
// void  readAllSensors();

void turnLeft();
void turnRight();
void turnAround();
void turnDegrees(float degrees);
void resetEncoders();
void setMotors(int leftSpeed, int rightSpeed);
void moveDistanceCM(float distanceCM, int speed);
void stopMotors();
void leftEncoderISR();
void rightEncoderISR();
void setPWM(uint8_t pin, uint32_t freq, uint8_t dutyCycle);
#endif // MOTOR_DRIVER_H
