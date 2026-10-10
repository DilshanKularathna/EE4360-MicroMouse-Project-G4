#ifndef MOTOR_DRIVER_H
#define MOTOR_DRIVER_H

#include <Arduino.h>

// void setMotors(int leftSpeed, int rightSpeed);
// void stopMotors();

// float readUltrasonic(int trig, int echo);
// void  readAllSensors();

// void leftEncoderISR();
// void rightEncoderISR();
// void resetEncoders();
// void moveDistanceCM(float distanceCM, int speed);
// void turnDegrees(float degrees, int speed);
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