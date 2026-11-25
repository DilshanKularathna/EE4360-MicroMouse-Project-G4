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

struct TurnResult {
    float finalError;       // deg, after the robot has stopped
    float overshoot;        // deg past the target (0 if none)
    unsigned long timeMs;
    bool timedOut;
};
TurnResult turnDegreesWith(float degrees, float kp, int minDuty, int maxDuty);

// Optional: called every loop of a turn (for plotting/debugging). nullptr = off.
extern void (*turnLogHook)(float yaw, float err, int duty);

void resetEncoders();
void setMotors(int leftSpeed, int rightSpeed);
void moveDistanceCM(float distanceCM, int speed);
void stopMotors();
void initEncoders();     // call once in setup(): pins + 4x quadrature interrupts
void leftEncoderISR();
void rightEncoderISR();
void setPWM(uint8_t pin, uint32_t freq, uint8_t dutyCycle);
#endif // MOTOR_DRIVER_H