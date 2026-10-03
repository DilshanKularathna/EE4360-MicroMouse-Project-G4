#ifndef MOTOR_DRIVER_H
#define MOTOR_DRIVER_H

#include <Arduino.h>

class MotorDriver {
private:
    uint8_t leftRpwm, leftLpwm;
    uint8_t rightRpwm, rightLpwm;
    uint8_t enablePin;
    uint8_t leftEncA, leftEncB;
    uint8_t rightEncA, rightEncB;

public:
    MotorDriver(uint8_t lRpwm, uint8_t lLpwm, 
                uint8_t rRpwm, uint8_t rLpwm, 
                uint8_t enPin,
                uint8_t lEncA, uint8_t lEncB, 
                uint8_t rEncA, uint8_t rEncB);

    void begin();
    
    // Raw speed outputs (-255 to 255)
    void setSpeeds(int leftSpeed, int rightSpeed);
    void stop();

    // Encoder management
    void resetEncoders();
    long getLeftPulses();
    long getRightPulses();

    // Controlled closed-loop movements using Encoders & PI controller
    void moveCells(float numCells, int basePwm = 70);
    void turnRight90(int basePwm = 70);
    void turn180(int basePwm = 70);

    // Static ISR Callback Routines
    static void handleLeftEncoder();
    static void handleRightEncoder();
};

#endif // MOTOR_DRIVER_H