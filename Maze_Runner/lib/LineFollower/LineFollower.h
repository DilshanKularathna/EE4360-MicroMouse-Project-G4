#ifndef LINE_FOLLOWER_H
#define LINE_FOLLOWER_H

#include <Arduino.h>
#include "IRArray.h"

/**
 * PID line follower for the bridge (black line on white background).
 * Drives the motors through MotorDriver's setMotors(), in duty percent.
 */
class LineFollower {
private:
    IRArray& irSensors;

    int basePwm;    // duty %
    int maxPwm;     // duty %

    float kp, ki, kd;
    int lastError;
    float integral;

    const int* whiteVals;
    const int* blackVals;

public:
    explicit LineFollower(IRArray& irRef);

    void begin(const int* white, const int* black);
    void setPIDGains(float p, float i, float d);
    void setSpeeds(int baseSpeed, int maxSpeed);

    int getNormalizedPosition();
    void update(); // Runs one iteration of PID calculation and motor control
    void stop();
};

#endif // LINE_FOLLOWER_H
