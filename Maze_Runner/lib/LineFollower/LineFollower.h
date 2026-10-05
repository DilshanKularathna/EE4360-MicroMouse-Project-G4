#ifndef LINE_FOLLOWER_H
#define LINE_FOLLOWER_H

#include <Arduino.h>
#include "MotorDriver.h"
#include "IRArray.h"

class LineFollower {
private:
    MotorDriver& motors;
    IRArray& irSensors;

    int basePwm;
    int maxPwm;
    
    float kp, ki, kd;
    int lastError;
    float integral;

    const int* whiteVals;
    const int* blackVals;

public:
    LineFollower(MotorDriver& motorRef, IRArray& irRef);

    void begin(const int* white, const int* black);
    void setPIDGains(float p, float i, float d);
    void setSpeeds(int baseSpeed, int maxSpeed);

    int getNormalizedPosition();
    void update(); // Runs one iteration of PID calculation and motor control
};

#endif // LINE_FOLLOWER_H