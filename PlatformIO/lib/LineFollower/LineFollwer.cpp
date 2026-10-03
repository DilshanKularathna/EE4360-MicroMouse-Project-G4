#include "LineFollower.h"
#include "config.h"

LineFollower::LineFollower(MotorDriver& motorRef, IRArray& irRef)
    : motors(motorRef), irSensors(irRef),
      basePwm(DEFAULT_BASE_PWM), maxPwm(DEFAULT_MAX_PWM),
      kp(DEFAULT_KP), ki(DEFAULT_KI), kd(DEFAULT_KD),
      lastError(0), integral(0), whiteVals(nullptr), blackVals(nullptr) {}

void LineFollower::begin(const int* white, const int* black) {
    whiteVals = white;
    blackVals = black;
}

void LineFollower::setPIDGains(float p, float i, float d) {
    kp = p;
    ki = i;
    kd = d;
}

void LineFollower::setSpeeds(int baseSpeed, int maxSpeed) {
    basePwm = baseSpeed;
    maxPwm = maxSpeed;
}

int LineFollower::getNormalizedPosition() {
    int raw[8];
    irSensors.readRaw(raw);

    long weightedSum = 0;
    long totalValue = 0;

    for (int i = 0; i < 8; i++) {
        int clamped = constrain(raw[i], whiteVals[i], blackVals[i]);
        int normalized = map(clamped, whiteVals[i], blackVals[i], 0, 1000);

        weightedSum += (long)normalized * (i * 1000);
        totalValue += normalized;
    }

    // Line lost detection
    if (totalValue < 200) {
        return (lastError < 0) ? 0 : 7000;
    }

    return weightedSum / totalValue;
}

void LineFollower::update() {
    int position = getNormalizedPosition();

    // Setpoint is center (3500)
    int error = position - 3500;

    integral += error;
    integral = constrain(integral, -3000, 3000);

    int derivative = error - lastError;
    lastError = error;

    // Steering Adjustment
    float adjustment = (kp * error) + (ki * integral) + (kd * derivative);

    int leftMotorSpeed = constrain((int)(basePwm - adjustment), -maxPwm, maxPwm);
    int rightMotorSpeed = constrain((int)(basePwm + adjustment), -maxPwm, maxPwm);

    motors.setSpeeds(leftMotorSpeed, rightMotorSpeed);
}