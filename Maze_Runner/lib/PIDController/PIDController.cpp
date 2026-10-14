#include "PIDController.h"
#include "config.h"

PIDController::PIDController(float kp, float ki, float kd) {
    _kp = kp;
    _ki = ki;
    _kd = kd;
    
    _lastError = 0;
    _integral = 0;
    _integralMin = -3000.0;
    _integralMax = 3000.0;

    _useBatteryCorrection = false;
    _batteryPin = -1;
    _vMax = 12.6;
    _criticalLowV = 9.9;
    _r1 = 10000.0;
    _r2 = 4700.0;
    _vRef = 5.0;
}

void PIDController::setupBattery(int pin, float vMax, float criticalLowV, float r1, float r2, float vRef) {
    _batteryPin = pin;
    _vMax = vMax;
    _criticalLowV = criticalLowV;
    _r1 = r1;
    _r2 = r2;
    _vRef = vRef;
    
    if (_batteryPin >= 0) {
        pinMode(_batteryPin, INPUT);
    }
}

void PIDController::setBatteryCorrection(bool enable) {
    _useBatteryCorrection = enable;
}

void PIDController::setIntegralLimits(float minVal, float maxVal) {
    _integralMin = minVal;
    _integralMax = maxVal;
}

float PIDController::readBatteryVoltage() {
    if (_batteryPin < 0) return _vMax; // Return max default if pin is unconfigured

    int rawADC = analogRead(_batteryPin);
    float pinVoltage = (rawADC * _vRef) / 1023.0;
    float dividerFactor = (_r1 + _r2) / _r2;
    return pinVoltage * dividerFactor;
}

bool PIDController::isBatteryCritical() {
    if (_batteryPin < 0) return false; // Safety bypass if hardware is unconfigured
    return (readBatteryVoltage() < _criticalLowV);
}

float PIDController::computeAdjustment(int error) {
    _integral += error;
    _integral = constrain(_integral, _integralMin, _integralMax);

    int derivative = error - _lastError;
    _lastError = error;

    return (_kp * error) + (_ki * _integral) + (_kd * derivative);
}

int PIDController::scalePWM(int baseSpeed) {
    // If running a stable LiPo or correction is disabled, skip scaling
    if (!_useBatteryCorrection || _batteryPin < 0) {
        return baseSpeed;
    }

    float vCurrent = readBatteryVoltage();
    
    // Safety guard to avoid division by zero or massive spikes if a wire pops out
    if (vCurrent < 5.0) {
        vCurrent = 5.0;
    }

    float compensationRatio = _vMax / vCurrent;
    return (int)(baseSpeed * compensationRatio);
}

void PIDController::reset() {
    _integral = 0;
    _lastError = 0;
}
