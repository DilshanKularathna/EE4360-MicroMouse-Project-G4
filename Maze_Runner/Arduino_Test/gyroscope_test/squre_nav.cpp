#include <Arduino.h>
#include <math.h>
#include <Wire.h>
#include <util/atomic.h>
#include "config.h"
#include "Gyroscope.h"

// MPU6050 is mounted upside down; the wrapper reports left turns as positive.
Gyroscope gyroscope(Wire, true);

volatile long leftEncoderTicks = 0;
volatile long rightEncoderTicks = 0;

static const float MAX_TILT_FROM_START_DEG = 30.0f;
static const unsigned long GYRO_SAMPLE_INTERVAL_US = 10000UL;

void leftEncoderISR() {
    if (digitalRead(PIN_LEFT_ENC_B) == HIGH) --leftEncoderTicks;
    else ++leftEncoderTicks;
}

void rightEncoderISR() {
    if (digitalRead(PIN_RIGHT_ENC_B) == HIGH) ++rightEncoderTicks;
    else --rightEncoderTicks;
}

void readEncoderTicks(long &left, long &right) {
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        left = leftEncoderTicks;
        right = rightEncoderTicks;
    }
}

void resetEncoderTicks() {
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        leftEncoderTicks = 0;
        rightEncoderTicks = 0;
    }
}

void setWheelSpeeds(int leftPercent, int rightPercent) {
    leftPercent = constrain(leftPercent, -DEFAULT_MAX_PWM, DEFAULT_MAX_PWM);
    rightPercent = constrain(rightPercent, -DEFAULT_MAX_PWM, DEFAULT_MAX_PWM);

    const uint8_t leftPwm = (uint8_t)(abs(leftPercent) * 255L / 100L);
    const uint8_t rightPwm = (uint8_t)(abs(rightPercent) * 255L / 100L);
    analogWrite(PIN_LEFT_RPWM, leftPercent > 0 ? leftPwm : 0);
    analogWrite(PIN_LEFT_LPWM, leftPercent < 0 ? leftPwm : 0);
    analogWrite(PIN_RIGHT_RPWM, rightPercent > 0 ? rightPwm : 0);
    analogWrite(PIN_RIGHT_LPWM, rightPercent < 0 ? rightPwm : 0);
}

void stopMotors() {
    analogWrite(PIN_LEFT_RPWM, 0);
    analogWrite(PIN_LEFT_LPWM, 0);
    analogWrite(PIN_RIGHT_RPWM, 0);
    analogWrite(PIN_RIGHT_LPWM, 0);
}

bool tiltIsSafe(float startTiltX, float startTiltY) {
    const float tiltX = gyroscope.getAccelAngleXDeg() - startTiltX;
    const float tiltY = gyroscope.getAccelAngleYDeg() - startTiltY;
    return fabs(tiltX) <= MAX_TILT_FROM_START_DEG &&
           fabs(tiltY) <= MAX_TILT_FROM_START_DEG;
}

bool driveOneSide(uint8_t side, float startTiltX, float startTiltY) {
    resetEncoderTicks();
    gyroscope.update();
    const float headingTarget = gyroscope.getYawAngleDeg();
    const long targetTicks = (long)(TICKS_PER_CELL_25CM + 0.5f);
    const unsigned long startMs = millis();
    unsigned long nextSampleUs = micros();
    long leftTicks = 0;
    long rightTicks = 0;
    bool reached = false;
    bool tiltFault = false;

    while (millis() - startMs < STRAIGHT_TEST_TIMEOUT_MS) {
        if ((long)(micros() - nextSampleUs) < 0) continue;
        nextSampleUs += GYRO_SAMPLE_INTERVAL_US;

        gyroscope.update();
        if (!tiltIsSafe(startTiltX, startTiltY)) {
            tiltFault = true;
            break;
        }

        readEncoderTicks(leftTicks, rightTicks);
        const float averageTicks = (leftTicks + rightTicks) * 0.5f;
        if (averageTicks >= targetTicks) {
            reached = true;
            break;
        }

        const float headingError = gyroscope.getYawAngleDeg() - headingTarget;
        const float correction = constrain(
            headingError * STRAIGHT_TEST_GYRO_KP,
            -STRAIGHT_TEST_MAX_CORRECTION,
            STRAIGHT_TEST_MAX_CORRECTION);
        const int leftSpeed = (int)lroundf(
            DEFAULT_BASE_PWM + correction + STRAIGHT_TEST_LEFT_TRIM);
        const int rightSpeed = (int)lroundf(
            DEFAULT_BASE_PWM - correction + STRAIGHT_TEST_RIGHT_TRIM);
        setWheelSpeeds(leftSpeed, rightSpeed);
    }

    stopMotors();
    delay(200);
    gyroscope.update();
    readEncoderTicks(leftTicks, rightTicks);

    Serial.print(F("side,"));
    Serial.print(side + 1);
    Serial.print(F(",status,"));
    Serial.print(tiltFault ? F("TILT_ABORT") : (reached ? F("PASS") : F("TIMEOUT")));
    Serial.print(F(",target_ticks,"));
    Serial.print(targetTicks);
    Serial.print(F(",left_ticks,"));
    Serial.print(leftTicks);
    Serial.print(F(",right_ticks,"));
    Serial.print(rightTicks);
    Serial.print(F(",heading_error_deg,"));
    Serial.println(gyroscope.getYawAngleDeg() - headingTarget, 2);
    return reached && !tiltFault;
}

bool turnLeft90(uint8_t turnNumber, float startTiltX, float startTiltY) {
    resetEncoderTicks();
    gyroscope.update();
    const float startAngle = gyroscope.getYawAngleDeg();
    const float targetDegrees = 90.0f + TURN_LEFT_CORRECTION_DEG;
    const unsigned long startMs = millis();
    unsigned long nextSampleUs = micros();
    float turnedDegrees = 0.0f;
    bool tiltFault = false;

    while (fabs(targetDegrees - turnedDegrees) > TURN_STOP_TOLERANCE_DEG &&
           millis() - startMs < TURN_90_TIMEOUT_MS) {
        if ((long)(micros() - nextSampleUs) < 0) continue;
        nextSampleUs += GYRO_SAMPLE_INTERVAL_US;

        gyroscope.update();
        if (!tiltIsSafe(startTiltX, startTiltY)) {
            tiltFault = true;
            break;
        }
        turnedDegrees = gyroscope.getYawAngleDeg() - startAngle;
        const float error = targetDegrees - turnedDegrees;
        const float absError = fabs(error);
        int duty = TURN_SPEED;

        if (absError < TURN_SLOWDOWN_START_ERROR_DEG) {
            const float range = TURN_SLOWDOWN_START_ERROR_DEG -
                                TURN_STOP_TOLERANCE_DEG;
            const float progress = constrain(
                (absError - TURN_STOP_TOLERANCE_DEG) / range, 0.0f, 1.0f);
            duty = (int)lroundf(
                TURN_MIN_DUTY + progress * (TURN_SPEED - TURN_MIN_DUTY));
        }

        const int reverseDuty = (int)lroundf(
            duty * constrain(TURN_REVERSE_SPEED_RATIO, 0.0f, 1.0f));
        // With this motor wiring, left wheel reverse + right wheel forward is left.
        setWheelSpeeds(-reverseDuty, duty);
    }

    stopMotors();
    delay(250);
    gyroscope.update();
    turnedDegrees = gyroscope.getYawAngleDeg() - startAngle;

    long leftTicks = 0;
    long rightTicks = 0;
    readEncoderTicks(leftTicks, rightTicks);
    const bool reached = !tiltFault &&
        fabs(targetDegrees - turnedDegrees) <= TURN_STOP_TOLERANCE_DEG;

    Serial.print(F("turn,left,"));
    Serial.print(turnNumber);
    Serial.print(F(",target_deg,"));
    Serial.print(targetDegrees, 1);
    Serial.print(F(",measured_deg,"));
    Serial.print(turnedDegrees, 2);
    Serial.print(F(",error_from_90_deg,"));
    Serial.print(turnedDegrees - 90.0f, 2);
    Serial.print(F(",left_ticks,"));
    Serial.print(leftTicks);
    Serial.print(F(",right_ticks,"));
    Serial.print(rightTicks);
    Serial.print(F(",status,"));
    Serial.println(tiltFault ? F("TILT_ABORT") : (reached ? F("PASS") : F("TIMEOUT/MISS")));
    return reached;
}

void runSquare() {
    const float startYaw = gyroscope.getYawAngleDeg();
    const float startTiltX = gyroscope.getAccelAngleXDeg();
    const float startTiltY = gyroscope.getAccelAngleYDeg();
    bool complete = true;

    Serial.println(F("Starting 25 cm square: four gyro-corrected straights and left turns."));
    Serial.println(F("Starting in 3 seconds; put the robot on a clear, level surface."));
    delay(3000);
    Serial.println(F("Per-side and per-turn measurements:"));

    for (uint8_t side = 0; side < 4; ++side) {
        if (!driveOneSide(side, startTiltX, startTiltY)) {
            complete = false;
            break;
        }
        if (!turnLeft90(side + 1, startTiltX, startTiltY)) {
            complete = false;
            break;
        }
    }

    stopMotors();
    gyroscope.update();
    const float finalYaw = gyroscope.getYawAngleDeg();

    Serial.println(F("Square result:"));
    Serial.print(F("status,"));
    Serial.println(complete ? F("COMPLETE") : F("INCOMPLETE"));
    Serial.print(F("target_ticks_per_side,"));
    Serial.println((long)(TICKS_PER_CELL_25CM + 0.5f));
    Serial.print(F("final_heading_change_deg,"));
    Serial.println(finalYaw - startYaw, 2);
    Serial.println(F("Accelerometer tilt guard compares X/Y tilt with the calibrated start angle."));
}

void setup() {
    Serial.begin(115200);
    delay(1000);

    pinMode(PIN_RIGHT_RPWM, OUTPUT);
    pinMode(PIN_RIGHT_LPWM, OUTPUT);
    pinMode(PIN_LEFT_RPWM, OUTPUT);
    pinMode(PIN_LEFT_LPWM, OUTPUT);
    pinMode(PIN_LEFT_ENC_A, INPUT_PULLUP);
    pinMode(PIN_LEFT_ENC_B, INPUT_PULLUP);
    pinMode(PIN_RIGHT_ENC_A, INPUT_PULLUP);
    pinMode(PIN_RIGHT_ENC_B, INPUT_PULLUP);
    stopMotors();

    attachInterrupt(digitalPinToInterrupt(PIN_LEFT_ENC_A), leftEncoderISR, RISING);
    attachInterrupt(digitalPinToInterrupt(PIN_RIGHT_ENC_A), rightEncoderISR, RISING);

    Serial.println(F("Keep the robot stationary and level during MPU6050 calibration."));
    if (!gyroscope.begin(0, 0)) {
        Serial.println(F("MPU6050 initialization failed; motors remain stopped."));
        while (true) delay(1000);
    }
    delay(1000);
    if (!gyroscope.calibrate()) {
        Serial.println(F("Gyro calibration failed; motors remain stopped."));
        while (true) delay(1000);
    }
    if (!gyroscope.calibrateAccelerometer()) {
        Serial.println(F("Accelerometer calibration failed; motors remain stopped."));
        while (true) delay(1000);
    }

    runSquare();
}

void loop() {
    // The square runs once after reset.
}
