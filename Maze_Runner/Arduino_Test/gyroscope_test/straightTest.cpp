#include <Arduino.h>
#include <math.h>
#include <Wire.h>
#include <util/atomic.h>
#include "config.h"
#include "Gyroscope.h"

Gyroscope gyroscope(Wire, true);

volatile long leftEncoderTicks = 0;
volatile long rightEncoderTicks = 0;

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
    leftPercent = constrain(leftPercent, 0, DEFAULT_MAX_PWM);
    rightPercent = constrain(rightPercent, 0, DEFAULT_MAX_PWM);

    analogWrite(PIN_LEFT_RPWM, (uint8_t)(leftPercent * 255L / 100L));
    analogWrite(PIN_LEFT_LPWM, 0);
    analogWrite(PIN_RIGHT_RPWM, (uint8_t)(rightPercent * 255L / 100L));
    analogWrite(PIN_RIGHT_LPWM, 0);
}

void stopMotors() {
    analogWrite(PIN_LEFT_RPWM, 0);
    analogWrite(PIN_LEFT_LPWM, 0);
    analogWrite(PIN_RIGHT_RPWM, 0);
    analogWrite(PIN_RIGHT_LPWM, 0);
}

void runStraightTest() {
    resetEncoderTicks();
    gyroscope.update();
    const float startHeading = gyroscope.getYawAngleDeg();
    const long targetTicks = (long)(TICKS_PER_CELL_25CM + 0.5f);
    long leftTicks = 0;
    long rightTicks = 0;
    float headingError = 0.0f;

    Serial.println(F("Straight gyro test starts in 3 seconds."));
    Serial.print(F("Target ticks per wheel average: "));
    Serial.println(targetTicks);
    Serial.println(F("Place the robot on a clear, straight path."));
    delay(3000);

    const unsigned long driveStartMs = millis();
    unsigned long nextUpdateUs = micros();
    bool reachedTarget = false;

    while (millis() - driveStartMs < STRAIGHT_TEST_TIMEOUT_MS) {
        if ((long)(micros() - nextUpdateUs) < 0) continue;
        nextUpdateUs += 10000UL;

        gyroscope.update();
        headingError = gyroscope.getYawAngleDeg() - startHeading;

        readEncoderTicks(leftTicks, rightTicks);
        const float averageTicks = (leftTicks + rightTicks) * 0.5f;
        if (averageTicks >= targetTicks) {
            reachedTarget = true;
            break;
        }

        // Positive yaw is leftward. Speed up the left wheel and slow the right
        // wheel to steer back right; reverse that correction for rightward drift.
        const float gyroCorrection = constrain(
            headingError * STRAIGHT_TEST_GYRO_KP,
            -STRAIGHT_TEST_MAX_CORRECTION,
            STRAIGHT_TEST_MAX_CORRECTION);

        const int leftSpeed = (int)lroundf(
            DEFAULT_BASE_PWM + gyroCorrection + STRAIGHT_TEST_LEFT_TRIM);
        const int rightSpeed = (int)lroundf(
            DEFAULT_BASE_PWM - gyroCorrection + STRAIGHT_TEST_RIGHT_TRIM);
        setWheelSpeeds(leftSpeed, rightSpeed);
    }

    stopMotors();
    delay(200);
    gyroscope.update();
    headingError = gyroscope.getYawAngleDeg() - startHeading;
    readEncoderTicks(leftTicks, rightTicks);

    Serial.println(F("Straight test result:"));
    Serial.println(F("status,target_ticks,left_ticks,right_ticks,average_ticks,heading_deviation_deg"));
    Serial.print(reachedTarget ? F("PASS,") : F("TIMEOUT,"));
    Serial.print(targetTicks);
    Serial.print(',');
    Serial.print(leftTicks);
    Serial.print(',');
    Serial.print(rightTicks);
    Serial.print(',');
    Serial.print((leftTicks + rightTicks) * 0.5f, 1);
    Serial.print(',');
    Serial.println(headingError, 2);
    Serial.println(F("Edit STRAIGHT_TEST_LEFT_TRIM / STRAIGHT_TEST_RIGHT_TRIM in config.h for wheel bias."));
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

    Serial.println(F("Keep robot still while the MPU6050 initializes and calibrates."));
    if (!gyroscope.begin(0, 0)) {
        Serial.println(F("MPU6050 initialization failed; motors remain stopped."));
        while (true) delay(1000);
    }
    delay(1000);
    Serial.println(F("Calibrating gyro; do not move the robot."));
    if (!gyroscope.calibrate()) {
        Serial.println(F("Gyro calibration failed; motors remain stopped."));
        while (true) delay(1000);
    }

    runStraightTest();
}

void loop() {
    // Run once per reset.
}
