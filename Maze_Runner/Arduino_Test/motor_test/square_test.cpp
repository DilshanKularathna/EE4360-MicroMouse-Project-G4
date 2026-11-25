// Square test: drive one cell, turn right, repeat 4 times.
// The robot should end on its start mark.

#include <Arduino.h>
#include "config.h"
#include "MotorDriver.h"
#include "globals.h"
#include "Gyro.h"

#define TEST_DUTY 30

void driveOneCell() {
    int leftDuty  = TEST_DUTY;
    int rightDuty = (int)(TEST_DUTY * RIGHT_DUTY_TRIM + 0.5f);

    resetEncoders();
    unsigned long start = millis();
    setMotors(leftDuty, rightDuty);
    while ((labs(leftEncoderCount) + labs(rightEncoderCount)) / 2 < TICKS_PER_CELL_25CM
           && millis() - start < 8000) {}
    stopMotors();
    delay(500);
}

void setup() {
    Serial.begin(115200);
    delay(500);

    pinMode(PIN_LEFT_RPWM, OUTPUT);
    pinMode(PIN_LEFT_LPWM, OUTPUT);
    pinMode(PIN_RIGHT_RPWM, OUTPUT);
    pinMode(PIN_RIGHT_LPWM, OUTPUT);

    initEncoders();
    stopMotors();

    if (!gyroBegin()) Serial.println("MPU-6050 NOT FOUND: using encoder turns");
    if (!gyroCalibrate()) Serial.println("Warning: robot moved during gyro calibration");

    Serial.println("Place the robot on the start mark. Starting in 3 s...");
    delay(3000);

    for (int i = 0; i < 4; i++) {
        driveOneCell();
        Serial.print("Cell ");
        Serial.print(i + 1);
        Serial.println(" done");
        turnRight();
    }

    Serial.println("Square finished. Measure the distance from the start mark.");
}

void loop() {}
