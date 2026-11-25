// Straight test: drive one 25 cm cell using the calibrated trim and ticks.

#include <Arduino.h>
#include "config.h"
#include "MotorDriver.h"
#include "globals.h"

#define TEST_DUTY 30

void setup() {
    Serial.begin(115200);
    delay(500);

    pinMode(PIN_LEFT_RPWM, OUTPUT);
    pinMode(PIN_LEFT_LPWM, OUTPUT);
    pinMode(PIN_RIGHT_RPWM, OUTPUT);
    pinMode(PIN_RIGHT_LPWM, OUTPUT);

    initEncoders();
    stopMotors();

    int leftDuty  = TEST_DUTY;
    int rightDuty = (int)(TEST_DUTY * RIGHT_DUTY_TRIM + 0.5f);

    Serial.println("Place the robot on the start line. Starting in 3 s...");
    delay(3000);

    resetEncoders();
    unsigned long start = millis();
    setMotors(leftDuty, rightDuty);
    while ((labs(leftEncoderCount) + labs(rightEncoderCount)) / 2 < TICKS_PER_CELL_25CM
           && millis() - start < 8000) {}
    stopMotors();

    Serial.print("Counts  L = ");
    Serial.print(leftEncoderCount);
    Serial.print("   R = ");
    Serial.println(rightEncoderCount);
    Serial.println("Measure the distance from the start line. Target: 25 cm.");
}

void loop() {}
