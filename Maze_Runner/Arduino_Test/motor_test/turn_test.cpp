// Turn test: gyro closed-loop 90 degree turn using the library turnRight()/turnLeft().
// Set TURN_RIGHT to 1 for a right turn, 0 for a left turn.

#include <Arduino.h>
#include "config.h"
#include "MotorDriver.h"
#include "globals.h"
#include "Gyro.h"

#define TURN_RIGHT 0
#define TURN_180   1   // 1 = one continuous 180 degree turn, 0 = 90 degrees
#define PLOT       0   // 1 = print Teleplot lines (>yaw, >error, >duty) during the turn

void plotHook(float yaw, float err, int duty) {
    static unsigned long last = 0;
    if (millis() - last < 20) return;   // 50 lines/s so printing does not slow the loop
    last = millis();
    Serial.print(">yaw:");   Serial.println(yaw, 1);
    Serial.print(">error:"); Serial.println(err, 1);
    Serial.print(">duty:");  Serial.println(duty);
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

    if (!gyroBegin()) Serial.println("MPU-6050 NOT FOUND: using encoder turn");
    Serial.println("Keep the robot still. Calibrating gyro...");
    if (!gyroCalibrate()) Serial.println("Warning: robot moved during calibration");

    Serial.println("Mark the robot's heading with tape. Starting in 3 s...");
    delay(3000);

    if (PLOT) turnLogHook = plotHook;

    unsigned long t = millis();
    float angle = (TURN_180 ? 180.0f : 90.0f) * (TURN_RIGHT ? 1.0f : -1.0f);
    turnDegrees(angle);
    t = millis() - t;

    unsigned long settle = millis();
    while (millis() - settle < 300) gyroUpdate();   // catch any coasting
    float total = gyroYaw();

    Serial.print("Counts  L = ");
    Serial.print(leftEncoderCount);
    Serial.print("   R = ");
    Serial.println(rightEncoderCount);
    Serial.print("Turn time = ");
    Serial.print(t);
    Serial.println(" ms");
    Serial.print("Total angle = ");
    Serial.print(total, 1);
    Serial.println(TURN_180 ? " deg (expected +-180)" : " deg (expected +-90)");
}

void loop() {}
