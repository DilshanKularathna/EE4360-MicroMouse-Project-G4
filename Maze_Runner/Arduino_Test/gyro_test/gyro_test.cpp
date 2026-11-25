// Gyro test: prints rate and yaw from the Gyro library. Send 'r' to reset yaw.

#include <Arduino.h>
#include "Gyro.h"

void setup() {
    Serial.begin(115200);
    delay(500);

    if (!gyroBegin()) {
        Serial.println("MPU-6050 NOT FOUND. Check SDA(20), SCL(21), VCC, GND.");
        while (1) {}
    }

    Serial.println("Keep the robot still. Calibrating bias...");
    if (!gyroCalibrate()) Serial.println("Warning: robot moved during calibration. Press reset.");
    Serial.println("Ready.");
}

void loop() {
    if (Serial.available() && Serial.read() == 'r') {
        gyroResetYaw();
        Serial.println("Yaw reset");
    }

    gyroUpdate();

    static unsigned long lastPrint = 0;
    if (millis() - lastPrint >= 50) {
        lastPrint = millis();
        Serial.print("rate = ");
        Serial.print(gyroRate(), 2);
        Serial.print(" deg/s   yaw = ");
        Serial.println(gyroYaw(), 2);
    }
}
