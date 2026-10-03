#include <Arduino.h>
#include "config.h"
#include "MotorDriver.h"

MotorDriver motors(
    PIN_LEFT_RPWM, PIN_LEFT_LPWM,
    PIN_RIGHT_RPWM, PIN_RIGHT_LPWM,
    PIN_MOTOR_EN,
    PIN_LEFT_ENC_A, PIN_LEFT_ENC_B,
    PIN_RIGHT_ENC_A, PIN_RIGHT_ENC_B
);

void setup() {
    Serial.begin(115200);
    motors.begin();
    delay(2000);
}

void loop() {
    Serial.println("Moving 1 Cell Forward (25 cm)...");
    motors.moveCells(1.0);

    delay(500);

    Serial.println("Turning Right 90 Degrees...");
    motors.turnRight90();

    delay(500);

    Serial.println("Moving 1 Cell Forward (25 cm)...");
    motors.moveCells(1.0);

    delay(500);

    Serial.println("Performing 180 Degree U-Turn...");
    motors.turn180();

    motors.stop();
    Serial.println("Sequence Finished!");
    while (1);
}