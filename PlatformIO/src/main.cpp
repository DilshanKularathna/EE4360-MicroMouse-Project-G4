#include <Arduino.h>
#include "config.h"
#include "MotorDriver.h"
#include "IRArray.h"
#include "LineFollower.h"

MotorDriver motors(
    PIN_LEFT_RPWM, PIN_LEFT_LPWM,
    PIN_RIGHT_RPWM, PIN_RIGHT_LPWM,
    PIN_MOTOR_EN,
    PIN_LEFT_ENC_A, PIN_LEFT_ENC_B,
    PIN_RIGHT_ENC_A, PIN_RIGHT_ENC_B
);

IRArray irSensors(IR_PINS, PIN_IR_EN, NUM_IR_SENSORS);
LineFollower lineFollower(motors, irSensors);

void setup() {
    Serial.begin(115200);

    motors.begin();
    irSensors.begin();
    lineFollower.begin(WHITE_VALUES, BLACK_VALUES);

    delay(2000);
}

void loop() {
    lineFollower.update();
    delay(5);
}