#include <Arduino.h>
#include "config.h"
#include "MotorDriver.h"

#define TEST_FREQ     20000

void setup()
{
    pinMode(PIN_RIGHT_RPWM, OUTPUT);
    pinMode(PIN_RIGHT_LPWM, OUTPUT);

    pinMode(PIN_LEFT_RPWM, OUTPUT);
    pinMode(PIN_LEFT_LPWM, OUTPUT);
}

void loop()
{
    // Motors keep
    moveDistanceCM(50.0f, 30);
    delay(2000);
}