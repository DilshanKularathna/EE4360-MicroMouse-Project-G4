#include <Arduino.h>
#include "config.h"

void setup()
{
    pinMode(PIN_RIGHT_RPWM, OUTPUT);
    pinMode(PIN_RIGHT_LPWM, OUTPUT);

    pinMode(PIN_LEFT_RPWM, OUTPUT);
    pinMode(PIN_LEFT_LPWM, OUTPUT);

    // Move both motors forward
    analogWrite(PIN_RIGHT_RPWM, DEFAULT_BASE_PWM);
    analogWrite(PIN_RIGHT_LPWM, 0);

    analogWrite(PIN_LEFT_RPWM, DEFAULT_BASE_PWM);
    analogWrite(PIN_LEFT_LPWM, 0);
}

void loop()
{
    // Motors keep running forward
}