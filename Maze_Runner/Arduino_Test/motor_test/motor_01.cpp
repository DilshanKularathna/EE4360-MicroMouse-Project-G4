#include <Arduino.h>
#include "config.h"
#include "MotorDriver.h"

void setup()
{
    Serial.begin(115200);

    // Pin setups using your config.h definitions
    pinMode(PIN_RIGHT_RPWM, OUTPUT);
    pinMode(PIN_RIGHT_LPWM, OUTPUT);
    pinMode(PIN_LEFT_RPWM, OUTPUT);
    pinMode(PIN_LEFT_LPWM, OUTPUT);

    pinMode(PIN_LEFT_ENC_A, INPUT_PULLUP);
    pinMode(PIN_LEFT_ENC_B, INPUT_PULLUP);
    pinMode(PIN_RIGHT_ENC_A, INPUT_PULLUP);
    pinMode(PIN_RIGHT_ENC_B, INPUT_PULLUP);

    // Attach hardware interrupts for quadrature encoders
    attachInterrupt(digitalPinToInterrupt(PIN_LEFT_ENC_A), leftEncoderISR, RISING);
    attachInterrupt(digitalPinToInterrupt(PIN_RIGHT_ENC_A), rightEncoderISR, RISING);

    Serial.println("--- 90-Degree Turn Test ---");
    Serial.println("Starting in 3 seconds...");
    delay(3000);

    // Turn 90 degrees right (in-place) and stop
    Serial.println("Turning 90 degrees Right...");
    resetEncoders(); // Reset encoder counts before the turn
    turnDegrees(-90.0f);
    resetEncoders(); // Reset encoder counts after the turn
    delay(2000);
    setMotors(30, 30);
    stopMotors();
    resetEncoders(); // Reset encoder counts before the next turn
    delay(2000);
    turnDegrees(180.0f);
    // Wait for a second before the next turn

    Serial.println("Turn complete. Motors stopped.");
}

void loop() {
    // Keep loop empty so the robot turns only once during setup
}