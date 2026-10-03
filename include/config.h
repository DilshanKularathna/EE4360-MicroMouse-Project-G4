#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>

// --- Motor Driver Pins ---
#define PIN_MOTOR_EN         7
#define PIN_LEFT_RPWM        4
#define PIN_LEFT_LPWM        6
#define PIN_RIGHT_RPWM       9
#define PIN_RIGHT_LPWM       8

#define PIN_LEFT_ENC_A       2
#define PIN_LEFT_ENC_B       3
#define PIN_RIGHT_ENC_A      18
#define PIN_RIGHT_ENC_B      19

// --- IR Sensor Array Pins ---
#define PIN_IR_EN            49
const uint8_t IR_PINS[8] = {A15, A14, A13, A12, A11, A10, A9, A8};
#define NUM_IR_SENSORS       8

// Default Thresholds (Updated via calibration utility)
const int DEFAULT_IR_THRESHOLDS[8] = {500, 500, 500, 500, 500, 500, 500, 500};

#endif // CONFIG_H