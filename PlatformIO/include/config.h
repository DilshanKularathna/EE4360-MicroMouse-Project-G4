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

// Calibrated Values (From line_follower_test.ino)
const int WHITE_VALUES[8] = {288, 158, 195, 324, 382, 516, 492, 460};
const int BLACK_VALUES[8] = {1023, 1020, 1021, 1021, 1017, 1023, 1018, 1021};

// Default PID Parameters
#define DEFAULT_BASE_PWM    30
#define DEFAULT_MAX_PWM     120
#define DEFAULT_KP          0.030f
#define DEFAULT_KI          0.000f
#define DEFAULT_KD          0.100f

// --- Ultrasonic Sensor Pins ---
#define PIN_ULTRA_TRIG       12
#define PIN_ULTRA_ECHO       13

#endif // CONFIG_H