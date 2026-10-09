#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>

// --- Motor Driver Pins ---
//#define PIN_MOTOR_EN         7
#define PIN_RIGHT_RPWM        5 //4
#define PIN_RIGHT_LPWM        6
#define PIN_LEFT_RPWM       7 //9 //physical wiring color changed 
#define PIN_LEFT_LPWM       8

#define PIN_RIGHT_ENC_A       2
#define PIN_RIGHT_ENC_B       3
#define PIN_LEFT_ENC_A      18
#define PIN_LEFT_ENC_B      19

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
#define DEFAULT_BASE_PWM    100
#define DEFAULT_MAX_PWM     120
#define DEFAULT_KP          0.030f
#define DEFAULT_KI          0.000f
#define DEFAULT_KD          0.100f

// Calibrated Calibration Values
// 288 ticks = 25 cm (1 cell) -> 11.52 ticks/cm
const float TICKS_PER_CELL_25CM = 288.0;
const float TICKS_PER_90_DEG = 116.5;  // Average of 116-117 ticks
const float TICKS_PER_180_DEG = 232.5; // Average of 232-233 ticks

#define WHEEL_DIAMETER_CM 6.0f
#define WHEELBASE_CM 15.5f
#define ENCODER_COUNTS_PER_REV 360
static const float CM_PER_COUNT           = (3.14159f * WHEEL_DIAMETER_CM / ENCODER_COUNTS_PER_REV);
#define TURN_SPEED 25

#define WALL_THRESHOLD 20.0f
#define FRONT_OBSTACLE WALL_THRESHOLD
#define CENTER_DIST 5.0f
#define SAFETY_DIST 7.5f

// --- Ultrasonic Sensor Pins ---
#define PIN_ULTRA_FRONT_TRIG       24 // Black wire - Echo, Blue - Trig
#define PIN_ULTRA_FRONT_ECHO       31
#define PIN_ULTRA_LEFT_TRIG       22
#define PIN_ULTRA_LEFT_ECHO       35
#define PIN_ULTRA_RIGHT_TRIG       26
#define PIN_ULTRA_RIGHT_ECHO       33

#define NORTH 0
#define EAST 1
#define SOUTH 2
#define WEST 3

#endif // CONFIG_H