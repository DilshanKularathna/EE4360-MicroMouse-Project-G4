#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>

// --- Motor Driver Pins (e.g., L298N / TB6612FNG) ---
#define PIN_MOTOR_LEFT_PWM    2
#define PIN_MOTOR_LEFT_DIR1   3
#define PIN_MOTOR_LEFT_DIR2   4

#define PIN_MOTOR_RIGHT_PWM   5
#define PIN_MOTOR_RIGHT_DIR1  6
#define PIN_MOTOR_RIGHT_DIR2  7

// --- Ultrasonic Sensor Pins ---
#define PIN_ULTRA_TRIG        8
#define PIN_ULTRA_ECHO        9

// --- IR Sensor Array Pins ---
const uint8_t IR_PINS[] = {A0, A1, A2, A3, A4};
const uint8_t NUM_IR_SENSORS = 5;

#endif // CONFIG_H