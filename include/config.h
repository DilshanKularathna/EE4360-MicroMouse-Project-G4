#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>

// --- Motor Driver Pins (IBT-2 / BTS7960 Style Drivers) ---
// Enable pins (Both Left and Right drivers enabled via Pin 7)
#define PIN_MOTOR_EN         7

// Left Motor Driver Pins
#define PIN_LEFT_RPWM        4
#define PIN_LEFT_LPWM        6

// Right Motor Driver Pins
#define PIN_RIGHT_RPWM       9
#define PIN_RIGHT_LPWM       8

// --- Encoder Pins (Arduino Mega 2560 Hardware Interrupts) ---
#define PIN_LEFT_ENC_A       2  // Hardware Interrupt 0
#define PIN_LEFT_ENC_B       3  // Hardware Interrupt 1
#define PIN_RIGHT_ENC_A      18 // Hardware Interrupt 5
#define PIN_RIGHT_ENC_B      19 // Hardware Interrupt 4

// --- Movement & Calibration Constants ---
#define BASE_PWM             70
#define TICKS_PER_CELL_25CM  288.0f // 288 ticks = 25 cm cell
#define TICKS_PER_90_DEG     116.5f // ~116-117 ticks for 90° pivot turn
#define TICKS_PER_180_DEG    232.5f // ~232-233 ticks for 180° U-turn

#endif // CONFIG_H