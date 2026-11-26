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

// Motor and encoder calibration (4x decoding)
#define ENCODER_COUNTS_PER_REV   897
#define WHEEL_DIAMETER_CM        6.43f
#define WHEELBASE_CM             18.5f        // centre to centre of the wheels (measured)
const float TICKS_PER_CELL_25CM  = 1111.0f;
#define RIGHT_DUTY_TRIM          0.9768f
#define MIN_DUTY_LEFT            16
#define MIN_DUTY_RIGHT           13
const float TICKS_PER_90_DEG     = 478.0f;    // right turn
const float TICKS_PER_90_LEFT    = 526.0f;    // left turn

// Gyro scale: 5 full turns read 1747 deg instead of 1800 -> multiply by 1800/1747
#define GYRO_SCALE               1.030f

// Gyro closed-loop turn
#define TURN_KP                  1.2f         // duty % per degree of error (autotuned)
#define TURN_MIN_DUTY            22           // lowest duty that still turns the robot
#define TURN_TOLERANCE_DEG       1.0f
#define TURN_BALANCE_GAIN        0.1f         // duty % per tick of left/right difference
const float TICKS_PER_180_DEG    = 956.0f;    // provisional: 2 x 90 deg, not measured yet

static const float CM_PER_COUNT = (3.14159f * WHEEL_DIAMETER_CM / ENCODER_COUNTS_PER_REV);
// #define TURN_SPEED 20

#define WALL_THRESHOLD 9.0f      // side wall present if below this (walls read 0-8; 10-15 = posts/far walls)
#define FRONT_WALL_CM 15.0f      // front wall in this cell if F is below this (centred: 4.3, next cell: ~29)
#define FRONT_OBSTACLE FRONT_WALL_CM
#define WALL_CENTER_CM 3.2f      // side reading when centred in a cell (measured)
#define FRONT_CENTER_CM 2.3f     // front reading with the AXLE on the cell centre (measured)
#define FRONT_MARGIN_CM 1.0f     // extra gap kept from a front wall (0 = axle exactly on the centre)
#define FRONT_TARGET_CM (FRONT_CENTER_CM + FRONT_MARGIN_CM)
#define FRONT_SLOW_CM 2.0f       // a move stops this far before the target, then creeps in slowly

// Robot geometry (measured)
#define CELL_SIZE_CM 25.0f
#define SIDE_SENSOR_AHEAD_CM 5.3f   // side sensors are this far in front of the wheel axle
#define EDGE_TO_CENTER_CM (CELL_SIZE_CM / 2.0f + SIDE_SENSOR_AHEAD_CM)   // drive after a side wall starts/ends
#define TICKS_PER_CM (TICKS_PER_CELL_25CM / CELL_SIZE_CM)
#define EDGE_CORRECTION 0           // 1 = finish moves using side-wall edges (off: ultrasonic edges are +-5 cm)
#define FRONT_ALIGN_DUTY 20      // duty used to square up on a front wall
#define WALL_STEER_GAIN 2.0f     // duty % per cm of side error (used without the gyro)
#define HEADING_KP 1.5f          // duty % per degree of heading error (gyro, autotuned)
#define WALL_YAW_GAIN 3.0f       // degrees of heading offset per cm of wall error (autotuned)
#define MAX_WALL_YAW 10.0f       // limit on that offset (degrees, autotuned)

// --- Ultrasonic Sensor Pins ---
#define PIN_ULTRA_FRONT_TRIG       24 // Black wire - Echo, Blue - Trig
#define PIN_ULTRA_FRONT_ECHO       31
#define PIN_ULTRA_LEFT_TRIG       22
#define PIN_ULTRA_LEFT_ECHO       35
#define PIN_ULTRA_RIGHT_TRIG       26
#define PIN_ULTRA_RIGHT_ECHO       33

// Ultrasonic correction: real_cm = (raw_cm - offset) / scale
#define ULTRA_FRONT_SCALE          0.928f
#define ULTRA_FRONT_OFFSET         2.34f
#define ULTRA_LEFT_SCALE           0.938f
#define ULTRA_LEFT_OFFSET          4.39f
#define ULTRA_RIGHT_SCALE          0.936f
#define ULTRA_RIGHT_OFFSET         4.72f

#define NORTH 0
#define EAST 1
#define SOUTH 2
#define WEST 3

// --- Battery Monitoring & Safety Settings ---
#define PIN_BATTERY_ADC       A7       // The analog pin connected to your voltage divider
#define BATTERY_V_MAX         12.6f    // Full charge voltage (e.g., 12.6V for 3S, 16.8V for 4S)
#define BATTERY_CRITICAL_LOW  9.9f     // Cutoff voltage to protect cells (e.g., 3.3V per cell minimum)
#define BATTERY_R1            10000.0f // Top resistor in Ohms (10k)
#define BATTERY_R2            4700.0f  // Bottom resistor in Ohms (4.7k)
#define ARDUINO_V_REF         5.0f     // Regulated 5V supplying your Arduino from the buck

#endif // CONFIG_H