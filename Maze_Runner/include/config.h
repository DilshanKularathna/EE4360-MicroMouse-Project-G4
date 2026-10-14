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
#define DEFAULT_BASE_PWM    15
#define DEFAULT_MAX_PWM     80
#define DEFAULT_KP          0.030f
#define DEFAULT_KI          0.000f
#define DEFAULT_KD          0.100f

// Calibrated Calibration Values
// 288 ticks = 25 cm (1 cell) -> 11.52 ticks/cm
const float TICKS_PER_CELL_25CM = 282.0;
// const float TICKS_PER_90_DEG = 116.5;  // Average of 116-117 ticks
const float TICKS_PER_180_DEG = 232.0; // Average of 232-233 ticks

const float TICKS_PER_90_DEG = 140.0;  // Average of 116-117 ticks
// const float TICKS_PER_180_DEG = 280.2; // Average of 232-233 ticks

#define WHEEL_DIAMETER_CM 6.0f
#define WHEELBASE_CM 18.0f
#define MAZE_CELL_SIZE_CM 25.0f
#define TURN_AXLE_TO_FARTHEST_END_CM 12.6f
#define TURN_CHASSIS_WIDTH_CM 18.0f
#define ENCODER_COUNTS_PER_REV 224 
static const float CM_PER_COUNT           = (3.14159f * WHEEL_DIAMETER_CM / ENCODER_COUNTS_PER_REV);
#define TURN_SPEED 15
// User-tuned upper bound for reverse-side wheel speed. Side-clearance geometry
// may lower the ratio on a turn that needs more center shift toward the opening.
#define TURN_REVERSE_SPEED_RATIO 0.85f
// Extra sweep clearance beyond the geometric minimum.
#define TURN_SIDE_CLEARANCE_MARGIN_CM 0.5f
// Gyro turn controller tuning.
// User-tuned turn calibration values. Preserve these during tuning.
#define TURN_LEFT_CORRECTION_DEG      -3.0f
#define TURN_RIGHT_CORRECTION_DEG     -2.9f
#define USE_TWO_90_TURNS_FOR_180      1
#define TURN_STOP_TOLERANCE_DEG       5.0f
#define TURN_SLOWDOWN_START_ERROR_DEG 20.0f
#define TURN_MIN_DUTY                 15
#define TURN_90_TIMEOUT_MS            5000UL
#define TURN_180_TIMEOUT_MS           5000UL

// Straight-line gyro test tuning. The tick target uses TICKS_PER_CELL_25CM.
// Positive trim increases that wheel's PWM; use small +/- values for tuning.
#define STRAIGHT_TEST_LEFT_TRIM       0
#define STRAIGHT_TEST_RIGHT_TRIM      0
#define STRAIGHT_TEST_GYRO_KP         2.0f
#define STRAIGHT_TEST_MAX_CORRECTION  10.0f
#define STRAIGHT_TEST_TIMEOUT_MS      10000UL

#define WALL_THRESHOLD 20.0f
#define FRONT_OBSTACLE WALL_THRESHOLD
#define CENTER_DIST 4.0f
#define SAFETY_DIST 5.0f

// Optional extra forward travel immediately before a 90-degree turn.
// Positive values move the robot closer to the front of the current cell;
// keep at 0 until a specific maze turn has been measured and tuned.
#define LEFT_TURN_ENTRY_ADVANCE_CM  0.0f
#define RIGHT_TURN_ENTRY_ADVANCE_CM 0.0f
#define TURN_ENTRY_SPEED            18

// --- Ultrasonic Sensor Pins ---
#define PIN_ULTRA_FRONT_TRIG       24 // Black wire - Echo, Blue - Trig
#define PIN_ULTRA_FRONT_ECHO       31
#define PIN_ULTRA_LEFT_TRIG        22
#define PIN_ULTRA_LEFT_ECHO        35
#define PIN_ULTRA_RIGHT_TRIG       26
#define PIN_ULTRA_RIGHT_ECHO       33

// --- Ultrasonic Calibration (Calibrated = Raw * SCALE + OFFSET) ---
#define ULTRA_FRONT_SCALE          1.0f    // Update with your measured scale (m)
#define ULTRA_FRONT_OFFSET         0.0f    // Update with your measured offset (c) in cm
#define ULTRA_LEFT_SCALE           1.136f
#define ULTRA_LEFT_OFFSET          -3.067f
#define ULTRA_RIGHT_SCALE          0.833f
#define ULTRA_RIGHT_OFFSET         0.085f

#define PIN_GYRO_SCL    21  
#define PIN_GYRO_SDA    20

// MPU6050 mounted with +X toward the robot front, +Y to the right, +Z down.
#define MPU6050_FRONT_AXIS_INDEX  0
#define MPU6050_RIGHT_AXIS_INDEX  1
#define MPU6050_DOWN_AXIS_INDEX   2
#define MPU6050_YAW_AXIS_INDEX    MPU6050_DOWN_AXIS_INDEX

#define NORTH 0
#define EAST 1
#define SOUTH 2
#define WEST 3

#endif // CONFIG_H
