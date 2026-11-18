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

// Maze-to-line bridge marker is read as a sequence of broad floor bands:
// white, black, white, black. A broad band must cover most of the sensor array.
#define SWITCH_MARKER_WHITE_MAX_BLACK_SENSORS 2
#define SWITCH_MARKER_BLACK_MIN_BLACK_SENSORS 6
#define SWITCH_MARKER_STABLE_SAMPLES          3
#define SWITCH_TO_LINE_CREEP_SPEED            12
#define SWITCH_TO_LINE_CREEP_MAX_CM           15.0f
#define SWITCH_TO_LINE_CREEP_TIMEOUT_MS       8000UL
#define SWITCH_TO_LINE_MIN_WHITE_SENSORS      5
#define SWITCH_TO_LINE_MIN_BLACK_SENSORS      1
#define SWITCH_TO_LINE_BASE_SPEED             10
#define SWITCH_TO_LINE_MAX_SPEED              35
#define SWITCH_TO_LINE_KP                     0.030f
#define SWITCH_TO_LINE_KI                     0.000f
#define SWITCH_TO_LINE_KD                     0.100f

// Default PID Parameters
#define DEFAULT_BASE_PWM    10
#define DEFAULT_MAX_PWM     50
#define DEFAULT_KP          0.030f
#define DEFAULT_KI          0.000f
#define DEFAULT_KD          0.100f

// Calibrated Calibration Values
// 288 ticks = 25 cm (1 cell) -> 11.52 ticks/cm
const float TICKS_PER_CELL_25CM = 282.0;
// Encoder target for an in-place 90 degree turn.
const float TICKS_PER_90_DEG = 116.5;  // Earlier calibration
const float TICKS_PER_180_DEG = 232.0;  // Average of 232-233 ticks

// const float TICKS_PER_90_DEG = 140.0;
// const float TICKS_PER_180_DEG = 280.2; // Average of 232-233 ticks

#define WHEEL_DIAMETER_CM 6.0f
#define WHEELBASE_CM 18.0f
#define ENCODER_COUNTS_PER_REV 224 
static const float CM_PER_COUNT           = (3.14159f * WHEEL_DIAMETER_CM / ENCODER_COUNTS_PER_REV);
#define TURN_SPEED 20
// User-tuned reverse-side wheel speed ratio during gyro turns.
#define TURN_REVERSE_SPEED_RATIO 1.0f
// Gyro turn controller tuning.
// User-tuned turn calibration values. Preserve these during tuning.
#define TURN_LEFT_CORRECTION_DEG      4.5f
#define TURN_RIGHT_CORRECTION_DEG     0.0f
#define USE_TWO_90_TURNS_FOR_180      0
#define TURN_STOP_TOLERANCE_DEG       2.0f
#define TURN_SLOWDOWN_START_ERROR_DEG 20.0f
#define TURN_MIN_DUTY                 15
#define TURN_90_TIMEOUT_MS            5000UL
#define TURN_180_TIMEOUT_MS           5000UL

// Select how 90 degree pivots are measured:
// TURN_90_MODE_GYRO uses gyro yaw; TURN_90_MODE_ENCODER uses encoder ticks.
#define TURN_90_MODE_GYRO              0
#define TURN_90_MODE_ENCODER           1
#define TURN_90_MODE                   TURN_90_MODE_ENCODER
#define TURN_STALL_MIN_PROGRESS_TICKS  2

// Straight-line gyro test tuning. The tick target uses TICKS_PER_CELL_25CM.
// Positive trim increases that wheel's PWM; use small +/- values for tuning.
#define STRAIGHT_TEST_LEFT_TRIM       0
#define STRAIGHT_TEST_RIGHT_TRIM      0
#define STRAIGHT_TEST_GYRO_KP         2.0f
#define STRAIGHT_TEST_MAX_CORRECTION  10.0f
#define STRAIGHT_TEST_TIMEOUT_MS      10000UL

#define WALL_THRESHOLD 20.0f
#define LEFT_WALL_THRESHOLD  20.0f
#define RIGHT_WALL_THRESHOLD 20.0f
#define FRONT_OBSTACLE WALL_THRESHOLD
// Desired clearance from either side wall. Change this one value to tune both.
#define SIDE_WALL_TARGET_CM 6.5f
#define CENTER_DIST         SIDE_WALL_TARGET_CM
#define LEFT_CENTER_DIST    SIDE_WALL_TARGET_CM
#define RIGHT_CENTER_DIST   SIDE_WALL_TARGET_CM
#define SAFETY_DIST 4.5f
#define NO_WALL_CREEP_DUTY 14.0f
#define MAZE_MAP_WIDTH 9
#define MAZE_MAP_HEIGHT 9

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

// ---------------------------------------------------------------
// Tuning block. Every value also has a default in main_updated.cpp,
// so this is optional; it just lets you tune everything in one place.
// ---------------------------------------------------------------

// --- Robot geometry (cm). 0 = not set (boot check is skipped).
#define ROBOT_WIDTH_CM            18.0f  // widest point of the body, wheels included
#define AXLE_TO_REAR_CM           12.6f  // axle centreline -> rearmost point of the body
#define AXLE_TO_FRONT_CM          4.1f   // axle centreline -> frontmost point (16.7 total length)
#define CELL_INNER_WIDTH_CM       25.0f  // MEASURE: real free gap between facing walls
#define TRACK_WIDTH_CM            18.0f  // MEASURE: distance between the two wheel CENTRES
#define PIVOT_CLEARANCE_MARGIN_CM 1.0f   // spare room wanted on each side

// --- Side-wall correction. Direct duty steering lets the ultrasonic sensors
// turn away from a nearby wall; 0 disables it, 1 is nominal, higher is stronger.
#define WALL_STEER_AS_HEADING        0
#define ULTRASONIC_STEER_WEIGHT      5.0f
#define WALL_HEADING_GAIN_DEG_PER_CM 1.5f
#define WALL_HEADING_MAX_DEG         4.0f

// --- Arc turns: OFF, so 90 degree turns are pivots around the axle centre.
//     (Arc code stays in the main file; set to 1 to use it again.)
#define USE_ARC_TURNS             0
#define ARC_TURN_RADIUS_CM        10.0f   // tail swing = sqrt(rear^2 + (R + width/2)^2) - R
#define ARC_BACKUP_DUTY           10     // also the reverse duty for pivots; 10 was below TURN_MIN_DUTY
#define ARC_BACKUP_TRIM_CM        0.0f   // reverse overshoot in cm: stop the reverse this much earlier
#define ARC_TURN_SPEED            10     // outer-wheel duty; inner gets this x ratio

// --- Reverse before pivoting
#define REVERSE_BEFORE_PIVOT      1      // 1 = back up before 90 degree pivots and 180 turns
#define PIVOT_BACKUP_CM           1.5f   // reverse distance before a 90 degree pivot
#define TURN_AROUND_BACKUP_CM     6.0f   // reverse distance before a 180 degree turn

// --- 90 degree pivot style (only used when USE_ARC_TURNS is 0)
//     0 = spin about the axle centre (both wheels counter-rotate)
//     1 = pivot about one wheel (left turn about left wheel, right turn about right wheel)
//     For a TRUE one-wheel pivot also set TURN_REVERSE_SPEED_RATIO to 0.0 above.
#define TURN90_WHEEL_LOCK         1

// --- Stall detection during turns
#define TURN_STALL_WINDOW_MS         400UL
#define TURN_STALL_MIN_PROGRESS_DEG  2.0f

// --- Median of 3 sensor readings for turn decisions
#define USE_SENSOR_MEDIAN         1

#endif // CONFIG_H
