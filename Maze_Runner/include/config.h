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

// --- 4-way DIP switch (mode select) ---
// Wiring: resistor from each pin to GND, other side of the switch to 5V -> ON reads HIGH.
// Never configure these pins as outputs.
const uint8_t DIP_PINS[4] = {49, 47, 45, 43};   // [0] = SW1 ... [3] = SW4
#define DIP_ON_LEVEL         HIGH

// --- IR Sensor Array Pins ---
#define PIN_IR_EN            52
const uint8_t IR_PINS[8] = {A7, A6, A5, A4, A3, A2, A1, A0};   // [0] = robot's right most, [7] = left most
#define NUM_IR_SENSORS       8

// IR calibration (from ir_02.cpp 'c' command, pins A7..A0). Raw reads HIGH on black.
const int WHITE_VALUES[8] = {805, 722, 731, 733, 720, 727, 743, 740};
const int BLACK_VALUES[8] = {1022, 1017, 1020, 1019, 1017, 1016, 1018, 1017};

// Line follower (tuned in ir_02.cpp). Speeds are on the 0-255 analogWrite scale used
// there; the LineFollower library converts them to duty % for setMotors().
#define LINE_BASE_PWM        50
#define LINE_MAX_PWM         100
#define LINE_KP              0.025f
#define LINE_KI              0.000f
#define LINE_KD              0.053f
#define LINE_SENSOR_EXPONENT 2.5f
#define LINE_LOST_SUM        200     // total normalised value below this = line lost
#define LINE_IS_WHITE        0       // 0 = black line on white (as tuned), 1 = white line on black

// Marker cell + line mode (LineFollower library). Maze floor is black. Seen while driving,
// the special cell is: WHITE strip ~3.5 cm -> BLACK square ~12 cm -> WHITE strip ~3.5 cm
// -> small black part -> black line starts. Lengths are measured with the encoders.
#define MARKER_FULL_SENSORS  6       // this many sensors the same colour = a full-width band
#define MARKER_W_MIN_CM      2.0f    // white strip length limits (nominal 3.5 cm)
#define MARKER_W_MAX_CM      6.0f
#define MARKER_B_MIN_CM      8.5f    // black square limits (nominal 12; motors 11.5-11.6, hand 9.6-11.0; a false 5.3 was seen)
#define MARKER_B_MAX_CM      16.0f
#define MARKER_W2_CONFIRM_CM 1.5f    // second white strip seen this long = marker found
#define MARKER_FINISH_MAX_CM 30.0f   // a move ended mid-pattern: drive at most this far to finish it
#define LINE_SEARCH_DUTY     20      // duty % used to drive straight while looking for the line
#define LINE_SEARCH_MAX_CM   60.0f   // give up if no line is found within this distance
#define LINE_FOUND_CM        1.0f    // line pattern must last this long to count as found
#define LINE_END_SENSORS     6       // this many sensors black = back on the black maze floor
#define LINE_END_MAX_CONTRAST 60     // ...and max-min raw below this (floor 20-50; grey background + line 70-130)
#define LINE_END_CM          6.0f    // ...for at least this distance = end of the line section
#define LINE_MIN_FOLLOW_CM   30.0f   // no 'line ended' check until this far along the line (ramp at the start lifts the IR array)
#define LINE_LOST_MS         500     // line lost (all white) this long = stop line mode

// Motor and encoder calibration (4x decoding)
#define ENCODER_COUNTS_PER_REV   897
#define WHEEL_DIAMETER_CM        6.43f
#define WHEELBASE_CM             18.5f        // centre to centre of the wheels (measured)
const float TICKS_PER_CELL_25CM  = 1134.0f;   // was 1111: drove 24.5 cm (rechecked 2026-11-26)
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
#define WALL_CENTER_CM 3.0f      // side reading when centred in a cell (measured)
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
#define HEADING_KP 2.0f          // duty % per degree of heading error (gyro, autotuned)
#define HEADING_KD 0.0f          // duty % per deg/s of turn rate (gyro): damps wobble, 0 = off
#define ENC_SYNC_KP 0.1f         // tuning stage 2: duty % per tick of left-right difference
#define WALL_YAW_GAIN 1.0f       // degrees of heading offset per cm of wall error (autotuned)
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

#endif // CONFIG_H