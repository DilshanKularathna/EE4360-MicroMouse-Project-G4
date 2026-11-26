#ifndef MOTION_H
#define MOTION_H

#include <Arduino.h>

/**
 * ============================================================================
 * Cell-level motion for the maze runner
 * ----------------------------------------------------------------------------
 * Ported from the tested left-wall follower
 * (Arduino_Test/maze_solve_test/maze_solver_small.cpp) so the main firmware
 * uses the same tuned moves:
 *  - one-cell forward move: encoder PID + gyro heading hold + wall centring
 *  - front-wall alignment (stops the axle on the cell centre)
 *  - turns to an absolute heading, so turn errors do not add up
 * All gains and distances come from config.h / globals.h.
 * ============================================================================
 */

// Sets up encoders, motor and ultrasonic pins, and the gyro (falls back to
// encoder-only turns if the MPU-6050 is not found). Robot must be still.
void motionBegin();

// Drives one 25 cm cell. Stops early if a wall comes up in front.
// Returns the distance actually driven (cm, from the encoders).
float motionForwardCell();

// If a wall is in front of this cell, creeps until the axle is on the cell centre.
void motionAlignFront();

// Turns by a multiple of 90 degrees (+ = right). The target heading is kept
// as an absolute value, so small turn errors do not build up.
void motionTurnBy(float deltaDeg);

// Makes the current heading the new reference for motionTurnBy(). Call after
// anything that turns the robot outside this lib (e.g. line following).
void motionResetHeading();

// Gyro yaw minus the target heading (deg, + = right of target). 0 without a gyro.
float motionHeadingError();

// Refreshes the gyro bias. Call only while the robot is standing still.
void motionRecalibrateGyro();

// Median of 3 readings of one ultrasonic sensor (cm, corrected).
float motionReadDistance(int trigPin, int echoPin);

#endif // MOTION_H
