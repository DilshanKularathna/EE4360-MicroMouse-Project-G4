#include <Arduino.h>
#include <Wire.h>
#include <util/atomic.h>
#include <math.h>
#include "config.h"
#include "Ultrasonic.h"
#include "motorDriver.h"
#include "globals.h"
#include "Gyroscope.h"

// ================================================================
// CONSTANTS
// ================================================================
const long TARGET_TICKS_FWD = (long)(TICKS_PER_CELL_25CM + 0.5f);

// Geometry used by the in-maze turn-clearance diagnostic. These are measured
// from the drive-axle midpoint to the farthest chassis end and across the body.
float getTurnSweepRadiusCm() {
    const float halfWidth = TURN_CHASSIS_WIDTH_CM * 0.5f;
    return sqrtf(
        TURN_AXLE_TO_FARTHEST_END_CM * TURN_AXLE_TO_FARTHEST_END_CM +
        halfWidth * halfWidth);
}

void printTurnSweepGeometry() {
    const float sweepRadiusCm = getTurnSweepRadiusCm();
    Serial.println(F("=== Turn clearance geometry ==="));
    Serial.print(F("Axle to farthest chassis end: "));
    Serial.print(TURN_AXLE_TO_FARTHEST_END_CM, 1);
    Serial.println(F(" cm"));
    Serial.print(F("Chassis width: "));
    Serial.print(TURN_CHASSIS_WIDTH_CM, 1);
    Serial.println(F(" cm"));
    Serial.print(F("Estimated sweep radius: "));
    Serial.print(sweepRadiusCm, 2);
    Serial.println(F(" cm"));
    Serial.print(F("Estimated sweep-circle diameter: "));
    Serial.print(2.0f * sweepRadiusCm, 2);
    Serial.println(F(" cm"));
    Serial.print(F("Maze cell: "));
    Serial.print(MAZE_CELL_SIZE_CM, 1);
    Serial.print(F(" x "));
    Serial.print(MAZE_CELL_SIZE_CM, 1);
    Serial.println(F(" cm"));
    Serial.print(F("Centered-cell lateral shift allowance: "));
    Serial.print((MAZE_CELL_SIZE_CM - TURN_CHASSIS_WIDTH_CM) * 0.5f, 2);
    Serial.println(F(" cm"));
    Serial.println(F("Geometry estimate; turn arc adapts to the outside-side sensor."));
}

bool          started = false;
bool          turnFault = false;
unsigned long t0      = 0;

const float MAX_DUTY = 28.0f;
const float WALL_STEER_GAIN = 1.5f;
const float MAX_WALL_STEER_DUTY = 8.0f;
const float WALL_ERROR_DEADBAND_CM = 0.5f;

const float GYRO_HEADING_GAIN = STRAIGHT_TEST_GYRO_KP;
const float MAX_GYRO_STEER_DUTY = STRAIGHT_TEST_MAX_CORRECTION;
const float MAX_COMBINED_STEER_DUTY = 10.0f;
const float GYRO_HEADING_DEADBAND_DEG = 1.0f;

const float ENCODER_SYNC_GAIN = 0.35f;    // duty percentage per tick of wheel progress mismatch
const float MAX_ENCODER_SYNC_DUTY = 8.0f;
const long ENCODER_SYNC_DEADBAND_TICKS = 1;

const float ULTRASONIC_STEER_STEP_DUTY = 5.0f;
const float SINGLE_WALL_STEER_MIN_DIST_CM = 4.0f;

// Sensor is mounted upside down; Gyroscope corrects yaw so left is positive.
Gyroscope gyroscope(Wire, true);
const unsigned long GYRO_SAMPLE_INTERVAL_US = 10000UL;
bool gyroReady = false;
float gyroHeadingDeg = 0.0f;
float gyroHeadingTargetDeg = 0.0f;

// ================================================================
// GLOBAL VARIABLES
// ================================================================
float eprevL = 0, eintegralL = 0;
float eprevR = 0, eintegralR = 0;
long  prevT = 0;

const float kpL = 4.5f, kdL = 0.0f, kiL = 0.0f;
const float kpR = 4.5f, kdR = 0.0f, kiR = 0.0f;

float frontDist, leftDist, rightDist;

enum RelativeDir { REL_LEFT = -1, REL_FRONT = 0, REL_RIGHT = 1, REL_DEAD = 99 };

void moveForwardPID();
void  navigateStep();
bool initializeGyroscope();
bool turnWithGyroscope(float targetDegrees, float reverseSpeedRatio);
bool advanceForTurn(float distanceCm);
float calculateTurnReverseRatio(float targetDegrees);

// Optional, measured pre-turn adjustment. Positive distance advances toward
// the front of the current cell; zero leaves the existing turn position intact.
bool advanceForTurn(float distanceCm) {
    if (distanceCm <= 0.0f) return true;

    const long targetTicks = lroundf(
        distanceCm * TICKS_PER_CELL_25CM / 25.0f);
    if (targetTicks <= 0) return true;

    resetEncoders();
    const unsigned long startMs = millis();
    unsigned long lastFrontCheckMs = 0;
    const unsigned long timeoutMs = 1000UL + (unsigned long)(distanceCm * 250.0f);

    Serial.print(F("Turn entry advance: "));
    Serial.print(distanceCm, 1);
    Serial.print(F(" cm ("));
    Serial.print(targetTicks);
    Serial.println(F(" ticks)"));

    while (millis() - startMs < timeoutMs) {
        if (millis() - lastFrontCheckMs >= 50UL) {
            const float front = readUltrasonic(
                PIN_ULTRA_FRONT_TRIG, PIN_ULTRA_FRONT_ECHO);
            if (front > 0.1f && front < SAFETY_DIST) {
                stopMotors();
                Serial.println(F("Turn entry advance stopped: front obstacle."));
                return false;
            }
            lastFrontCheckMs = millis();
        }

        long leftPos;
        long rightPos;
        ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
            leftPos = leftEncoderCount;
            rightPos = rightEncoderCount;
        }

        if (leftPos >= targetTicks - 2 && rightPos >= targetTicks - 2) {
            stopMotors();
            delay(100);
            return true;
        }

        const long difference = leftPos - rightPos;
        const float sync = constrain(
            difference * ENCODER_SYNC_GAIN,
            -MAX_ENCODER_SYNC_DUTY,
            MAX_ENCODER_SYNC_DUTY);
        const float leftDuty = (leftPos >= targetTicks - 2)
            ? 0.0f : constrain(TURN_ENTRY_SPEED - sync, 0.0f, MAX_DUTY);
        const float rightDuty = (rightPos >= targetTicks - 2)
            ? 0.0f : constrain(TURN_ENTRY_SPEED + sync, 0.0f, MAX_DUTY);
        setMotors(leftDuty, rightDuty);
    }

    stopMotors();
    Serial.println(F("Turn entry advance failed: timeout."));
    return false;
}

// Estimate the lateral center shift needed during a turn from the outside-wall
// reading. The reverse/forward wheel ratio sets the center's arc radius:
// r = track*(1-ratio)/(2*(1+ratio)). A turn-side open cell allows the center
// to move toward that opening while the gyro controls the final heading.
float calculateTurnReverseRatio(float targetDegrees) {
    const bool leftTurn = targetDegrees > 0.0f;
    const float outsideWallGap = leftTurn ? rightDist : leftDist;
    const bool outsideWallSeen =
        outsideWallGap > 0.1f && outsideWallGap <= WALL_THRESHOLD;

    if (!outsideWallSeen) {
        Serial.println(F("Turn sweep: no outside wall detected; using tuned ratio."));
        return constrain(TURN_REVERSE_SPEED_RATIO, 0.0f, 1.0f);
    }

    const float halfWidth = TURN_CHASSIS_WIDTH_CM * 0.5f;
    const float sweepRadius = getTurnSweepRadiusCm();
    const float pivotToOutsideWall = halfWidth + outsideWallGap;
    const float maxCenterShift =
        (MAZE_CELL_SIZE_CM - TURN_CHASSIS_WIDTH_CM) * 0.5f;
    const float neededShift = max(
        0.0f,
        sweepRadius + TURN_SIDE_CLEARANCE_MARGIN_CM - pivotToOutsideWall);
    const float centerShift = min(neededShift, maxCenterShift);

    const float theta = fabs(targetDegrees) * (PI / 180.0f);
    const float lateralFactor = 1.0f - cosf(theta);
    float geometryRatio = 1.0f;
    if (centerShift > 0.0f && lateralFactor > 0.001f) {
        const float centerArcRadius = centerShift / lateralFactor;
        geometryRatio = (WHEELBASE_CM - 2.0f * centerArcRadius) /
                        (WHEELBASE_CM + 2.0f * centerArcRadius);
        geometryRatio = constrain(geometryRatio, 0.0f, 1.0f);
    }

    // Keep at least the user-tuned forward arc; add more shift only when the
    // side-wall geometry calls for it.
    const float ratio = min(
        geometryRatio,
        constrain(TURN_REVERSE_SPEED_RATIO, 0.0f, 1.0f));

    Serial.print(F("Turn sweep side gap="));
    Serial.print(outsideWallGap, 1);
    Serial.print(F(" cm; needed shift="));
    Serial.print(neededShift, 1);
    Serial.print(F(" cm; applied shift="));
    Serial.print(centerShift, 1);
    Serial.print(F(" cm; reverse ratio="));
    Serial.println(ratio, 2);
    if (neededShift > maxCenterShift) {
        Serial.println(F("Turn clearance estimate exceeds available centered-cell offset."));
    }
    return ratio;
}

RelativeDir findNextUnvisitedNeighbor() {
    bool leftWall  = isWallOnLeft();
    bool rightWall = isWallOnRight();
    bool frontOpen = isPathClearFront();

// 1st Priority: Turn left if there is an opening on the left
    if (!leftWall) {
        return REL_LEFT;
    }
    
    // 2nd Priority: Move forward if front is open (hug the left wall)
    if (frontOpen) {
        return REL_FRONT;
    }

    // 3rd Priority: Turn right only if left AND front are blocked
    if (!rightWall) {
        return REL_RIGHT;
    }

    // 4th Priority: U-Turn at dead ends
    return REL_DEAD;
}

void navigateStep() {
    if (turnFault) {
        stopMotors();
        return;
    }
    readAllSensors();

// --- ADD THIS SERIAL PRINT BLOCK HERE ---
    Serial.println("----------------------------------------");
    Serial.print("Sensors -> L: ");
    Serial.print(leftDist);
    Serial.print(" cm | F: ");
    Serial.print(frontDist);
    Serial.print(" cm | R: ");
    Serial.print(rightDist);
    Serial.println(" cm");
    Serial.println("----------------------------------------");
    // ----------------------------------------

    RelativeDir nextDir = findNextUnvisitedNeighbor();

    if (nextDir == REL_DEAD) {
        Serial.println("Dead end -> turn around");
        if (USE_TWO_90_TURNS_FOR_180) {
            const float correctedLeft90 = 90.0f + TURN_LEFT_CORRECTION_DEG;
            if (!turnWithGyroscope(correctedLeft90, TURN_REVERSE_SPEED_RATIO)) {
                turnFault = true;
            } else if (!turnWithGyroscope(correctedLeft90, TURN_REVERSE_SPEED_RATIO)) {
                turnFault = true;
            }
        } else if (!turnWithGyroscope(180.0f, TURN_REVERSE_SPEED_RATIO)) {
            turnFault = true;
        }
        return;
    }

    if (nextDir == REL_LEFT) {
        Serial.println("Turning left");
        if (!advanceForTurn(LEFT_TURN_ENTRY_ADVANCE_CM)) {
            turnFault = true;
            return;
        }
        const float targetDegrees = 90.0f + TURN_LEFT_CORRECTION_DEG;
        const float reverseRatio = calculateTurnReverseRatio(targetDegrees);
        if (!turnWithGyroscope(targetDegrees, reverseRatio)) {
            turnFault = true;
            return;
        }
        moveForwardPID();
        return;
    }

    if (nextDir == REL_RIGHT) {
        Serial.println("Turning right");
        if (!advanceForTurn(RIGHT_TURN_ENTRY_ADVANCE_CM)) {
            turnFault = true;
            return;
        }
        const float targetDegrees = -90.0f - TURN_RIGHT_CORRECTION_DEG;
        const float reverseRatio = calculateTurnReverseRatio(targetDegrees);
        if (!turnWithGyroscope(targetDegrees, reverseRatio)) {
            turnFault = true;
            return;
        }
        moveForwardPID();
        return;
    }

    Serial.println("Moving forward");
    moveForwardPID();
}

bool initializeGyroscope() {
    if (!gyroscope.begin(0, 0)) { // +/-250 dps gyro; accelerometer range is unused.
        Serial.println(F("MPU6050 initialization failed."));
        return false;
    }

    Serial.println(F("Keep robot still: calibrating gyro bias."));
    delay(1000);
    if (!gyroscope.calibrate()) return false;
    gyroscope.update();
    gyroHeadingDeg = gyroscope.getYawAngleDeg();
    gyroHeadingTargetDeg = gyroHeadingDeg;
    return true;
}

bool turnWithGyroscope(float targetDegrees, float reverseSpeedRatio) {
    if (!gyroReady) {
        stopMotors();
        Serial.println(F("Turn refused: MPU6050 is not ready."));
        return false;
    }

    const unsigned long timeoutMs = (fabs(targetDegrees) > 90.0f)
        ? TURN_180_TIMEOUT_MS
        : TURN_90_TIMEOUT_MS;
    const unsigned long startMs = millis();
    unsigned long nextSampleUs = micros();
    const float startAngle = gyroscope.getYawAngleDeg();
    float angle = 0.0f;

    // navigateStep passes the calibrated target (90 plus/minus its correction).
    // Keep comparing against that exact target, including after coast-down.
    Serial.print(F("Gyro corrected turn target: "));
    Serial.print(targetDegrees, 1);
    Serial.println(F(" deg"));

    for (uint8_t attempt = 0; attempt < 3; ++attempt) {
        angle = gyroscope.getYawAngleDeg() - startAngle;
        while (fabs(targetDegrees - angle) > TURN_STOP_TOLERANCE_DEG &&
               millis() - startMs < timeoutMs) {
            if ((long)(micros() - nextSampleUs) < 0) continue;
            gyroscope.update();
            angle = gyroscope.getYawAngleDeg() - startAngle;
            gyroHeadingDeg = gyroscope.getYawAngleDeg();

            const float error = targetDegrees - angle;
            // Positive gyro angle is left; this motor wiring turns left with
            // left reverse and right forward.
            const int direction = (error > 0.0f) ? -1 : 1;
            const float absError = fabs(error);
            int duty = TURN_SPEED;
            if (absError < TURN_SLOWDOWN_START_ERROR_DEG) {
                const float range = TURN_SLOWDOWN_START_ERROR_DEG -
                                    TURN_STOP_TOLERANCE_DEG;
                const float progress = constrain(
                    (absError - TURN_STOP_TOLERANCE_DEG) / range,
                    0.0f,
                    1.0f);
                duty = (int)lroundf(
                    TURN_MIN_DUTY + progress * (TURN_SPEED - TURN_MIN_DUTY));
            }
            const int reverseDuty = (int)lroundf(
                duty * constrain(reverseSpeedRatio, 0.0f, 1.0f));

            if (direction < 0) setMotors(-reverseDuty, duty);
            else setMotors(duty, -reverseDuty);
            nextSampleUs += GYRO_SAMPLE_INTERVAL_US;
        }

        stopMotors();
        delay(200);
        gyroscope.update();
        angle = gyroscope.getYawAngleDeg() - startAngle;
        gyroHeadingDeg = gyroscope.getYawAngleDeg();
        const float finalError = targetDegrees - angle;

        Serial.print(F("Measured=")); Serial.print(angle, 2);
        Serial.print(F(" deg; target=")); Serial.print(targetDegrees, 2);
        Serial.print(F(" deg; error=")); Serial.println(finalError, 2);

        if (fabs(finalError) <= TURN_STOP_TOLERANCE_DEG) {
            gyroHeadingTargetDeg = gyroHeadingDeg;
            return true;
        }
        if (millis() - startMs >= timeoutMs) break;
        Serial.println(F("Outside tolerance after settling; correcting again."));
    }

    Serial.println(F("Turn did not reach corrected target tolerance."));
    return false;
}
/*
void moveForwardPID() {
    resetEncoders();

    long targetL = TARGET_TICKS_FWD;
    long targetR = TARGET_TICKS_FWD;

    eprevL     = 0;
    eintegralL = 0;
    eprevR     = 0;
    eintegralR = 0;
    prevT      = micros();

    bool          reached            = false;
    bool          leftDone           = false;
    bool          rightDone          = false;
    unsigned long startTime          = millis();
    unsigned long lastCheck          = 0;
    unsigned long nextGyroSampleUs   = micros();
    float ultrasonicSteering         = 0.0f;
    float gyroSteering               = 0.0f;
    float         steeringAdjustment = 0;
    unsigned long lastDriveReportMs = startTime;

    while (!reached) {
        if (millis() - startTime > 4000)
            break;

        if (millis() - lastCheck > 50) {
            float f = readUltrasonic(PIN_ULTRA_FRONT_TRIG, PIN_ULTRA_FRONT_ECHO);
            if (f < SAFETY_DIST && f > 0.1f) {
                Serial.println("OBSTACLE - STOP");
                reached = true;
                stopMotors();
                break;
            }

            float r = readUltrasonic(PIN_ULTRA_RIGHT_TRIG, PIN_ULTRA_RIGHT_ECHO);
            float l = readUltrasonic(PIN_ULTRA_LEFT_TRIG, PIN_ULTRA_LEFT_ECHO);

            const bool rightWallSeen = r > 0.1f && r <= WALL_THRESHOLD;
            const bool leftWallSeen  = l > 0.1f && l <= WALL_THRESHOLD;
            float wallError = 0.0f;
            if (leftWallSeen && rightWallSeen) {
                // Positive error means closer to the right wall: steer left.
                wallError = l - r;
            } else if (rightWallSeen) {
                wallError = CENTER_DIST - r;
            } else if (leftWallSeen) {
                wallError = l - CENTER_DIST;
            }
            if (fabs(wallError) < WALL_ERROR_DEADBAND_CM) wallError = 0.0f;
            ultrasonicSteering = constrain(
                wallError * WALL_STEER_GAIN,
                -MAX_WALL_STEER_DUTY,
                MAX_WALL_STEER_DUTY);

            lastCheck = millis();
        }

        // Keep a persistent yaw reference across consecutive cells, correcting
        // accumulated drift from both long straight runs and slightly imperfect turns.
        if ((long)(micros() - nextGyroSampleUs) >= 0) {
            gyroscope.update();
            gyroHeadingDeg = gyroscope.getYawAngleDeg();

            // Positive left-heading error requires a rightward wheel bias.
            float headingError = gyroHeadingDeg - gyroHeadingTargetDeg;
            if (fabs(headingError) < GYRO_HEADING_DEADBAND_DEG) headingError = 0.0f;
            gyroSteering = constrain(
                -headingError * GYRO_HEADING_GAIN,
                -MAX_GYRO_STEER_DUTY,
                MAX_GYRO_STEER_DUTY);
            nextGyroSampleUs += GYRO_SAMPLE_INTERVAL_US;
        }
        steeringAdjustment = constrain(
            ultrasonicSteering + gyroSteering,
            -MAX_COMBINED_STEER_DUTY,
            MAX_COMBINED_STEER_DUTY);

        long  currT  = micros();
        float deltaT = (currT - prevT) / 1e6f;
        prevT        = currT;
        if (deltaT <= 0)
            deltaT = 1e-3f;

        long leftPos, rightPos;
        ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
            leftPos  = leftEncoderCount;
            rightPos = rightEncoderCount;
        }

        float eL    = (float)targetL - (float)leftPos;
        float dedtL = (eL - eprevL) / deltaT;
        eintegralL += eL * deltaT;
        float uL = kpL * eL + kdL * dedtL + kiL * eintegralL;

        float eR    = (float)targetR - (float)rightPos;
        float dedtR = (eR - eprevR) / deltaT;
        eintegralR += eR * deltaT;
        float uR = kpR * eR + kdR * dedtR + kiR * eintegralR;

        // Latch each wheel as complete independently. Do not reverse or keep
        // driving a wheel while waiting for the other encoder to catch up.
        if (leftPos >= targetL - 5) leftDone = true;
        if (rightPos >= targetR - 5) rightDone = true;

        // Balance wheel progress directly. A positive value means the left
        // wheel is ahead, so reduce its duty and give the right wheel more.
        long encoderDifference = leftPos - rightPos;
        if (labs(encoderDifference) <= ENCODER_SYNC_DEADBAND_TICKS) encoderDifference = 0;
        const float encoderSync = constrain(
            encoderDifference * ENCODER_SYNC_GAIN,
            -MAX_ENCODER_SYNC_DUTY,
            MAX_ENCODER_SYNC_DUTY);
        const float totalSteering = constrain(
            steeringAdjustment + encoderSync,
            -MAX_COMBINED_STEER_DUTY - MAX_ENCODER_SYNC_DUTY,
            MAX_COMBINED_STEER_DUTY + MAX_ENCODER_SYNC_DUTY);

        float pwrL = fabs(uL);
        float pwrR = fabs(uR);

        if (pwrL > MAX_DUTY)
            pwrL = MAX_DUTY;
        if (pwrR > MAX_DUTY)
            pwrR = MAX_DUTY;

        // Reduce coast near the destination; the proportional controller is
        // otherwise saturated at MAX_DUTY until only a few ticks remain.
        if (!leftDone && eL > 0 && eL <= 25 && pwrL > 20.0f) pwrL = 20.0f;
        if (!rightDone && eR > 0 && eR <= 25 && pwrR > 20.0f) pwrR = 20.0f;

        pwrL -= totalSteering;
        pwrR += totalSteering;

        if (pwrL > MAX_DUTY + 5)
            pwrL = MAX_DUTY + 5;
        if (pwrL < 0)
            pwrL = 0;
        if (pwrR > MAX_DUTY + 5)
            pwrR = MAX_DUTY + 5;
        if (pwrR < 0)
            pwrR = 0;

        if (leftDone) pwrL = 0;
        if (rightDone) pwrR = 0;

        int dirL = (uL > 0) ? 1 : -1;
        int dirR = (uR > 0) ? 1 : -1;

        if (leftDone && rightDone) {
            reached = true;
            stopMotors();
        } else {
            setMotors(leftDone ? 0 : dirL * pwrL,
                      rightDone ? 0 : dirR * pwrR);
        }

        if (millis() - lastDriveReportMs >= 250UL) {
            Serial.print(F("FWD ticks L/R=")); Serial.print(leftPos); Serial.print('/'); Serial.print(rightPos);
            Serial.print(F(" yaw err=")); Serial.print(gyroHeadingDeg - gyroHeadingTargetDeg, 1);
            Serial.print(F(" steer U/G/E=")); Serial.print(ultrasonicSteering, 1); Serial.print('/');
            Serial.print(gyroSteering, 1); Serial.print('/'); Serial.print(encoderSync, 1);
            Serial.print(F(" PWM L/R=")); Serial.print(pwrL, 1); Serial.print('/'); Serial.println(pwrR, 1);
            lastDriveReportMs = millis();
        }

        eprevL = eL;
        eprevR = eR;
    }
    stopMotors();
    delay(200);
}
*/

void moveForwardPID() {
    resetEncoders();

    const long targetL = TARGET_TICKS_FWD;
    const long targetR = TARGET_TICKS_FWD;

    eprevL = 0;
    eintegralL = 0;
    eprevR = 0;
    eintegralR = 0;
    prevT = micros();

    bool reached = false;
    bool leftDone = false;
    bool rightDone = false;

    const unsigned long startTime = millis();
    unsigned long lastCheck = 0;
    unsigned long nextGyroSampleUs = micros();
    unsigned long lastDriveReportMs = startTime;

    // Keep these corrections separate so you can tune them independently.
    float ultrasonicSteering = 0.0f;
    float gyroSteering = 0.0f;
    float steeringAdjustment = 0.0f;

    while (!reached) {
        if (millis() - startTime > 4000UL) {
            break;
        }

        // Refresh ultrasonic steering every 50 ms.
        if (millis() - lastCheck > 50UL) {
            const float front = readUltrasonic(
                PIN_ULTRA_FRONT_TRIG, PIN_ULTRA_FRONT_ECHO);

            if (front > 0.1f && front < SAFETY_DIST) {
                Serial.println(F("OBSTACLE - STOP"));
                stopMotors();
                break;
            }

            const float r = readUltrasonic(
                PIN_ULTRA_RIGHT_TRIG, PIN_ULTRA_RIGHT_ECHO);
            const float l = readUltrasonic(
                PIN_ULTRA_LEFT_TRIG, PIN_ULTRA_LEFT_ECHO);

            // readUltrasonic() returns 100 cm when there is no echo.
            const bool rightWallSeen =
                r > 0.1f && r <= WALL_THRESHOLD;
            const bool leftWallSeen =
                l > 0.1f && l <= WALL_THRESHOLD;

            ultrasonicSteering = 0.0f;

            if (rightWallSeen && leftWallSeen) {
                if (r < CENTER_DIST) {
                    ultrasonicSteering = ULTRASONIC_STEER_STEP_DUTY;
                } else if (l < CENTER_DIST) {
                    ultrasonicSteering = -ULTRASONIC_STEER_STEP_DUTY;
                }
            } else if (rightWallSeen) {
                if (r > SINGLE_WALL_STEER_MIN_DIST_CM) {
                    ultrasonicSteering = ULTRASONIC_STEER_STEP_DUTY;
                }
            } else if (leftWallSeen) {
                if (l > SINGLE_WALL_STEER_MIN_DIST_CM) {
                    ultrasonicSteering = -ULTRASONIC_STEER_STEP_DUTY;
                }
            }

            lastCheck = millis();
        }

        // Refresh gyro heading correction every 10 ms.
        if ((long)(micros() - nextGyroSampleUs) >= 0) {
            gyroscope.update();
            gyroHeadingDeg = gyroscope.getYawAngleDeg();

            float headingError =
                gyroHeadingDeg - gyroHeadingTargetDeg;

            if (fabs(headingError) < GYRO_HEADING_DEADBAND_DEG) {
                headingError = 0.0f;
            }

            gyroSteering = constrain(
                -headingError * GYRO_HEADING_GAIN,
                -MAX_GYRO_STEER_DUTY,
                MAX_GYRO_STEER_DUTY);

            nextGyroSampleUs += GYRO_SAMPLE_INTERVAL_US;
        }

        // Combine ultrasonic and gyro corrections only after calculating each
        // independently. Encoder synchronization is added separately below.
        steeringAdjustment = constrain(
            ultrasonicSteering + gyroSteering,
            -MAX_COMBINED_STEER_DUTY,
            MAX_COMBINED_STEER_DUTY);

        const long currT = micros();
        float deltaT = (currT - prevT) / 1e6f;
        prevT = currT;
        if (deltaT <= 0.0f) {
            deltaT = 1e-3f;
        }

        long leftPos;
        long rightPos;
        ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
            leftPos = leftEncoderCount;
            rightPos = rightEncoderCount;
        }

        const float eL = (float)targetL - (float)leftPos;
        const float dedtL = (eL - eprevL) / deltaT;
        eintegralL += eL * deltaT;
        const float uL =
            kpL * eL + kdL * dedtL + kiL * eintegralL;

        const float eR = (float)targetR - (float)rightPos;
        const float dedtR = (eR - eprevR) / deltaT;
        eintegralR += eR * deltaT;
        const float uR =
            kpR * eR + kdR * dedtR + kiR * eintegralR;

        if (leftPos >= targetL - 5) leftDone = true;
        if (rightPos >= targetR - 5) rightDone = true;

        long encoderDifference = leftPos - rightPos;
        if (labs(encoderDifference) <= ENCODER_SYNC_DEADBAND_TICKS) {
            encoderDifference = 0;
        }

        const float encoderSync = constrain(
            encoderDifference * ENCODER_SYNC_GAIN,
            -MAX_ENCODER_SYNC_DUTY,
            MAX_ENCODER_SYNC_DUTY);

        const float totalSteering = constrain(
            steeringAdjustment + encoderSync,
            -MAX_COMBINED_STEER_DUTY - MAX_ENCODER_SYNC_DUTY,
            MAX_COMBINED_STEER_DUTY + MAX_ENCODER_SYNC_DUTY);

        float pwrL = min(fabs(uL), MAX_DUTY);
        float pwrR = min(fabs(uR), MAX_DUTY);

        if (!leftDone && eL > 0 && eL <= 25 && pwrL > 20.0f) {
            pwrL = 20.0f;
        }
        if (!rightDone && eR > 0 && eR <= 25 && pwrR > 20.0f) {
            pwrR = 20.0f;
        }

        pwrL = constrain(pwrL - totalSteering, 0.0f, MAX_DUTY + 5.0f);
        pwrR = constrain(pwrR + totalSteering, 0.0f, MAX_DUTY + 5.0f);

        const int dirL = (uL > 0.0f) ? 1 : -1;
        const int dirR = (uR > 0.0f) ? 1 : -1;

        if (leftDone && rightDone) {
            reached = true;
            stopMotors();
        } else {
            setMotors(leftDone ? 0 : dirL * pwrL,
                      rightDone ? 0 : dirR * pwrR);
        }

        if (millis() - lastDriveReportMs >= 250UL) {
            Serial.print(F("Steer U/G/E="));
            Serial.print(ultrasonicSteering, 1);
            Serial.print('/');
            Serial.print(gyroSteering, 1);
            Serial.print('/');
            Serial.println(encoderSync, 1);
            lastDriveReportMs = millis();
        }

        eprevL = eL;
        eprevR = eR;
    }

    stopMotors();
    delay(200);
}


void setup() {
    Serial.begin(115200);
    delay(1000);
    printTurnSweepGeometry();

    pinMode(PIN_LEFT_ENC_A, INPUT_PULLUP);
    pinMode(PIN_LEFT_ENC_B, INPUT_PULLUP);
    pinMode(PIN_RIGHT_ENC_A, INPUT_PULLUP);
    pinMode(PIN_RIGHT_ENC_B, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(PIN_LEFT_ENC_A), leftEncoderISR, RISING);
    attachInterrupt(digitalPinToInterrupt(PIN_RIGHT_ENC_A), rightEncoderISR, RISING);

    pinMode(PIN_LEFT_RPWM, OUTPUT);
    pinMode(PIN_LEFT_LPWM, OUTPUT);
    pinMode(PIN_RIGHT_RPWM, OUTPUT);
    pinMode(PIN_RIGHT_LPWM, OUTPUT);

    pinMode(PIN_ULTRA_FRONT_TRIG, OUTPUT);
    pinMode(PIN_ULTRA_FRONT_ECHO, INPUT);
    pinMode(PIN_ULTRA_LEFT_TRIG, OUTPUT);
    pinMode(PIN_ULTRA_LEFT_ECHO, INPUT);
    pinMode(PIN_ULTRA_RIGHT_TRIG, OUTPUT);
    pinMode(PIN_ULTRA_RIGHT_ECHO, INPUT);

    prevT = micros();

    gyroReady = initializeGyroscope();
    t0 = millis();
    if (!gyroReady) {
        stopMotors();
        Serial.println(F("MPU6050 not detected or calibration read failed. Auto drive disabled."));
    }

    Serial.println("========================================");
    Serial.println("WALL FOLLOWER - READY");
    Serial.println(gyroReady ? F("Gyro yaw correction enabled. Auto start in 3 seconds...") : F("Auto start disabled until MPU6050 works."));
    Serial.println("Priority: Left -> Right -> Front");
    Serial.println("========================================");
}

void loop() {
    if (gyroReady && !started && millis() - t0 >= 3000) {
        started = true;
        Serial.println("\n=== STARTING AUTO DRIVE ===");
    }

    if (started) {
        navigateStep();
    }

    delay(50);
}
