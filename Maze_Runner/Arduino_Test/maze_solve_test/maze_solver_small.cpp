#include <Arduino.h>
#include <Wire.h>
#include <util/atomic.h>
#include "config.h"
#include "Ultrasonic.h"
#include "motorDriver.h"
#include "Gyroscope.h"
#include "globals.h"

// ================================================================
// CONSTANTS
// ================================================================
const long TARGET_TICKS_FWD = 288;

bool          started = false;
bool          turnFault = false;
unsigned long t0      = 0;

#define TURN_SPEED 25
const float MAX_DUTY = 28.0f;
const float WALL_STEER_GAIN = 1.5f;
const float MAX_WALL_STEER_DUTY = 8.0f;
const float WALL_ERROR_DEADBAND_CM = 0.5f;
const float GYRO_HEADING_GAIN = 0.8f;
const float MAX_GYRO_STEER_DUTY = 6.0f;
const float MAX_COMBINED_STEER_DUTY = 10.0f;
const float GYRO_HEADING_DEADBAND_DEG = 1.0f;
const float ENCODER_SYNC_GAIN = 0.35f;    // duty percentage per tick of wheel progress mismatch
const float MAX_ENCODER_SYNC_DUTY = 8.0f;
const long ENCODER_SYNC_DEADBAND_TICKS = 1;


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
bool readGyro(int16_t gyroRaw[3]);
bool turnWithGyroscope(float targetDegrees);

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
        if (!turnWithGyroscope(180.0f)) turnFault = true;
        return;
    }

    if (nextDir == REL_LEFT) {
        Serial.println("Turning left");
        if (!turnWithGyroscope(-90.0f)) {
            turnFault = true;
            return;
        }
        moveForwardPID();
        return;
    }

    if (nextDir == REL_RIGHT) {
        Serial.println("Turning right");
        if (!turnWithGyroscope(90.0f)) {
            turnFault = true;
            return;
        }
        moveForwardPID();
        return;
    }

    Serial.println("Moving forward");
    moveForwardPID();
}


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
    unsigned long lastGyroSampleUs   = nextGyroSampleUs;
    float ultrasonicSteering         = 0.0f;
    float gyroSteering               = 0.0f;
    float         steeringAdjustment = 0;
    int16_t gyroRaw[3];
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
            if (!readGyro(gyroRaw)) {
                stopMotors();
                turnFault = true;
                Serial.println(F("MPU6050 read failed during forward move; motors stopped."));
                return;
            }
            const unsigned long sampleUs = micros();
            const float gyroDt = (sampleUs - lastGyroSampleUs) / 1000000.0f;
            lastGyroSampleUs = sampleUs;
            const uint8_t yawAxis = MPU6050_YAW_AXIS_INDEX;
            const float yawRateDps = (gyroRaw[yawAxis] - gyroOffsets[yawAxis]) / MPU6050_GYRO_LSB_PER_DPS;
            gyroHeadingDeg += yawRateDps * gyroDt;

            // Positive clockwise heading error requires a counter-clockwise
            // wheel bias; ultrasonic correction is added below.
            float headingError = gyroHeadingDeg - gyroHeadingTargetDeg;
            if (fabs(headingError) < GYRO_HEADING_DEADBAND_DEG) headingError = 0.0f;
            gyroSteering = constrain(
                headingError * GYRO_HEADING_GAIN,
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

void setup() {
    Serial.begin(115200);
    delay(1000);

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
