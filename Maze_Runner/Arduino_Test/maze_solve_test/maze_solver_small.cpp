#include <Arduino.h>
#include <Wire.h>
#include <util/atomic.h>
#include "config.h"
#include "Ultrasonic.h"
#include "MotorDriver.h"
#include "Gyroscope.h"

// ================================================================
// CONSTANTS
// ================================================================
const long TARGET_TICKS_FWD = 288;
const float CRUISE_DUTY = 25.0f;
float syncTrim = 0.0f;

Gyroscope gyroscope(Wire, true);
const unsigned long GYRO_SAMPLE_INTERVAL_US = 10000UL;
bool gyroReady = false;
float gyroHeadingDeg = 0.0f;
float gyroHeadingTargetDeg = 0.0f;

bool          started = false;
bool          turnFault = false;
unsigned long t0      = 0;

#define TURN_SPEED 25
const float MAX_DUTY = 28.0f;
const float MIN_DUTY          = 14.0f;

const float RAMP_UP_TICKS     = 40.0f;  // ticks to go from MIN_DUTY to CRUISE_DUTY
const float RAMP_DOWN_TICKS   = 70.0f;  // start slowing this many ticks before the target
const float SYNC_KP           = 0.35f;  // duty % per tick of (L - R) error
const float SYNC_KI           = 0.40f;  // duty % per (tick * second) -> learns the motor mismatch
const float SYNC_MAX_DUTY     = 8.0f;
const float WALL_TICKS_PER_CM = 0.5f;   // wall error (cm) -> wanted (L - R) offset (ticks)

const float WALL_MAX_TICKS    = 6.0f;
const long  STOP_WINDOW_TICKS = 3;
const unsigned long CTRL_PERIOD_US = 10000;   // control loop = 10 ms

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

const float kpL = 6.5f, kdL = 0.0f, kiL = 0.0f;
const float kpR = 6.5f, kdR = 0.0f, kiR = 0.0f;

float frontDist, leftDist, rightDist;

enum RelativeDir { REL_LEFT = -1, REL_FRONT = 0, REL_RIGHT = 1, REL_DEAD = 99 };

void moveForwardPID();
void moveForwardGyroPID();
void  navigateStep();
bool initializeGyroscope();
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
        turnAround();
        return;
    }

    if (nextDir == REL_LEFT) {
        Serial.println("Turning left");
        turnLeft();
        moveForwardPID();
        return;
    }

    if (nextDir == REL_RIGHT) {
        Serial.println("Turning right");
        turnRight();
        moveForwardPID();
        return;
    }

    Serial.println("Moving forward");
    moveForwardPID();
}

void navigateGyroStep() {
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
        moveForwardGyroPID();
        return;
    }

    if (nextDir == REL_RIGHT) {
        Serial.println("Turning right");
        if (!turnWithGyroscope(90.0f)) {
            turnFault = true;
            return;
        }
        moveForwardGyroPID();
        return;
    }

    Serial.println("Moving forward");
    moveForwardGyroPID();
}

// void moveForwardPID() {
//     resetEncoders();

//     long targetL = TARGET_TICKS_FWD;
//     long targetR = TARGET_TICKS_FWD;

//     eprevL     = 0;
//     eintegralL = 0;
//     eprevR     = 0;
//     eintegralR = 0;
//     prevT      = micros();

//     bool          reached            = false;
//     unsigned long startTime          = millis();
//     unsigned long lastCheck          = 0;
//     float         steeringAdjustment = 0;

//     while (!reached) {
//         if (millis() - startTime > 4000)
//             break;

//         if (millis() - lastCheck > 50) {
//             float f = readUltrasonic(PIN_ULTRA_FRONT_TRIG, PIN_ULTRA_FRONT_ECHO);
//             if (f < SAFETY_DIST && f > 0.1f) {
//                 Serial.println("OBSTACLE - STOP");
//                 reached = true;
//                 stopMotors();
//                 break;
//             }

//             float r = readUltrasonic(PIN_ULTRA_RIGHT_TRIG, PIN_ULTRA_RIGHT_ECHO);
//             float l = readUltrasonic(PIN_ULTRA_LEFT_TRIG, PIN_ULTRA_LEFT_ECHO);

//             steeringAdjustment = 0;

//             if (r > 0.1f && l > 0.1f) {
//                 if (r < CENTER_DIST) {
//                     steeringAdjustment = 5.0f;
//                 } else if (l < CENTER_DIST) {
//                     steeringAdjustment = -5.0f;
//                 }
//             } else if (r > 0.1f && l <= 0.1f) {
//                 if (r > 4.0f) {
//                     steeringAdjustment = 5.0f;
//                 }
//             } else if (l > 0.1f && r <= 0.1f) {
//                 if (l > 4.0f) {
//                     steeringAdjustment = -5.0f;
//                 }
//             }

//             lastCheck = millis();
//         }

//         long  currT  = micros();
//         float deltaT = (currT - prevT) / 1e6f;
//         prevT        = currT;
//         if (deltaT <= 0)
//             deltaT = 1e-3f;

//         long leftPos, rightPos;
//         ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
//             leftPos  = leftEncoderCount;
//             rightPos = rightEncoderCount;
//         }

//         float eL    = (float)targetL - (float)leftPos;
//         float dedtL = (eL - eprevL) / deltaT;
//         eintegralL += eL * deltaT;
//         float uL = kpL * eL + kdL * dedtL + kiL * eintegralL;

//         float eR    = (float)targetR - (float)rightPos;
//         float dedtR = (eR - eprevR) / deltaT;
//         eintegralR += eR * deltaT;
//         float uR = kpR * eR + kdR * dedtR + kiR * eintegralR;

//         float pwrL = fabs(uL);
//         float pwrR = fabs(uR);

//         if (pwrL > MAX_DUTY)
//             pwrL = MAX_DUTY;
//         if (pwrR > MAX_DUTY)
//             pwrR = MAX_DUTY;

//         pwrL -= steeringAdjustment;
//         pwrR += steeringAdjustment;

//         if (pwrL > MAX_DUTY + 5)
//             pwrL = MAX_DUTY + 5;
//         if (pwrL < 0)
//             pwrL = 0;
//         if (pwrR > MAX_DUTY + 5)
//             pwrR = MAX_DUTY + 5;
//         if (pwrR < 0)
//             pwrR = 0;

//         int dirL = (uL > 0) ? 1 : -1;
//         int dirR = (uR > 0) ? 1 : -1;

//         if (abs(eL) < 5 && abs(eR) < 5) {
//             reached = true;
//             stopMotors();
//         } else {
//             setMotors(dirL * pwrL, dirR * pwrR);
//         }

//         eprevL = eL;
//         eprevR = eR;
//     }
//     stopMotors();
//     delay(200);
// }

void moveForwardPID() {
    resetEncoders();
 
    const unsigned long startMs = millis();
    unsigned long lastSenseMs   = 0;
    unsigned long lastUs        = micros();
    unsigned long nextUs        = lastUs + CTRL_PERIOD_US;
    uint8_t senseIdx            = 0;
    float f = 999.0f, l = 0.0f, r = 0.0f;
    float diffTarget            = 0.0f;   // wanted (L - R) in ticks, set by wall sensing
 
    while (true) {
        if (millis() - startMs > 4000) break;
 
        // ---- sensors: ONE ultrasonic every 25 ms so the loop is never blocked for long ----
        if (millis() - lastSenseMs >= 25) {
            lastSenseMs = millis();
            if (senseIdx == 0)      f = readUltrasonic(PIN_ULTRA_FRONT_TRIG, PIN_ULTRA_FRONT_ECHO);
            else if (senseIdx == 1) r = readUltrasonic(PIN_ULTRA_RIGHT_TRIG, PIN_ULTRA_RIGHT_ECHO);
            else                    l = readUltrasonic(PIN_ULTRA_LEFT_TRIG,  PIN_ULTRA_LEFT_ECHO);
            senseIdx = (senseIdx + 1) % 3;
 
            if (f < SAFETY_DIST && f > 0.1f) {
                Serial.println("OBSTACLE - STOP");
                break;
            }
 
            // Wall error: positive means "steer left".
            const bool rSeen = (r > 0.1f && r <= WALL_THRESHOLD);
            const bool lSeen = (l > 0.1f && l <= WALL_THRESHOLD);
            float wallErr = 0.0f;
            if (lSeen && rSeen) wallErr = l - r;
            else if (rSeen)     wallErr = CENTER_DIST - r;
            else if (lSeen)     wallErr = l - CENTER_DIST;
            if (fabs(wallErr) < 0.5f) wallErr = 0.0f;
 
            // Steer left = right wheel must travel further = (L - R) should go negative.
            // The wall term MOVES the sync target instead of fighting the sync loop.
            diffTarget = -constrain(wallErr * WALL_TICKS_PER_CM, -WALL_MAX_TICKS, WALL_MAX_TICKS);
        }
 
        // ---- fixed-rate control update ----
        if ((long)(micros() - nextUs) < 0) continue;
        const unsigned long now = micros();
        const float dt = (now - lastUs) / 1e6f;
        lastUs  = now;
        nextUs += CTRL_PERIOD_US;
        if ((long)(now - nextUs) > 0) nextUs = now + CTRL_PERIOD_US;   // fell behind, resync
 
        long lp, rp;
        ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
            lp = leftEncoderCount;
            rp = rightEncoderCount;
        }
 
        const float avg       = 0.5f * (float)(lp + rp);
        const float remaining = (float)TARGET_TICKS_FWD - avg;
        if (remaining <= STOP_WINDOW_TICKS) break;      // both wheels stop together
 
        // ---- shared speed profile (same duty for both wheels) ----
        const float up   = MIN_DUTY + (CRUISE_DUTY - MIN_DUTY) * (avg / RAMP_UP_TICKS);
        const float down = MIN_DUTY + (CRUISE_DUTY - MIN_DUTY) * (remaining / RAMP_DOWN_TICKS);
        float duty = min(CRUISE_DUTY, min(up, down));
        duty = max(duty, MIN_DUTY);
 
        // ---- wheel sync: P on the tick difference + learned trim ----
        const float diffErr = (float)(lp - rp) - diffTarget;   // + means left is ahead
        syncTrim += SYNC_KI * diffErr * dt;
        syncTrim  = constrain(syncTrim, -SYNC_MAX_DUTY, SYNC_MAX_DUTY);
        const float steer = constrain(SYNC_KP * diffErr + syncTrim, -SYNC_MAX_DUTY, SYNC_MAX_DUTY);
 
        const float pwrL = constrain(duty - steer, 0.0f, CRUISE_DUTY + SYNC_MAX_DUTY);
        const float pwrR = constrain(duty + steer, 0.0f, CRUISE_DUTY + SYNC_MAX_DUTY);
        setMotors((int)(pwrL + 0.5f), (int)(pwrR + 0.5f));
    }
 
    stopMotors();
    delay(200);
 
    long lEnd, rEnd;
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        lEnd = leftEncoderCount;
        rEnd = rightEncoderCount;
    }
    Serial.print(F("Move done  L="));
    Serial.print(lEnd);
    Serial.print(F("  R="));
    Serial.print(rEnd);
    Serial.print(F("  trim="));
    Serial.println(syncTrim);
}

void moveForwardGyroPID() {
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

        if ((long)(micros() - nextGyroSampleUs) >= 0) {
            gyroscope.update();
            const unsigned long sampleUs = micros();
            const float gyroDt = (sampleUs - lastGyroSampleUs) / 1000000.0f;
            lastGyroSampleUs = sampleUs;
            gyroHeadingDeg += gyroscope.getYawRateDps() * gyroDt;

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

        if (leftPos >= targetL - 5) leftDone = true;
        if (rightPos >= targetR - 5) rightDone = true;

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

        if (pwrL > MAX_DUTY) pwrL = MAX_DUTY;
        if (pwrR > MAX_DUTY) pwrR = MAX_DUTY;

        if (!leftDone && eL > 0 && eL <= 25 && pwrL > 20.0f) pwrL = 20.0f;
        if (!rightDone && eR > 0 && eR <= 25 && pwrR > 20.0f) pwrR = 20.0f;

        pwrL -= totalSteering;
        pwrR += totalSteering;

        if (pwrL > MAX_DUTY + 5) pwrL = MAX_DUTY + 5;
        if (pwrL < 0) pwrL = 0;
        if (pwrR > MAX_DUTY + 5) pwrR = MAX_DUTY + 5;
        if (pwrR < 0) pwrR = 0;

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

        eprevL = eL;
        eprevR = eR;
    }
    stopMotors();
    delay(200);
}

bool initializeGyroscope() {
    if (!gyroscope.begin() || !gyroscope.calibrate()) return false;
    gyroscope.update();
    gyroHeadingDeg = gyroscope.getYawAngleDeg();
    gyroHeadingTargetDeg = gyroHeadingDeg;
    return true;
}

bool turnWithGyroscope(float targetDegrees) {
    if (fabs(targetDegrees) >= 180.0f) turnAround();
    else if (targetDegrees < 0.0f) turnLeft();
    else turnRight();
    gyroHeadingTargetDeg += targetDegrees;
    return true;
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
