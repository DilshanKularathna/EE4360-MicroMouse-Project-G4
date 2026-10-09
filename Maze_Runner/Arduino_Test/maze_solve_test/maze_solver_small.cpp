#include <Arduino.h>
#include <Wire.h>
#include <util/atomic.h>
#include "config.h"
#include "Ultrasonic.h"
#include "motorDriver.h"
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

// MPU6050 is mounted with +Z down, so Z gyro rate is positive for a
// clockwise (right) turn. Calibrate the stationary bias at every boot.
const uint8_t MPU6050_ADDRESS = 0x68;
const uint8_t MPU6050_REG_PWR_MGMT_1 = 0x6B;
const uint8_t MPU6050_REG_GYRO_CONFIG = 0x1B;
const uint8_t MPU6050_REG_ACCEL_XOUT_H = 0x3B;
const float MPU6050_GYRO_LSB_PER_DPS = 131.0f;
const unsigned long GYRO_SAMPLE_INTERVAL_US = 10000UL;
const unsigned long GYRO_BIAS_CALIBRATION_MS = 3000UL;
const float TURN_STOP_TOLERANCE_DEG = 4.0f;
int16_t gyroOffsets[3] = {0, 0, 0};
bool gyroReady = false;
float gyroHeadingDeg = 0.0f;
float gyroHeadingTargetDeg = 0.0f;

// ================================================================
// GLOBAL VARIABLES
// ================================================================
float eprevL = 0, eintegralL = 0;
float eprevR = 0, eintegralR = 0;
long  prevT = 0;

const float kpL = 2.5f, kdL = 0.0f, kiL = 0.0f;
const float kpR = 2.5f, kdR = 0.0f, kiR = 0.0f;

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

bool readGyro(int16_t gyroRaw[3]) {
    Wire.beginTransmission(MPU6050_ADDRESS);
    Wire.write(MPU6050_REG_ACCEL_XOUT_H + 8); // GYRO_XOUT_H
    if (Wire.endTransmission(false) != 0 || Wire.requestFrom(MPU6050_ADDRESS, (uint8_t)6) != 6) {
        while (Wire.available()) Wire.read();
        return false;
    }
    for (uint8_t axis = 0; axis < 3; ++axis) {
        gyroRaw[axis] = (int16_t)((Wire.read() << 8) | Wire.read());
    }
    return true;
}

bool initializeGyroscope() {
    Wire.begin();
    Wire.setClock(100000);

    Wire.beginTransmission(MPU6050_ADDRESS);
    Wire.write(MPU6050_REG_PWR_MGMT_1);
    Wire.write(0x00); // Wake the MPU6050 and use the internal clock.
    if (Wire.endTransmission() != 0) return false;
    delay(100);

    Wire.beginTransmission(MPU6050_ADDRESS);
    Wire.write(MPU6050_REG_GYRO_CONFIG);
    Wire.write(0x00); // +/-250 degrees/second, 131 LSB per degree/second.
    if (Wire.endTransmission() != 0) return false;

    int32_t sums[3] = {0, 0, 0};
    uint16_t samples = 0;
    int16_t raw[3];
    const unsigned long startMs = millis();
    unsigned long nextSampleUs = micros();
    Serial.println(F("Keep robot still: calibrating gyro bias for 3 seconds."));
    while (millis() - startMs < GYRO_BIAS_CALIBRATION_MS) {
        if ((long)(micros() - nextSampleUs) >= 0) {
            if (!readGyro(raw)) return false;
            for (uint8_t axis = 0; axis < 3; ++axis) sums[axis] += raw[axis];
            ++samples;
            nextSampleUs += GYRO_SAMPLE_INTERVAL_US;
        }
    }
    if (samples == 0) return false;
    for (uint8_t axis = 0; axis < 3; ++axis) gyroOffsets[axis] = (int16_t)(sums[axis] / samples);
    Serial.print(F("Gyro bias raw X/Y/Z: "));
    Serial.print(gyroOffsets[0]); Serial.print('/');
    Serial.print(gyroOffsets[1]); Serial.print('/');
    Serial.println(gyroOffsets[2]);
    return true;
}

bool turnWithGyroscope(float targetDegrees) {
    if (!gyroReady) {
        stopMotors();
        Serial.println(F("Turn refused: MPU6050 is not ready."));
        return false;
    }

    const uint8_t yawAxis = MPU6050_YAW_AXIS_INDEX;
    const unsigned long timeoutMs = (fabs(targetDegrees) > 90.0f) ? 4500UL : 3000UL;
    const unsigned long startMs = millis();
    unsigned long nextSampleUs = micros();
    unsigned long lastSampleUs = nextSampleUs;
    float angle = 0.0f;
    int motorYawSign = 1;
    bool motorDirectionVerified = false;
    int16_t raw[3];

    Serial.print(F("Gyro turn target: ")); Serial.print(targetDegrees, 0); Serial.println(F(" deg"));
    while (fabs(targetDegrees - angle) > TURN_STOP_TOLERANCE_DEG && millis() - startMs < timeoutMs) {
        if ((long)(micros() - nextSampleUs) < 0) continue;
        if (!readGyro(raw)) {
            stopMotors();
            Serial.println(F("MPU6050 read failed during turn; motors stopped."));
            return false;
        }

        const unsigned long sampleUs = micros();
        const float dt = (sampleUs - lastSampleUs) / 1000000.0f;
        lastSampleUs = sampleUs;
        const float rateDps = (raw[yawAxis] - gyroOffsets[yawAxis]) / MPU6050_GYRO_LSB_PER_DPS;
        angle += rateDps * dt;
        gyroHeadingDeg += rateDps * dt;

        if (!motorDirectionVerified && millis() - startMs >= 200UL && fabs(angle) >= 2.0f) {
            motorYawSign = (angle * targetDegrees > 0.0f) ? 1 : -1;
            motorDirectionVerified = true;
        }

        const float error = targetDegrees - angle;
        const int direction = ((error > 0.0f) ? 1 : -1) * motorYawSign;
        const int duty = (fabs(error) < 20.0f) ? 18 : TURN_SPEED;
        setMotors(direction * duty, -direction * duty);
        nextSampleUs += GYRO_SAMPLE_INTERVAL_US;
    }

    stopMotors();
    delay(200); // Let the robot settle before reporting the final integrated angle.
    if (!readGyro(raw)) {
        Serial.println(F("MPU6050 read failed after turn."));
        return false;
    }
    const bool reached = fabs(targetDegrees - angle) <= TURN_STOP_TOLERANCE_DEG;
    if (reached) gyroHeadingTargetDeg += targetDegrees;
    Serial.print(F("Turn angle=")); Serial.print(angle, 1);
    Serial.print(F(" deg; target=")); Serial.print(targetDegrees, 0);
    Serial.println(reached ? F("; reached") : F("; timeout/under-turn"));
    return reached;
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
    unsigned long startTime          = millis();
    unsigned long lastCheck          = 0;
    unsigned long nextGyroSampleUs   = micros();
    unsigned long lastGyroSampleUs   = nextGyroSampleUs;
    float ultrasonicSteering         = 0.0f;
    float gyroSteering               = 0.0f;
    float         steeringAdjustment = 0;
    int16_t gyroRaw[3];

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

        float pwrL = fabs(uL);
        float pwrR = fabs(uR);

        if (pwrL > MAX_DUTY)
            pwrL = MAX_DUTY;
        if (pwrR > MAX_DUTY)
            pwrR = MAX_DUTY;

        pwrL -= steeringAdjustment;
        pwrR += steeringAdjustment;

        if (pwrL > MAX_DUTY + 5)
            pwrL = MAX_DUTY + 5;
        if (pwrL < 0)
            pwrL = 0;
        if (pwrR > MAX_DUTY + 5)
            pwrR = MAX_DUTY + 5;
        if (pwrR < 0)
            pwrR = 0;

        int dirL = (uL > 0) ? 1 : -1;
        int dirR = (uR > 0) ? 1 : -1;

        if (abs(eL) < 5 && abs(eR) < 5) {
            reached = true;
            stopMotors();
        } else {
            setMotors(dirL * pwrL, dirR * pwrR);
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
