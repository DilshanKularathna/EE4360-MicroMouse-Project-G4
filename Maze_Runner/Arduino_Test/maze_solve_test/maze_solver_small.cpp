#include <Arduino.h>
#include <util/atomic.h>
#include "config.h"
#include "Ultrasonic.h"
#include "motorDriver.h"

// ================================================================
// CONSTANTS
// ================================================================
const long TARGET_TICKS_FWD = 288;

bool          started = false;
unsigned long t0      = 0;

#define TURN_SPEED 25
const float MAX_DUTY = 28.0f;

// ================================================================
// GLOBAL VARIABLES
// ================================================================
float eprevL = 0, eintegralL = 0;
float eprevR = 0, eintegralR = 0;
long  prevT = 0;

const float kpL = 4.5f, kdL = 0.0f, kiL = 0.0f;
const float kpR = 4.0f, kdR = 0.0f, kiR = 0.0f;

float frontDist, leftDist, rightDist;

enum RelativeDir { REL_LEFT = -1, REL_FRONT = 0, REL_RIGHT = 1, REL_DEAD = 99 };

void moveForwardPID();
void  navigateStep();

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
    float         steeringAdjustment = 0;

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

            steeringAdjustment = 0;

            if (r > 0.1f && l > 0.1f) {
                if (r < CENTER_DIST) {
                    steeringAdjustment = 5.0f;
                } else if (l < CENTER_DIST) {
                    steeringAdjustment = -5.0f;
                }
            } else if (r > 0.1f && l <= 0.1f) {
                if (r > 4.0f) {
                    steeringAdjustment = 5.0f;
                }
            } else if (l > 0.1f && r <= 0.1f) {
                if (l > 4.0f) {
                    steeringAdjustment = -5.0f;
                }
            }

            lastCheck = millis();
        }

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
    t0    = millis();

    Serial.println("========================================");
    Serial.println("WALL FOLLOWER - READY");
    Serial.println("Auto start in 3 seconds...");
    Serial.println("Priority: Left -> Right -> Front");
    Serial.println("========================================");
}

void loop() {
    if (!started && millis() - t0 >= 3000) {
        started = true;
        Serial.println("\n=== STARTING AUTO DRIVE ===");
    }

    if (started) {
        navigateStep();
    }

    delay(50);
}