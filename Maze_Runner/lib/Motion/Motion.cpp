#include "Motion.h"

#include <util/atomic.h>
#include "config.h"
#include "globals.h"
#include "MotorDriver.h"
#include "Ultrasonic.h"
#include "Gyro.h"

// Wheel position P control (same gains as the tested sketch; its I and D were 0)
static const float KP_LEFT  = 4.0f;
static const float KP_RIGHT = 4.5f;

static const unsigned long MOVE_TIMEOUT_MS   = 4000;
static const unsigned long SENSOR_PERIOD_MS  = 25;     // one sensor per slot, so echoes do not mix
static const unsigned long ALIGN_TIMEOUT_MS  = 1500;
static const float         ALIGN_TOLERANCE_CM = 0.4f;

static float headingRef = 0.0f;   // absolute heading target (deg, multiple of 90, right = +)

static float median3(float a, float b, float c) {
    if (a > b) { float t = a; a = b; b = t; }
    if (b > c) { float t = b; b = c; c = t; }
    if (a > b) { float t = a; a = b; b = t; }
    return b;
}

// Positive = robot is too close to the right wall (needs to turn left).
static float wallError(float l, float r) {
    bool wallL = l < WALL_THRESHOLD;
    bool wallR = r < WALL_THRESHOLD;
    if (wallL && wallR) return l - r;
    if (wallR)          return WALL_CENTER_CM - r;
    if (wallL)          return l - WALL_CENTER_CM;
    return 0.0f;
}

static void readEncoders(long& l, long& r) {
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        l = leftEncoderCount;
        r = rightEncoderCount;
    }
}

void motionBegin() {
    initEncoders();

    pinMode(PIN_LEFT_RPWM, OUTPUT);
    pinMode(PIN_LEFT_LPWM, OUTPUT);
    pinMode(PIN_RIGHT_RPWM, OUTPUT);
    pinMode(PIN_RIGHT_LPWM, OUTPUT);
    stopMotors();

    pinMode(PIN_ULTRA_FRONT_TRIG, OUTPUT);
    pinMode(PIN_ULTRA_FRONT_ECHO, INPUT);
    pinMode(PIN_ULTRA_LEFT_TRIG, OUTPUT);
    pinMode(PIN_ULTRA_LEFT_ECHO, INPUT);
    pinMode(PIN_ULTRA_RIGHT_TRIG, OUTPUT);
    pinMode(PIN_ULTRA_RIGHT_ECHO, INPUT);

    if (gyroBegin()) {
        gyroCalibrate();
        gyroResetYaw();
    }
    headingRef = 0.0f;
}

void motionResetHeading() {
    if (gyroReady()) {
        gyroUpdate();
        headingRef = gyroYaw();
    }
}

float motionHeadingError() {
    if (!gyroReady()) return 0.0f;
    gyroUpdate();
    return gyroYaw() - headingRef;
}

void motionRecalibrateGyro() {
    if (gyroReady()) gyroCalibrate(100);
}

float motionReadDistance(int trigPin, int echoPin) {
    float a = readUltrasonic(trigPin, echoPin);
    delay(10);
    float b = readUltrasonic(trigPin, echoPin);
    delay(10);
    float c = readUltrasonic(trigPin, echoPin);
    return median3(a, b, c);
}

void motionAlignFront() {
    unsigned long start = millis();
    while (millis() - start < ALIGN_TIMEOUT_MS) {
        float f = readUltrasonic(PIN_ULTRA_FRONT_TRIG, PIN_ULTRA_FRONT_ECHO);
        if (f > FRONT_WALL_CM) break;                 // no wall ahead after all
        float err = f - FRONT_TARGET_CM;              // > 0: too far from the wall
        if (fabs(err) <= ALIGN_TOLERANCE_CM) break;
        int duty = (err > 0) ? FRONT_ALIGN_DUTY : -FRONT_ALIGN_DUTY;
        setMotors(duty, duty);
        delay(30);
    }
    stopMotors();
    delay(100);
}

void motionTurnBy(float deltaDeg) {
    if (!gyroReady()) {
        turnDegrees(deltaDeg);
        return;
    }
    headingRef += deltaDeg;
    turnDegrees(headingRef - gyroYaw());
}

float motionForwardCell() {
    resetEncoders();

    const long target = (long)TICKS_PER_CELL_25CM;

    unsigned long startTime = millis();
    unsigned long lastCheck = 0;
    float steering   = 0.0f;        // positive = turn left
    float desiredYaw = headingRef;

    // Last 3 side readings (median filter). Start "no wall" so the first
    // samples cannot cause a sudden steer.
    float lBuf[3] = {100.0f, 100.0f, 100.0f};
    float rBuf[3] = {100.0f, 100.0f, 100.0f};
    int   sensorTurn = 0;           // 0 = front, 1 = left, 2 = right

    while (millis() - startTime < MOVE_TIMEOUT_MS) {
        if (millis() - lastCheck >= SENSOR_PERIOD_MS) {
            lastCheck  = millis();
            sensorTurn = (sensorTurn + 1) % 3;

            if (sensorTurn == 0) {
                float f = readUltrasonic(PIN_ULTRA_FRONT_TRIG, PIN_ULTRA_FRONT_ECHO);
                if (f > 0.1f && f <= FRONT_TARGET_CM + FRONT_SLOW_CM) break;   // wall ahead
            } else {
                float d = (sensorTurn == 1) ? readUltrasonic(PIN_ULTRA_LEFT_TRIG, PIN_ULTRA_LEFT_ECHO)
                                            : readUltrasonic(PIN_ULTRA_RIGHT_TRIG, PIN_ULTRA_RIGHT_ECHO);
                float* buf = (sensorTurn == 1) ? lBuf : rBuf;
                if (d >= 0.0f) {
                    buf[0] = buf[1];
                    buf[1] = buf[2];
                    buf[2] = d;
                }
                float wallErr = wallError(median3(lBuf[0], lBuf[1], lBuf[2]),
                                          median3(rBuf[0], rBuf[1], rBuf[2]));
                if (gyroReady())    // walls only nudge the heading target
                    desiredYaw = headingRef + constrain(-WALL_YAW_GAIN * wallErr, -MAX_WALL_YAW, MAX_WALL_YAW);
                else
                    steering = constrain(WALL_STEER_GAIN * wallErr, -5.0f, 5.0f);
            }
        }

        if (gyroReady()) {          // the gyro holds the heading on every loop
            gyroUpdate();
            steering = constrain(HEADING_KP * (gyroYaw() - desiredYaw), -5.0f, 5.0f);
        }

        long posL, posR;
        readEncoders(posL, posR);
        float eL = (float)(target - posL);
        float eR = (float)(target - posR);

        if (fabs(eL) < 5 && fabs(eR) < 5) break;

        float uL = KP_LEFT * eL;
        float uR = KP_RIGHT * eR;
        float pwrL = constrain(fabs(uL), 0.0f, MAX_DUTY) - steering;
        float pwrR = constrain(fabs(uR), 0.0f, MAX_DUTY) + steering;
        pwrL = constrain(pwrL, 0.0f, MAX_DUTY + 5);
        pwrR = constrain(pwrR, 0.0f, MAX_DUTY + 5);
        if (pwrL > 0 && pwrL < MIN_DUTY_LEFT)  pwrL = MIN_DUTY_LEFT;
        if (pwrR > 0 && pwrR < MIN_DUTY_RIGHT) pwrR = MIN_DUTY_RIGHT;

        setMotors((uL > 0 ? 1 : -1) * (int)pwrL, (uR > 0 ? 1 : -1) * (int)pwrR);
    }

    stopMotors();
    delay(200);

    long posL, posR;
    readEncoders(posL, posR);
    return (posL + posR) / 2.0f / TICKS_PER_CM;
}
