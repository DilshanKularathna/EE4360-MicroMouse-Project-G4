#include <Arduino.h>
#include <Wire.h>
#include <util/atomic.h>
#include "config.h"
#include "MotorDriver.h"
#include "Gyroscope.h"

const long TARGET_TICKS_PER_CELL = 288;
const uint8_t TEST_CELLS = 6;
const float CRUISE_DUTY = 25.0f;
const float MIN_DUTY = 14.0f;
const float RAMP_UP_TICKS = 40.0f;
const float RAMP_DOWN_TICKS = 70.0f;
const float SYNC_KP = 0.50f;
const float SYNC_KI = 0.02f; // Reduced because the synchronized error now includes gyro heading error.
const float SYNC_MAX_DUTY = 8.0f;
const float GYRO_HEADING_DEADBAND_DEG = 0.2f;
const float GYRO_DIFF_TICKS_PER_DEG = 2.0f * TICKS_PER_90_DEG / 90.0f;
const float GYRO_HEADING_RESPONSE_PER_SEC = 1.0f;
const float ENCODER_RATE_FILTER_SECONDS = 0.20f;
const float GYRO_RATE_LOOKAHEAD_SECONDS = 0.0f; // Avoid steering oscillations from gyro-rate spikes.
const long STOP_WINDOW_TICKS = 3;
const unsigned long CTRL_PERIOD_US = 10000UL;
const unsigned long MOVE_TIMEOUT_MS = 25000UL;

Gyroscope gyroscope(Wire, false); // Sensor is upright.
bool gyroReady = false;
bool encoderCheckMode = false;
float startHeadingDeg = 0.0f;
unsigned long lastEncoderReportMs = 0;

void runContinuousForwardTest();
void printEncoderCounts();

void setup() {
    Serial.begin(115200);

    pinMode(PIN_LEFT_ENC_A, INPUT_PULLUP);
    pinMode(PIN_LEFT_ENC_B, INPUT_PULLUP);
    pinMode(PIN_RIGHT_ENC_A, INPUT_PULLUP);
    pinMode(PIN_RIGHT_ENC_B, INPUT_PULLUP);
    pinMode(PIN_LEFT_RPWM, OUTPUT);
    pinMode(PIN_LEFT_LPWM, OUTPUT);
    pinMode(PIN_RIGHT_RPWM, OUTPUT);
    pinMode(PIN_RIGHT_LPWM, OUTPUT);
    attachInterrupt(digitalPinToInterrupt(PIN_LEFT_ENC_A), leftEncoderISR, RISING);
    attachInterrupt(digitalPinToInterrupt(PIN_RIGHT_ENC_A), rightEncoderISR, RISING);
    stopMotors();

    Serial.println(F("Keep robot still: starting gyro calibration."));
    delay(1000);
    gyroReady = gyroscope.begin() && gyroscope.calibrate();
    if (gyroReady) {
        gyroscope.update();
        startHeadingDeg = gyroscope.getYawAngleDeg();
        Serial.println(F("Gyro ready. Send e for encoder check or r for 6-cell run."));
    } else {
        Serial.println(F("Gyro initialization failed; forward test disabled."));
    }
}

void loop() {
    if (Serial.available()) {
        const char command = Serial.read();
        if (command == 'e' || command == 'E') {
            stopMotors();
            resetEncoders();
            encoderCheckMode = true;
            lastEncoderReportMs = 0;
            Serial.println(F("Push robot forward by hand; both counts should increase. Send s to stop."));
        } else if (command == 's' || command == 'S') {
            encoderCheckMode = false;
            stopMotors();
            Serial.print(F("Encoder check: "));
            printEncoderCounts();
        } else if (command == 'r' || command == 'R') {
            encoderCheckMode = false;
            if (gyroReady) {
                runContinuousForwardTest();
                Serial.println(F("Ready. Send r to repeat, or e to check encoder signs."));
            }
        }
    }

    if (encoderCheckMode && millis() - lastEncoderReportMs >= 250) {
        lastEncoderReportMs = millis();
        printEncoderCounts();
    }
}

void printEncoderCounts() {
    long leftTicks, rightTicks;
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        leftTicks = leftEncoderCount;
        rightTicks = rightEncoderCount;
    }
    Serial.print(F("L="));
    Serial.print(leftTicks);
    Serial.print(F(" R="));
    Serial.println(rightTicks);
}

void runContinuousForwardTest() {
    const long targetTicks = TARGET_TICKS_PER_CELL * TEST_CELLS;
    long nextCellReport = TARGET_TICKS_PER_CELL;
    float syncTrim = 0.0f;
    float filteredWheelDiffRate = 0.0f;
    bool timedOut = false;

    resetEncoders();
    long previousLeftTicks = 0;
    long previousRightTicks = 0;
    gyroscope.update();
    startHeadingDeg = gyroscope.getYawAngleDeg();
    const unsigned long startMs = millis();
    unsigned long lastUs = micros();
    unsigned long nextUs = lastUs + CTRL_PERIOD_US;
    Serial.println(F("Starting continuous 6-cell gyro + encoder run."));

    while (true) {
        if (millis() - startMs >= MOVE_TIMEOUT_MS) {
            timedOut = true;
            break;
        }
        if ((long)(micros() - nextUs) < 0) continue;

        const unsigned long now = micros();
        const float dt = (now - lastUs) / 1000000.0f;
        lastUs = now;
        nextUs += CTRL_PERIOD_US;
        if ((long)(now - nextUs) > 0) nextUs = now + CTRL_PERIOD_US;

        long leftTicks, rightTicks;
        ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
            leftTicks = leftEncoderCount;
            rightTicks = rightEncoderCount;
        }

        const float averageTicks = 0.5f * (leftTicks + rightTicks);
        const float remainingTicks = targetTicks - averageTicks;
        if (remainingTicks <= STOP_WINDOW_TICKS) break;

        gyroscope.update();
        float rawHeadingError = gyroscope.getYawAngleDeg() - startHeadingDeg;
        if (rawHeadingError > 180.0f) rawHeadingError -= 360.0f;
        if (rawHeadingError < -180.0f) rawHeadingError += 360.0f;
        const float yawRateDps = gyroscope.getYawRateDps();
        float headingError = rawHeadingError + yawRateDps * GYRO_RATE_LOOKAHEAD_SECONDS;
        if (headingError > 180.0f) headingError -= 360.0f;
        if (headingError < -180.0f) headingError += 360.0f;
        if (fabs(headingError) < GYRO_HEADING_DEADBAND_DEG) headingError = 0.0f;
        const float leftDelta = leftTicks - previousLeftTicks;
        const float rightDelta = rightTicks - previousRightTicks;
        previousLeftTicks = leftTicks;
        previousRightTicks = rightTicks;
        const float measuredWheelDiffRate = (leftDelta - rightDelta) / dt;
        const float filterAlpha = dt / (ENCODER_RATE_FILTER_SECONDS + dt);
        filteredWheelDiffRate += filterAlpha *
                                 (measuredWheelDiffRate - filteredWheelDiffRate);
        const float gyroDiffRateTarget = headingError * GYRO_DIFF_TICKS_PER_DEG *
                                         GYRO_HEADING_RESPONSE_PER_SEC;

        const float rampUpDuty = MIN_DUTY +
            (CRUISE_DUTY - MIN_DUTY) * (averageTicks / RAMP_UP_TICKS);
        const float rampDownDuty = MIN_DUTY +
            (CRUISE_DUTY - MIN_DUTY) * (remainingTicks / RAMP_DOWN_TICKS);
        float duty = min(CRUISE_DUTY, min(rampUpDuty, rampDownDuty));
        duty = max(duty, MIN_DUTY);

        const float syncErrorRate = filteredWheelDiffRate - gyroDiffRateTarget;
        syncTrim = constrain(syncTrim + SYNC_KI * syncErrorRate * dt,
                             -SYNC_MAX_DUTY, SYNC_MAX_DUTY);
        const float encoderSteering = constrain(SYNC_KP * syncErrorRate + syncTrim,
                                                -SYNC_MAX_DUTY, SYNC_MAX_DUTY);
        const float steering = encoderSteering;

        while (averageTicks >= nextCellReport && nextCellReport <= targetTicks) {
            Serial.print(F("Cell "));
            Serial.print(nextCellReport / TARGET_TICKS_PER_CELL);
            Serial.print(F(" L="));
            Serial.print(leftTicks);
            Serial.print(F(" R="));
            Serial.print(rightTicks);
            Serial.print(F(" encTrim="));
            Serial.print(syncTrim, 2);
            Serial.print(F(" syncErrRate="));
            Serial.print(syncErrorRate, 1);
            Serial.print(F(" wheelDiffRate="));
            Serial.print(filteredWheelDiffRate, 1);
            Serial.print(F(" gyroTargetRate="));
            Serial.print(gyroDiffRateTarget, 1);
            Serial.print(F(" steer="));
            Serial.print(steering, 2);
            Serial.print(F(" yaw="));
            Serial.print(rawHeadingError, 2);
            Serial.print(F(" yawRate="));
            Serial.print(yawRateDps, 2);
            Serial.print(F(" predYaw="));
            Serial.println(headingError, 2);
            nextCellReport += TARGET_TICKS_PER_CELL;
        }

        const float leftDuty = constrain(duty - steering, 0.0f,
                                         CRUISE_DUTY + SYNC_MAX_DUTY);
        const float rightDuty = constrain(duty + steering, 0.0f,
                                          CRUISE_DUTY + SYNC_MAX_DUTY);
        setMotors((int)(leftDuty + 0.5f), (int)(rightDuty + 0.5f));
    }

    gyroscope.update();
    const float yawBeforeStop = gyroscope.getYawAngleDeg() - startHeadingDeg;
    const float yawRateBeforeStop = gyroscope.getYawRateDps();
    stopMotors();
    delay(200);
    long leftEnd, rightEnd;
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        leftEnd = leftEncoderCount;
        rightEnd = rightEncoderCount;
    }
    gyroscope.update();
    const float yawAfterCoast = gyroscope.getYawAngleDeg() - startHeadingDeg;
    Serial.print(F("Final L="));
    Serial.print(leftEnd);
    Serial.print(F(" R="));
    Serial.print(rightEnd);
    Serial.print(F(" encTrim="));
    Serial.print(syncTrim, 2);
    Serial.print(F(" yawBeforeStop="));
    Serial.print(yawBeforeStop, 2);
    Serial.print(F(" yawRateBeforeStop="));
    Serial.print(yawRateBeforeStop, 2);
    Serial.print(F(" yawAfterCoast="));
    Serial.println(yawAfterCoast, 2);
    if (timedOut) Serial.println(F("Move timed out before reaching target."));
}