#include "SwitchToLine.h"

#include <util/atomic.h>
#include <stdlib.h>

#include "config.h"
#include "MotorDriver.h"

void SwitchToLine::begin() {
    pinMode(PIN_IR_EN, OUTPUT);
    digitalWrite(PIN_IR_EN, HIGH);
    for (uint8_t i = 0; i < NUM_IR_SENSORS; ++i) {
        pinMode(IR_PINS[i], INPUT);
    }
}

int SwitchToLine::readSensors(int *rawValues) const {
    int blackCount = 0;
    for (uint8_t i = 0; i < NUM_IR_SENSORS; ++i) {
        rawValues[i] = analogRead(IR_PINS[i]);
        const int threshold = (WHITE_VALUES[i] + BLACK_VALUES[i]) / 2;
        if (rawValues[i] > threshold) {
            ++blackCount;
        }
    }
    return blackCount;
}

bool SwitchToLine::observeMarker() {
    if (state != SEARCHING_FOR_MARKER) {
        return false;
    }

    int rawValues[NUM_IR_SENSORS];
    const int blackCount = readSensors(rawValues);
    int surface = 0;
    if (blackCount <= SWITCH_MARKER_WHITE_MAX_BLACK_SENSORS) {
        surface = 1;
    } else if (blackCount >= SWITCH_MARKER_BLACK_MIN_BLACK_SENSORS) {
        surface = -1;
    }

    const int expectedSurface = (markerStage % 2 == 0) ? 1 : -1;
    if (surface == expectedSurface) {
        if (++stableSamples >= SWITCH_MARKER_STABLE_SAMPLES) {
            ++markerStage;
            stableSamples = 0;
            if (markerStage == 4) {
                state = CREEPING_TO_LINE;
                creepStartMs = millis();
                resetEncoders();
                Serial.println(F("Maze-to-line marker detected; creeping forward to find the line."));
                return true;
            }
        }
    } else {
        stableSamples = 0;
        if (surface == 1 && markerStage > 0) {
            markerStage = 1;
            stableSamples = 1;
        }
    }
    return false;
}

bool SwitchToLine::isLinePresent(int blackCount,
                                int whiteCount) const {
    return whiteCount >= SWITCH_TO_LINE_MIN_WHITE_SENSORS &&
           blackCount >= SWITCH_TO_LINE_MIN_BLACK_SENSORS;
}

void SwitchToLine::followLine(const int *rawValues) {
    long weightedSum = 0;
    long totalBlackness = 0;

    for (uint8_t i = 0; i < NUM_IR_SENSORS; ++i) {
        const int range = BLACK_VALUES[i] - WHITE_VALUES[i];
        const int blackness = constrain(
            (long)(rawValues[i] - WHITE_VALUES[i]) * 1000L / range,
            0L,
            1000L);
        weightedSum += (long)blackness * (i * 1000L);
        totalBlackness += blackness;
    }

    const int position = (totalBlackness > 0)
        ? (int)(weightedSum / totalBlackness)
        : (previousError < 0 ? 0 : 7000);
    const int error = position - 3500;
    integral = constrain(integral + error, -3000.0f, 3000.0f);
    const int derivative = error - previousError;
    previousError = error;

    const float correction = SWITCH_TO_LINE_KP * error +
                            SWITCH_TO_LINE_KI * integral +
                            SWITCH_TO_LINE_KD * derivative;
    const int leftSpeed = constrain(
        (int)(SWITCH_TO_LINE_BASE_SPEED - correction),
        -SWITCH_TO_LINE_MAX_SPEED,
        SWITCH_TO_LINE_MAX_SPEED);
    const int rightSpeed = constrain(
        (int)(SWITCH_TO_LINE_BASE_SPEED + correction),
        -SWITCH_TO_LINE_MAX_SPEED,
        SWITCH_TO_LINE_MAX_SPEED);
    setMotors(leftSpeed, rightSpeed);
}

void SwitchToLine::update() {
    int rawValues[NUM_IR_SENSORS];
    const int blackCount = readSensors(rawValues);
    const int whiteCount = NUM_IR_SENSORS - blackCount;

    if (state == CREEPING_TO_LINE) {
        if (isLinePresent(blackCount, whiteCount)) {
            stopMotors();
            state = FOLLOWING_LINE;
            integral = 0.0f;
            previousError = 0;
            Serial.println(F("Line acquired; maze navigation disabled, following line."));
            followLine(rawValues);
            return;
        }

        long leftTicks;
        long rightTicks;
        ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
            leftTicks = leftEncoderCount;
            rightTicks = rightEncoderCount;
        }
        const long progress = (labs(leftTicks) + labs(rightTicks)) / 2;
        const long maxTicks = (long)(
            SWITCH_TO_LINE_CREEP_MAX_CM * (TICKS_PER_CELL_25CM / 25.0f) + 0.5f);
        if (progress >= maxTicks ||
            millis() - creepStartMs >= SWITCH_TO_LINE_CREEP_TIMEOUT_MS) {
            stopMotors();
            state = BRIDGE_FAILED;
            Serial.println(F("Line not found within the configured creep distance/time; stopped."));
            failureReported = true;
            return;
        }

        setMotors(SWITCH_TO_LINE_CREEP_SPEED, SWITCH_TO_LINE_CREEP_SPEED);
        return;
    }

    if (state == FOLLOWING_LINE) {
        followLine(rawValues);
    } else if (state == BRIDGE_FAILED && !failureReported) {
        stopMotors();
        failureReported = true;
    }
}