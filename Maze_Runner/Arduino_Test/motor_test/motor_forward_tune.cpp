#include <Arduino.h>
#include <util/atomic.h>
#include "config.h"
#include "MotorDriver.h"

// Continuous straight-line tuning run. Change these values as you tune.
const long TARGET_TICKS_PER_CELL = 288;
const uint8_t TEST_CELLS = 6;
const float CRUISE_DUTY = 25.0f;
const float MIN_DUTY = 14.0f;       // Raise just above the measured reliable-move threshold.
const float RAMP_UP_TICKS = 40.0f;
const float RAMP_DOWN_TICKS = 70.0f;
const float SYNC_KP = 0.57f;
const float SYNC_KI = 0.40f;
const float SYNC_MAX_DUTY = 8.0f;
const long STOP_WINDOW_TICKS = 3;
const unsigned long CTRL_PERIOD_US = 10000UL;
const unsigned long MOVE_TIMEOUT_MS = 25000UL;

bool encoderCheckMode = false;
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

    Serial.println(F("Forward tuning sketch ready."));
    Serial.println(F("Send e: manual encoder sign check; push forward, then send s."));
    Serial.println(F("Send r: run 6 cells (150 cm) continuously with encoder sync."));
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
            Serial.print(F("Encoder check ended: "));
            printEncoderCounts();
        } else if (command == 'r' || command == 'R') {
            encoderCheckMode = false;
            runContinuousForwardTest();
            Serial.println(F("Ready. Send r to repeat, or e to check encoder signs."));
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
    bool timedOut = false;

    resetEncoders();
    const unsigned long startMs = millis();
    unsigned long lastUs = micros();
    unsigned long nextUs = lastUs + CTRL_PERIOD_US;
    Serial.print(F("Starting continuous run: "));
    Serial.print(TEST_CELLS);
    Serial.println(F(" cells."));

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

        while (averageTicks >= nextCellReport && nextCellReport <= targetTicks) {
            Serial.print(F("Cell "));
            Serial.print(nextCellReport / TARGET_TICKS_PER_CELL);
            Serial.print(F(" L="));
            Serial.print(leftTicks);
            Serial.print(F(" R="));
            Serial.print(rightTicks);
            Serial.print(F(" trim="));
            Serial.println(syncTrim, 2);
            nextCellReport += TARGET_TICKS_PER_CELL;
        }

        const float rampUpDuty = MIN_DUTY +
            (CRUISE_DUTY - MIN_DUTY) * (averageTicks / RAMP_UP_TICKS);
        const float rampDownDuty = MIN_DUTY +
            (CRUISE_DUTY - MIN_DUTY) * (remainingTicks / RAMP_DOWN_TICKS);
        float duty = min(CRUISE_DUTY, min(rampUpDuty, rampDownDuty));
        duty = max(duty, MIN_DUTY);

        const float tickError = leftTicks - rightTicks;
        syncTrim = constrain(syncTrim + SYNC_KI * tickError * dt,
                             -SYNC_MAX_DUTY, SYNC_MAX_DUTY);
        const float steering = constrain(SYNC_KP * tickError + syncTrim,
                                         -SYNC_MAX_DUTY, SYNC_MAX_DUTY);
        const float leftDuty = constrain(duty - steering, 0.0f,
                                         CRUISE_DUTY + SYNC_MAX_DUTY);
        const float rightDuty = constrain(duty + steering, 0.0f,
                                          CRUISE_DUTY + SYNC_MAX_DUTY);
        setMotors((int)(leftDuty + 0.5f), (int)(rightDuty + 0.5f));
    }

    stopMotors();
    delay(200);

    long leftEnd, rightEnd;
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        leftEnd = leftEncoderCount;
        rightEnd = rightEncoderCount;
    }
    const float averageEnd = 0.5f * (leftEnd + rightEnd);
    Serial.print(F("Final L="));
    Serial.print(leftEnd);
    Serial.print(F(" R="));
    Serial.print(rightEnd);
    Serial.print(F(" trim="));
    Serial.println(syncTrim, 2);
    Serial.print(F("Distance: "));
    Serial.print(averageEnd * 25.0f / TARGET_TICKS_PER_CELL, 1);
    Serial.println(F(" cm"));
    if (timedOut) Serial.println(F("Move timed out before reaching target."));
}
