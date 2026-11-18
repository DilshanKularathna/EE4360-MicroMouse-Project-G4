#include <Arduino.h>
#include <util/atomic.h>
#include "config.h"
#include "MotorDriver.h"

// Continuous straight-line tuning using normalized wheel rotations.
const float LEFT_COUNTS_PER_REV = 8976.0f;
const float RIGHT_COUNTS_PER_REV = 8968.0f;
const uint8_t TEST_CELLS = 6;
const float CELL_DISTANCE_CM = 25.0f;

const float CRUISE_DUTY = 25.0f;
const float MIN_DUTY = 14.0f;       // Raise just above the measured reliable-move threshold.
const float RAMP_UP_DISTANCE_CM = 5.0f;
const float RAMP_DOWN_DISTANCE_CM = 7.0f;
const float SYNC_KP = 0.57f;
const float SYNC_KI = 0.40f;
const float SYNC_MAX_DUTY = 8.0f;
const float STOP_WINDOW_CM = 0.3f;
const unsigned long CTRL_PERIOD_US = 10000UL;
const unsigned long MOVE_TIMEOUT_MS = 60000UL;

const float WHEEL_CIRCUMFERENCE_CM = PI * WHEEL_DIAMETER_CM;
const float AVG_COUNTS_PER_REV = 0.5f *
    (LEFT_COUNTS_PER_REV + RIGHT_COUNTS_PER_REV);

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

    Serial.println(F("Rotation-normalized forward tuning sketch ready."));
    Serial.println(F("Send e: manual encoder sign check; push forward, then send s."));
    Serial.println(F("Send r: run 6 cells continuously with normalized encoder sync."));
    Serial.print(F("Wheel circumference from config: "));
    Serial.print(WHEEL_CIRCUMFERENCE_CM, 2);
    Serial.println(F(" cm"));
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
    const float targetDistanceCm = TEST_CELLS * CELL_DISTANCE_CM;
    const float targetRevolutions = targetDistanceCm / WHEEL_CIRCUMFERENCE_CM;
    const float averageStartCountsPerRev = AVG_COUNTS_PER_REV;
    float syncTrim = 0.0f;
    float nextCellReportCm = CELL_DISTANCE_CM;
    bool timedOut = false;

    resetEncoders();
    const unsigned long startMs = millis();
    unsigned long lastUs = micros();
    unsigned long nextUs = lastUs + CTRL_PERIOD_US;
    Serial.print(F("Starting continuous run: "));
    Serial.print(TEST_CELLS);
    Serial.print(F(" cells / "));
    Serial.print(targetDistanceCm, 1);
    Serial.println(F(" cm."));

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

        const float leftRevolutions = leftTicks / LEFT_COUNTS_PER_REV;
        const float rightRevolutions = rightTicks / RIGHT_COUNTS_PER_REV;
        const float averageRevolutions = 0.5f * (leftRevolutions + rightRevolutions);
        const float distanceCm = averageRevolutions * WHEEL_CIRCUMFERENCE_CM;
        const float remainingCm = targetDistanceCm - distanceCm;
        if (remainingCm <= STOP_WINDOW_CM) break;

        while (distanceCm >= nextCellReportCm && nextCellReportCm <= targetDistanceCm) {
            Serial.print(F("Distance "));
            Serial.print(nextCellReportCm, 0);
            Serial.print(F(" cm  L="));
            Serial.print(leftTicks);
            Serial.print(F(" R="));
            Serial.print(rightTicks);
            Serial.print(F(" trim="));
            Serial.println(syncTrim, 2);
            nextCellReportCm += CELL_DISTANCE_CM;
        }

        const float rampUpDuty = MIN_DUTY +
            (CRUISE_DUTY - MIN_DUTY) * (distanceCm / RAMP_UP_DISTANCE_CM);
        const float rampDownDuty = MIN_DUTY +
            (CRUISE_DUTY - MIN_DUTY) * (remainingCm / RAMP_DOWN_DISTANCE_CM);
        float duty = min(CRUISE_DUTY, min(rampUpDuty, rampDownDuty));
        duty = max(duty, MIN_DUTY);

        // Express wheel progress mismatch in average-counts-per-revolution units,
        // so the same gains work despite the two encoders' slightly different CPR.
        const float rotationError = leftRevolutions - rightRevolutions;
        const float normalizedError = rotationError * averageStartCountsPerRev;
        syncTrim = constrain(syncTrim + SYNC_KI * normalizedError * dt,
                             -SYNC_MAX_DUTY, SYNC_MAX_DUTY);
        const float steering = constrain(SYNC_KP * normalizedError + syncTrim,
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
    const float leftDistanceCm =
        (leftEnd / LEFT_COUNTS_PER_REV) * WHEEL_CIRCUMFERENCE_CM;
    const float rightDistanceCm =
        (rightEnd / RIGHT_COUNTS_PER_REV) * WHEEL_CIRCUMFERENCE_CM;
    const float measuredDistanceCm = 0.5f * (leftDistanceCm + rightDistanceCm);

    Serial.print(F("Target revolutions/wheel: "));
    Serial.println(targetRevolutions, 3);
    Serial.print(F("Final L="));
    Serial.print(leftEnd);
    Serial.print(F(" R="));
    Serial.print(rightEnd);
    Serial.print(F(" trim="));
    Serial.println(syncTrim, 2);
    Serial.print(F("Measured distance: "));
    Serial.print(measuredDistanceCm, 1);
    Serial.println(F(" cm"));
    if (timedOut) Serial.println(F("Move timed out before reaching target."));
}
