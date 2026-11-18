#include <Arduino.h>
#include <util/atomic.h>
#include "config.h"
#include "MotorDriver.h"

// One-cell forward accuracy test, using the project motor driver and encoders.
const long TARGET_TICKS_FWD = 288;
const float CRUISE_DUTY = 25.0f;
const float MIN_DUTY = 14.0f;
const float RAMP_UP_TICKS = 40.0f;
const float RAMP_DOWN_TICKS = 70.0f;
const float SYNC_KP = 0.35f;
const float SYNC_KI = 0.40f;
const float SYNC_MAX_DUTY = 8.0f;
const long STOP_WINDOW_TICKS = 3;
const unsigned long CTRL_PERIOD_US = 10000UL;
const unsigned long MOVE_TIMEOUT_MS = 5000UL;

void moveCells(float numCells);

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
    delay(2000);
}

void loop() {
    Serial.println(F("Forward accuracy test: 1 cell (25 cm)"));
    moveCells(1.0f);

    stopMotors();
    Serial.println(F("Test complete. Reset the board to run again."));
    while (true) delay(1000);
}

void moveCells(float numCells) {
    const long targetTicks = (long)(numCells * TARGET_TICKS_FWD + 0.5f);
    float syncTrim = 0.0f;

    resetEncoders();
    const unsigned long startMs = millis();
    unsigned long lastUs = micros();
    unsigned long nextUs = lastUs + CTRL_PERIOD_US;
    bool timedOut = false;

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

    Serial.print(F("Target ticks: "));
    Serial.println(targetTicks);
    Serial.print(F("Final L/R ticks: "));
    Serial.print(leftEnd);
    Serial.print('/');
    Serial.println(rightEnd);
    Serial.print(F("Average ticks: "));
    Serial.println(averageEnd, 1);
    Serial.print(F("Distance: "));
    Serial.print(averageEnd * 25.0f / TARGET_TICKS_FWD, 1);
    Serial.println(F(" cm"));
    Serial.print(F("Wheel difference: "));
    Serial.println(leftEnd - rightEnd);
    if (timedOut) Serial.println(F("Move timed out before reaching target."));
}
