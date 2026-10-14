#include <Arduino.h>
#include <EEPROM.h>
#include <Wire.h>
#include <util/atomic.h>
#include "config.h"
#include "Gyroscope.h"

// MPU6050 is mounted upside down; this wrapper reports left turns as positive.
Gyroscope gyroscope(Wire, true);

struct TurnResult {
    uint16_t magic;
    uint8_t turnsCompleted;
    bool reachedTarget[2];
    float targetAngles[2];
    long leftTicks[2];
    long rightTicks[2];
    float measuredAngles[2];
    float deviationFrom90[2];
    float averageLeftTicks;
    float averageRightTicks;
};

const int RESULT_EEPROM_ADDRESS = 128;
const uint16_t RESULT_MAGIC = 0x5234;

volatile long leftEncoderTicks = 0;
volatile long rightEncoderTicks = 0;
TurnResult result;
bool runFinished = false;
bool ledOn = false;
unsigned long lastLedToggleMs = 0;

void leftEncoderISR() {
    if (digitalRead(PIN_LEFT_ENC_B) == HIGH) --leftEncoderTicks;
    else ++leftEncoderTicks;
}

void rightEncoderISR() {
    if (digitalRead(PIN_RIGHT_ENC_B) == HIGH) ++rightEncoderTicks;
    else --rightEncoderTicks;
}

void resetTurnTicks() {
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        leftEncoderTicks = 0;
        rightEncoderTicks = 0;
    }
}

void readTurnTicks(long &left, long &right) {
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        left = leftEncoderTicks;
        right = rightEncoderTicks;
    }
}

void setWheelDuty(int leftPercent, int rightPercent) {
    leftPercent = constrain(leftPercent, -100, 100);
    rightPercent = constrain(rightPercent, -100, 100);
    const uint8_t leftPwm = (uint8_t)(abs(leftPercent) * 255L / 100L);
    const uint8_t rightPwm = (uint8_t)(abs(rightPercent) * 255L / 100L);

    analogWrite(PIN_LEFT_RPWM, leftPercent > 0 ? leftPwm : 0);
    analogWrite(PIN_LEFT_LPWM, leftPercent < 0 ? leftPwm : 0);
    analogWrite(PIN_RIGHT_RPWM, rightPercent > 0 ? rightPwm : 0);
    analogWrite(PIN_RIGHT_LPWM, rightPercent < 0 ? rightPwm : 0);
}

void stopMotors() {
    analogWrite(PIN_LEFT_RPWM, 0);
    analogWrite(PIN_LEFT_LPWM, 0);
    analogWrite(PIN_RIGHT_RPWM, 0);
    analogWrite(PIN_RIGHT_LPWM, 0);
}

// Positive angle is left; negative angle is right. Encoder counts are read
// after each turn for reporting only and never affect turn control.
bool turnByGyro(uint8_t turnIndex, float targetDegrees) {
    resetTurnTicks();
    gyroscope.update();
    const float startAngle = gyroscope.getYawAngleDeg();
    const float idealDegrees = (turnIndex == 0) ? 90.0f : -90.0f;
    float turnedDegrees = 0.0f;
    const unsigned long startMs = millis();

    Serial.print(F("Turn "));
    Serial.print(turnIndex == 0 ? F("left") : F("right"));
    Serial.print(F(" target="));
    Serial.println(targetDegrees, 0);

    while (fabs(targetDegrees - turnedDegrees) > TURN_STOP_TOLERANCE_DEG &&
           millis() - startMs < TURN_90_TIMEOUT_MS) {
        gyroscope.update();
        turnedDegrees = gyroscope.getYawAngleDeg() - startAngle;

        const float error = targetDegrees - turnedDegrees;
        // Positive wheel command (left forward, right reverse) is a right
        // turn in MotorDriver, while Gyroscope reports left turns as positive.
        const int direction = (error > 0.0f) ? -1 : 1;
        const int duty = (fabs(error) < TURN_SLOWDOWN_START_ERROR_DEG)
            ? TURN_MIN_DUTY
            : TURN_SPEED;
        setWheelDuty(direction * duty, -direction * duty);
        delay(10);
    }

    stopMotors();

    // Include coast-down angle and encoder ticks in the saved report.
    const unsigned long settleStart = millis();
    while (millis() - settleStart < 300UL) {
        gyroscope.update();
        turnedDegrees = gyroscope.getYawAngleDeg() - startAngle;
        delay(5);
    }
    const float deviation = turnedDegrees - idealDegrees;
    const bool reachedTarget = fabs(deviation) <= TURN_STOP_TOLERANCE_DEG;

    long leftTicks = 0;
    long rightTicks = 0;
    readTurnTicks(leftTicks, rightTicks);
    result.leftTicks[turnIndex] = labs(leftTicks);
    result.rightTicks[turnIndex] = labs(rightTicks);
    result.measuredAngles[turnIndex] = turnedDegrees;
    result.targetAngles[turnIndex] = targetDegrees;
    result.deviationFrom90[turnIndex] = deviation;
    result.reachedTarget[turnIndex] = reachedTarget;
    ++result.turnsCompleted;

    Serial.print(turnIndex == 0 ? F("Left") : F("Right"));
    Serial.print(F(" turn deviation from ideal 90 deg="));
    Serial.print(deviation, 1);
    Serial.print(F(" deg: "));
    Serial.println(reachedTarget ? F("PASS") : F("FAIL"));
    return reachedTarget;
}

void printResult(const TurnResult &report, const __FlashStringHelper *heading) {
    Serial.println();
    Serial.println(heading);
    Serial.println(F("turn,commanded_deg,measured_deg,deviation_from_90_deg,status"));
    for (uint8_t i = 0; i < report.turnsCompleted; ++i) {
        Serial.print(i == 0 ? F("left") : F("right"));
        Serial.print(',');
        Serial.print(report.targetAngles[i], 1);
        Serial.print(',');
        Serial.print(report.measuredAngles[i], 1);
        Serial.print(',');
        Serial.print(report.deviationFrom90[i], 1);
        Serial.print(',');
        Serial.println(report.reachedTarget[i] ? F("PASS") : F("FAIL"));
    }
}

void printSavedResult() {
    TurnResult saved;
    EEPROM.get(RESULT_EEPROM_ADDRESS, saved);
    if (saved.magic != RESULT_MAGIC || saved.turnsCompleted == 0 || saved.turnsCompleted > 2) return;
    printResult(saved, F("Previous run saved in EEPROM:"));
}

void finishRun() {
    if (result.turnsCompleted > 0) {
        long leftTotal = 0;
        long rightTotal = 0;
        for (uint8_t i = 0; i < result.turnsCompleted; ++i) {
            leftTotal += result.leftTicks[i];
            rightTotal += result.rightTicks[i];
        }
        result.averageLeftTicks = leftTotal / (float)result.turnsCompleted;
        result.averageRightTicks = rightTotal / (float)result.turnsCompleted;
    }

    EEPROM.put(RESULT_EEPROM_ADDRESS, result);
    printResult(result, F("This run:"));
    Serial.println(F("Run over. Edit correction values in config.h to tune the next run."));
    runFinished = true;
    lastLedToggleMs = millis();
}

void runTurns() {
    result.magic = RESULT_MAGIC;
    result.turnsCompleted = 0;
    result.averageLeftTicks = 0.0f;
    result.averageRightTicks = 0.0f;

    Serial.println(F("Starting gyro-only left 90, then right 90 in 3 seconds."));
    delay(3000);

    const float leftTarget = 90.0f + TURN_LEFT_CORRECTION_DEG;
    const float rightTarget = -90.0f - TURN_RIGHT_CORRECTION_DEG;
    const bool leftReached = turnByGyro(0, leftTarget);
    if (!leftReached) Serial.println(F("Left turn missed its gyro target; continuing with right turn."));
    delay(250);

    const bool rightReached = turnByGyro(1, rightTarget);
    if (!rightReached) Serial.println(F("Right turn missed its gyro target."));

    finishRun();
}

void setup() {
    Serial.begin(115200);
    delay(1000);
    Serial.println(F("MPU6050 90-degree left/right turn test"));
    printSavedResult();

    pinMode(PIN_RIGHT_RPWM, OUTPUT);
    pinMode(PIN_RIGHT_LPWM, OUTPUT);
    pinMode(PIN_LEFT_RPWM, OUTPUT);
    pinMode(PIN_LEFT_LPWM, OUTPUT);
    pinMode(PIN_LEFT_ENC_A, INPUT_PULLUP);
    pinMode(PIN_LEFT_ENC_B, INPUT_PULLUP);
    pinMode(PIN_RIGHT_ENC_A, INPUT_PULLUP);
    pinMode(PIN_RIGHT_ENC_B, INPUT_PULLUP);
    pinMode(LED_BUILTIN, OUTPUT);
    stopMotors();

    attachInterrupt(digitalPinToInterrupt(PIN_LEFT_ENC_A), leftEncoderISR, RISING);
    attachInterrupt(digitalPinToInterrupt(PIN_RIGHT_ENC_A), rightEncoderISR, RISING);

    Serial.println(F("Keep robot still while the MPU6050 initializes and calibrates."));
    if (!gyroscope.begin(0, 0)) {
        Serial.println(F("MPU6050 initialization failed; motors remain stopped."));
        while (true) delay(1000);
    }
    delay(1000);
    Serial.println(F("Calibrating gyro; do not move the robot."));
    if (!gyroscope.calibrate()) {
        Serial.println(F("Gyro calibration failed; motors remain stopped."));
        while (true) delay(1000);
    }

    gyroscope.update();
    Serial.println(F("Calibration complete."));
    runTurns();
}

void loop() {
    if (!runFinished) return;

    // Fast blink indicates the two-turn test has finished. Sending any
    // character prints the result again if the board stayed powered.
    if (millis() - lastLedToggleMs >= 100UL) {
        ledOn = !ledOn;
        digitalWrite(LED_BUILTIN, ledOn ? HIGH : LOW);
        lastLedToggleMs = millis();
    }
    if (Serial.available() > 0) {
        while (Serial.available() > 0) Serial.read();
        printResult(result, F("This run:"));
    }
}
