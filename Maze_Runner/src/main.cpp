// Main firmware: the DIP switch selects the mode.
// The switches are read ONCE at start-up; press reset after changing them.
//   pio run -e megaatmega2560 -t upload

#include <Arduino.h>
#include "config.h"
#include "globals.h"
#include "ModeSelect.h"
#include "MotorDriver.h"
#include "Ultrasonic.h"
#include "Gyro.h"

float frontDist, leftDist, rightDist;   // used by the Ultrasonic library

static uint8_t activeMode = MODE_IDLE;

// Waits while keeping the gyro heading up to date.
static void waitMs(unsigned long ms) {
    unsigned long start = millis();
    while (millis() - start < ms)
        if (gyroReady()) gyroUpdate();
}

// Motor modes start only after a hand covers the front sensor and is taken away.
static void waitForHandSignal() {
    Serial.println("Cover the front sensor with your hand, then take it away.");
    int seen = 0;
    while (seen < 3) {
        seen = (readUltrasonic(PIN_ULTRA_FRONT_TRIG, PIN_ULTRA_FRONT_ECHO) < 5.0f) ? seen + 1 : 0;
        delay(60);
    }
    Serial.println("Hand seen - take it away.");
    while (readUltrasonic(PIN_ULTRA_FRONT_TRIG, PIN_ULTRA_FRONT_ECHO) < 10.0f) delay(60);
    Serial.println("Starting in 3 s.");
    delay(3000);
}

// ---------------- Mode: idle ----------------
static void idleStep() {
    readAllSensors();
    Serial.print("L=");   Serial.print(leftDist, 1);
    Serial.print(" F=");  Serial.print(frontDist, 1);
    Serial.print(" R=");  Serial.print(rightDist, 1);
    Serial.print(" | ");
    modePrint(modeReadRaw());   // shows the switches now; reset to apply
    delay(500);
}

// ---------------- Mode: turn test ----------------
static void turnTest() {
    waitForHandSignal();
    if (gyroReady()) gyroCalibrate();

    const float turns[] = {90.0f, -90.0f, 180.0f};
    for (uint8_t i = 0; i < 3; i++) {
        TurnResult r = turnDegreesWith(turns[i], TURN_KP, TURN_MIN_DUTY, TURN_SPEED);
        Serial.print("turn ");     Serial.print(turns[i], 0);
        Serial.print("  error=");  Serial.print(r.finalError, 1);
        Serial.print("  ms=");     Serial.print(r.timeMs);
        Serial.println(r.timedOut ? "  (TIMEOUT)" : "");
        waitMs(1000);
    }
    Serial.println("Turn test done.");
}

void setup() {
    Serial.begin(115200);

    pinMode(PIN_LEFT_RPWM, OUTPUT);
    pinMode(PIN_LEFT_LPWM, OUTPUT);
    pinMode(PIN_RIGHT_RPWM, OUTPUT);
    pinMode(PIN_RIGHT_LPWM, OUTPUT);
    stopMotors();
    initEncoders();

    pinMode(PIN_ULTRA_FRONT_TRIG, OUTPUT);
    pinMode(PIN_ULTRA_FRONT_ECHO, INPUT);
    pinMode(PIN_ULTRA_LEFT_TRIG, OUTPUT);
    pinMode(PIN_ULTRA_LEFT_ECHO, INPUT);
    pinMode(PIN_ULTRA_RIGHT_TRIG, OUTPUT);
    pinMode(PIN_ULTRA_RIGHT_ECHO, INPUT);

    modeBegin();
    activeMode = modeReadStable();
    Serial.print("Selected mode: ");
    modePrint(activeMode);

    if (!gyroBegin()) Serial.println("MPU-6050 NOT FOUND");
    else if (!gyroCalibrate()) Serial.println("Warning: robot moved during gyro calibration");

    switch (activeMode) {
        case MODE_IDLE:
            break;

        case MODE_TEST_TURN:
            turnTest();
            break;

        // Add each mode here as it is implemented.
        default:
            Serial.println("This mode is not implemented yet. Motors stay off.");
            break;
    }
}

void loop() {
    if (activeMode == MODE_IDLE) idleStep();
}
