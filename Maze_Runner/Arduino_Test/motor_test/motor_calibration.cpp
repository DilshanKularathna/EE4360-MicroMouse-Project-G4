// ================================================================
// motor_calibration.cpp
// Motor + encoder calibration for the MicroMouse (Arduino Mega 2560).
//
// Uses the pins from config.h and the drive/encoder code in
// lib/MotorDriver, so you measure the same code the robot runs.
//
// Serial Monitor: 115200 baud, line ending = "Newline".
//
// Order of tests (do them in this order):
//   1  Direction check      - robot LIFTED, wheels free
//   2  Dead-band            - robot on the floor
//   3  Distance / ticks-per-cm and right-wheel trim - robot on the floor
//   4  Ticks per wheel revolution (hand-turn each wheel)
//   5  Print results and the lines to paste into config.h
// ================================================================
#include <Arduino.h>
#include <util/atomic.h>
#include "config.h"
#include "globals.h"
#include "MotorDriver.h"

// MotorDriver.cpp pulls in Ultrasonic.cpp, which reads these globals.
float frontDist = 0.0f;
float leftDist  = 0.0f;
float rightDist = 0.0f;

// ---------------- test settings ----------------
const uint8_t       TEST_DUTY        = 40;    // duty % for straight-line tests (0-100)
const uint8_t       MAX_SEARCH_DUTY  = 60;    // dead-band search upper limit (safety)
const long          DISTANCE_TICKS   = 1000;  // encoder ticks to drive in test 3
const unsigned long DEADBAND_STEP_MS = 100;   // time per duty step in test 2
const long          MIN_MOVE_TICKS   = 3;     // ticks that count as "the wheel moved"

// ---------------- results (filled in by tests) ----------------
bool  haveDistance   = false;
float ticksPerCmL    = 0.0f;
float ticksPerCmR    = 0.0f;
float rightTrim      = 1.0f;   // multiply right duty by this to drive straight
bool  haveRevL       = false;
bool  haveRevR       = false;
float ticksPerRevL   = 0.0f;
float ticksPerRevR   = 0.0f;
int   deadbandL      = -1;     // -1 = not measured / no movement
int   deadbandR      = -1;

// ================================================================
// Low-level helpers
// ================================================================
long readLeftTicks() {
    long v;
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { v = leftEncoderCount; }
    return v;
}

long readRightTicks() {
    long v;
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { v = rightEncoderCount; }
    return v;
}

// setMotors(a, b): a drives the left wheel, b drives the right wheel.
// Positive = forward, negative = backward. 0 = off.
void driveLeftOnly(int duty)  { setMotors(duty, 0); }
void driveRightOnly(int duty) { setMotors(0, duty); }

void clearSerial() {
    while (Serial.available()) Serial.read();
}

void waitForKey(const char* msg) {
    Serial.println(msg);
    Serial.println("-> Send any character to continue.");
    clearSerial();
    while (!Serial.available()) {}
    clearSerial();
}

float askFloat(const char* prompt) {
    Serial.print(prompt);
    clearSerial();
    while (!Serial.available()) {}
    float v = Serial.parseFloat();
    clearSerial();
    Serial.println(v, 3);
    return v;
}

// Print live encoder counts every 100 ms for `ms` milliseconds.
void liveCounts(unsigned long ms) {
    unsigned long start = millis();
    while (millis() - start < ms) {
        Serial.print("  L = "); Serial.print(readLeftTicks());
        Serial.print("   R = "); Serial.println(readRightTicks());
        delay(100);
    }
}

// Print live encoder counts until any key is pressed.
void liveCountsUntilKey() {
    Serial.println("Live encoder counts (send any character to stop):");
    clearSerial();
    while (!Serial.available()) {
        Serial.print("  L = "); Serial.print(readLeftTicks());
        Serial.print("   R = "); Serial.println(readRightTicks());
        delay(100);
    }
    clearSerial();
}

void printMenu() {
    Serial.println();
    Serial.println("==================== MOTOR CALIBRATION ====================");
    Serial.println("1  Direction check        (robot LIFTED, wheels free)");
    Serial.println("2  Dead-band (min duty)   (robot on floor)");
    Serial.println("3  Distance + right trim  (robot on floor, 1.5 m clear)");
    Serial.println("4  Ticks per wheel rev    (hand-turn each wheel)");
    Serial.println("5  Print results + config.h lines");
    Serial.println("6  Live encoder counts    (any key to stop)");
    Serial.println("x  EMERGENCY STOP");
    Serial.println("===========================================================");
}

// ================================================================
// TEST 1 - Direction check
// Both wheels driven "forward" with positive duty must give POSITIVE ticks.
// ================================================================
void testDirection() {
    waitForKey("\n[TEST 1] DIRECTION CHECK. Lift the robot so both wheels spin freely.");

    resetEncoders();
    driveLeftOnly(TEST_DUTY);
    liveCounts(500);
    stopMotors();
    delay(300);
    Serial.print("Left  wheel, +duty -> ticks = ");
    Serial.println(readLeftTicks());

    resetEncoders();
    driveRightOnly(TEST_DUTY);
    liveCounts(500);
    stopMotors();
    delay(300);
    Serial.print("Right wheel, +duty -> ticks = ");
    Serial.println(readRightTicks());

    Serial.println("Expected: both values POSITIVE and both wheels turning the way the robot goes forward.");
    Serial.println("If a value is NEGATIVE: flip the ++/-- in that wheel's ISR in lib/MotorDriver/MotorDriver.cpp.");
    Serial.println("If a wheel turns backwards: swap that motor's RPWM/LPWM pins in config.h.");
}

// ================================================================
// TEST 2 - Dead-band: smallest duty that makes each wheel move
// ================================================================
int findDeadband(bool leftWheel) {
    resetEncoders();
    for (int duty = 0; duty <= MAX_SEARCH_DUTY; duty++) {
        if (leftWheel) driveLeftOnly(duty);
        else           driveRightOnly(duty);

        delay(DEADBAND_STEP_MS);

        long ticks = leftWheel ? readLeftTicks() : readRightTicks();
        if (labs(ticks) >= MIN_MOVE_TICKS) {
            stopMotors();
            return duty;
        }
    }
    stopMotors();
    return -1;
}

void testDeadband() {
    waitForKey("\n[TEST 2] DEAD-BAND. Robot on the floor, wheels loaded normally, clear space.");

    Serial.println("Ramping LEFT wheel...");
    deadbandL = findDeadband(true);
    delay(800);

    Serial.println("Ramping RIGHT wheel...");
    deadbandR = findDeadband(false);
    delay(800);

    Serial.print("Left  min duty = "); Serial.println(deadbandL);
    Serial.print("Right min duty = "); Serial.println(deadbandR);
    if (deadbandL < 0 || deadbandR < 0) {
        Serial.println("WARNING: a wheel did not move up to MAX_SEARCH_DUTY. Check wiring and battery.");
    }
    Serial.println("Note: the drive code should use a duty above the larger of these two values.");
}

// ================================================================
// TEST 3 - Distance calibration (ticks per cm) and right-wheel trim
// Robot drives straight for DISTANCE_TICKS; you measure the real distance.
// ================================================================
void testDistance() {
    waitForKey("\n[TEST 3] DISTANCE. Mark the robot's start position on the floor with tape.\n"
               "Robot drives straight until ~1000 ticks, then stops. Send 'x' at any time to abort.");

    resetEncoders();
    setMotors(TEST_DUTY, TEST_DUTY);

    bool aborted = false;
    while (true) {
        long l = readLeftTicks();
        long r = readRightTicks();
        if (max(labs(l), labs(r)) >= DISTANCE_TICKS) break;

        if (Serial.available()) {
            if (Serial.read() == 'x') { aborted = true; break; }
        }
    }
    stopMotors();
    delay(500);

    if (aborted) {
        Serial.println("Aborted.");
        return;
    }

    long l = readLeftTicks();
    long r = readRightTicks();
    Serial.print("Ticks  L = "); Serial.print(l);
    Serial.print("   R = ");     Serial.println(r);
    Serial.println("Mark where the robot stopped and measure the STRAIGHT distance with the tape.");

    float cm = askFloat("Distance travelled (cm): ");
    if (cm <= 0.0f || labs(l) == 0 || labs(r) == 0) {
        Serial.println("Invalid input or a wheel did not count. Repeat test 3.");
        return;
    }

    ticksPerCmL = (float)labs(l) / cm;
    ticksPerCmR = (float)labs(r) / cm;
    rightTrim   = (float)labs(l) / (float)labs(r);
    haveDistance = true;

    Serial.print("Ticks per cm  L = "); Serial.println(ticksPerCmL, 3);
    Serial.print("Ticks per cm  R = "); Serial.println(ticksPerCmR, 3);
    Serial.print("Right trim      = "); Serial.println(rightTrim, 4);
    Serial.println("Right trim: multiply the right-wheel duty by this to make the robot drive straight.");
    Serial.println("Repeat test 3 at least 3 times and average the ticks-per-cm values.");
}

// ================================================================
// TEST 4 - Ticks per wheel revolution (wheel turned by hand)
// ================================================================
void testTicksPerRev(bool leftWheel) {
    waitForKey(leftWheel
        ? "\n[TEST 4a] LEFT wheel. Put a tape mark on the wheel. Robot lifted."
        : "\n[TEST 4b] RIGHT wheel. Put a tape mark on the wheel. Robot lifted.");

    resetEncoders();
    Serial.println("Turn the wheel BY HAND, FORWARD, exactly 10 full revolutions (stop on the tape mark).");
    liveCountsUntilKey();   // watch the counts while turning; press any key when finished
    float revs = askFloat("Revolutions turned (type the number, e.g. 10): ");
    if (revs <= 0.0f) {
        Serial.println("Invalid number. Repeat test 4.");
        return;
    }

    long t = labs(leftWheel ? readLeftTicks() : readRightTicks());
    if (t == 0) {
        Serial.println("No ticks counted. Check the encoder wiring.");
        return;
    }

    if (leftWheel) { ticksPerRevL = (float)t / revs; haveRevL = true; }
    else           { ticksPerRevR = (float)t / revs; haveRevR = true; }

    Serial.print("Ticks per wheel revolution = ");
    Serial.println(leftWheel ? ticksPerRevL : ticksPerRevR, 2);
}

// ================================================================
// TEST 5 - Print the summary and the config.h lines
// ================================================================
void printResults() {
    Serial.println("\n==================== RESULTS ====================");
    Serial.print("Dead-band  L = "); Serial.print(deadbandL);
    Serial.print("%   R = ");        Serial.print(deadbandR); Serial.println("%");

    if (!haveDistance) {
        Serial.println("Test 3 not done yet: no distance data.");
    } else {
        Serial.print("Ticks per cm   L = "); Serial.println(ticksPerCmL, 3);
        Serial.print("Ticks per cm   R = "); Serial.println(ticksPerCmR, 3);
        Serial.print("Right trim       = "); Serial.println(rightTrim, 4);
    }

    if (!haveRevL || !haveRevR) {
        Serial.println("Test 4 not finished for both wheels: no wheel-revolution data.");
        Serial.println("=================================================");
        return;
    }

    float ticksPerRev = (ticksPerRevL + ticksPerRevR) / 2.0f;
    float ticksPerCm  = haveDistance ? (ticksPerCmL + ticksPerCmR) / 2.0f : 0.0f;
    float cmPerRev    = ticksPerRev / ticksPerCm;
    float diameterCm  = cmPerRev / PI;

    Serial.println("\n----- Paste into include/config.h (replace the old values) -----");
    Serial.print("#define ENCODER_COUNTS_PER_REV  "); Serial.println((long)(ticksPerRev + 0.5f));
    Serial.print("#define WHEEL_DIAMETER_CM       "); Serial.println(diameterCm, 3);
    Serial.print("#define TICKS_PER_CELL_25CM     "); Serial.println(ticksPerCm * 25.0f, 1);
    Serial.print("// right-wheel duty trim: "); Serial.println(rightTrim, 4);
    Serial.print("// min duty to move: left "); Serial.print(deadbandL);
    Serial.print("%, right "); Serial.print(deadbandR); Serial.println("%");
    Serial.println("----------------------------------------------------------------");
    Serial.println("Note: TICKS_PER_90_DEG / TICKS_PER_180_DEG depend on these values; re-measure them after this.");
    Serial.println("=================================================");
}

// ================================================================
// Arduino entry points
// ================================================================
void setup() {
    Serial.begin(115200);
    delay(500);

    initEncoders();   // 4x quadrature decoding (both channels, both edges)

    pinMode(PIN_LEFT_RPWM,  OUTPUT);
    pinMode(PIN_LEFT_LPWM,  OUTPUT);
    pinMode(PIN_RIGHT_RPWM, OUTPUT);
    pinMode(PIN_RIGHT_LPWM, OUTPUT);

    stopMotors();
    printMenu();
}

void loop() {
    if (!Serial.available()) return;

    char c = Serial.read();
    switch (c) {
        case '1': testDirection();          break;
        case '2': testDeadband();           break;
        case '3': testDistance();           break;
        case '4':
            testTicksPerRev(true);
            testTicksPerRev(false);
            break;
        case '5': printResults();           break;
        case '6': liveCountsUntilKey();     break;
        case 'x':
        case 'X':
            stopMotors();
            Serial.println("STOPPED");
            break;
        case '\r':
        case '\n':
            break;
        default:
            printMenu();
            break;
    }

    stopMotors();
    clearSerial();
}
