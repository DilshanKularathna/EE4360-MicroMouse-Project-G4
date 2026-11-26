// Tests the LineFollower library: marker-cell detection and line mode.
//
//   pio run -e line_mode_test -t upload   then   pio device monitor
//
// Serial commands (type a letter + ENTER, 115200 baud). Any key stops a running test.
//   v  view, motors OFF: push the robot by hand forward through the special cell.
//      Shows the black/white bar, distance, the pattern step and each band's length.
//   g  plain line follow (same as ir_02 'g', now through the library + setMotors)
//   m  marker run: drive straight slowly from the black floor, stop when the pattern is found
//   l  line mode only: start on the second white strip, finds and follows the line
//   f  full test: drive until the marker, then line mode (what the maze will do)

#include <Arduino.h>
#include "config.h"
#include "globals.h"
#include "MotorDriver.h"
#include "LineFollower.h"

static void flushSerial() {
    delay(20);
    while (Serial.available()) Serial.read();
}

static bool keyPressed() {
    if (!Serial.available()) return false;
    flushSerial();
    return true;
}

static void printBar() {
    for (uint8_t i = 0; i < NUM_IR_SENSORS; i++) Serial.print(lineIsBlack(i) ? '#' : '.');
}

static void viewMode() {
    stopMotors();
    Serial.println("VIEW (motors off). '#' = black. Push the robot FORWARD by hand; any key stops.");
    Serial.println("'marker:' lines show each band and its measured length.");
    lineOdoReset();
    markerReset();
    unsigned long lastPrint = 0;
    bool announced = false;
    while (!keyPressed()) {
        float cm = lineOdoCm();
        MarkerState m = markerUpdate(cm);       // sampled fast so band lengths are accurate
        if (m == MARKER_FOUND && !announced) {
            announced = true;
            Serial.print(">>> MARKER FOUND at "); Serial.print(cm, 1); Serial.println(" cm <<<");
        }

        if (millis() - lastPrint >= 150) {
            lastPrint = millis();
            Serial.print("raw ");
            for (uint8_t i = 0; i < NUM_IR_SENSORS; i++) { Serial.print(lineRaw(i)); Serial.print(' '); }
            Serial.print("| ");
            printBar();
            Serial.print(" black="); Serial.print(lineBlackCount());
            Serial.print(" cm=");    Serial.print(cm, 1);
            Serial.print(" step=");  Serial.println(markerStepName());
        }
        delay(5);
    }
    Serial.println("stopped ('v' again to reset the pattern)");
}

static void plainFollow() {
    Serial.println("LINE FOLLOW: place on the line. Starting in 2 s (any key stops).");
    delay(2000);
    flushSerial();
    lineReset();
    unsigned long lastPrint = 0;
    while (!keyPressed()) {
        LineStep s = lineFollowStep(true);
        if (millis() - lastPrint > 200) {
            lastPrint = millis();
            Serial.print("pos "); Serial.print(s.position);
            Serial.print(" L ");  Serial.print(s.leftPwm);
            Serial.print(" R ");  Serial.print(s.rightPwm);
            Serial.println(s.lost ? " LOST" : "");
        }
        delay(5);
    }
    stopMotors();
    Serial.println("stopped");
}

// Drives straight until the marker is seen. Returns false if stopped or not found.
static bool driveToMarker(float maxCm) {
    lineOdoReset();
    markerReset();
    while (true) {
        if (keyPressed()) { stopMotors(); Serial.println("stopped"); return false; }
        float cm = lineOdoCm();
        if (cm > maxCm) {
            stopMotors();
            Serial.print("No marker within "); Serial.print(maxCm, 0); Serial.println(" cm");
            return false;
        }
        if (markerUpdate(cm) == MARKER_FOUND) {
            stopMotors();
            Serial.print("MARKER found at "); Serial.print(cm, 1); Serial.print(" cm  ");
            printBar();
            Serial.println();
            return true;
        }
        lineDriveStraight(LINE_SEARCH_DUTY);
        delay(5);
    }
}

static void startDelay(const char *what) {
    Serial.print(what);
    Serial.println(" Starting in 2 s (any key stops).");
    delay(2000);
    flushSerial();
}

void setup() {
    Serial.begin(115200);

    pinMode(PIN_LEFT_RPWM, OUTPUT);
    pinMode(PIN_LEFT_LPWM, OUTPUT);
    pinMode(PIN_RIGHT_RPWM, OUTPUT);
    pinMode(PIN_RIGHT_LPWM, OUTPUT);
    stopMotors();
    initEncoders();
    lineBegin();
    delay(500);

    Serial.println();
    Serial.println("=== Line mode test ===");
    Serial.println("Commands: v=view  g=follow  m=marker run  l=line mode  f=full (marker + line)");
}

void loop() {
    if (!Serial.available()) return;
    char cmd = Serial.read();
    flushSerial();

    switch (cmd) {
        case 'v': viewMode(); break;
        case 'g': plainFollow(); break;
        case 'm':
            startDelay("MARKER RUN: place on the black floor before the marker cell.");
            driveToMarker(3 * CELL_SIZE_CM);
            break;
        case 'l': {
            startDelay("LINE MODE: place on the second white strip, facing the line.");
            LineModeResult r = runLineMode(false, keyPressed);
            Serial.print("Result: "); Serial.println(lineModeResultName(r));
            break;
        }
        case 'f': {
            startDelay("FULL: place on the black floor before the marker cell.");
            if (driveToMarker(3 * CELL_SIZE_CM)) {
                delay(300);
                LineModeResult r = runLineMode(true, keyPressed);
                Serial.print("Result: "); Serial.println(lineModeResultName(r));
            }
            break;
        }
        default: break;
    }
    Serial.println("Commands: v=view  g=follow  m=marker run  l=line mode  f=full (marker + line)");
}
