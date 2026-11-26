#include <Arduino.h>
#include "config.h"
#include "IRArray.h"
#include "LineFollower.h"
#include "Ultrasonic.h"
#include "hardware_adapter.h"
#include "maze_explorer.h"

// Group 4 hardware
IRArray      irSensorArray(IR_PINS, PIN_IR_EN, NUM_IR_SENSORS);
LineFollower lineFollower(irSensorArray);

// Hardware & line following adapters
G4RobotPlatform       platform(irSensorArray);
G4LineFollowerAdapter lineFollowerAdapter(lineFollower, irSensorArray);

// Maze explorer engine
MazeExplorer explorer(platform, &lineFollowerAdapter);

// Blocks until the run is started. With a start button: press and release it.
// Without one: cover the front sensor with a hand (3 readings under 5 cm),
// then take the hand away. Nobody touches the robot either way.
void waitForStartSignal() {
#if PIN_START_BUTTON >= 0
    pinMode(PIN_START_BUTTON, INPUT_PULLUP);
    Serial.println(F("Press the start button."));
    while (digitalRead(PIN_START_BUTTON) == HIGH) delay(10);
    while (digitalRead(PIN_START_BUTTON) == LOW)  delay(10);
#else
    Serial.println(F("Cover the front sensor with your hand, then take it away."));
    int seen = 0;
    while (seen < 3) {
        seen = (readUltrasonic(PIN_ULTRA_FRONT_TRIG, PIN_ULTRA_FRONT_ECHO) < 5.0f) ? seen + 1 : 0;
        delay(60);
    }
    while (readUltrasonic(PIN_ULTRA_FRONT_TRIG, PIN_ULTRA_FRONT_ECHO) < 10.0f) delay(60);
#endif
    Serial.println(F("Starting in 2 s - keep clear."));
    delay(2000);
}

void printExplorationResults(const ExplorationResult& res) {
    Serial.println(F("\n========================================"));
    Serial.println(F("     PHASE 1 EXPLORATION RESULTS        "));
    Serial.println(F("========================================"));
    Serial.print(F("Status: "));
    Serial.println(res.completed ? F("COMPLETED SUCCESSFULLY") : F("INCOMPLETE / PARTIAL"));

    Serial.print(F("Start Tile Section A: ("));
    Serial.print(res.startA.x);
    Serial.print(F(", "));
    Serial.print(res.startA.y);
    Serial.println(F(")"));

    Serial.print(F("Bridge Ramp Section A: ("));
    Serial.print(res.bridgeEntryA.x);
    Serial.print(F(", "));
    Serial.print(res.bridgeEntryA.y);
    Serial.println(F(")"));

    Serial.print(F("Bridge Exit Section B: ("));
    Serial.print(res.bridgeExitB.x);
    Serial.print(F(", "));
    Serial.print(res.bridgeExitB.y);
    Serial.println(F(")"));

    Serial.print(F("Goal Tile Section B: ("));
    Serial.print(res.goalB.x);
    Serial.print(F(", "));
    Serial.print(res.goalB.y);
    Serial.print(F(") - Found: "));
    Serial.println(res.goalFound ? F("YES") : F("NO"));

    Serial.println(F("\n--- Section A Wall Bitmasks (4x4) ---"));
    for (int y = MAZE_A_SIZE - 1; y >= 0; y--) {
        Serial.print(F("Y="));
        Serial.print(y);
        Serial.print(F(" | "));
        for (int x = 0; x < MAZE_A_SIZE; x++) {
            Serial.print(F("0x"));
            if (res.mazeA[y][x] < 16) Serial.print(F("0"));
            Serial.print(res.mazeA[y][x], HEX);
            Serial.print(F(" "));
        }
        Serial.println();
    }

    Serial.println(F("\n--- Section B Wall Bitmasks (9x9) ---"));
    for (int y = MAZE_B_SIZE - 1; y >= 0; y--) {
        Serial.print(F("Y="));
        Serial.print(y);
        Serial.print(F(" | "));
        for (int x = 0; x < MAZE_B_SIZE; x++) {
            Serial.print(F("0x"));
            if (res.mazeB[y][x] < 16) Serial.print(F("0"));
            Serial.print(res.mazeB[y][x], HEX);
            Serial.print(F(" "));
        }
        Serial.println();
    }
    Serial.println(F("========================================\n"));
}

void setup() {
    Serial.begin(115200);
    delay(500);

    Serial.println(F("=== EE4360 Micromouse Maze Explorer ==="));
    Serial.println(F("Initializing hardware - keep the robot still..."));

    // Motors, encoders, ultrasonics, gyro and IR array. Needed before the
    // start signal, which uses the front sensor.
    platform.begin();

    irSensorArray.setThresholds(DEFAULT_IR_THRESHOLDS);
    lineFollower.begin(WHITE_VALUES, BLACK_VALUES);
    lineFollower.setPIDGains(DEFAULT_KP, DEFAULT_KI, DEFAULT_KD);
    lineFollower.setSpeeds(DEFAULT_BASE_PWM, DEFAULT_MAX_PWM);

    waitForStartSignal();

    // Section A (4x4) -> Bridge Marker -> Line Follow Bridge -> Section B (9x9) -> Goal
    ExplorationResult result = explorer.runExploration();
    printExplorationResults(result);
}

void loop() {
    // Exploration complete, motors stopped
    delay(1000);
}
