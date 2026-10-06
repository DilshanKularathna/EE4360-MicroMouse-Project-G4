#include <Arduino.h>
#include "config.h"
#include "MotorDriver.h"
#include "IRArray.h"
#include "Ultrasonic.h"
#include "LineFollower.h"
#include "hardware_adapter.h"
#include "maze_explorer.h"

// Instantiate Group 4 Hardware Components
MotorDriver motorDriver(
    PIN_LEFT_RPWM, PIN_LEFT_LPWM,
    PIN_RIGHT_RPWM, PIN_RIGHT_LPWM,
    PIN_MOTOR_EN,
    PIN_LEFT_ENC_A, PIN_LEFT_ENC_B,
    PIN_RIGHT_ENC_A, PIN_RIGHT_ENC_B
);

IRArray irSensorArray(IR_PINS, PIN_IR_EN, NUM_IR_SENSORS);
Ultrasonics frontUltrasonic(PIN_ULTRA_TRIG, PIN_ULTRA_ECHO);
LineFollower lineFollower(motorDriver, irSensorArray);

// Hardware & Line Following Adapters
G4RobotPlatform platform(motorDriver, irSensorArray, frontUltrasonic);
G4LineFollowerAdapter lineFollowerAdapter(lineFollower, irSensorArray);

// Maze Explorer Engine
MazeExplorer explorer(platform, &lineFollowerAdapter);

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
    while (!Serial) {
        ; // Wait for serial monitor connection
    }

    Serial.println(F("=== EE4360 Micromouse Maze Explorer ==="));
    Serial.println(F("Initializing hardware..."));

    // Calibrate / load thresholds into IR Array
    irSensorArray.setThresholds(DEFAULT_IR_THRESHOLDS);
    lineFollower.begin(WHITE_VALUES, BLACK_VALUES);
    lineFollower.setPIDGains(DEFAULT_KP, DEFAULT_KI, DEFAULT_KD);
    lineFollower.setSpeeds(DEFAULT_BASE_PWM, DEFAULT_MAX_PWM);

    Serial.println(F("Starting Phase 1 Exploration..."));
    delay(2000); // 2 second pause before movement

    // Run the full exploration sequence:
    // Section A (4x4) -> Bridge Marker -> Line Follow Bridge -> Section B (9x9) -> Goal
    ExplorationResult result = explorer.runExploration();

    // Print mapped results
    printExplorationResults(result);
}

void loop() {
    // Exploration complete, idle motors stopped
    delay(1000);
}