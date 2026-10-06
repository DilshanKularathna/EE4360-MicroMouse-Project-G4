#include "hardware_adapter.h"

G4RobotPlatform::G4RobotPlatform(MotorDriver& motors, IRArray& ir, Ultrasonics& frontUltra, float wallThresh)
    : motorDriver(motors),
      irArray(ir),
      frontUltrasonic(frontUltra),
      wallThresholdCM(wallThresh),
      markerState(MARKER_IDLE),
      markerStateTime(0)
{
}

void G4RobotPlatform::begin() {
    motorDriver.begin();
    irArray.begin();
    irArray.enable();
    frontUltrasonic.begin();
    markerState = MARKER_IDLE;
    markerStateTime = 0;
}

WallSensors G4RobotPlatform::readWalls() {
    WallSensors ws;
    ws.wallFront = false;
    ws.wallLeft  = false;
    ws.wallRight = false;

    float frontDist = frontUltrasonic.getDistanceCM();
    if (frontDist > 0.0f && frontDist < wallThresholdCM) {
        ws.wallFront = true;
    }

    // Note: If additional side ultrasonics or a servo-mounted scanner
    // are added by the hardware team, query them here.
    return ws;
}

TileType G4RobotPlatform::readFloorTile() {
    int rawIR[8];
    irArray.readRaw(rawIR);

    uint8_t whiteCount = 0;
    uint8_t blackCount = 0;

    for (int i = 0; i < 8; i++) {
        // Higher value = darker/black, lower value = lighter/white
        if (rawIR[i] < DEFAULT_IR_THRESHOLDS[i]) {
            whiteCount++;
        } else {
            blackCount++;
        }
    }

    // 1. Full White Tile Check (START or FINISH tile)
    if (whiteCount >= 7) {
        return TILE_WHITE;
    }

    // 2. Approach Marker Detection State Machine
    // Pattern: Black floor -> White outer (9cm) -> Black center (3cm) -> White outer
    unsigned long now = millis();

    switch (markerState) {
        case MARKER_IDLE:
            if (whiteCount >= 4) {
                markerState = MARKER_WHITE_1;
                markerStateTime = now;
            }
            break;

        case MARKER_WHITE_1:
            if (blackCount >= 4) {
                markerState = MARKER_BLACK_CENTER;
                markerStateTime = now;
            } else if (now - markerStateTime > 1500) {
                markerState = MARKER_IDLE; // Timeout reset
            }
            break;

        case MARKER_BLACK_CENTER:
            if (whiteCount >= 4) {
                markerState = MARKER_WHITE_2;
                markerStateTime = now;
                return TILE_BRIDGE_MARKER; // Marker pattern confirmed!
            } else if (now - markerStateTime > 1500) {
                markerState = MARKER_IDLE;
            }
            break;

        case MARKER_WHITE_2:
            if (now - markerStateTime > 500) {
                markerState = MARKER_IDLE;
            }
            break;
    }

    return TILE_NORMAL_BLACK;
}

bool G4RobotPlatform::moveForwardOneCell() {
    // 288 ticks = 25 cm (1 cell)
    motorDriver.moveCells(1.0f);
    return true;
}

void G4RobotPlatform::turn(int relativeTurns) {
    if (relativeTurns == 1) {
        motorDriver.turnRight90();
    } else if (relativeTurns == -1) {
        // Turn left 90°: turn 180° then right 90° (or 3x right 90°)
        motorDriver.turn180();
        motorDriver.turnRight90();
    } else if (relativeTurns == 2 || relativeTurns == -2) {
        motorDriver.turn180();
    }
}

void G4RobotPlatform::stop() {
    motorDriver.stop();
}

// ============================================================================
// G4LineFollowerAdapter Implementation
// ============================================================================

G4LineFollowerAdapter::G4LineFollowerAdapter(LineFollower& follower, IRArray& ir, unsigned long maxTimeoutMs)
    : lineFollower(follower),
      irSensors(ir),
      bridgeStartTime(0),
      maxBridgeCrossTimeMs(maxTimeoutMs)
{
}

void G4LineFollowerAdapter::begin() {
    lineFollower.begin(WHITE_VALUES, BLACK_VALUES);
    bridgeStartTime = millis();
}

bool G4LineFollowerAdapter::step() {
    // Run one PID loop cycle for line following
    lineFollower.update();

    // Check if bridge exit has been reached:
    // Line ends and robot transitions back onto normal black maze surface
    int rawIR[8];
    irSensors.readRaw(rawIR);

    uint8_t blackCount = 0;
    for (int i = 0; i < 8; i++) {
        if (rawIR[i] >= DEFAULT_IR_THRESHOLDS[i]) {
            blackCount++;
        }
    }

    // Safety timeout or all black floor detected after bridge entry
    if (millis() - bridgeStartTime > 3000) { // Traversed bridge ramp and span
        if (blackCount >= 7) {
            // Re-entered black floor of Section B!
            return true;
        }
    }

    if (millis() - bridgeStartTime > maxBridgeCrossTimeMs) {
        // Safety timeout
        return true;
    }

    return false;
}

void G4LineFollowerAdapter::stop() {
    // Stop motor output
}
