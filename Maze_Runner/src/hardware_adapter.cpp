#include "hardware_adapter.h"
#include "config.h"
#include "MotorDriver.h"
#include "Motion.h"

// Ultrasonic.cpp declares these as extern (used by its readAllSensors()).
float frontDist = 0.0f;
float leftDist  = 0.0f;
float rightDist = 0.0f;

bool adapterLog = LOG_ADAPTER;

// A forward move that covers less than this did not reach the next cell.
static const float MIN_CELL_PROGRESS_CM = CELL_SIZE_CM / 2.0f;

G4RobotPlatform::G4RobotPlatform(IRArray& ir)
    : irArray(ir),
      initialized(false),
      markerState(MARKER_IDLE),
      markerStateTime(0)
{
}

void G4RobotPlatform::begin() {
    markerState = MARKER_IDLE;
    markerStateTime = 0;
    if (initialized) {
        motionRecalibrateGyro();
        return;
    }
    motionBegin();
    irArray.begin();
    irArray.enable();
    initialized = true;
}

WallSensors G4RobotPlatform::readWalls() {
    motionRecalibrateGyro();   // robot is standing still here: refresh the gyro bias

    // One sensor at a time so they cannot hear each other's echoes.
    // Negative readings mean the wall is very close, so they count as walls.
    frontDist = motionReadDistance(PIN_ULTRA_FRONT_TRIG, PIN_ULTRA_FRONT_ECHO);
    leftDist  = motionReadDistance(PIN_ULTRA_LEFT_TRIG, PIN_ULTRA_LEFT_ECHO);
    rightDist = motionReadDistance(PIN_ULTRA_RIGHT_TRIG, PIN_ULTRA_RIGHT_ECHO);

    WallSensors ws;
    ws.wallFront = frontDist < FRONT_WALL_CM;
    ws.wallLeft  = leftDist  < WALL_THRESHOLD;
    ws.wallRight = rightDist < WALL_THRESHOLD;
    ws.frontCM   = frontDist;
    ws.leftCM    = leftDist;
    ws.rightCM   = rightDist;

    if (adapterLog) {
        Serial.print(F("WALLS F="));  Serial.print(frontDist, 1);
        Serial.print(F(" L="));       Serial.print(leftDist, 1);
        Serial.print(F(" R="));       Serial.print(rightDist, 1);
        Serial.print(F(" -> F"));     Serial.print(ws.wallFront);
        Serial.print(F(" L"));        Serial.print(ws.wallLeft);
        Serial.print(F(" R"));        Serial.println(ws.wallRight);
    }
    return ws;
}

static const char* tileName(TileType t) {
    switch (t) {
        case TILE_WHITE:         return "white";
        case TILE_BRIDGE_MARKER: return "marker";
        default:                 return "black";
    }
}

TileType G4RobotPlatform::readFloorTile() {
    TileType t = classifyFloor();
    if (adapterLog) {
        Serial.print(F("FLOOR "));
        Serial.println(tileName(t));
    }
    return t;
}

TileType G4RobotPlatform::classifyFloor() {
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
    unsigned long t0 = millis();
    float driven = motionForwardCell();
    motionAlignFront();   // no-op unless a wall is in front of the new cell
    bool ok = driven >= MIN_CELL_PROGRESS_CM;
    if (adapterLog) {
        Serial.print(ok ? F("MOVE ok ") : F("MOVE SHORT "));
        Serial.print(driven, 1);
        Serial.print(F("cm "));
        Serial.print(millis() - t0);
        Serial.println(F("ms"));
    }
    return ok;
}

void G4RobotPlatform::turn(int relativeTurns) {
    if (relativeTurns == -2) relativeTurns = 2;
    unsigned long t0 = millis();
    motionTurnBy(90.0f * relativeTurns);
    if (adapterLog) {
        Serial.print(F("TURN "));
        if (relativeTurns > 0) Serial.print('+');
        Serial.print(relativeTurns);
        Serial.print(F(" err "));
        Serial.print(motionHeadingError(), 1);
        Serial.print(F("deg "));
        Serial.print(millis() - t0);
        Serial.println(F("ms"));
    }
}

void G4RobotPlatform::stop() {
    stopMotors();
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
    lineFollower.stop();
    // The corridor turned the robot; its heading now is Section B's NORTH.
    // TODO (robot): square up on the bridge-in tile walls before this.
    motionResetHeading();
}
