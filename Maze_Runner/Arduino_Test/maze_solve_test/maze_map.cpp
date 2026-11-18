#include <Arduino.h>
#include <Wire.h>
#include <util/atomic.h>
#include <math.h>
#include "config.h"
#include "Ultrasonic.h"
#include "MotorDriver.h"   // was "motorDriver.h": wrong case fails on Linux/macOS builds
#include "globals.h"
#include "Gyroscope.h"

// ================================================================
// NEW TUNING KNOBS
// Defaults live here so this file compiles on its own. Copy them into
// config.h if you prefer to tune them there (config.h wins if defined).
// ================================================================

// --- Robot geometry, in cm (0 skips the boot check).
#ifndef ROBOT_WIDTH_CM
#define ROBOT_WIDTH_CM            18.0f  // widest point of the body, wheels included
#endif
#ifndef AXLE_TO_REAR_CM
#define AXLE_TO_REAR_CM           12.6f  // axle centreline -> rearmost point of the body
#endif
#ifndef AXLE_TO_FRONT_CM
#define AXLE_TO_FRONT_CM          4.1f   // axle centreline -> frontmost point (16.7 total length)
#endif
#ifndef CELL_INNER_WIDTH_CM
#define CELL_INNER_WIDTH_CM       25.0f  // free width between facing walls. Walls have thickness,
                                         // so measure the real gap and replace this value.
#endif
#ifndef TRACK_WIDTH_CM
#define TRACK_WIDTH_CM            WHEELBASE_CM  // distance between the two wheel CENTRES
#endif
#ifndef PIVOT_CLEARANCE_MARGIN_CM
#define PIVOT_CLEARANCE_MARGIN_CM 1.0f   // spare room wanted on each side
#endif

// --- Wall centering: 1 = wall error nudges the heading target (cascade),
//     0 = old behaviour (wall steering added directly to wheel duty).
#ifndef WALL_STEER_AS_HEADING
#define WALL_STEER_AS_HEADING        1
#endif
#ifndef WALL_HEADING_GAIN_DEG_PER_CM
#define WALL_HEADING_GAIN_DEG_PER_CM 1.5f
#endif
#ifndef WALL_HEADING_MAX_DEG
#define WALL_HEADING_MAX_DEG         4.0f
#endif
#ifndef ULTRASONIC_STEER_WEIGHT
#define ULTRASONIC_STEER_WEIGHT      1.0f
#endif
#ifndef LEFT_WALL_THRESHOLD
#define LEFT_WALL_THRESHOLD          WALL_THRESHOLD
#endif
#ifndef RIGHT_WALL_THRESHOLD
#define RIGHT_WALL_THRESHOLD         WALL_THRESHOLD
#endif
#ifndef LEFT_CENTER_DIST
#define LEFT_CENTER_DIST             CENTER_DIST
#endif
#ifndef RIGHT_CENTER_DIST
#define RIGHT_CENTER_DIST            CENTER_DIST
#endif
#ifndef NO_WALL_CREEP_DUTY
#define NO_WALL_CREEP_DUTY           18.0f
#endif

// --- Arc turns (experimental). When 1, a 90 degree turn becomes:
//     reverse ARC_TURN_RADIUS_CM, then a forward arc whose axle path has that
//     radius. The tail swings out less than in a pivot, and the robot ends up
//     centred in the new corridor. The next cell move is shortened to match.
//     Set to 0 to go back to plain in-place pivots (the current default in config.h).
//     The very first turn (before one cell move has completed) stays a pivot,
//     because the tail may be against the start wall and nothing can see behind.
#ifndef USE_ARC_TURNS
#define USE_ARC_TURNS             0
#endif
#ifndef ARC_TURN_RADIUS_CM
#define ARC_TURN_RADIUS_CM        18.0f  // tail swing = sqrt(rear^2 + (R + width/2)^2) - R
#endif
// Reverse duty. Also used for the pre-pivot reverse and the pre-180 reverse.
#ifndef ARC_BACKUP_DUTY
#define ARC_BACKUP_DUTY           20
#endif
// The robot coasts a little after the motors stop, so it reverses further than
// requested. Measure how many cm too far it goes and put that here:
// the reverse then stops that much earlier. The arc itself is not affected.
#ifndef ARC_BACKUP_TRIM_CM
#define ARC_BACKUP_TRIM_CM        0.0f
#endif
// Outer-wheel duty during an arc. Higher than TURN_SPEED on purpose: the slower
// inner wheel gets duty * ratio, and very low duty will not turn the motor,
// which would silently shrink the real radius.
#ifndef ARC_TURN_SPEED
#define ARC_TURN_SPEED            40
#endif

// --- Reverse before pivoting. Only applied once reverseSafe is true (after one
//     full cell move), so the first turn never backs into the start wall.
#ifndef REVERSE_BEFORE_PIVOT
#define REVERSE_BEFORE_PIVOT      1
#endif
#ifndef PIVOT_BACKUP_CM
#define PIVOT_BACKUP_CM           3.0f   // reverse before a 90 degree pivot
#endif
#ifndef TURN_AROUND_BACKUP_CM
#define TURN_AROUND_BACKUP_CM     6.0f   // reverse before a 180 degree turn
#endif
#ifndef TURN90_WHEEL_LOCK
#define TURN90_WHEEL_LOCK         0
#endif

// --- Stall detection: stop a turn that stops rotating (robot is touching a wall)
#ifndef TURN_STALL_WINDOW_MS
#define TURN_STALL_WINDOW_MS         400UL
#endif
#ifndef TURN_STALL_MIN_PROGRESS_DEG
#define TURN_STALL_MIN_PROGRESS_DEG  2.0f  // less rotation than this per window = stalled
#endif

// --- 1 = each decision uses the median of 3 readings per sensor, so one missed
//     echo (reported as 100 cm = "open") cannot trigger a false turn.
#ifndef USE_SENSOR_MEDIAN
#define USE_SENSOR_MEDIAN         1
#endif
#ifndef MAZE_MAP_WIDTH
#define MAZE_MAP_WIDTH             16
#endif
#ifndef MAZE_MAP_HEIGHT
#define MAZE_MAP_HEIGHT            16
#endif
#if (MAZE_MAP_WIDTH * MAZE_MAP_HEIGHT) > 256
#error "Maze map is limited to 256 cells on this implementation."
#endif

// Turn drive modes (plain ints so the file also works as an .ino).
#define TURN_MODE_NORMAL     0   // both wheels counter-rotate (spin about the axle centre)
#define TURN_MODE_LOCK_LEFT  1   // right wheel drives, left counter-rotates
#define TURN_MODE_LOCK_RIGHT 2   // left wheel drives (reverse), right counter-rotates
#define TURN_MODE_ARC        3   // forward arc, inner wheel slower

// ================================================================
// CONSTANTS
// ================================================================
const long TARGET_TICKS_FWD = (long)(TICKS_PER_CELL_25CM + 0.5f);

bool          started = false;
bool          turnFault = false;
unsigned long t0      = 0;

const float MAX_DUTY = 15.0f;
const float WALL_STEER_GAIN = 1.5f;
const float MAX_WALL_STEER_DUTY = 8.0f;
const float WALL_ERROR_DEADBAND_CM = 0.5f;
const float GYRO_HEADING_GAIN = STRAIGHT_TEST_GYRO_KP;
const float MAX_GYRO_STEER_DUTY = STRAIGHT_TEST_MAX_CORRECTION;
const float MAX_COMBINED_STEER_DUTY = 10.0f;
const float GYRO_HEADING_DEADBAND_DEG = 1.0f;

const float ENCODER_SYNC_GAIN = 0.35f;    // duty percentage per tick of wheel progress mismatch
const float MAX_ENCODER_SYNC_DUTY = 8.0f;
const long ENCODER_SYNC_DEADBAND_TICKS = 1;

static void updateWallCorrection(float leftDistanceCm,
                                 float rightDistanceCm,
                                 float &wallError,
                                 float &wallHeadingOffsetDeg,
                                 float &ultrasonicSteering,
                                 bool &searchingForSideWall) {
    const bool rightWallSeen = rightDistanceCm > 0.1f &&
                               rightDistanceCm <= RIGHT_WALL_THRESHOLD;
    const bool leftWallSeen = leftDistanceCm > 0.1f &&
                              leftDistanceCm <= LEFT_WALL_THRESHOLD;
    searchingForSideWall = !leftWallSeen && !rightWallSeen;

    wallError = 0.0f;
    if (leftWallSeen && rightWallSeen) {
        // Positive means shifted right, so steering should turn left.
        const float leftError = leftDistanceCm - LEFT_CENTER_DIST;
        const float rightError = rightDistanceCm - RIGHT_CENTER_DIST;
        wallError = 0.5f * (leftError - rightError);
    } else if (rightWallSeen) {
        wallError = RIGHT_CENTER_DIST - rightDistanceCm;
    } else if (leftWallSeen) {
        wallError = leftDistanceCm - LEFT_CENTER_DIST;
    }

    if (fabs(wallError) < WALL_ERROR_DEADBAND_CM) {
        wallError = 0.0f;
    }

#if WALL_STEER_AS_HEADING
    wallHeadingOffsetDeg = constrain(
        wallError * WALL_HEADING_GAIN_DEG_PER_CM,
        -WALL_HEADING_MAX_DEG,
        WALL_HEADING_MAX_DEG);
    ultrasonicSteering = 0.0f;
#else
    wallHeadingOffsetDeg = 0.0f;
    ultrasonicSteering = constrain(
        wallError * WALL_STEER_GAIN * ULTRASONIC_STEER_WEIGHT,
        -MAX_WALL_STEER_DUTY,
        MAX_WALL_STEER_DUTY);
#endif
}

// Sensor is mounted upside down; Gyroscope corrects yaw so left is positive.
Gyroscope gyroscope(Wire, true);
const unsigned long GYRO_SAMPLE_INTERVAL_US = 10000UL;
bool gyroReady = false;
float gyroHeadingDeg = 0.0f;

// IDEAL heading, always (start heading + n * 90). Turns update this by exactly
// +/-90, never from the measured yaw, so a turn that ends 3 degrees off is
// corrected on the next straight instead of becoming the new reference.
float gyroHeadingTargetDeg = 0.0f;

// ================================================================
// GLOBAL VARIABLES
// ================================================================
float eprevL = 0, eintegralL = 0;
float eprevR = 0, eintegralR = 0;
long  prevT = 0;

const float kpL = 4.5f, kdL = 0.0f, kiL = 0.0f;
const float kpR = 4.5f, kdR = 0.0f, kiR = 0.0f;

float frontDist, leftDist, rightDist;

// Compact cell map: known edges, wall edges, and whether the cell was visited.
struct MazeCell {
    uint8_t knownEdges;
    uint8_t wallEdges;
    bool visited;
};

static MazeCell mazeMap[MAZE_MAP_WIDTH][MAZE_MAP_HEIGHT] = {};
static uint8_t mapX = 0;
static uint8_t mapY = 0;
static uint8_t mapHeading = NORTH;
static bool mapComplete = false;
static bool moveTimedOut = false;

bool moveForwardPID(long targetTicks);
void navigateStep();
bool initializeGyroscope();
void reportPivotClearance();
bool turnAroundWithGyroscopePivots();

// ================================================================
// HELPERS
// ================================================================
static long ticksForCm(float cm) {
    return (long)(cm * (TICKS_PER_CELL_25CM / 25.0f) + 0.5f);
}

// True once at least one cell move has finished, so the tail is known to sit in
// a corridor and reversing is safe. Set in moveForwardPID().
static bool reverseSafe = false;
// True when the last 90 degree turn was an arc (it already advanced the robot).
static bool lastTurnWasArc = false;

// Ticks to drive after a 90 degree turn so the robot ends on the next cell centre.
static long forwardTicksAfterTurn() {
    if (!lastTurnWasArc) return TARGET_TICKS_FWD;
    // The arc already carried the axle ARC_TURN_RADIUS_CM along the new heading.
    const long t = TARGET_TICKS_FWD - ticksForCm(ARC_TURN_RADIUS_CM);
    return (t > ticksForCm(5.0f)) ? t : ticksForCm(5.0f);
}

// ---- Sensor filtering -------------------------------------------------
static float median3(float a, float b, float c) {
    float t;
    if (a > b) { t = a; a = b; b = t; }
    if (b > c) { t = b; b = c; c = t; }
    if (a > b) { t = a; a = b; b = t; }
    return b;
}

static float readMedian(int trig, int echo) {
    const float a = readUltrasonic(trig, echo);
    delay(8);
    const float b = readUltrasonic(trig, echo);
    delay(8);
    const float c = readUltrasonic(trig, echo);
    return median3(a, b, c);
}

// Replacement for readAllSensors() used for turn decisions.
static void readAllSensorsForDecision() {
#if USE_SENSOR_MEDIAN
    frontDist = readMedian(PIN_ULTRA_FRONT_TRIG, PIN_ULTRA_FRONT_ECHO);
    rightDist = readMedian(PIN_ULTRA_RIGHT_TRIG, PIN_ULTRA_RIGHT_ECHO);
    leftDist  = readMedian(PIN_ULTRA_LEFT_TRIG,  PIN_ULTRA_LEFT_ECHO);
#else
    readAllSensors();
#endif
}

static bool mapInBounds(int x, int y) {
    return x >= 0 && x < MAZE_MAP_WIDTH && y >= 0 && y < MAZE_MAP_HEIGHT;
}

static uint8_t mapEdgeBit(uint8_t direction) {
    return (uint8_t)(1U << direction);
}

static uint8_t oppositeDirection(uint8_t direction) {
    return (uint8_t)((direction + 2U) & 0x03U);
}

static bool mapNeighbor(int x, int y, uint8_t direction, int &nx, int &ny) {
    nx = x;
    ny = y;
    switch (direction) {
        case NORTH: ++ny; break;
        case EAST:  ++nx; break;
        case SOUTH: --ny; break;
        default:    --nx; break; // WEST
    }
    return mapInBounds(nx, ny);
}

static void mapSetEdge(int x, int y, uint8_t direction, bool isWall) {
    if (!mapInBounds(x, y)) return;

    int nx, ny;
    if (!mapNeighbor(x, y, direction, nx, ny)) {
        isWall = true; // The configured map boundary is a wall.
    }

    const uint8_t bit = mapEdgeBit(direction);
    mazeMap[x][y].knownEdges |= bit;
    if (isWall) mazeMap[x][y].wallEdges |= bit;
    else         mazeMap[x][y].wallEdges &= (uint8_t)~bit;

    if (mapInBounds(nx, ny)) {
        const uint8_t reverseBit = mapEdgeBit(oppositeDirection(direction));
        mazeMap[nx][ny].knownEdges |= reverseBit;
        if (isWall) mazeMap[nx][ny].wallEdges |= reverseBit;
        else        mazeMap[nx][ny].wallEdges &= (uint8_t)~reverseBit;
    }
}

static bool mapEdgeIsKnownOpen(int x, int y, uint8_t direction) {
    const uint8_t bit = mapEdgeBit(direction);
    return (mazeMap[x][y].knownEdges & bit) != 0 &&
           (mazeMap[x][y].wallEdges & bit) == 0;
}

static void mapObserveCurrentCell() {
    mazeMap[mapX][mapY].visited = true;

    const uint8_t leftDirection = (uint8_t)((mapHeading + 3U) & 0x03U);
    const uint8_t rightDirection = (uint8_t)((mapHeading + 1U) & 0x03U);
    mapSetEdge(mapX, mapY, mapHeading,
               !(frontDist > 0.1f && frontDist > FRONT_OBSTACLE));
    mapSetEdge(mapX, mapY, leftDirection,
               !(leftDist > 0.1f && leftDist > LEFT_WALL_THRESHOLD));
    mapSetEdge(mapX, mapY, rightDirection,
               !(rightDist > 0.1f && rightDist > RIGHT_WALL_THRESHOLD));
}

static uint8_t mapCellId(int x, int y) {
    return (uint8_t)(y * MAZE_MAP_WIDTH + x);
}

static void mapCellFromId(uint8_t id, int &x, int &y) {
    x = id % MAZE_MAP_WIDTH;
    y = id / MAZE_MAP_WIDTH;
}

static bool mapChooseNextDirection(uint8_t &direction) {
    const uint8_t relativePreference[4] = {
        (uint8_t)((mapHeading + 3U) & 0x03U), // left
        mapHeading,                           // straight
        (uint8_t)((mapHeading + 1U) & 0x03U), // right
        (uint8_t)((mapHeading + 2U) & 0x03U)  // back
    };

    // Explore an adjacent open, unvisited cell using the existing left-first bias.
    for (uint8_t i = 0; i < 4; ++i) {
        const uint8_t candidate = relativePreference[i];
        int nx, ny;
        if (!mapEdgeIsKnownOpen(mapX, mapY, candidate) ||
            !mapNeighbor(mapX, mapY, candidate, nx, ny)) continue;
        if (!mazeMap[nx][ny].visited) {
            direction = candidate;
            return true;
        }
    }

    // No local frontier: breadth-first search through mapped open corridors to
    // the nearest visited cell that borders an open, unvisited cell.
    const uint16_t cellCount = (uint16_t)MAZE_MAP_WIDTH * MAZE_MAP_HEIGHT;
    if (cellCount == 0 || cellCount > 256) return false;

    static uint8_t queue[256];
    static uint8_t parent[256];
    static uint8_t firstStepDirection[256];
    static bool seen[256];
    for (uint16_t i = 0; i < cellCount; ++i) seen[i] = false;
    uint16_t head = 0;
    uint16_t tail = 0;
    const uint8_t startId = mapCellId(mapX, mapY);
    queue[tail++] = startId;
    seen[startId] = true;

    while (head < tail) {
        const uint8_t currentId = queue[head++];
        int x, y;
        mapCellFromId(currentId, x, y);

        bool bordersFrontier = false;
        for (uint8_t d = 0; d < 4; ++d) {
            int nx, ny;
            if (mapEdgeIsKnownOpen(x, y, d) && mapNeighbor(x, y, d, nx, ny) &&
                !mazeMap[nx][ny].visited) {
                bordersFrontier = true;
                break;
            }
        }

        if (bordersFrontier && currentId != startId) {
            uint8_t stepId = currentId;
            while (parent[stepId] != startId) stepId = parent[stepId];
            direction = firstStepDirection[stepId];
            return true;
        }

        for (uint8_t d = 0; d < 4; ++d) {
            int nx, ny;
            if (!mapEdgeIsKnownOpen(x, y, d) || !mapNeighbor(x, y, d, nx, ny) ||
                !mazeMap[nx][ny].visited) continue;

            const uint8_t neighborId = mapCellId(nx, ny);
            if (seen[neighborId]) continue;
            seen[neighborId] = true;
            parent[neighborId] = currentId;
            firstStepDirection[neighborId] = currentId == startId
                ? d : firstStepDirection[currentId];
            queue[tail++] = neighborId;
        }
    }

    return false; // Every reachable mapped frontier has been explored.
}

static bool mapAdvanceToNeighbor(uint8_t direction) {
    int nx, ny;
    if (!mapNeighbor(mapX, mapY, direction, nx, ny)) return false;
    mapSetEdge(mapX, mapY, direction, false);
    mapX = (uint8_t)nx;
    mapY = (uint8_t)ny;
    return true;
}

// Boot-time check: can the body spin around the axle inside a corridor?
// The swept circle has radius sqrt(overhang^2 + (width/2)^2).
void reportPivotClearance() {
    if (ROBOT_WIDTH_CM <= 0.0f || CELL_INNER_WIDTH_CM <= 0.0f ||
        (AXLE_TO_REAR_CM <= 0.0f && AXLE_TO_FRONT_CM <= 0.0f)) {
        Serial.println(F("Geometry not set in config: pivot clearance check skipped."));
        return;
    }
    const float halfW  = ROBOT_WIDTH_CM * 0.5f;
    const float rearR  = sqrtf(AXLE_TO_REAR_CM  * AXLE_TO_REAR_CM  + halfW * halfW);
    const float frontR = sqrtf(AXLE_TO_FRONT_CM * AXLE_TO_FRONT_CM + halfW * halfW);
    const float swingR = (rearR > frontR) ? rearR : frontR;
    const float room   = CELL_INNER_WIDTH_CM * 0.5f;
    const float spare  = room - swingR;

    Serial.print(F("Pivot swing radius: ")); Serial.print(swingR, 1);
    Serial.print(F(" cm; half corridor: ")); Serial.print(room, 1);
    Serial.print(F(" cm; spare: ")); Serial.print(spare, 1);
    Serial.println(F(" cm"));
    if (spare < PIVOT_CLEARANCE_MARGIN_CM) {
        Serial.println(F("WARNING: in-place turns will touch the walls. Shorten the tail,"));
        Serial.println(F("move the axle toward the middle, or use USE_ARC_TURNS."));
    } else {
        Serial.println(F("Pivot clearance OK."));
    }

#if REVERSE_BEFORE_PIVOT
    Serial.print(F("Reverse before pivot: "));
    Serial.print(PIVOT_BACKUP_CM, 1);
    Serial.print(F(" cm (90), "));
    Serial.print(TURN_AROUND_BACKUP_CM, 1);
    Serial.println(F(" cm (180)."));
#endif

#if USE_ARC_TURNS
    // Outer rear corner of an arc whose axle path has radius R, measured from the
    // corridor centreline: sqrt(rear^2 + (R + width/2)^2) - R.
    const float outerReach = ARC_TURN_RADIUS_CM + halfW;
    const float arcSwing = sqrtf(AXLE_TO_REAR_CM * AXLE_TO_REAR_CM +
                                 outerReach * outerReach) - ARC_TURN_RADIUS_CM;
    const float arcSpare = room - arcSwing;
    Serial.print(F("Arc turn tail swing: ")); Serial.print(arcSwing, 1);
    Serial.print(F(" cm; spare: ")); Serial.print(arcSpare, 1);
    Serial.println(F(" cm"));
    if (arcSpare < PIVOT_CLEARANCE_MARGIN_CM) {
        Serial.println(F("Arc margin is thin: expect wall contact unless centring is within about 0.5 cm."));
    }
#endif
}

bool initializeGyroscope() {
    if (!gyroscope.begin(0, 0)) { // +/-250 dps gyro; accelerometer range is unused.
        Serial.println(F("MPU6050 initialization failed."));
        return false;
    }

    Serial.println(F("Keep robot still: calibrating gyro bias."));
    delay(1000);
    if (!gyroscope.calibrate()) return false;
    gyroscope.update();
    gyroHeadingDeg = gyroscope.getYawAngleDeg();
    gyroHeadingTargetDeg = gyroHeadingDeg;
    return true;
}

// ================================================================
// TURNING
// ================================================================

// One drive command for the current turn error (positive error = turn left).
static void driveTurn(float error, int duty, int mode) {
    const int rev = (int)lroundf(
        duty * constrain(TURN_REVERSE_SPEED_RATIO, 0.0f, 1.0f));

    if (mode == TURN_MODE_ARC) {
        // Inner/outer wheel speed ratio for an axle path of ARC_TURN_RADIUS_CM.
        // Below half the track width the ratio goes negative (inner wheel reverses).
        // "duty" runs from TURN_SPEED down to TURN_MIN_DUTY near the goal; scale it
        // up to ARC_TURN_SPEED so the inner wheel still gets enough duty to turn.
        const float half  = TRACK_WIDTH_CM * 0.5f;
        const float ratio = (ARC_TURN_RADIUS_CM - half) / (ARC_TURN_RADIUS_CM + half);
        const int outer   = (int)lroundf(duty * ((float)ARC_TURN_SPEED / (float)TURN_SPEED));
        const int inner   = (int)lroundf(outer * ratio);
        if (error > 0.0f) setMotors(inner, outer);  // left arc: right wheel is outer
        else              setMotors(outer, inner);  // right arc: left wheel is outer
        return;
    }

    if (mode == TURN_MODE_NORMAL) {
        // Positive gyro angle is left; this wiring turns left with
        // left reverse and right forward.
        if (error > 0.0f) setMotors(-rev, duty);
        else              setMotors(duty, -rev);
        return;
    }

    // Pivot variants used for the dead-end U-turn: the "moving" wheel drives
    // the turn, the other counter-rotates at TURN_REVERSE_SPEED_RATIO.
    int moving;
    if (mode == TURN_MODE_LOCK_LEFT) moving = (error > 0.0f) ? duty : -duty;
    else                             moving = (error > 0.0f) ? -duty : duty;
    const int counter = (moving > 0 ? -1 : 1) * rev;
    if (mode == TURN_MODE_LOCK_LEFT) setMotors(counter, moving);
    else                             setMotors(moving, counter);
}

// Turn until the yaw reaches (newGoalDeg + stopBiasDeg), an ABSOLUTE heading.
// stopBiasDeg carries your TURN_*_CORRECTION_DEG coast compensation.
// On success the ideal heading becomes exactly newGoalDeg.
static bool gyroTurnToGoal(float newGoalDeg, float stopBiasDeg, int mode) {
    if (!gyroReady) {
        stopMotors();
        Serial.println(F("Turn refused: MPU6050 is not ready."));
        return false;
    }

    const float stopHeading = newGoalDeg + stopBiasDeg;
    const float turnSizeDeg = fabs(newGoalDeg - gyroHeadingTargetDeg);
    const unsigned long timeoutMs =
        (turnSizeDeg > 90.5f) ? TURN_180_TIMEOUT_MS : TURN_90_TIMEOUT_MS;
    const unsigned long startMs = millis();
    unsigned long nextSampleUs = micros();
    float yaw = 0.0f;

    Serial.print(F("Gyro turn: goal "));
    Serial.print(newGoalDeg, 1);
    Serial.print(F(" deg, stop at "));
    Serial.print(stopHeading, 1);
    Serial.println(F(" deg"));

    for (uint8_t attempt = 0; attempt < 3; ++attempt) {
        // Corrections after an arc are plain in-place pivots.
        const int activeMode =
            (mode == TURN_MODE_ARC && attempt > 0) ? TURN_MODE_NORMAL : mode;

        gyroscope.update();   // the stored yaw can be 200+ ms old here
        yaw = gyroscope.getYawAngleDeg();

        // Stall watch: if the yaw barely changes over a window while the motors
        // are driving, the robot is touching something. Stop instead of pushing.
        unsigned long stallRefMs = millis();
        float stallRefYaw = yaw;
        bool stalled = false;

        while (fabs(stopHeading - yaw) > TURN_STOP_TOLERANCE_DEG &&
               millis() - startMs < timeoutMs) {
            if ((long)(micros() - nextSampleUs) < 0) continue;
            gyroscope.update();
            yaw = gyroscope.getYawAngleDeg();
            gyroHeadingDeg = yaw;

            if (millis() - stallRefMs >= TURN_STALL_WINDOW_MS) {
                if (fabs(yaw - stallRefYaw) < TURN_STALL_MIN_PROGRESS_DEG) {
                    stalled = true;
                    break;
                }
                stallRefYaw = yaw;
                stallRefMs = millis();
            }

            const float error = stopHeading - yaw;
            const float absError = fabs(error);
            int duty = TURN_SPEED;
            if (absError < TURN_SLOWDOWN_START_ERROR_DEG) {
                const float range = TURN_SLOWDOWN_START_ERROR_DEG -
                                    TURN_STOP_TOLERANCE_DEG;
                const float progress = constrain(
                    (absError - TURN_STOP_TOLERANCE_DEG) / range,
                    0.0f,
                    1.0f);
                duty = (int)lroundf(
                    TURN_MIN_DUTY + progress * (TURN_SPEED - TURN_MIN_DUTY));
            }

            driveTurn(error, duty, activeMode);
            nextSampleUs += GYRO_SAMPLE_INTERVAL_US;
        }

        stopMotors();
        if (stalled) {
            Serial.println(F("Turn blocked: no rotation, likely touching a wall. Stopped."));
            return false;
        }
        delay(200);
        gyroscope.update();
        yaw = gyroscope.getYawAngleDeg();
        gyroHeadingDeg = yaw;
        const float finalError = stopHeading - yaw;

        Serial.print(F("Yaw=")); Serial.print(yaw, 2);
        Serial.print(F(" deg; stop target=")); Serial.print(stopHeading, 2);
        Serial.print(F(" deg; error=")); Serial.println(finalError, 2);

        if (fabs(finalError) <= TURN_STOP_TOLERANCE_DEG) {
            gyroHeadingTargetDeg = newGoalDeg;   // ideal heading, not the measured one
            return true;
        }
        if (millis() - startMs >= timeoutMs) break;

        nextSampleUs = micros();   // avoid a burst of catch-up samples after the delay
        Serial.println(F("Outside tolerance after settling; correcting again."));
    }

    stopMotors();
    Serial.println(F("Turn did not reach target tolerance."));
    return false;
}

// Reverse a fixed distance (encoders only; no gyro hold for such a short move).
static void reverseByCm(float cm) {
    const long target = ticksForCm(cm);
    if (target <= 0) return;

    resetEncoders();
    const unsigned long startMs = millis();
    while (millis() - startMs < 3000UL) {
        long l, r;
        ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
            l = leftEncoderCount;
            r = rightEncoderCount;
        }
        if ((labs(l) + labs(r)) / 2 >= target) break;
        setMotors(-ARC_BACKUP_DUTY, -ARC_BACKUP_DUTY);
    }
    stopMotors();
    delay(100);
}

// 90 degree turn to the left or right using the configured style.
static bool turn90(bool left) {
    const float newGoal = gyroHeadingTargetDeg + (left ? 90.0f : -90.0f);
    // Same meaning as before: left stops at 90 - LEFT_CORR, right at -90 + RIGHT_CORR.
    const float bias = left ? -TURN_LEFT_CORRECTION_DEG : TURN_RIGHT_CORRECTION_DEG;
#if USE_ARC_TURNS
    if (reverseSafe) {
        lastTurnWasArc = true;
        reverseByCm(ARC_TURN_RADIUS_CM - ARC_BACKUP_TRIM_CM);
        return gyroTurnToGoal(newGoal, bias, TURN_MODE_ARC);
    }
    Serial.println(F("First turn: no known free space behind, using a pivot."));
#endif
    lastTurnWasArc = false;
#if REVERSE_BEFORE_PIVOT
    if (reverseSafe) {
        reverseByCm(PIVOT_BACKUP_CM - ARC_BACKUP_TRIM_CM);   // blind reverse: only after a full cell move
    }
#endif
#if TURN90_WHEEL_LOCK
    const int pivotMode = left ? TURN_MODE_LOCK_LEFT : TURN_MODE_LOCK_RIGHT;
#else
    const int pivotMode = TURN_MODE_NORMAL;
#endif
    // Normal mode spins about the axle; lock mode pivots about one wheel when
    // TURN_REVERSE_SPEED_RATIO is also set to 0.
    return gyroTurnToGoal(newGoal, bias, pivotMode);
}

// Dead-end turn: two gyro-measured 90 degree phases, left each time.
bool turnAroundWithGyroscopePivots() {
    const float g = gyroHeadingTargetDeg;
    const float bias = -TURN_LEFT_CORRECTION_DEG;
    if (!gyroTurnToGoal(g + 90.0f, bias, TURN_MODE_LOCK_LEFT)) return false;
    return gyroTurnToGoal(g + 180.0f, bias, TURN_MODE_LOCK_RIGHT);
}

// ================================================================
// NAVIGATION
// ================================================================
static bool turnToMapDirection(uint8_t targetDirection) {
    const uint8_t turnAmount = (uint8_t)((targetDirection - mapHeading) & 0x03U);
    if (turnAmount == 0) return true;

    if (turnAmount == 1) {
        if (!turn90(false)) return false;
    } else if (turnAmount == 3) {
        if (!turn90(true)) return false;
    } else {
#if REVERSE_BEFORE_PIVOT
        if (reverseSafe) {
            reverseByCm(TURN_AROUND_BACKUP_CM - ARC_BACKUP_TRIM_CM);
        }
#endif
        const bool turned = USE_TWO_90_TURNS_FOR_180
            ? turnAroundWithGyroscopePivots()
            : gyroTurnToGoal(gyroHeadingTargetDeg + 180.0f,
                             0.0f,
                             TURN_MODE_NORMAL);
        if (!turned) return false;
        lastTurnWasArc = false;
    }

    mapHeading = targetDirection;
    return true;
}

void navigateStep() {
    if (turnFault) {
        stopMotors();
        return;
    }
    readAllSensorsForDecision();
    mapObserveCurrentCell();

    Serial.println("----------------------------------------");
    Serial.print("Mapped cell (");
    Serial.print(mapX);
    Serial.print(',');
    Serial.print(mapY);
    Serial.print(F(") heading "));
    Serial.println(mapHeading);
    Serial.print("Sensors -> L: ");
    Serial.print(leftDist);
    Serial.print(" cm | F: ");
    Serial.print(frontDist);
    Serial.print(" cm | R: ");
    Serial.print(rightDist);
    Serial.println(" cm");
    Serial.println("----------------------------------------");

    uint8_t targetDirection;
    if (!mapChooseNextDirection(targetDirection)) {
        if (!mapComplete) {
            Serial.println(F("Map exploration complete: no reachable open frontier."));
            mapComplete = true;
        }
        stopMotors();
        return;
    }
    mapComplete = false;

    if (!turnToMapDirection(targetDirection)) {
        Serial.println(F("Turn failed; stopping with the map position unchanged."));
        turnFault = true;
        stopMotors();
        return;
    }

    const long moveTicks = lastTurnWasArc
        ? forwardTicksAfterTurn() : TARGET_TICKS_FWD;
    if (!moveForwardPID(moveTicks)) {
        Serial.println(F("Cell move incomplete; keeping the current map coordinate."));
        if (moveTimedOut) {
            Serial.println(F("Encoder move timed out; stopping because position is uncertain."));
            turnFault = true;
            stopMotors();
        }
        return;
    }

    if (!mapAdvanceToNeighbor(mapHeading)) {
        Serial.println(F("Map boundary reached; stopping."));
        turnFault = true;
        stopMotors();
        return;
    }
    lastTurnWasArc = false;
}

bool moveForwardPID(long targetTicks) {
    moveTimedOut = false;
    // Take a filtered wall snapshot while stopped so the first forward motor
    // command already includes the wall correction. The loop keeps refreshing it.
    stopMotors();
    const float frontAtStart = readUltrasonic(
        PIN_ULTRA_FRONT_TRIG, PIN_ULTRA_FRONT_ECHO);
    if (frontAtStart > 0.1f && frontAtStart < SAFETY_DIST) {
        Serial.println(F("OBSTACLE before cell move - staying stopped."));
        return false;
    }

    const float rightAtStart = readMedian(
        PIN_ULTRA_RIGHT_TRIG, PIN_ULTRA_RIGHT_ECHO);
    const float leftAtStart = readMedian(
        PIN_ULTRA_LEFT_TRIG, PIN_ULTRA_LEFT_ECHO);

    resetEncoders();

    const long targetL = targetTicks;
    const long targetR = targetTicks;

    eprevL = 0;
    eintegralL = 0;
    eprevR = 0;
    eintegralR = 0;
    prevT = micros();

    bool reached = false;
    bool leftDone = false;
    bool rightDone = false;
    bool obstacleDuringMove = false;

    unsigned long lastCheck = millis();
    unsigned long nextGyroSampleUs = micros();
    unsigned long lastDriveReportMs = lastCheck;
    const char *stopReason = "tick target";

    float ultrasonicSteering = 0.0f;
    float wallHeadingOffsetDeg = 0.0f;
    float gyroSteering = 0.0f;
    float steeringAdjustment = 0.0f;
    float leftWallDistanceCm = leftAtStart;
    float rightWallDistanceCm = rightAtStart;
    float wallError = 0.0f;
    bool searchingForSideWall = true;

    updateWallCorrection(leftWallDistanceCm, rightWallDistanceCm,
                         wallError, wallHeadingOffsetDeg,
                         ultrasonicSteering, searchingForSideWall);

    // Start timeout accounting after the stationary sensor snapshot.
    const unsigned long startTime = millis();

    while (!reached) {
        if (millis() - startTime > STRAIGHT_TEST_TIMEOUT_MS) {
            stopReason = "timeout";
            moveTimedOut = true;
            break;
        }

        // Refresh front safety and side-wall centering every 50 ms.
        if (millis() - lastCheck > 50UL) {
            const float front = readUltrasonic(
                PIN_ULTRA_FRONT_TRIG, PIN_ULTRA_FRONT_ECHO);

            if (front > 0.1f && front < SAFETY_DIST) {
                Serial.println(F("OBSTACLE - STOP"));
                stopReason = "front obstacle";
                obstacleDuringMove = true;
                stopMotors();
                break;
            }

            rightWallDistanceCm = readUltrasonic(
                PIN_ULTRA_RIGHT_TRIG, PIN_ULTRA_RIGHT_ECHO);
            leftWallDistanceCm = readUltrasonic(
                PIN_ULTRA_LEFT_TRIG, PIN_ULTRA_LEFT_ECHO);

            updateWallCorrection(leftWallDistanceCm, rightWallDistanceCm,
                                 wallError, wallHeadingOffsetDeg,
                                 ultrasonicSteering, searchingForSideWall);

            lastCheck = millis();
        }

        // Refresh gyro heading correction every 10 ms.
        if ((long)(micros() - nextGyroSampleUs) >= 0) {
            gyroscope.update();
            gyroHeadingDeg = gyroscope.getYawAngleDeg();

            float headingError =
                gyroHeadingDeg - (gyroHeadingTargetDeg + wallHeadingOffsetDeg);

            if (fabs(headingError) < GYRO_HEADING_DEADBAND_DEG) {
                headingError = 0.0f;
            }

            gyroSteering = constrain(
                -headingError * GYRO_HEADING_GAIN,
                -MAX_GYRO_STEER_DUTY,
                MAX_GYRO_STEER_DUTY);

            nextGyroSampleUs += GYRO_SAMPLE_INTERVAL_US;
        }

        steeringAdjustment = constrain(
            ultrasonicSteering + gyroSteering,
            -MAX_COMBINED_STEER_DUTY,
            MAX_COMBINED_STEER_DUTY);

        const long currT = micros();
        float deltaT = (currT - prevT) / 1e6f;
        prevT = currT;
        if (deltaT <= 0.0f) {
            deltaT = 1e-3f;
        }

        long leftPos;
        long rightPos;
        ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
            leftPos = leftEncoderCount;
            rightPos = rightEncoderCount;
        }

        const float eL = (float)targetL - (float)leftPos;
        const float dedtL = (eL - eprevL) / deltaT;
        eintegralL += eL * deltaT;
        const float uL =
            kpL * eL + kdL * dedtL + kiL * eintegralL;

        const float eR = (float)targetR - (float)rightPos;
        const float dedtR = (eR - eprevR) / deltaT;
        eintegralR += eR * deltaT;
        const float uR =
            kpR * eR + kdR * dedtR + kiR * eintegralR;

        if (leftPos >= targetL) leftDone = true;
        if (rightPos >= targetR) rightDone = true;

        long encoderDifference = leftPos - rightPos;
        if (labs(encoderDifference) <= ENCODER_SYNC_DEADBAND_TICKS) {
            encoderDifference = 0;
        }

        const float encoderSync = constrain(
            encoderDifference * ENCODER_SYNC_GAIN,
            -MAX_ENCODER_SYNC_DUTY,
            MAX_ENCODER_SYNC_DUTY);

        const float totalSteering = constrain(
            steeringAdjustment + encoderSync,
            -MAX_COMBINED_STEER_DUTY - MAX_ENCODER_SYNC_DUTY,
            MAX_COMBINED_STEER_DUTY + MAX_ENCODER_SYNC_DUTY);

        float pwrL = min(fabs(uL), MAX_DUTY);
        float pwrR = min(fabs(uR), MAX_DUTY);

        // Preserve enough torque near the target to avoid stalling short of
        // the requested encoder count; stop each wheel at its exact target.
        if (!leftDone && eL > 0 && eL <= 25 && pwrL < 20.0f) {
            pwrL = 20.0f;
        }
        if (!rightDone && eR > 0 && eR <= 25 && pwrR < 20.0f) {
            pwrR = 20.0f;
        }

        pwrL = constrain(pwrL - totalSteering, 0.0f, MAX_DUTY + 5.0f);
        pwrR = constrain(pwrR + totalSteering, 0.0f, MAX_DUTY + 5.0f);

        // With no side-wall reference, keep advancing slowly while the next
        // sensor samples search for either wall. Resume normal power on detection.
        if (searchingForSideWall) {
            pwrL = min(pwrL, NO_WALL_CREEP_DUTY);
            pwrR = min(pwrR, NO_WALL_CREEP_DUTY);
        }

        const int dirL = (uL > 0.0f) ? 1 : -1;
        const int dirR = (uR > 0.0f) ? 1 : -1;

        if (leftDone && rightDone) {
            reached = true;
            stopMotors();
        } else {
            setMotors(leftDone ? 0 : dirL * pwrL,
                      rightDone ? 0 : dirR * pwrR);
        }

        if (millis() - lastDriveReportMs >= 250UL) {
            Serial.print(F("Steer U/G/E="));
            Serial.print(ultrasonicSteering, 1);
            Serial.print('/');
            Serial.print(gyroSteering, 1);
            Serial.print('/');
            Serial.print(encoderSync, 1);
            Serial.print(F(" wallOffset="));
            Serial.print(wallHeadingOffsetDeg, 1);
            Serial.print(F(" side L/R="));
            Serial.print(leftWallDistanceCm, 1);
            Serial.print('/');
            Serial.print(rightWallDistanceCm, 1);
            Serial.print(F(" wallErr="));
            Serial.println(wallError, 1);
            lastDriveReportMs = millis();
        }

        eprevL = eL;
        eprevR = eR;
    }

    stopMotors();
    delay(200);

    if (reached) reverseSafe = true;   // a full move finished: the tail is in a corridor

    long finalLeftPos;
    long finalRightPos;
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        finalLeftPos = leftEncoderCount;
        finalRightPos = rightEncoderCount;
    }

    // If a new obstacle appeared during this cell move, reverse the measured
    // partial progress so the map coordinate still corresponds to this cell.
    if (obstacleDuringMove) {
        const long traveledTicks = (labs(finalLeftPos) + labs(finalRightPos)) / 2;
        if (traveledTicks > 0) {
            const float rollbackCm =
                traveledTicks * (25.0f / TICKS_PER_CELL_25CM);
            Serial.print(F("Collision recovery: backing up "));
            Serial.print(rollbackCm, 1);
            Serial.println(F(" cm to the cell start."));
            reverseByCm(rollbackCm);
        }
    }

    gyroscope.update();
    gyroHeadingDeg = gyroscope.getYawAngleDeg();

    Serial.print(F("Cell move ended: L="));
    Serial.print(finalLeftPos);
    Serial.print(F("/"));
    Serial.print(targetL);
    Serial.print(F(" ticks, R="));
    Serial.print(finalRightPos);
    Serial.print(F("/"));
    Serial.print(targetR);
    Serial.print(F(" ticks; yaw err="));
    Serial.print(gyroHeadingDeg - gyroHeadingTargetDeg, 1);
    Serial.print(F(" deg; reason="));
    Serial.println(stopReason);
    return reached;
}

// ================================================================
// SETUP / LOOP
// ================================================================
void setup() {
    Serial.begin(115200);
    delay(1000);
    pinMode(PIN_LEFT_ENC_A, INPUT_PULLUP);
    pinMode(PIN_LEFT_ENC_B, INPUT_PULLUP);
    pinMode(PIN_RIGHT_ENC_A, INPUT_PULLUP);
    pinMode(PIN_RIGHT_ENC_B, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(PIN_LEFT_ENC_A), leftEncoderISR, RISING);
    attachInterrupt(digitalPinToInterrupt(PIN_RIGHT_ENC_A), rightEncoderISR, RISING);

    pinMode(PIN_LEFT_RPWM, OUTPUT);
    pinMode(PIN_LEFT_LPWM, OUTPUT);
    pinMode(PIN_RIGHT_RPWM, OUTPUT);
    pinMode(PIN_RIGHT_LPWM, OUTPUT);

    pinMode(PIN_ULTRA_FRONT_TRIG, OUTPUT);
    pinMode(PIN_ULTRA_FRONT_ECHO, INPUT);
    pinMode(PIN_ULTRA_LEFT_TRIG, OUTPUT);
    pinMode(PIN_ULTRA_LEFT_ECHO, INPUT);
    pinMode(PIN_ULTRA_RIGHT_TRIG, OUTPUT);
    pinMode(PIN_ULTRA_RIGHT_ECHO, INPUT);

    prevT = micros();

    gyroReady = initializeGyroscope();
    t0 = millis();
    if (!gyroReady) {
        stopMotors();
        Serial.println(F("MPU6050 not detected or calibration read failed. Auto drive disabled."));
    }

    Serial.println("========================================");
    Serial.println("WALL FOLLOWER - READY");
    Serial.println(gyroReady ? F("Gyro yaw correction enabled. Auto start in 3 seconds...") : F("Auto start disabled until MPU6050 works."));
    Serial.println("Priority: Left -> Front -> Right");
    reportPivotClearance();
    Serial.println("========================================");
}

void loop() {
    if (gyroReady && !started && millis() - t0 >= 3000) {
        started = true;
        Serial.println("\n=== STARTING AUTO DRIVE ===");
    }

    if (started) {
        navigateStep();
    }

    delay(50);
}
