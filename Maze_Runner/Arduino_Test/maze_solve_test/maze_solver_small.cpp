#include <Arduino.h>
#include <util/atomic.h>
#include "config.h"
#include "Ultrasonic.h"
#include "MotorDriver.h"
#include "globals.h"
#include "Gyro.h"

// ================================================================
// CONSTANTS
// ================================================================
const long TARGET_TICKS_FWD = (long)TICKS_PER_CELL_25CM;

#define LOG_MOVE 1         // 0 = off, 1 = text lines for the log file, 2 = Teleplot graph lines
#define AUTOTUNE_WALLS 0   // 1 = run the wall-centring autotune in a straight corridor instead of solving
#define TUNE_START_SET 0   // first set to run (0-11): use it to continue after a crash
#define TUNE_MOVES 4       // cells to drive each way (the end wall stops it early if the corridor is shorter)

// Straight-move speed profile (duty %). MAX_DUTY (cruise) is in globals.h.
#define MOVE_START_DUTY      18      // duty at the start of the ramp (just above the stall duty)
#define MOVE_RAMP_MS         250     // time to ramp from MOVE_START_DUTY up to MAX_DUTY
#define MOVE_DECEL_PER_TICK  0.07f   // braking: duty = min duty + this x remaining ticks (~4 cm zone)
#define MOVE_STOP_TICKS      10      // cut the motors this close to the target and let it coast (~0.2 cm)
#define MOVE_MAX_STEER       5.0f    // limit on the left/right duty difference
#define HEADING_KD           0.05f   // duty % per deg/s of yaw rate: damps the weaving
#define ENC_BALANCE_KP       0.05f   // no gyro: duty % per tick of left/right encoder difference
#define WALL_OFFSET_FILTER   0.3f    // 0..1 smoothing of the wall heading nudge (lower = smoother)

bool          started = false;
unsigned long t0      = 0;

// turn speed, max duty inside globals.h

// ================================================================
// GLOBAL VARIABLES
// ================================================================
float frontDist, leftDist, rightDist;

enum RelativeDir { REL_LEFT = -1, REL_FRONT = 0, REL_RIGHT = 1, REL_DEAD = 99 };

// ================================================================
// PROTOTYPES
// ================================================================
void leftEncoderISR();
void rightEncoderISR();
void setPWM(uint8_t pin, uint32_t freq, uint8_t dutyCycle);
void setMotors(int leftDuty, int rightDuty);
void stopMotors();
void resetEncoders();
void moveForwardPID();
void turnDegrees(float degrees);
void turnLeft();
void turnRight();
void turnAround();

float readUltrasonic(int trigPin, int echoPin);
void  readAllSensors();
bool  isPathClearFront();
bool  isWallOnRight();
bool  isWallOnLeft();
void  navigateStep();

// Left-hand wall follower (no memory of visited cells).
RelativeDir chooseNextDirection() {
    bool leftWall  = isWallOnLeft();
    bool rightWall = isWallOnRight();
    bool frontOpen = isPathClearFront();

// 1st Priority: Turn left if there is an opening on the left
    if (!leftWall) {
        return REL_LEFT;
    }
    
    // 2nd Priority: Move forward if front is open (hug the left wall)
    if (frontOpen) {
        return REL_FRONT;
    }

    // 3rd Priority: Turn right only if left AND front are blocked
    if (!rightWall) {
        return REL_RIGHT;
    }

    // 4th Priority: U-Turn at dead ends
    return REL_DEAD;
}

// Drives slowly forward/back until the front reading equals the centred distance.
void alignToFrontWall() {
    unsigned long start = millis();
    while (millis() - start < 1500) {
        float f = readUltrasonic(PIN_ULTRA_FRONT_TRIG, PIN_ULTRA_FRONT_ECHO);
        if (f > FRONT_WALL_CM) break;                 // no wall ahead after all
        float err = f - FRONT_TARGET_CM;              // > 0: too far from the wall
        if (fabs(err) <= 0.4f) break;
        // Slow down as it gets close instead of banging back and forth at full align duty
        int mag  = constrain((int)(8.0f * fabs(err)), max(MIN_DUTY_LEFT, MIN_DUTY_RIGHT), FRONT_ALIGN_DUTY);
        int duty = (err > 0) ? mag : -mag;
        setMotors(duty, duty);
        delay(30);
    }
    stopMotors();
    delay(100);
}

float headingRef = 0.0f;   // absolute heading target (deg, multiple of 90, right = +)

// Turns so the heading lands on an exact multiple of 90 degrees (no drift build-up).
void turnBy(float delta) {
    currentHeading = (currentHeading + 4 + (int)(delta / 90.0f)) % 4;
    if (!gyroReady()) {
        turnDegrees(delta);
        return;
    }
    headingRef += delta;
    turnDegrees(headingRef - gyroYaw());
}

void navigateStep() {
    if (gyroReady()) gyroCalibrate(100);   // robot is still: refresh the gyro bias
    readAllSensors();

    if (frontDist < FRONT_WALL_CM) {       // wall ahead: move to the exact cell centre first
        alignToFrontWall();
        readAllSensors();
    }

// --- ADD THIS SERIAL PRINT BLOCK HERE ---
    Serial.println("----------------------------------------");
    Serial.print("Sensors -> L: ");
    Serial.print(leftDist);
    Serial.print(" cm | F: ");
    Serial.print(frontDist);
    Serial.print(" cm | R: ");
    Serial.print(rightDist);
    Serial.println(" cm");
    Serial.println("----------------------------------------");
    // ----------------------------------------

    RelativeDir nextDir = chooseNextDirection();

    if (nextDir == REL_DEAD) {
        Serial.println("Dead end -> turn around");
        turnBy(180.0f);
        return;
    }

    if (nextDir == REL_LEFT) {
        Serial.println("Turning left");
        turnBy(-90.0f);
        moveForwardPID();
        return;
    }

    if (nextDir == REL_RIGHT) {
        Serial.println("Turning right");
        turnBy(90.0f);
        moveForwardPID();
        return;
    }

    Serial.println("Moving forward");
    moveForwardPID();
}

// Positive = robot is too close to the right wall (needs to turn left).
float wallError(float l, float r) {
    bool wallL = l < WALL_THRESHOLD;
    bool wallR = r < WALL_THRESHOLD;

    if (wallL && wallR) return l - r;
    if (wallR)          return WALL_CENTER_CM - r;
    if (wallL)          return l - WALL_CENTER_CM;
    return 0.0f;
}

// Centring gains (variables so the autotune can change them)
float wallYawGain = WALL_YAW_GAIN;
float maxWallYaw  = MAX_WALL_YAW;
float headingKp   = HEADING_KP;

// Centring statistics collected during moves (used by the autotune)
float statSumErr2 = 0.0f;   // sum of wall error^2 (cm^2)
float statSumYaw2 = 0.0f;   // sum of heading deviation^2 (deg^2)
int   statN       = 0;
int   statClose   = 0;      // samples with a wall closer than 1 cm

float median3(const float *v) {
    float a = v[0], b = v[1], c = v[2];
    if (a > b) { float t = a; a = b; b = t; }
    if (b > c) { float t = b; b = c; c = t; }
    if (a > b) { float t = a; a = b; b = t; }
    return b;
}

// Drives one cell straight ahead.
//
// Speed: one profile for the whole robot, based on the AVERAGE distance of the two wheels:
//   ramp up from MOVE_START_DUTY to MAX_DUTY, cruise, then brake in proportion to the
//   distance left, and cut the motors MOVE_STOP_TICKS before the target (no reversing).
// Direction: a single steering term (gyro heading PD, nudged by the side walls).
//   Without the gyro: wall error + keep both encoders equal.
void moveForwardPID() {
    resetEncoders();

    long          target     = TARGET_TICKS_FWD;   // average ticks of both wheels
    unsigned long startTime  = millis();
    unsigned long lastCheck  = 0;
    float         wallSteer  = 0.0f;         // no gyro: steering from the walls
    float         wallOffset = 0.0f;         // gyro: filtered heading nudge from the walls (deg)
    float         desiredYaw = headingRef;   // heading target (deg, right = positive)
    const int     minBase    = max(MIN_DUTY_LEFT, MIN_DUTY_RIGHT);

    // Last 3 side readings, started from the readings taken before the move
    float lBuf[3] = {max(leftDist, 0.0f),  max(leftDist, 0.0f),  max(leftDist, 0.0f)};
    float rBuf[3] = {max(rightDist, 0.0f), max(rightDist, 0.0f), max(rightDist, 0.0f)};
    int   sensorTurn = 0;   // 0 = front, 1 = left, 2 = right
    bool  wallLPrev  = lBuf[2] < WALL_THRESHOLD;
    bool  wallRPrev  = rBuf[2] < WALL_THRESHOLD;
    bool  edgeDone   = false;

    while (millis() - startTime < 4000) {
        // One sensor every 25 ms, in turn, so one sensor cannot hear another's echo
        if (millis() - lastCheck >= 25) {
            lastCheck  = millis();
            sensorTurn = (sensorTurn + 1) % 3;

            if (sensorTurn == 0) {
                // No echo reads 100; a very close wall can read below 0, so no lower bound here
                float f = readUltrasonic(PIN_ULTRA_FRONT_TRIG, PIN_ULTRA_FRONT_ECHO);
                if (f <= FRONT_TARGET_CM + FRONT_SLOW_CM) {   // wall ahead: stop, then creep in
                    Serial.println("Front wall - stop, align slowly");
                    break;
                }
            } else {
                float  d   = (sensorTurn == 1) ? readUltrasonic(PIN_ULTRA_LEFT_TRIG, PIN_ULTRA_LEFT_ECHO)
                                               : readUltrasonic(PIN_ULTRA_RIGHT_TRIG, PIN_ULTRA_RIGHT_ECHO);
                float *buf = (sensorTurn == 1) ? lBuf : rBuf;
                buf[0] = buf[1];
                buf[1] = buf[2];
                buf[2] = max(d, 0.0f);   // below 0 = touching the wall: keep it as 0, don't drop it

                float l = median3(lBuf);
                float r = median3(rBuf);

                // A side wall starting or ending marks the cell boundary (logged for calibration).
                // With EDGE_CORRECTION the move is re-aimed to stop on the next cell centre.
                bool wallLNow = l < WALL_THRESHOLD;
                bool wallRNow = r < WALL_THRESHOLD;
                bool flipL    = wallLNow != wallLPrev;
                if (flipL || wallRNow != wallRPrev) {
                    long lp, rp;
                    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
                        lp = leftEncoderCount;
                        rp = rightEncoderCount;
                    }
                    float doneCm  = (lp + rp) / 2.0f / TICKS_PER_CM;
                    bool  nowWall = flipL ? wallLNow : wallRNow;

                    if (LOG_MOVE) {
                        Serial.print("edge ");
                        Serial.print(flipL ? "L" : "R");
                        Serial.print(nowWall ? " open->wall at " : " wall->open at ");
                        Serial.print(doneCm, 1);
                        Serial.println(" cm (geometry: 7.2)");
                    }

                    if (EDGE_CORRECTION && !edgeDone && doneCm > 2.0f && doneCm < 15.0f) {
                        target   = (lp + rp) / 2 + (long)(EDGE_TO_CENTER_CM * TICKS_PER_CM);
                        edgeDone = true;
                    }
                }
                wallLPrev = wallLNow;
                wallRPrev = wallRNow;

                float wallErr = wallError(l, r);
                if (gyroReady()) {
                    // The walls only nudge the heading target, smoothed so a wall appearing or
                    // ending does not make a step in the heading
                    float raw   = constrain(-wallYawGain * wallErr, -maxWallYaw, maxWallYaw);
                    wallOffset += WALL_OFFSET_FILTER * (raw - wallOffset);
                    desiredYaw  = headingRef + wallOffset;
                } else {
                    wallSteer = constrain(WALL_STEER_GAIN * wallErr, -MOVE_MAX_STEER, MOVE_MAX_STEER);
                }

                if (sensorTurn == 2 && (l < WALL_THRESHOLD || r < WALL_THRESHOLD)) {
                    float yawDev = gyroYaw() - headingRef;
                    statSumErr2 += wallErr * wallErr;
                    statSumYaw2 += yawDev * yawDev;
                    statN++;
                    if (l < 1.0f || r < 1.0f) statClose++;
                }

                if (LOG_MOVE == 1 && sensorTurn == 2) {          // text, for the log file
                    Serial.print("mv L="); Serial.print(l, 1);
                    Serial.print(" R=");   Serial.print(r, 1);
                    Serial.print(" werr=");   Serial.print(wallErr, 1);
                    Serial.print(" desYaw="); Serial.print(desiredYaw - headingRef, 1);
                    Serial.print(" yaw=");    Serial.println(gyroYaw() - headingRef, 1);
                }
                if (LOG_MOVE == 2 && sensorTurn == 2) {          // Teleplot graph lines
                    Serial.print(">L:");      Serial.println(l, 1);
                    Serial.print(">R:");      Serial.println(r, 1);
                    Serial.print(">werr:");   Serial.println(wallErr, 1);
                    Serial.print(">desYaw:"); Serial.println(desiredYaw - headingRef, 1);
                    Serial.print(">yaw:");    Serial.println(gyroYaw() - headingRef, 1);
                }
            }
        }

        long leftPos, rightPos;
        ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
            leftPos  = leftEncoderCount;
            rightPos = rightEncoderCount;
        }

        long remaining = target - (leftPos + rightPos) / 2;
        if (remaining <= MOVE_STOP_TICKS)
            break;

        // --- Speed: ramp up, cruise, brake ---
        float rampDuty  = MOVE_START_DUTY + (MAX_DUTY - MOVE_START_DUTY) * (millis() - startTime) / (float)MOVE_RAMP_MS;
        float brakeDuty = minBase + MOVE_DECEL_PER_TICK * remaining;
        float base      = min(MAX_DUTY, min(rampDuty, brakeDuty));
        if (base < minBase) base = minBase;

        // --- Direction: positive = turn left ---
        float steer;
        if (gyroReady()) {
            gyroUpdate();
            steer = headingKp * (gyroYaw() - desiredYaw) + HEADING_KD * gyroRate();
        } else {
            steer = wallSteer + ENC_BALANCE_KP * (leftPos - rightPos);   // left ahead = veering right
        }
        steer = constrain(steer, -MOVE_MAX_STEER, MOVE_MAX_STEER);

        float pwrL = constrain(base - steer,                   (float)MIN_DUTY_LEFT,  MAX_DUTY + MOVE_MAX_STEER);
        float pwrR = constrain(base * RIGHT_DUTY_TRIM + steer, (float)MIN_DUTY_RIGHT, MAX_DUTY + MOVE_MAX_STEER);
        setMotors((int)(pwrL + 0.5f), (int)(pwrR + 0.5f));
    }
    stopMotors();
    delay(200);
}

// Wall-centring autotune, one set at a time. Needs a straight corridor with walls on
// both sides and at both ends; robot axle on the centre of cell 1, facing down it.
// Each set: wait for the hand signal, drive TUNE_MOVES cells, turn 180, drive back,
// turn 180 (ends at the start facing forward), print the result, then wait again.
// Cost (lower is better) = RMS wall error (cm) + 0.1 x RMS heading deviation (deg)
//                          + 5 x fraction of samples with a wall closer than 1 cm.
const float TUNE_GAINS[]  = {2.0f, 3.0f, 4.0f};
const float TUNE_LIMITS[] = {6.0f, 10.0f};
const float TUNE_KPS[]    = {1.0f, 1.5f};
const int   TUNE_SETS     = 12;

void tuneParams(int i, float &g, float &lim, float &kp) {
    g   = TUNE_GAINS[i / 4];
    lim = TUNE_LIMITS[(i / 2) % 2];
    kp  = TUNE_KPS[i % 2];
}

// Waits for a hand over the front sensor (3 readings under 5 cm), then for it to go away.
void waitForHandSignal() {
    Serial.println("Centre the robot in cell 1, then cover the front sensor with your hand and take it away.");
    int seen = 0;
    while (seen < 3) {
        seen = (readUltrasonic(PIN_ULTRA_FRONT_TRIG, PIN_ULTRA_FRONT_ECHO) < 5.0f) ? seen + 1 : 0;
        delay(60);
    }
    Serial.println("Hand seen - take it away.");
    while (readUltrasonic(PIN_ULTRA_FRONT_TRIG, PIN_ULTRA_FRONT_ECHO) < 10.0f) delay(60);
    Serial.println("Starting in 3 s. Keep clear and do not touch the robot.");
    delay(3000);
}

void runWallAutotune() {
    float costs[TUNE_SETS];
    bool  ran[TUNE_SETS] = {false};

    for (int i = TUNE_START_SET; i < TUNE_SETS; i++) {
        float g, lim, kp;
        tuneParams(i, g, lim, kp);
        Serial.print("Next: set ");   Serial.print(i);
        Serial.print("  wallGain=");  Serial.print(g, 1);
        Serial.print("  maxYaw=");    Serial.print(lim, 1);
        Serial.print("  headKp=");    Serial.println(kp, 1);

        waitForHandSignal();
        if (gyroReady()) gyroCalibrate();   // robot is still: fresh bias
        gyroResetYaw();
        headingRef = 0.0f;                  // straight ahead = the way you placed it

        wallYawGain = g;
        maxWallYaw  = lim;
        headingKp   = kp;
        statSumErr2 = statSumYaw2 = 0.0f;
        statN = statClose = 0;

        for (int leg = 0; leg < 2; leg++) {
            for (int c = 0; c < TUNE_MOVES; c++) {
                if (gyroReady()) gyroCalibrate(100);
                readAllSensors();
                moveForwardPID();
            }
            readAllSensors();
            if (frontDist < FRONT_WALL_CM) alignToFrontWall();
            turnBy(180.0f);
        }

        if (statN == 0) {
            Serial.println("No walls seen: check the corridor. Set skipped.");
            continue;
        }

        float rmsErr    = sqrt(statSumErr2 / statN);
        float rmsYaw    = sqrt(statSumYaw2 / statN);
        float closeFrac = (float)statClose / statN;
        costs[i] = rmsErr + 0.1f * rmsYaw + 5.0f * closeFrac;
        ran[i]   = true;

        Serial.println("tune,set,wallGain,maxYaw,headKp,rmsErr,rmsYaw,closeFrac,cost");
        Serial.print("tune,");        Serial.print(i);         Serial.print(',');
        Serial.print(g, 1);           Serial.print(',');
        Serial.print(lim, 1);         Serial.print(',');
        Serial.print(kp, 1);          Serial.print(',');
        Serial.print(rmsErr, 2);      Serial.print(',');
        Serial.print(rmsYaw, 2);      Serial.print(',');
        Serial.print(closeFrac, 2);   Serial.print(',');
        Serial.println(costs[i], 2);
    }

    int best = -1;
    for (int i = TUNE_START_SET; i < TUNE_SETS; i++)
        if (ran[i] && (best < 0 || costs[i] < costs[best])) best = i;

    if (best >= 0) {
        float g, lim, kp;
        tuneParams(best, g, lim, kp);
        Serial.println("===== BEST =====");
        Serial.print("#define WALL_YAW_GAIN "); Serial.print(g, 1);   Serial.println("f");
        Serial.print("#define MAX_WALL_YAW ");  Serial.print(lim, 1); Serial.println("f");
        Serial.print("#define HEADING_KP ");    Serial.print(kp, 1);  Serial.println("f");
        Serial.print("// cost = ");             Serial.println(costs[best], 2);
    }
}

void setup() {
    Serial.begin(115200);
    delay(1000);

    initEncoders();

    if (!gyroBegin()) {
        Serial.println("MPU-6050 NOT FOUND: no heading hold, encoder turns");
    } else {
        Serial.println("Calibrating gyro, keep the robot still...");
        if (!gyroCalibrate()) Serial.println("Warning: robot moved during gyro calibration");
    }

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

    t0 = millis();

    Serial.println("========================================");
    Serial.println("WALL FOLLOWER - READY");
    Serial.println("Auto start in 3 seconds...");
    Serial.println("Priority: Left -> Front -> Right");
    Serial.println("========================================");

    if (AUTOTUNE_WALLS) runWallAutotune();
}

void loop() {
    if (AUTOTUNE_WALLS) return;   // autotune runs once in setup()

    if (!started && millis() - t0 >= 3000) {
        started = true;
        Serial.println("\n=== STARTING AUTO DRIVE ===");
    }

    if (started) {
        navigateStep();
    }

    delay(50);
}