#include <Arduino.h>
#include <util/atomic.h>
#include "config.h"
#include "Ultrasonic.h"
#include "MotorDriver.h"
#include "globals.h"
#include "Gyro.h"
#include "LineFollower.h"

// 1 = watch the IR array during moves for the special cell (white strip / black square /
// white strip). When found, the move stops and line mode runs. Test first with env:line_mode_test.
#ifndef LINE_MODE
#define LINE_MODE 0
#endif
bool  lineModePending = false;
float mazeOdoCm       = 0.0f;   // distance driven in straight moves, kept across cells

// ================================================================
// CONSTANTS
// ================================================================
const long TARGET_TICKS_FWD = (long)TICKS_PER_CELL_25CM;

#define LOG_MOVE 1         // 0 = off, 1 = text lines for the log file, 2 = Teleplot graph lines
#define AUTOTUNE_WALLS 0   // 1 = run the wall-centring autotune in a straight corridor instead of solving
#define TUNE_START_SET 0   // first set to run (0-11): use it to continue after a crash
#define TUNE_MOVES 4       // cells to drive each way (the end wall stops it early if the corridor is shorter)

// ---- Straight-line tuning stages (guide: Doc/straight_tuning.md) ----
// Set from platformio.ini (env tune1_distance ... tune4_walls) or change the default here.
// 0 = normal maze run
// 1 = distance only: no steering at all, measure the distance with a tape
// 2 = encoders only: keep the left and right tick counts equal (ENC_SYNC_KP)
// 3 = gyro only: hold the start heading (HEADING_KP, HEADING_KD), side walls ignored
// 4 = gyro + side walls: the same steering as the maze run, in a straight corridor
// In stages 1-4 every run starts with the hand signal and drives TUNE_CELLS cells.
#ifndef TUNE_STAGE
#define TUNE_STAGE 0
#endif
#ifndef TUNE_CELLS
#define TUNE_CELLS 1
#endif
#define STEER_WALLS (TUNE_STAGE == 0 || TUNE_STAGE == 4)
#define STEER_GYRO  (TUNE_STAGE == 0 || TUNE_STAGE == 3 || TUNE_STAGE == 4)
#define STEER_ENC   (TUNE_STAGE == 2)

bool          started = false;
unsigned long t0      = 0;

// turn speed, max duty inside globals.h

// ================================================================
// GLOBAL VARIABLES
// ================================================================
float eprevL = 0, eintegralL = 0;
float eprevR = 0, eintegralR = 0;
long  prevT = 0;

const float kpL = 4.0f, kdL = 0.0f, kiL = 0.0f;
const float kpR = 4.5f, kdR = 0.0f, kiR = 0.0f;

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

RelativeDir findNextUnvisitedNeighbor() {
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
        int duty = (err > 0) ? FRONT_ALIGN_DUTY : -FRONT_ALIGN_DUTY;
        setMotors(duty, duty);
        delay(30);
    }
    stopMotors();
    delay(100);
}

float headingRef = 0.0f;   // absolute heading target (deg, multiple of 90, right = +)

// Turns so the heading lands on an exact multiple of 90 degrees (no drift build-up).
void turnBy(float delta) {
    if (LINE_MODE) markerReset();   // a new direction: start the pattern again
    currentHeading = (currentHeading + 4 + (int)(delta / 90.0f)) % 4;
    if (!gyroReady()) {
        turnDegrees(delta);
        return;
    }
    headingRef += delta;
    turnDegrees(headingRef - gyroYaw());
}

void navigateStep() {
    // Special cell found (or the last move ended in the middle of its pattern)
    if (LINE_MODE && (lineModePending || markerState() == MARKER_STARTED)) {
        lineModePending = false;
        Serial.println("Special cell -> line mode");
        LineModeResult r = runLineMode(true);
        markerReset();
        Serial.print("Line mode done: ");
        Serial.println(lineModeResultName(r));
        // TODO: after the line section the robot is not on a cell centre / 90 deg heading.
        // Re-align (front wall / gyro headingRef) here before maze solving continues.
    }

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

    RelativeDir nextDir = findNextUnvisitedNeighbor();

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

long lastMoveL = 0, lastMoveR = 0;   // encoder counts at the end of the last move
void waitForHandSignal();

float median3(const float *v) {
    float a = v[0], b = v[1], c = v[2];
    if (a > b) { float t = a; a = b; b = t; }
    if (b > c) { float t = b; b = c; c = t; }
    if (a > b) { float t = a; a = b; b = t; }
    return b;
}

void moveForwardPID() {
    resetEncoders();

    long targetL = TARGET_TICKS_FWD;
    long targetR = TARGET_TICKS_FWD;

    eprevL     = 0;
    eintegralL = 0;
    eprevR     = 0;
    eintegralR = 0;
    prevT      = micros();

    bool          reached            = false;
    unsigned long startTime          = millis();
    unsigned long lastCheck          = 0;
    float         steeringAdjustment = 0;   // positive = turn left
    float         desiredYaw         = headingRef;   // heading target (deg, right = positive)

    // Last 3 side readings, started from the readings taken before the move
    float lBuf[3] = {max(leftDist, 0.0f),  max(leftDist, 0.0f),  max(leftDist, 0.0f)};
    float rBuf[3] = {max(rightDist, 0.0f), max(rightDist, 0.0f), max(rightDist, 0.0f)};
    int   sensorTurn = 0;   // 0 = front, 1 = left, 2 = right
    bool  wallLPrev  = lBuf[2] < WALL_THRESHOLD;
    bool  wallRPrev  = rBuf[2] < WALL_THRESHOLD;
    bool  edgeDone   = false;
    unsigned long lastIrCheck = 0;

    while (!reached) {
        if (millis() - startTime > 4000)
            break;

        // One sensor every 25 ms, in turn, so one sensor cannot hear another's echo
        if (millis() - lastCheck >= 25) {
            lastCheck  = millis();
            sensorTurn = (sensorTurn + 1) % 3;

            if (sensorTurn == 0) {
                float f = readUltrasonic(PIN_ULTRA_FRONT_TRIG, PIN_ULTRA_FRONT_ECHO);
                if (f > 0.1f && f <= FRONT_TARGET_CM + FRONT_SLOW_CM) {   // wall ahead: stop, then creep in
                    Serial.println("Front wall - stop, align slowly");
                    reached = true;
                    stopMotors();
                    break;
                }
            } else {
                float  d   = (sensorTurn == 1) ? readUltrasonic(PIN_ULTRA_LEFT_TRIG, PIN_ULTRA_LEFT_ECHO)
                                               : readUltrasonic(PIN_ULTRA_RIGHT_TRIG, PIN_ULTRA_RIGHT_ECHO);
                float *buf = (sensorTurn == 1) ? lBuf : rBuf;
                if (d >= 0.0f) {   // negative = bad echo: skip it
                    buf[0] = buf[1];
                    buf[1] = buf[2];
                    buf[2] = d;
                }

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
                        long remaining = (long)(EDGE_TO_CENTER_CM * TICKS_PER_CM);
                        targetL  = lp + remaining;
                        targetR  = rp + remaining;
                        edgeDone = true;
                    }
                }
                wallLPrev = wallLNow;
                wallRPrev = wallRNow;

                float wallErr = wallError(l, r);
                if (STEER_WALLS && gyroReady())   // the walls only nudge the heading target
                    desiredYaw = headingRef + constrain(-wallYawGain * wallErr, -maxWallYaw, maxWallYaw);

                if (sensorTurn == 2 && (l < WALL_THRESHOLD || r < WALL_THRESHOLD)) {
                    float yawDev = gyroYaw() - headingRef;
                    statSumErr2 += wallErr * wallErr;
                    statSumYaw2 += yawDev * yawDev;
                    statN++;
                    if (l < 1.0f || r < 1.0f) statClose++;
                }
                else if (STEER_WALLS)
                    steeringAdjustment = constrain(WALL_STEER_GAIN * wallErr, -5.0f, 5.0f);

                if (LOG_MOVE == 1 && sensorTurn == 2) {          // text, for the log file
                    long lp, rp;
                    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
                        lp = leftEncoderCount;
                        rp = rightEncoderCount;
                    }
                    Serial.print("mv cm="); Serial.print((lp + rp) / 2.0f / TICKS_PER_CM, 1);
                    Serial.print(" LR=");   Serial.print(lp - rp);
                    Serial.print(" st=");   Serial.print(steeringAdjustment, 1);
                    Serial.print(" L=");    Serial.print(l, 1);
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

        if (gyroReady()) {   // the gyro holds that heading on every loop
            gyroUpdate();    // always, so the heading stays right for the next turn and the log
            if (STEER_GYRO)
                steeringAdjustment = constrain(headingKp * (gyroYaw() - desiredYaw) + HEADING_KD * gyroRate(),
                                               -5.0f, 5.0f);
        }

        long  currT  = micros();
        float deltaT = (currT - prevT) / 1e6f;
        prevT        = currT;
        if (deltaT <= 0)
            deltaT = 1e-3f;

        long leftPos, rightPos;
        ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
            leftPos  = leftEncoderCount;
            rightPos = rightEncoderCount;
        }

        if (LINE_MODE && micros() - lastIrCheck >= 3000) {   // marker check every 3 ms
            lastIrCheck = micros();
            if (markerUpdate(mazeOdoCm + (leftPos + rightPos) / 2.0f / TICKS_PER_CM) == MARKER_FOUND) {
                Serial.println("Special cell found - stopping the move");
                lineModePending = true;
                reached = true;
                stopMotors();
                break;
            }
        }

#if STEER_ENC
        // Left wheel ahead = robot turning right -> positive = steer left
        steeringAdjustment = constrain(ENC_SYNC_KP * (float)(leftPos - rightPos), -5.0f, 5.0f);
#endif

        // Distance uses the AVERAGE of both wheels. With a separate target per wheel, the wheel
        // that got ahead (because of steering) stopped first and the other one caught up,
        // spinning the robot at the end of every move and undoing the steering.
        float eAvg  = ((float)(targetL + targetR) - (float)(leftPos + rightPos)) / 2.0f;
        float eL    = eAvg;
        float dedtL = (eL - eprevL) / deltaT;
        eintegralL += eL * deltaT;
        float uL = kpL * eL + kdL * dedtL + kiL * eintegralL;

        float eR    = eAvg;
        float dedtR = (eR - eprevR) / deltaT;
        eintegralR += eR * deltaT;
        float uR = kpR * eR + kdR * dedtR + kiR * eintegralR;

        float pwrL = fabs(uL);
        float pwrR = fabs(uR);

        if (pwrL > MAX_DUTY)
            pwrL = MAX_DUTY;
        if (pwrR > MAX_DUTY)
            pwrR = MAX_DUTY;

        // When a wheel runs backwards (overshoot), the steering has to be flipped for it,
        // otherwise it turns the robot the wrong way
        pwrL -= (uL > 0) ? steeringAdjustment : -steeringAdjustment;
        pwrR += (uR > 0) ? steeringAdjustment : -steeringAdjustment;

        if (pwrL > MAX_DUTY + 5)
            pwrL = MAX_DUTY + 5;
        if (pwrL < 0)
            pwrL = 0;
        if (pwrR > MAX_DUTY + 5)
            pwrR = MAX_DUTY + 5;
        if (pwrR < 0)
            pwrR = 0;

        if (pwrL > 0 && pwrL < MIN_DUTY_LEFT)  pwrL = MIN_DUTY_LEFT;
        if (pwrR > 0 && pwrR < MIN_DUTY_RIGHT) pwrR = MIN_DUTY_RIGHT;

        int dirL = (uL > 0) ? 1 : -1;
        int dirR = (uR > 0) ? 1 : -1;

        if (abs(eL) < 5 && abs(eR) < 5) {
            reached = true;
            stopMotors();
        } else {
            setMotors(dirL * pwrL, dirR * pwrR);
        }

        eprevL = eL;
        eprevR = eR;
    }
    stopMotors();

    unsigned long t = millis();
    while (millis() - t < 200)              // let it settle; keep the gyro running to catch coasting
        if (gyroReady()) gyroUpdate();

    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        lastMoveL = leftEncoderCount;
        lastMoveR = rightEncoderCount;
    }
    mazeOdoCm += (lastMoveL + lastMoveR) / 2.0f / TICKS_PER_CM;
    if (LOG_MOVE) {
        Serial.print("move done: L=");  Serial.print(lastMoveL);
        Serial.print(" R=");            Serial.print(lastMoveR);
        Serial.print(" LR=");           Serial.print(lastMoveL - lastMoveR);
        Serial.print(" gyro=");         Serial.print(gyroYaw() - headingRef, 1);
        Serial.print(" ms=");           Serial.print(millis() - startTime);
        Serial.println(reached ? "" : " (TIMEOUT)");
    }
}

// Stages 1-4: one test run per hand signal, then a summary line.
void stageRun() {
    static int run = 0;
    waitForHandSignal();
    if (gyroReady()) gyroCalibrate();   // robot is still: fresh bias
    gyroResetYaw();
    headingRef = 0.0f;                  // straight ahead = the way you placed it
    statSumErr2 = statSumYaw2 = 0.0f;
    statN = statClose = 0;

    readAllSensors();
    Serial.print("start L="); Serial.print(leftDist, 1);
    Serial.print(" F=");      Serial.print(frontDist, 1);
    Serial.print(" R=");      Serial.println(rightDist, 1);

    long sumL = 0, sumR = 0;
    int  cells = 0;
    float maxYaw = 0.0f;
    for (int c = 0; c < TUNE_CELLS; c++) {
        if (c > 0) readAllSensors();
        if (frontDist > 0.1f && frontDist <= FRONT_TARGET_CM + FRONT_SLOW_CM) {
            Serial.println("End wall reached.");
            break;
        }
        moveForwardPID();
        sumL += lastMoveL;
        sumR += lastMoveR;
        cells++;
        if (fabs(gyroYaw()) > maxYaw) maxYaw = fabs(gyroYaw());
    }

    run++;
    Serial.print("RESULT run=");  Serial.print(run);
    Serial.print(" stage=");      Serial.print(TUNE_STAGE);
    Serial.print(" cells=");      Serial.print(cells);
    Serial.print(" L=");          Serial.print(sumL);
    Serial.print(" R=");          Serial.print(sumR);
    Serial.print(" LR=");         Serial.print(sumL - sumR);
    Serial.print(" encCm=");      Serial.print((sumL + sumR) / 2.0f / TICKS_PER_CM, 1);
    Serial.print(" gyroEnd=");    Serial.print(gyroYaw(), 1);
    Serial.print(" gyroMaxEnd="); Serial.print(maxYaw, 1);
    if (statN > 0) {
        Serial.print(" rmsWallErr="); Serial.print(sqrt(statSumErr2 / statN), 2);
        Serial.print(" closeFrac=");  Serial.print((float)statClose / statN, 2);
    }
    Serial.println();
    Serial.println("Now measure: distance driven (cm) and sideways drift from the start line (cm).");
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
    if (LINE_MODE) lineBegin();

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

    prevT = micros();
    t0    = millis();

    Serial.println("========================================");
    Serial.println("WALL FOLLOWER - READY");
    Serial.println("Auto start in 3 seconds...");
    Serial.println("Priority: Left -> Right -> Front");
    Serial.println("========================================");

    if (TUNE_STAGE != 0) {
        Serial.print("TUNE STAGE ");     Serial.print(TUNE_STAGE);
        Serial.print("  cells=");        Serial.print(TUNE_CELLS);
        Serial.print("  ticks/cell=");   Serial.print(TICKS_PER_CELL_25CM, 0);
        Serial.print("  kpL=");          Serial.print(kpL, 2);
        Serial.print("  kpR=");          Serial.print(kpR, 2);
        Serial.print("  encSyncKp=");    Serial.print(ENC_SYNC_KP, 3);
        Serial.print("  headKp=");       Serial.print(headingKp, 2);
        Serial.print("  headKd=");       Serial.print(HEADING_KD, 3);
        Serial.print("  wallYawGain=");  Serial.print(wallYawGain, 1);
        Serial.print("  maxWallYaw=");   Serial.println(maxWallYaw, 1);
        if (TUNE_STAGE >= 3 && !gyroReady()) Serial.println("WARNING: no gyro - stage 3/4 will not steer");
        return;
    }

    if (AUTOTUNE_WALLS) runWallAutotune();
}

void loop() {
    if (TUNE_STAGE != 0) { stageRun(); return; }
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