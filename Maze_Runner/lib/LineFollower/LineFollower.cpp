#include "LineFollower.h"
#include <util/atomic.h>
#include "config.h"
#include "globals.h"
#include "MotorDriver.h"

#define NUM_SENSORS NUM_IR_SENSORS

static int whiteValues[NUM_SENSORS];
static int blackValues[NUM_SENSORS];
static int rawSensors[NUM_SENSORS];
static int normalizedSensors[NUM_SENSORS];
static bool blackFlags[NUM_SENSORS];
static uint8_t blackCount = 0;
static int rawMin = 0, rawMax = 0;   // spread of the 8 raw readings (contrast)

static int   lastLineError = 0;
static float lineIntegral  = 0;

// ================================================================
// SENSORS
// ================================================================
void lineSetCalibration(const int *white, const int *black) {
    for (int i = 0; i < NUM_SENSORS; i++) {
        whiteValues[i] = white[i];
        blackValues[i] = black[i];
    }
}

void lineBegin() {
    pinMode(PIN_IR_EN, OUTPUT);
    digitalWrite(PIN_IR_EN, HIGH);
    lineSetCalibration(WHITE_VALUES, BLACK_VALUES);
    lineReset();
}

void lineReadSensors() {
    blackCount = 0;
    rawMin = 1023;
    rawMax = 0;
    for (int i = 0; i < NUM_SENSORS; i++) {
        rawSensors[i] = analogRead(IR_PINS[i]);

        blackFlags[i] = rawSensors[i] > (whiteValues[i] + blackValues[i]) / 2;
        if (blackFlags[i]) blackCount++;
        if (rawSensors[i] < rawMin) rawMin = rawSensors[i];
        if (rawSensors[i] > rawMax) rawMax = rawSensors[i];

        int clamped = constrain(rawSensors[i], whiteValues[i], blackValues[i]);
        float ratio = (float)(clamped - whiteValues[i]) / (float)(blackValues[i] - whiteValues[i]);
        if (LINE_IS_WHITE) ratio = 1.0f - ratio;
        float curvedRatio = pow(ratio, LINE_SENSOR_EXPONENT);
        normalizedSensors[i] = (int)(curvedRatio * 1000.0);
    }
}

int     lineRaw(uint8_t i)     { return rawSensors[i]; }
int     lineNorm(uint8_t i)    { return normalizedSensors[i]; }
bool    lineIsBlack(uint8_t i) { return blackFlags[i]; }
uint8_t lineBlackCount()       { return blackCount; }
uint8_t lineWhiteCount()       { return NUM_SENSORS - blackCount; }
int     lineContrast()         { return rawMax - rawMin; }

// ================================================================
// PID FOLLOWER (ir_02 followLinePID)
// ================================================================
void lineReset() {
    lastLineError = 0;
    lineIntegral  = 0;
}

static bool lastLost = false;

int linePosition() {
    lineReadSensors();

    long weightedSum = 0;
    long totalValue  = 0;

    for (int i = 0; i < NUM_SENSORS; i++) {
        weightedSum += (long)normalizedSensors[i] * (i * 1000);
        totalValue += normalizedSensors[i];
    }

    lastLost = totalValue < LINE_LOST_SUM;
    if (lastLost) {
        return (lastLineError < 0) ? 0 : 7000;
    }

    return weightedSum / totalValue;
}

// ir_02 drove the motors with analogWrite (0-255). setMotors() takes duty % (0-100).
static int pwmToDuty(int pwm) {
    pwm = constrain(pwm, -LINE_MAX_PWM, LINE_MAX_PWM);
    long d = ((long)abs(pwm) * 100L + 127L) / 255L;
    return (pwm < 0) ? -(int)d : (int)d;
}

LineStep lineFollowStep(bool driveMotors) {
    LineStep s;
    int position = linePosition();

    // Setpoint is center (3500)
    int error = position - 3500;

    lineIntegral += error;
    lineIntegral = constrain(lineIntegral, -3000, 3000);

    int derivative = error - lastLineError;
    lastLineError  = error;

    float adjustment = (LINE_KP * error) + (LINE_KI * lineIntegral) + (LINE_KD * derivative);

    int leftMotorSpeed  = LINE_BASE_PWM - adjustment;
    int rightMotorSpeed = LINE_BASE_PWM + adjustment;

    if (driveMotors) setMotors(pwmToDuty(leftMotorSpeed), pwmToDuty(rightMotorSpeed));

    s.position = position;
    s.error    = error;
    s.leftPwm  = leftMotorSpeed;
    s.rightPwm = rightMotorSpeed;
    s.lost     = lastLost;
    return s;
}

// ================================================================
// ODOMETRY / STRAIGHT DRIVE
// ================================================================
static long odoL0 = 0, odoR0 = 0;

static void readEncoders(long &l, long &r) {
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        l = leftEncoderCount;
        r = rightEncoderCount;
    }
}

void lineOdoReset() { readEncoders(odoL0, odoR0); }

float lineOdoCm() {
    long l, r;
    readEncoders(l, r);
    return ((l - odoL0) + (r - odoR0)) / 2.0f / TICKS_PER_CM;
}

void lineDriveStraight(int duty) {
    long l, r;
    readEncoders(l, r);
    float steer = constrain(ENC_SYNC_KP * (float)((l - odoL0) - (r - odoR0)), -5.0f, 5.0f);
    int left  = (int)(duty - steer + 0.5f);
    int right = (int)((duty + steer) * RIGHT_DUTY_TRIM + 0.5f);
    setMotors(left, right);
}

// ================================================================
// MARKER CELL
// ================================================================
// Steps of the pattern, in the order they are driven over
enum MarkerStep { M_FLOOR, M_WHITE1, M_BLACK, M_WHITE2, M_FOUND, M_WAIT_BLACK };
static const char *const STEP_NAMES[] = {"floor", "white1", "black", "white2", "FOUND", "wait-black"};

bool markerVerbose = true;
static MarkerStep mStep    = M_FLOOR;
static float      mStartCm = 0.0f;   // where the current band started
static float      mLastCm  = 0.0f;   // last distance passed to markerUpdate
static bool       mFresh   = true;   // after a reset: the next update starts the floor band

void markerReset() { mStep = M_FLOOR; mFresh = true; }

const char *markerStepName() { return STEP_NAMES[mStep]; }

MarkerState markerState() {
    if (mStep == M_FOUND) return MARKER_FOUND;
    if (mStep == M_BLACK || mStep == M_WHITE2) return MARKER_STARTED;
    return MARKER_NONE;
}

static void markerGo(MarkerStep next, float cm, const char *why) {
    if (markerVerbose) {
        Serial.print("marker: ");
        Serial.print(STEP_NAMES[mStep]);
        Serial.print(' ');
        Serial.print(cm - mStartCm, 1);
        Serial.print(" cm -> ");
        Serial.print(STEP_NAMES[next]);
        if (why) { Serial.print(" ("); Serial.print(why); Serial.print(')'); }
        Serial.print("  @");                       // where it happened + what the array saw
        Serial.print(cm, 1);
        Serial.print(" cm ");
        for (int i = 0; i < NUM_SENSORS; i++) Serial.print(blackFlags[i] ? '#' : '.');
        Serial.println();
    }
    mStep    = next;
    mStartCm = cm;
}

// Bands are only judged when MARKER_FULL_SENSORS agree; mixed readings (an edge, or the
// robot at an angle) keep the current step.
MarkerState markerUpdate(float cm) {
    mLastCm = cm;
    if (mFresh) { mStartCm = cm; mFresh = false; }
    lineReadSensors();
    bool  white = lineWhiteCount() >= MARKER_FULL_SENSORS;
    bool  black = lineBlackCount() >= MARKER_FULL_SENSORS;
    float len   = cm - mStartCm;

    switch (mStep) {
        case M_WAIT_BLACK:   // a long white area (not a strip): wait for black floor again
            if (black) markerGo(M_FLOOR, cm, nullptr);
            break;
        case M_FLOOR:
            if (white) markerGo(M_WHITE1, cm, nullptr);
            break;
        case M_WHITE1:
            if (black) {
                if (len >= MARKER_W_MIN_CM && len <= MARKER_W_MAX_CM) markerGo(M_BLACK, cm, nullptr);
                else markerGo(M_FLOOR, cm, "strip too short");
            } else if (len > MARKER_W_MAX_CM) {
                markerGo(M_WAIT_BLACK, cm, "white too long");
            }
            break;
        case M_BLACK:
            if (white) {
                if (len >= MARKER_B_MIN_CM && len <= MARKER_B_MAX_CM) markerGo(M_WHITE2, cm, nullptr);
                else markerGo(M_WHITE1, cm, "black too short, new strip?");
            } else if (len > MARKER_B_MAX_CM) {
                markerGo(M_FLOOR, cm, "black too long = floor");
            }
            break;
        case M_WHITE2:
            if (len >= MARKER_W2_CONFIRM_CM) markerGo(M_FOUND, cm, nullptr);
            else if (black) markerGo(M_FLOOR, cm, "strip too short");
            break;
        case M_FOUND:
            break;
    }
    return markerState();
}

// ================================================================
// LINE MODE
// ================================================================
// A line under the array: some sensors on the line, the rest on the background.
static bool linePatternNow() {
    uint8_t onLine = LINE_IS_WHITE ? lineWhiteCount() : lineBlackCount();
    return onLine >= 1 && onLine <= 4;
}

const char *lineModeResultName(LineModeResult r) {
    switch (r) {
        case LINE_DONE_END:  return "line ended (black floor)";
        case LINE_DONE_LOST: return "line lost";
        case LINE_NOT_FOUND: return "no line found";
        case LINE_NO_MARKER: return "marker pattern not completed";
        default:             return "aborted";
    }
}

LineModeResult runLineMode(bool finishMarker, bool (*abortCheck)()) {
    LineModeResult result;

    // 0) A move may have stopped in the middle of the pattern: finish it first
    if (finishMarker && markerState() != MARKER_FOUND) {
        Serial.print("LINE MODE: finishing the marker pattern (at ");
        Serial.print(markerStepName());
        Serial.println(")");
        float base = mLastCm;
        lineOdoReset();
        while (true) {
            if (abortCheck && abortCheck()) { stopMotors(); return LINE_ABORTED; }
            float cm = lineOdoCm();
            MarkerState m = markerUpdate(base + cm);
            if (m == MARKER_FOUND) break;
            if (m == MARKER_NONE || cm > MARKER_FINISH_MAX_CM) {
                stopMotors();
                Serial.println("LINE MODE: not the special cell, back to the maze");
                return LINE_NO_MARKER;
            }
            lineDriveStraight(LINE_SEARCH_DUTY);
            delay(5);
        }
    }

    // 1) Drive straight (over the rest of the white strip and the small black part)
    //    until the line pattern has lasted LINE_FOUND_CM
    Serial.println("LINE MODE: searching for the line");
    lineOdoReset();
    float patternStart = -1.0f;
    while (true) {
        if (abortCheck && abortCheck()) { stopMotors(); return LINE_ABORTED; }
        float cm = lineOdoCm();
        if (cm > LINE_SEARCH_MAX_CM) { stopMotors(); Serial.println("LINE MODE: no line found"); return LINE_NOT_FOUND; }

        lineReadSensors();
        if (linePatternNow()) {
            if (patternStart < 0) patternStart = cm;
            if (cm - patternStart >= LINE_FOUND_CM) break;
        } else {
            patternStart = -1.0f;
        }
        lineDriveStraight(LINE_SEARCH_DUTY);
        delay(5);
    }
    Serial.print("LINE MODE: line found after ");
    Serial.print(lineOdoCm(), 1);
    Serial.println(" cm, following");

    // 2) Follow it until the black floor returns, or the line is lost
    lineReset();
    lineOdoReset();
    float endStart = -1.0f;
    unsigned long lostSince = 0;
    unsigned long lastPrint = 0;
    while (true) {
        if (abortCheck && abortCheck()) { result = LINE_ABORTED; break; }

        LineStep s = lineFollowStep(true);
        float cm = lineOdoCm();

        // Black floor = (almost) all sensors black AND even readings. The line section's
        // background can read grey (870-940, 'black' by the midpoint) but the line on it
        // still stands out (~980-1000), so the contrast stays high there.
        // Ignored for the first LINE_MIN_FOLLOW_CM: on the ramp at the start of the line section
        // the IR array is lifted off the surface and every sensor reads dark and even.
        if (cm >= LINE_MIN_FOLLOW_CM &&
            lineBlackCount() >= LINE_END_SENSORS && lineContrast() < LINE_END_MAX_CONTRAST) {
            if (endStart < 0) endStart = cm;
            if (cm - endStart >= LINE_END_CM) { result = LINE_DONE_END; break; }
        } else {
            endStart = -1.0f;
        }

        if (s.lost) {
            if (lostSince == 0) lostSince = millis();
            if (millis() - lostSince > LINE_LOST_MS) { result = LINE_DONE_LOST; break; }
        } else {
            lostSince = 0;
        }

        if (millis() - lastPrint >= 50) {
            lastPrint = millis();
            Serial.print("line cm="); Serial.print(cm, 1);
            Serial.print(" pos=");    Serial.print(s.position);
            Serial.print(" L=");      Serial.print(s.leftPwm);
            Serial.print(" R=");      Serial.print(s.rightPwm);
            Serial.print(" black=");  Serial.print(lineBlackCount());
            Serial.print(" c=");      Serial.print(lineContrast());
            Serial.print(' ');
            for (int i = 0; i < NUM_SENSORS; i++) Serial.print(blackFlags[i] ? '#' : '.');
            Serial.println();
        }
        delay(5);
    }
    stopMotors();
    Serial.print("LINE MODE: ");
    Serial.print(lineModeResultName(result));
    Serial.print(" after ");
    Serial.print(lineOdoCm(), 1);
    Serial.println(" cm of following");
    return result;
}
