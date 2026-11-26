#ifndef LINE_FOLLOWER_H
#define LINE_FOLLOWER_H

#include <Arduino.h>

// IR line follower (algorithm and tuning from Arduino_Test/irarray_test/ir_02.cpp),
// plus marker-cell detection and a full "line mode" for the maze.
// Pins, calibration and gains are in include/config.h (IR_PINS, LINE_*, MARKER_*).
// Sensor 0 = robot's right most, 7 = left most. Position 0..7000, 3500 = centred.
// Motors go through setMotors() (duty %), so encoders/gyro/maze code are unaffected.

struct LineStep {
    int  position;   // 0..7000 (0 or 7000 when the line is lost)
    int  error;      // position - 3500
    int  leftPwm;    // 0-255 scale, as in ir_02 (before conversion to duty %)
    int  rightPwm;
    bool lost;       // total normalised value below LINE_LOST_SUM
};

enum LineModeResult {
    LINE_DONE_END,     // line section finished: back on the black maze floor
    LINE_DONE_LOST,    // line lost for LINE_LOST_MS
    LINE_NOT_FOUND,    // no line within LINE_SEARCH_MAX_CM
    LINE_NO_MARKER,    // the marker pattern did not complete (it was not the special cell)
    LINE_ABORTED       // abort callback returned true
};

enum MarkerState {
    MARKER_NONE,       // nothing yet
    MARKER_STARTED,    // white strip + start of the black square seen: may be the special cell
    MARKER_FOUND       // white - black - white pattern complete
};

// ---- Sensors ----
void    lineBegin();                                   // IR enable pin on, load config.h calibration
void    lineSetCalibration(const int *white, const int *black);
void    lineReadSensors();                             // read all 8 (raw, normalised, black/white)
int     lineRaw(uint8_t i);
int     lineNorm(uint8_t i);                           // 0 = background, 1000 = line (curved)
bool    lineIsBlack(uint8_t i);                        // raw above the white/black midpoint
uint8_t lineBlackCount();
uint8_t lineWhiteCount();

// ---- PID follower (same maths as ir_02 followLinePID) ----
void     lineReset();                                  // clear PID memory before following
int      linePosition();                               // reads the sensors
LineStep lineFollowStep(bool driveMotors = true);      // one PID step; call every ~5 ms

// ---- Odometry / straight drive helpers (encoders from MotorDriver) ----
void  lineOdoReset();                                  // remember the current encoder counts
float lineOdoCm();                                     // average distance since lineOdoReset()
void  lineDriveStraight(int duty);                     // one step at duty %, keeps L/R ticks equal

// ---- Marker cell: white strip / black square / white strip on the black maze floor ----
// travelledCm must keep increasing across moves (do not restart it at each cell).
void        markerReset();
MarkerState markerUpdate(float travelledCm);           // reads sensors, advances the pattern
MarkerState markerState();
const char *markerStepName();                          // current step: floor, white1, black, white2...
extern bool markerVerbose;                             // true = print each band with its length

// ---- Full line mode ----
// finishMarker = true: if the marker is only STARTED, first drive straight until it is FOUND
// (gives up with LINE_NO_MARKER). Then drive straight to the line, follow it to its end, stop.
// abortCheck (optional) is polled every step; return true to stop (e.g. key press).
LineModeResult runLineMode(bool finishMarker, bool (*abortCheck)() = nullptr);
const char    *lineModeResultName(LineModeResult r);

#endif // LINE_FOLLOWER_H
