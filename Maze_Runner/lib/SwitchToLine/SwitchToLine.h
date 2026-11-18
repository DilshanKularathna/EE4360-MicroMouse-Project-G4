#ifndef SWITCH_TO_LINE_H
#define SWITCH_TO_LINE_H

#include <Arduino.h>

class SwitchToLine {
public:
    void begin();
    bool observeMarker();
    void update();

private:
    enum State : uint8_t {
        SEARCHING_FOR_MARKER,
        CREEPING_TO_LINE,
        FOLLOWING_LINE,
        BRIDGE_FAILED
    };

    State state = SEARCHING_FOR_MARKER;
    uint8_t markerStage = 0;
    uint8_t stableSamples = 0;
    unsigned long creepStartMs = 0;
    float integral = 0.0f;
    int previousError = 0;
    bool failureReported = false;

    int readSensors(int *rawValues) const;
    bool isLinePresent(int blackCount, int whiteCount) const;
    void followLine(const int *rawValues);
};

#endif