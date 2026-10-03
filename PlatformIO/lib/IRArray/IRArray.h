#ifndef IR_ARRAY_H
#define IR_ARRAY_H

#include <Arduino.h>

class IRArray {
private:
    const uint8_t* sensorPins;
    uint8_t enablePin;
    uint8_t numSensors;
    int rawValues[8];
    int minValues[8];
    int maxValues[8];
    int thresholds[8];

public:
    IRArray(const uint8_t* pins, uint8_t enPin, uint8_t count);

    void begin();
    void enable();
    void disable();
    
    void readRaw(int* outputArray);
    void readBinary(bool* outputArray);
    
    // Line position calculation (-3500 to +3500) for PID line following
    int getLinePosition();

    // Calibration helpers
    void setThresholds(const int* newThresholds);
    void setCalibrationRanges(const int* minVals, const int* maxVals);
};

#endif // IR_ARRAY_H