#include "IRArray.h"

IRArray::IRArray(const uint8_t* pins, uint8_t enPin, uint8_t count)
    : sensorPins(pins), enablePin(enPin), numSensors(count) {
    for (uint8_t i = 0; i < numSensors; i++) {
        thresholds[i] = 500; // Default fallback
        minValues[i] = 1023;
        maxValues[i] = 0;
    }
}

void IRArray::begin() {
    pinMode(enablePin, OUTPUT);
    enable();

    for (uint8_t i = 0; i < numSensors; i++) {
        pinMode(sensorPins[i], INPUT);
    }
}

void IRArray::enable() {
    digitalWrite(enablePin, HIGH);
}

void IRArray::disable() {
    digitalWrite(enablePin, LOW);
}

void IRArray::readRaw(int* outputArray) {
    for (uint8_t i = 0; i < numSensors; i++) {
        rawValues[i] = analogRead(sensorPins[i]);
        if (outputArray != nullptr) {
            outputArray[i] = rawValues[i];
        }
    }
}

void IRArray::readBinary(bool* outputArray) {
    readRaw(nullptr);
    for (uint8_t i = 0; i < numSensors; i++) {
        outputArray[i] = (rawValues[i] > thresholds[i]);
    }
}

int IRArray::getLinePosition() {
    readRaw(nullptr);
    long weightedSum = 0;
    long sum = 0;

    for (uint8_t i = 0; i < numSensors; i++) {
        int value = rawValues[i];
        weightedSum += (long)value * (i * 1000);
        sum += value;
    }

    if (sum == 0) return 0;
    
    // Center offset calculation
    return (int)(weightedSum / sum) - ((numSensors - 1) * 500);
}

void IRArray::setThresholds(const int* newThresholds) {
    for (uint8_t i = 0; i < numSensors; i++) {
        thresholds[i] = newThresholds[i];
    }
}

void IRArray::setCalibrationRanges(const int* minVals, const int* maxVals) {
    for (uint8_t i = 0; i < numSensors; i++) {
        minValues[i] = minVals[i];
        maxValues[i] = maxVals[i];
        thresholds[i] = (minVals[i] + maxVals[i]) / 2;
    }
}