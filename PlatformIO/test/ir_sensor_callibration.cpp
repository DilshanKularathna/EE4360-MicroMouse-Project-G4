#include <Arduino.h>
#include "config.h"

int sensorValues[NUM_IR_SENSORS];
int sensorMin[NUM_IR_SENSORS];
int sensorMax[NUM_IR_SENSORS];
int sensorThreshold[NUM_IR_SENSORS];

void readIRSensors() {
    for (int i = 0; i < NUM_IR_SENSORS; i++) {
        sensorValues[i] = analogRead(IR_PINS[i]);
    }
}

void waitForUserPrompt() {
    while (Serial.available() > 0) Serial.read();
    while (Serial.available() == 0) delay(10);
    while (Serial.available() > 0) Serial.read();
}

void calibrateSensors() {
    Serial.println("========================================");
    Serial.println("    IR SENSOR CALIBRATION UTILITY       ");
    Serial.println("========================================");

    for (int i = 0; i < NUM_IR_SENSORS; i++) {
        sensorMin[i] = 1023;
        sensorMax[i] = 0;
    }

    // Phase 1: White Calibration
    Serial.println("PHASE 1: WHITE SURFACE CALIBRATION");
    Serial.println("Place ALL sensors on WHITE surface and press ENTER.");
    waitForUserPrompt();

    unsigned long startTime = millis();
    while (millis() - startTime < 5000) {
        readIRSensors();
        for (int i = 0; i < NUM_IR_SENSORS; i++) {
            if (sensorValues[i] < sensorMin[i]) sensorMin[i] = sensorValues[i];
        }
        delay(10);
    }

    // Phase 2: Black Calibration
    Serial.println("PHASE 2: BLACK SURFACE CALIBRATION");
    Serial.println("Place/sweep sensors over BLACK line and press ENTER.");
    waitForUserPrompt();

    startTime = millis();
    while (millis() - startTime < 5000) {
        readIRSensors();
        for (int i = 0; i < NUM_IR_SENSORS; i++) {
            if (sensorValues[i] > sensorMax[i]) sensorMax[i] = sensorValues[i];
        }
        delay(10);
    }

    // Compute thresholds
    for (int i = 0; i < NUM_IR_SENSORS; i++) {
        sensorThreshold[i] = (sensorMin[i] + sensorMax[i]) / 2;
    }

    // Output formatted arrays for config.h
    Serial.println("\n===== COPY-PASTE TO CONFIG.H =====");
    Serial.print("const int DEFAULT_IR_THRESHOLDS[8] = {");
    for (int i = 0; i < NUM_IR_SENSORS; i++) {
        Serial.print(sensorThreshold[i]);
        if (i < NUM_IR_SENSORS - 1) Serial.print(", ");
    }
    Serial.println("};");
}

void setup() {
    Serial.begin(115200);
    pinMode(PIN_IR_EN, OUTPUT);
    digitalWrite(PIN_IR_EN, HIGH);

    calibrateSensors();
}

void loop() {
    readIRSensors();
    Serial.print("Raw Values: ");
    for (int i = 0; i < NUM_IR_SENSORS; i++) {
        Serial.print(sensorValues[i]);
        Serial.print("\t");
    }
    Serial.println();
    delay(200);
}