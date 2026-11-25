// Ultrasonic calibration: prints the average distance of each sensor (cm).
// Compare each reading with a tape measure to find the offset.
// 100.0 means no echo (nothing in range).

#include <Arduino.h>
#include "config.h"
#include "Ultrasonic.h"

// Ultrasonic.cpp uses these globals
float frontDist = 0.0f;
float leftDist  = 0.0f;
float rightDist = 0.0f;

float averageDistance(int trig, int echo) {
    float sum = 0.0f;
    for (int i = 0; i < 10; i++) {
        sum += readUltrasonic(trig, echo);
        delay(30);
    }
    return sum / 10.0f;
}

void setup() {
    Serial.begin(115200);
    delay(500);

    pinMode(PIN_ULTRA_FRONT_TRIG, OUTPUT);
    pinMode(PIN_ULTRA_FRONT_ECHO, INPUT);
    pinMode(PIN_ULTRA_LEFT_TRIG, OUTPUT);
    pinMode(PIN_ULTRA_LEFT_ECHO, INPUT);
    pinMode(PIN_ULTRA_RIGHT_TRIG, OUTPUT);
    pinMode(PIN_ULTRA_RIGHT_ECHO, INPUT);

    Serial.println("Ultrasonic calibration started.");
}

void loop() {
    float f = averageDistance(PIN_ULTRA_FRONT_TRIG, PIN_ULTRA_FRONT_ECHO);
    float l = averageDistance(PIN_ULTRA_LEFT_TRIG, PIN_ULTRA_LEFT_ECHO);
    float r = averageDistance(PIN_ULTRA_RIGHT_TRIG, PIN_ULTRA_RIGHT_ECHO);

    Serial.print("F: ");
    Serial.print(f, 1);
    Serial.print(" cm   L: ");
    Serial.print(l, 1);
    Serial.print(" cm   R: ");
    Serial.print(r, 1);
    Serial.println(" cm");

    delay(300);
}
