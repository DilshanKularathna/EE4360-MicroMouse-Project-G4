#include "Ultrasonic.h"

Ultrasonics::Ultrasonics(uint8_t trig, uint8_t echo)
    : trigPin(trig), echoPin(echo) {}

void Ultrasonics::begin() {
    pinMode(trigPin, OUTPUT);
    pinMode(echoPin, INPUT);
    digitalWrite(trigPin, LOW);
}

float Ultrasonics::getDistanceCM() {
    // 1. Clear Trig pin
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);

    // 2. Pulse Trig HIGH for 10 microseconds
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);

    // 3. Read Echo pulse with a 30ms timeout (~5m max range)
    long duration = pulseIn(echoPin, HIGH, 30000);

    // 4. Calculate distance: speed of sound is 0.034 cm/us (divided by 2 for round-trip)
    if (duration == 0) {
        return -1.0f; // Timeout / Object out of range
    }

    return (float)(duration * 0.034f / 2.0f);
}