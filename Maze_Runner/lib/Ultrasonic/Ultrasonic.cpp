#include "Ultrasonic.h"
#include "config.h"

// Ultrasonics::Ultrasonics(uint8_t trig, uint8_t echo)
//     : trigPin(trig), echoPin(echo) {}

// void Ultrasonics::begin() {
//     pinMode(trigPin, OUTPUT);
//     pinMode(echoPin, INPUT);
//     digitalWrite(trigPin, LOW);
// }

// float Ultrasonics::getDistanceCM() {
//     // 1. Clear Trig pin
//     digitalWrite(trigPin, LOW);
//     delayMicroseconds(2);

//     // 2. Pulse Trig HIGH for 10 microseconds
//     digitalWrite(trigPin, HIGH);
//     delayMicroseconds(10);
//     digitalWrite(trigPin, LOW);

//     // 3. Read Echo pulse with a 30ms timeout (~5m max range)
//     long duration = pulseIn(echoPin, HIGH, 30000);

//     // 4. Calculate distance: speed of sound is 0.034 cm/us (divided by 2 for round-trip)
//     if (duration == 0) {
//         return -1.0f; // Timeout / Object out of range
//     }

//     return (float)(duration * 0.034f / 2.0f);
// }

bool isPathClearFront() {
    return frontDist > FRONT_OBSTACLE;
}

bool isWallOnRight() {
    return rightDist <= WALL_THRESHOLD;
}

bool isWallOnLeft() {
    return leftDist <= WALL_THRESHOLD;
}

float readUltrasonic(int trigPin, int echoPin) {
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);
    long duration = pulseIn(echoPin, HIGH, 9000);
    if (duration == 0)
        return 100.0f; // Path is clear
    return (duration * 0.0343f) / 2.0f;
}

void readAllSensors() {
    frontDist = readUltrasonic(PIN_ULTRA_FRONT_TRIG, PIN_ULTRA_FRONT_ECHO);
    delay(10);
    rightDist = readUltrasonic(PIN_ULTRA_RIGHT_TRIG, PIN_ULTRA_RIGHT_ECHO);
    delay(10);
    leftDist = readUltrasonic(PIN_ULTRA_LEFT_TRIG, PIN_ULTRA_LEFT_ECHO);
    delay(10);
}