#include <Arduino.h>
#include "config.h"
// WayFinder logic and values

// Map to your IR sensors from config.h (Assuming IR7 is Left-most, IR0 is Right-most)
const uint8_t IR_PINS[8] = {IR7, IR6, IR5, IR4, IR3, IR2, IR1, IR0};

const int THRESHOLDS[8] = {818, 814, 804, 796, 807, 808, 801, 837};

// If Black Line gives high analog reading (your blackValues are ~920, whiteValues ~700)
const bool SENSOR_ACTIVE_IS_HIGH = true;

// Previous Team's Base Values
#define BASE_SPEED 80
#define MAX_CORRECTION 150
#define TURN_SPEED 80
#define TURN_TIMEOUT 3000
#define CENTER_SENSOR_DETECTION 3

// Previous Team's PID Values
float Kp = 12.0f;
float Ki = 12.0f;
float Kd = 3.0f;
#define INTEGRAL_MAX 200.0f
#define INTEGRAL_MIN -200.0f

const int WEIGHTS[8] = {7, 5, 3, 1, -1, -3, -5, -7}; // 7(Leftmost) to -7(Rightmost)

#define WHITE_COUNT_THRESHOLD 6
#define FORWARD_AFTER_TURN 150

#define DEAD_ZONE 0.5f
#define SMOOTHING_FACTOR 0.35f

float         lastError        = 0.0f;
float         integralError    = 0.0f;
unsigned long lastTime         = 0;
float         lastPosition     = 0.0f;
bool          lastDetectedLeft = false;

bool prevBinary[8]     = {0};
int  whiteSensorCount  = 0;
bool lTurnDetected     = false;
bool turnDirectionLeft = false;

// Local motor control using analogWrite (0-255 scale) mapped to your RPWM/LPWM pins
void setMotors(int leftSpeed, int rightSpeed) {
    if (leftSpeed >= 0) {
        analogWrite(PIN_LEFT_RPWM, constrain(leftSpeed, 0, 255));
        analogWrite(PIN_LEFT_LPWM, 0);
    } else {
        analogWrite(PIN_LEFT_RPWM, 0);
        analogWrite(PIN_LEFT_LPWM, constrain(-leftSpeed, 0, 255));
    }

    if (rightSpeed >= 0) {
        analogWrite(PIN_RIGHT_RPWM, constrain(rightSpeed, 0, 255));
        analogWrite(PIN_RIGHT_LPWM, 0);
    } else {
        analogWrite(PIN_RIGHT_RPWM, 0);
        analogWrite(PIN_RIGHT_LPWM, constrain(-rightSpeed, 0, 255));
    }
}

void stopMotors() {
    setMotors(0, 0);
}

float readLinePosition(bool& detected, bool binaryOut[8]) {
    int  activeCount = 0;
    long weightedSum = 0;
    whiteSensorCount = 0;

    for (uint8_t i = 0; i < 8; i++) {
        int  raw     = analogRead(IR_PINS[i]);
        bool active  = SENSOR_ACTIVE_IS_HIGH ? (raw >= THRESHOLDS[i]) : (raw <= THRESHOLDS[i]);
        binaryOut[i] = active;

        if (active) {
            activeCount++;
            weightedSum += WEIGHTS[i];
        } else {
            whiteSensorCount++;
        }
    }

    if (activeCount > 0) {
        memcpy(prevBinary, binaryOut, sizeof(prevBinary));
    }

    if (activeCount == 0) {
        detected = false;
        return lastPosition;
    } else {
        detected  = true;
        float pos = (float)weightedSum / (float)activeCount;

        pos              = SMOOTHING_FACTOR * lastPosition + (1.0f - SMOOTHING_FACTOR) * pos;
        lastPosition     = pos;
        lastDetectedLeft = (pos < 0.0f);
        return pos;
    }
}

float computePID(float position, float dt) {
    if (fabs(position) < DEAD_ZONE) {
        integralError *= 0.95f;
        lastError = 0.0f;
        return 0.0f;
    }

    float error = 0.0f - position;

    if (fabs(error) > 1.0f) {
        integralError += error * dt;
    } else {
        integralError *= 0.98f;
    }
    integralError = constrain(integralError, INTEGRAL_MIN, INTEGRAL_MAX);

    float derivative = 0.0f;
    if (dt > 0.001f) {
        derivative = (error - lastError) / dt;
    }

    float dynamicKp = Kp;
    float dynamicKd = Kd;

    if (fabs(error) < 2.0f) {
        dynamicKp = Kp * 0.8f;
        dynamicKd = Kd * 1.5f;
    } else {
        dynamicKp = Kp * 1.2f;
        dynamicKd = Kd * 0.8f;
    }

    float correction = (dynamicKp * error) + (Ki * integralError) + (dynamicKd * derivative);
    correction       = constrain(correction, -MAX_CORRECTION, MAX_CORRECTION);

    lastError = error;
    return correction;
}

bool detectLTurn(bool currentBinary[8]) {
    if (whiteSensorCount >= WHITE_COUNT_THRESHOLD) {
        Serial.println("⚠  L-TURN DETECTED: Most sensors see white");
        return true;
    }
    return false;
}

bool determineTurnDirection(bool currentBinary[8]) {
    bool leftWasActive  = (prevBinary[0] || prevBinary[1] || prevBinary[2]);
    bool rightWasActive = (prevBinary[5] || prevBinary[6] || prevBinary[7]);

    Serial.print("Turn Decision - Left was active: ");
    Serial.print(leftWasActive);
    Serial.print(" | Right was active: ");
    Serial.println(rightWasActive);

    if (leftWasActive && rightWasActive) {
        return !lastDetectedLeft;
    }

    return leftWasActive;
}

void performLTurn(bool turnLeft) {
    Serial.println(turnLeft ? "🔄 Performing 90° LEFT L-turn..."
                            : "🔄 Performing 90° RIGHT L-turn...");

    stopMotors();
    delay(100);

    unsigned long turnStart     = millis();
    bool          turnCompleted = false;

    while (!turnCompleted && (millis() - turnStart < TURN_TIMEOUT)) {
        if (turnLeft) {
            setMotors(-TURN_SPEED, TURN_SPEED);
        } else {
            setMotors(TURN_SPEED, -TURN_SPEED);
        }

        bool detected;
        bool binary[8];
        readLinePosition(detected, binary);

        int centerSensorsActive = 0;
        for (int i = 2; i <= 5; i++) {
            if (binary[i])
                centerSensorsActive++;
        }

        if (centerSensorsActive >= CENTER_SENSOR_DETECTION) {
            turnCompleted = true;
            Serial.println("✅ 90° turn completed - line detected!");
        }

        delay(50);
    }

    stopMotors();
    delay(50);

    turnStart = millis();
    while (millis() - turnStart < FORWARD_AFTER_TURN) {
        setMotors(60, 60);

        bool detected;
        bool binary[8];
        readLinePosition(detected, binary);
        int centerActive = 0;
        for (int i = 3; i <= 4; i++) {
            if (binary[i])
                centerActive++;
        }
        if (centerActive >= 2)
            break;

        delay(10);
    }

    stopMotors();
    delay(100);

    integralError = 0;
    lastError     = 0;
    lastPosition  = 0.0f;
    lTurnDetected = false;

    Serial.println("✅ L-turn complete! Resuming line following...");
}

}  // namespace

void setup() {
    Serial.begin(115200);
    // PinModes required since we bypass MotorDriver.cpp initialization
    pinMode(PIN_LEFT_RPWM, OUTPUT);
    pinMode(PIN_LEFT_LPWM, OUTPUT);
    pinMode(PIN_RIGHT_RPWM, OUTPUT);
    pinMode(PIN_RIGHT_LPWM, OUTPUT);
    
    // Enable IR array if needed by your hardware
    pinMode(IR_EN, OUTPUT);
    digitalWrite(IR_EN, HIGH); 

    stopMotors();
    lastTime = millis();
    Serial.println("OPTIMIZED Line Follower - Initialized");
}

void loop() {
    unsigned long now = millis();
    float         dt  = (now - lastTime) / 1000.0f;
    if (dt <= 0.0f)
        dt = 0.001f;

    bool  detected  = false;
    bool  binary[8] = {0};
    float position  = readLinePosition(detected, binary);

    if (!lTurnDetected && detectLTurn(binary)) {
        lTurnDetected     = true;
        turnDirectionLeft = determineTurnDirection(binary);
        performLTurn(turnDirectionLeft);
        lastTime = millis();
        return;
    }

    float correction = 0.0f;
    if (detected) {
        correction    = computePID(position, dt);
        lTurnDetected = false;
    } else {
        correction    = lastDetectedLeft ? 100.0f : -100.0f;
        integralError = 0;
    }

    int leftPWM  = (int)constrain(BASE_SPEED + correction, -255, 255);
    int rightPWM = (int)constrain(BASE_SPEED - correction, -255, 255);

    setMotors(leftPWM, rightPWM);

    static unsigned long lastPrint = 0;
    if (now - lastPrint > 200) {
        Serial.print("Pos:");
        Serial.print(position, 2);
        Serial.print(" | Corr:");
        Serial.print(correction, 1);
        Serial.print(" | L:");
        Serial.print(leftPWM);
        Serial.print(" R:");
        Serial.println(rightPWM);
        lastPrint = now;
    }

    lastTime = now;
}
