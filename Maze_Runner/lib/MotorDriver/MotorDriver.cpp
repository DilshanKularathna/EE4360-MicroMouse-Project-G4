#include <Arduino.h>
#include <util/atomic.h>
#include "config.h"
#include "MotorDriver.h"
#include "Ultrasonic.h"


// // Global pulse counters required for ISR access
volatile long leftEncoderCount  = 0;
volatile long rightEncoderCount = 0;
float frontDistance = 0.0f;
float leftDistance = 0.0f;
float rightDistance = 0.0f;
bool isMovingForward = false;
bool isTurning = false;
float lastError = 0.0f;
float integralError = 0.0f;
int currentHeading = NORTH;

// ================================================================
// PROTOTYPES
// ================================================================
void turnDegrees(float degrees);
void resetEncoders();
void setMotors(int leftSpeed, int rightSpeed);
void stopMotors();
void leftEncoderISR();
void rightEncoderISR();
void setPWM(uint8_t pin, uint32_t freq, uint8_t dutyCycle);

void turnLeft() {
    turnDegrees(-90.0f);
    currentHeading = (currentHeading + 3) % 4;
}

void turnRight() {
    turnDegrees(90.0f);
    currentHeading = (currentHeading + 1) % 4;
}

void turnAround() {
    turnDegrees(180.0f);
    currentHeading = (currentHeading + 2) % 4;
}

void turnDegrees(float degrees) { // + value turns right, - value turns left
    resetEncoders();

    const float turnMagnitude = fabs(degrees);
    const float turnRatio = turnMagnitude / 90.0f;
    float targetCountsFloat;
    if (turnRatio <= 1.0f) {
        targetCountsFloat = TICKS_PER_90_DEG * turnRatio;
    } else {
        // Interpolate between the measured 90-degree and 180-degree values.
        targetCountsFloat = TICKS_PER_90_DEG +
            (TICKS_PER_180_DEG - TICKS_PER_90_DEG) * (turnRatio - 1.0f);
    }
    const long targetCounts = (long)(targetCountsFloat + 0.5f);

    long lCount = 0;
    long rCount = 0;

    while (abs(lCount) < targetCounts || abs(rCount) < targetCounts) {
        ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
            lCount = leftEncoderCount;
            rCount = rightEncoderCount;
        }

        if (degrees > 0) {
            setPWM(PIN_LEFT_RPWM, 20000, TURN_SPEED);
            setPWM(PIN_LEFT_LPWM, 20000, 0);
            setPWM(PIN_RIGHT_RPWM, 20000, 0);
            setPWM(PIN_RIGHT_LPWM, 20000, TURN_SPEED);
        } else {
            setPWM(PIN_LEFT_RPWM, 20000, 0);
            setPWM(PIN_LEFT_LPWM, 20000, TURN_SPEED);
            setPWM(PIN_RIGHT_RPWM, 20000, TURN_SPEED);
            setPWM(PIN_RIGHT_LPWM, 20000, 0);
        }
    }
    stopMotors();
    delay(500);
}

void setMotors(int leftSpeed, int rightSpeed) { // Set + value for forward, - value for backward
    if (leftSpeed > 0) {
        setPWM(PIN_LEFT_RPWM, 20000, leftSpeed);
        setPWM(PIN_LEFT_LPWM, 20000, 0);
    } else {
        setPWM(PIN_LEFT_RPWM, 20000, 0);
        setPWM(PIN_LEFT_LPWM, 20000, abs(leftSpeed));
    }

    if (rightSpeed > 0) {
        setPWM(PIN_RIGHT_RPWM, 20000, rightSpeed);
        setPWM(PIN_RIGHT_LPWM, 20000, 0);
    } else {
        setPWM(PIN_RIGHT_RPWM, 20000, 0);
        setPWM(PIN_RIGHT_LPWM, 20000, abs(rightSpeed));
    }
}


void moveDistanceCM(float distanceCM, int speed) {
    if (isMovingForward || isTurning)
        return;
    isMovingForward = true;

    integralError = 0.0f;
    lastError     = 0.0f;

    long targetCounts = (long)(distanceCM / CM_PER_COUNT);

    Serial.print("Moving forward ");
    Serial.print(distanceCM);
    Serial.print("cm - Target counts: ");
    Serial.println(targetCounts);

    resetEncoders();

    while (abs(leftEncoderCount) < targetCounts || abs(rightEncoderCount) < targetCounts) {
        float currentFront = readUltrasonic(PIN_ULTRA_FRONT_TRIG, PIN_ULTRA_FRONT_ECHO);
        if (currentFront < FRONT_OBSTACLE) {
            Serial.println("Front obstacle detected during forward move! Stopping.");
            stopMotors();
            isMovingForward = false;
            return;
        }

        setMotors(speed, speed);
        delay(10);
    }

    stopMotors();
    delay(200);

    Serial.print("Forward completed - L: ");
    Serial.print(leftEncoderCount);
    Serial.print(" R: ");
    Serial.println(rightEncoderCount);

    isMovingForward = false;
}

void resetEncoders() {
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        leftEncoderCount  = 0;
        rightEncoderCount = 0;
    }
}

void stopMotors() {
    setPWM(PIN_LEFT_RPWM, 20000, 0);
    setPWM(PIN_LEFT_LPWM, 20000, 0);
    setPWM(PIN_RIGHT_RPWM, 20000, 0);
    setPWM(PIN_RIGHT_LPWM, 20000, 0);
}

void leftEncoderISR() {
    if (digitalRead(PIN_LEFT_ENC_B) == HIGH)
        leftEncoderCount--;
    else
        leftEncoderCount++;
}

void rightEncoderISR() {
    if (digitalRead(PIN_RIGHT_ENC_B) == HIGH)
        rightEncoderCount++;
    else
        rightEncoderCount--;
}

// Dont change, Bare Metal for PWM Generation
void setPWM(uint8_t pin, uint32_t freq, uint8_t dutyCycle) {
    if (freq == 0) return;
    if (dutyCycle > 100) dutyCycle = 100;

    static uint32_t timer3Freq = 0;
    static uint32_t timer4Freq = 0;
    static uint16_t timer3Top  = 0;
    static uint16_t timer4Top  = 0;

    // --- TIMER 3 (Pin 5 - OCR3A / PE3) ---
    if (pin == 5) {
        // Reconfigure if frequency changed or not initialized
        if (timer3Freq != freq) {
            uint8_t prescalerBits = (freq > 30000) ? 1 : (freq > 4000) ? 2 : 3;
            uint16_t prescaler    = (prescalerBits == 1) ? 1 : (prescalerBits == 2) ? 8 : 64;
            uint32_t top          = (F_CPU / (prescaler * freq)) - 1;
            if (top > 65535) top  = 65535;

            timer3Top  = (uint16_t)top;
            timer3Freq = freq;

            TCCR3A = (1 << WGM31);                          // Fast PWM, Mode 14
            TCCR3B = (1 << WGM33) | (1 << WGM32) | prescalerBits;
            ICR3   = timer3Top;
        }

        if (dutyCycle == 0) {
            TCCR3A &= ~(1 << COM3A1); // Disconnect Timer from Pin (True 0V OFF)
            PORTE  &= ~(1 << PE3);    // Force Pin LOW
        } else {
            OCR3A   = (uint32_t)timer3Top * dutyCycle / 100;
            TCCR3A |= (1 << COM3A1);  // Connect Timer to Pin (Non-inverting)
        }
    }

    // --- TIMER 4 (Pins 6, 7, 8 - OCR4A, OCR4B, OCR4C) ---
    else if (pin == 6 || pin == 7 || pin == 8) {
        // Reconfigure if frequency changed or not initialized
        if (timer4Freq != freq) {
            uint8_t prescalerBits = (freq > 30000) ? 1 : (freq > 4000) ? 2 : 3;
            uint16_t prescaler    = (prescalerBits == 1) ? 1 : (prescalerBits == 2) ? 8 : 64;
            uint32_t top          = (F_CPU / (prescaler * freq)) - 1;
            if (top > 65535) top  = 65535;

            timer4Top  = (uint16_t)top;
            timer4Freq = freq;

            TCCR4A = (1 << WGM41);                          // Fast PWM, Mode 14
            TCCR4B = (1 << WGM43) | (1 << WGM42) | prescalerBits;
            ICR4   = timer4Top;
        }

        uint16_t ocr = (uint32_t)timer4Top * dutyCycle / 100;

        if (pin == 6) {
            if (dutyCycle == 0) {
                TCCR4A &= ~(1 << COM4A1);
                PORTH  &= ~(1 << PH3);
            } else {
                OCR4A   = ocr;
                TCCR4A |= (1 << COM4A1);
            }
        } else if (pin == 7) {
            if (dutyCycle == 0) {
                TCCR4A &= ~(1 << COM4B1);
                PORTH  &= ~(1 << PH4);
            } else {
                OCR4B   = ocr;
                TCCR4A |= (1 << COM4B1);
            }
        } else if (pin == 8) {
            if (dutyCycle == 0) {
                TCCR4A &= ~(1 << COM4C1);
                PORTH  &= ~(1 << PH5);
            } else {
                OCR4C   = ocr;
                TCCR4A |= (1 << COM4C1);
            }
        }
    }
}
