#include <Arduino.h>
#include <util/atomic.h>
#include "config.h"
#include "MotorDriver.h"
#include "Ultrasonic.h"
#include "globals.h"
#include "Gyro.h"

// // Global pulse counters required for ISR access
volatile long leftEncoderCount = 0;
volatile long rightEncoderCount = 0;

// Define and initialize the variables in memory
bool isMovingForward = false;
bool isTurning = false;
float integralError = 0.0f;
float lastError = 0.0f;

float frontDistance = 0.0f;
float leftDistance = 0.0f;
float rightDistance = 0.0f;
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

void (*turnLogHook)(float yaw, float err, int duty) = nullptr;

static long encoderTicks() {
    long l, r;
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        l = leftEncoderCount;
        r = rightEncoderCount;
    }
    return (labs(l) + labs(r)) / 2;
}

static void spin(float direction, int duty) {   // direction > 0 = right
    if (direction > 0) setMotors(duty, -duty);
    else               setMotors(-duty, duty);
}

// Keeps both wheels at the same tick count so the robot pivots on its centre.
static void spinBalanced(float direction, int duty, int maxDuty) {
    long l, r;
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        l = leftEncoderCount;
        r = rightEncoderCount;
    }
    float diff = TURN_BALANCE_GAIN * (labs(l) - labs(r));   // > 0: left wheel ahead
    int dl = constrain((int)(duty - diff), 0, maxDuty + 5);
    int dr = constrain((int)(duty + diff), 0, maxDuty + 5);

    if (direction > 0) setMotors(dl, -dr);
    else               setMotors(-dl, dr);
}

// Fallback when the gyro is not available: stop at the calibrated tick count.
static void turnByEncoders(float degrees) {
    long target = (degrees > 0) ? (long)TICKS_PER_90_DEG : (long)TICKS_PER_90_LEFT;
    resetEncoders();
    spin(degrees, TURN_SPEED);
    unsigned long start = millis();
    while (encoderTicks() < target && millis() - start < 5000) {}
    stopMotors();
    delay(300);
}

// In-place turn. + = right, - = left.
// Closed loop on the gyro: slows down near the target and corrects overshoot.
// The calibrated tick count is used as a safety limit.
TurnResult turnDegreesWith(float degrees, float kp, int minDuty, int maxDuty) {
    TurnResult res = {0.0f, 0.0f, 0, false};
    unsigned long start = millis();

    if (!gyroReady()) {
        turnByEncoders(degrees);
        res.timeMs = millis() - start;
        return res;
    }

    float ticks90  = (degrees > 0) ? TICKS_PER_90_DEG : TICKS_PER_90_LEFT;
    long tickLimit = (long)(1.5f * ticks90 * fabs(degrees) / 90.0f);
    float dir      = (degrees > 0) ? 1.0f : -1.0f;
    resetEncoders();
    float startYaw = gyroYaw();   // yaw is not reset, so the heading stays absolute

    while (true) {
        if (millis() - start >= 6000 || encoderTicks() >= tickLimit) {
            res.timedOut = true;
            break;
        }

        gyroUpdate();
        float yaw  = gyroYaw() - startYaw;               // angle turned so far
        float past = dir * (yaw - degrees);              // > 0 = past the target
        if (past > res.overshoot) res.overshoot = past;

        float err = degrees - yaw;
        if (fabs(err) <= TURN_TOLERANCE_DEG) {
            stopMotors();
            unsigned long t = millis();
            while (millis() - t < 150) gyroUpdate();      // check it has not coasted past
            if (fabs(degrees - (gyroYaw() - startYaw)) <= TURN_TOLERANCE_DEG) break;
            continue;
        }

        int duty = constrain((int)(kp * fabs(err)), minDuty, maxDuty);
        if (turnLogHook) turnLogHook(yaw, err, duty);
        spinBalanced(err, duty, maxDuty);
    }
    stopMotors();
    res.timeMs = millis() - start;

    unsigned long t = millis();
    while (millis() - t < 300) gyroUpdate();              // include coasting in the error
    res.finalError = degrees - (gyroYaw() - startYaw);
    return res;
}

void turnDegrees(float degrees) {
    turnDegreesWith(degrees, TURN_KP, TURN_MIN_DUTY, TURN_SPEED);
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

// ================================================================
// ENCODERS - 4x quadrature decoding
// Both channels (A and B) trigger the ISR on every edge (CHANGE), so
// each full encoder cycle gives 4 counts.
//
// State = (A << 1) | B. Forward sequence: 01 -> 11 -> 10 -> 00 -> 01.
// Table index = (previousState << 2) | currentState.
// +1 = one step forward, -1 = one step backward, 0 = no change / invalid.
// ================================================================
static const int8_t QUAD_DELTA[16] = {
     0, +1, -1,  0,
    -1,  0,  0, +1,
    +1,  0,  0, -1,
     0, -1, +1,  0
};

static volatile uint8_t leftPrevState  = 0;
static volatile uint8_t rightPrevState = 0;

static inline uint8_t readLeftState() {
    return (digitalRead(PIN_LEFT_ENC_A) << 1) | digitalRead(PIN_LEFT_ENC_B);
}

static inline uint8_t readRightState() {
    return (digitalRead(PIN_RIGHT_ENC_A) << 1) | digitalRead(PIN_RIGHT_ENC_B);
}

// Call once in setup(): configures the pins and attaches both channels of
// both encoders to the interrupts.
void initEncoders() {
    pinMode(PIN_LEFT_ENC_A,  INPUT_PULLUP);
    pinMode(PIN_LEFT_ENC_B,  INPUT_PULLUP);
    pinMode(PIN_RIGHT_ENC_A, INPUT_PULLUP);
    pinMode(PIN_RIGHT_ENC_B, INPUT_PULLUP);

    leftPrevState  = readLeftState();
    rightPrevState = readRightState();

    attachInterrupt(digitalPinToInterrupt(PIN_LEFT_ENC_A),  leftEncoderISR,  CHANGE);
    attachInterrupt(digitalPinToInterrupt(PIN_LEFT_ENC_B),  leftEncoderISR,  CHANGE);
    attachInterrupt(digitalPinToInterrupt(PIN_RIGHT_ENC_A), rightEncoderISR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(PIN_RIGHT_ENC_B), rightEncoderISR, CHANGE);
}

// The left motor is mounted mirrored, so its count is inverted
// to make "robot forward" positive for both wheels.
void leftEncoderISR() {
    uint8_t curr = readLeftState();
    leftEncoderCount -= QUAD_DELTA[(leftPrevState << 2) | curr];
    leftPrevState = curr;
}

void rightEncoderISR() {
    uint8_t curr = readRightState();
    rightEncoderCount += QUAD_DELTA[(rightPrevState << 2) | curr];
    rightPrevState = curr;
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
