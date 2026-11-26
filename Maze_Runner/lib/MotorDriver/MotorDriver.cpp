#include <Arduino.h>
#include <util/atomic.h>
#include "config.h"
#include "MotorDriver.h"
#include "Ultrasonic.h"
#include "globals.h"

// ============================================================================
//  Global state shared between the procedural layer and ISRs
// ============================================================================
volatile long leftEncoderCount  = 0;
volatile long rightEncoderCount = 0;

bool  isMovingForward = false;
bool  isTurning       = false;
float integralError   = 0.0f;
float lastError       = 0.0f;

float frontDistance  = 0.0f;
float leftDistance   = 0.0f;
float rightDistance  = 0.0f;
int   currentHeading = NORTH;

// Optional debug hook for Serial plotting during PID turns
void (*turnLogHook)(float yaw, float err, int duty) = nullptr;

// ============================================================================
//  OOP encoder globals (used by the class ISR callbacks)
// ============================================================================
volatile long global_left_pulses  = 0;
volatile long global_right_pulses = 0;
static uint8_t global_left_enc_b  = 0;
static uint8_t global_right_enc_b = 0;

// ============================================================================
//  Static ISR callbacks (class methods, attached in begin())
// ============================================================================
void MotorDriver::handleLeftEncoder() {
    if (digitalRead(global_left_enc_b) == HIGH) global_left_pulses++;
    else                                         global_left_pulses--;
}

void MotorDriver::handleRightEncoder() {
    if (digitalRead(global_right_enc_b) == HIGH) global_right_pulses--;
    else                                          global_right_pulses++;
}

// ============================================================================
//  Constructor & begin()
// ============================================================================
MotorDriver::MotorDriver(uint8_t lRpwm, uint8_t lLpwm,
                         uint8_t rRpwm, uint8_t rLpwm,
                         uint8_t enPin,
                         uint8_t lEncA, uint8_t lEncB,
                         uint8_t rEncA, uint8_t rEncB)
    : leftRpwm(lRpwm), leftLpwm(lLpwm),
      rightRpwm(rRpwm), rightLpwm(rLpwm),
      enablePin(enPin),
      leftEncA(lEncA), leftEncB(lEncB),
      rightEncA(rEncA), rightEncB(rEncB) {}

void MotorDriver::begin() {
    pinMode(leftRpwm,  OUTPUT);
    pinMode(leftLpwm,  OUTPUT);
    pinMode(rightRpwm, OUTPUT);
    pinMode(rightLpwm, OUTPUT);
    pinMode(enablePin, OUTPUT);
    digitalWrite(enablePin, HIGH);

    pinMode(leftEncA,  INPUT_PULLUP);
    pinMode(leftEncB,  INPUT_PULLUP);
    pinMode(rightEncA, INPUT_PULLUP);
    pinMode(rightEncB, INPUT_PULLUP);

    global_left_enc_b  = leftEncB;
    global_right_enc_b = rightEncB;

    attachInterrupt(digitalPinToInterrupt(leftEncA),  MotorDriver::handleLeftEncoder,  RISING);
    attachInterrupt(digitalPinToInterrupt(rightEncA), MotorDriver::handleRightEncoder, RISING);

    stop();
}

// ============================================================================
//  Encoder management (class methods)
// ============================================================================
void MotorDriver::resetEncoders() {
    noInterrupts();
    global_left_pulses  = 0;
    global_right_pulses = 0;
    interrupts();
}

long MotorDriver::getLeftPulses() {
    noInterrupts();
    long p = global_left_pulses;
    interrupts();
    return p;
}

long MotorDriver::getRightPulses() {
    noInterrupts();
    long p = global_right_pulses;
    interrupts();
    return p;
}

// ============================================================================
//  Raw speed output
// ============================================================================
void MotorDriver::setSpeeds(int leftSpeed, int rightSpeed) {
    if (leftSpeed >= 0) {
        analogWrite(leftRpwm, constrain(leftSpeed,  0, 255));
        analogWrite(leftLpwm, 0);
    } else {
        analogWrite(leftRpwm, 0);
        analogWrite(leftLpwm, constrain(-leftSpeed, 0, 255));
    }
    if (rightSpeed >= 0) {
        analogWrite(rightRpwm, constrain(rightSpeed,  0, 255));
        analogWrite(rightLpwm, 0);
    } else {
        analogWrite(rightRpwm, 0);
        analogWrite(rightLpwm, constrain(-rightSpeed, 0, 255));
    }
}

void MotorDriver::stop() {
    analogWrite(leftRpwm,  0); analogWrite(leftLpwm,  0);
    analogWrite(rightRpwm, 0); analogWrite(rightLpwm, 0);
}

// ============================================================================
//  Straight-line movement
// ============================================================================

// PI-corrected cell-based forward movement
void MotorDriver::moveCells(float numCells, int basePwm) {
    resetEncoders();
    long   target         = (long)(numCells * TICKS_PER_CELL_25CM);
    double kp             = 1.5;
    double ki             = 0.05;
    double integral_error = 0.0;

    while ((getLeftPulses() + getRightPulses()) / 2 < target) {
        double error   = getLeftPulses() - getRightPulses();
        integral_error = constrain(integral_error + error, -300.0, 300.0);
        double corr    = kp * error + ki * integral_error;
        setSpeeds(constrain((int)(basePwm - corr), 0, 255),
                  constrain((int)(basePwm + corr), 0, 255));
        delay(10);
    }
    stop();
}

// Ultrasonic-guarded distance movement (ported from main)
void MotorDriver::moveDistanceCM(float distanceCM, int speed) {
    if (isMovingForward || isTurning) return;
    isMovingForward = true;
    integralError   = 0.0f;
    lastError       = 0.0f;

    long targetCounts = (long)(distanceCM / CM_PER_COUNT);

    Serial.print("Moving forward "); Serial.print(distanceCM);
    Serial.print("cm - Target counts: "); Serial.println(targetCounts);

    resetEncoders();

    while (abs(leftEncoderCount) < targetCounts || abs(rightEncoderCount) < targetCounts) {
        float currentFront = readUltrasonic(PIN_ULTRA_FRONT_TRIG, PIN_ULTRA_FRONT_ECHO);
        if (currentFront < FRONT_OBSTACLE) {
            Serial.println("Front obstacle detected! Stopping.");
            stopMotors();
            isMovingForward = false;
            return;
        }
        setMotors(speed, speed);
        delay(10);
    }

    stopMotors();
    delay(200);
    Serial.print("Forward done - L:"); Serial.print(leftEncoderCount);
    Serial.print(" R:"); Serial.println(rightEncoderCount);
    isMovingForward = false;
}

// ============================================================================
//  Encoder-only turns (fallback, no gyro required)
// ============================================================================
void MotorDriver::turnRight90(int basePwm) {
    resetEncoders();
    while (getLeftPulses() < TICKS_PER_90_DEG) {
        analogWrite(leftRpwm, basePwm); analogWrite(leftLpwm, 0);
        analogWrite(rightRpwm, 0);     analogWrite(rightLpwm, basePwm);
        delay(10);
    }
    stop();
}

void MotorDriver::turnLeft90(int basePwm) {
    resetEncoders();
    while (getRightPulses() < TICKS_PER_90_LEFT) {
        analogWrite(leftRpwm, 0);      analogWrite(leftLpwm, basePwm);
        analogWrite(rightRpwm, basePwm); analogWrite(rightLpwm, 0);
        delay(10);
    }
    stop();
}

void MotorDriver::turn180(int basePwm) {
    resetEncoders();
    while (getLeftPulses() < TICKS_PER_180_DEG) {
        analogWrite(leftRpwm, basePwm); analogWrite(leftLpwm, 0);
        analogWrite(rightRpwm, 0);     analogWrite(rightLpwm, basePwm);
        delay(10);
    }
    stop();
}

// ============================================================================
//  Gyroscope-fused turns
//  Primary stop: gyro heading reaches target.
//  Safety watchdog: encoder tick limit (150 % of encoder-only target).
// ============================================================================

void MotorDriver::turnRight90Gyro(Gyroscope& gyro, int basePwm) {
    const long tickWatchdog = (long)(TICKS_PER_90_DEG * 1.5f);
    gyro.resetHeading();
    resetEncoders();
    while (true) {
        float heading = gyro.updateHeading();
        if (heading >= 90.0f)              break;
        if (getLeftPulses() >= tickWatchdog) break;
        analogWrite(leftRpwm,  basePwm); analogWrite(leftLpwm,  0);
        analogWrite(rightRpwm, 0);       analogWrite(rightLpwm, basePwm);
        delay(5);
    }
    stop();
}

void MotorDriver::turnLeft90Gyro(Gyroscope& gyro, int basePwm) {
    const long tickWatchdog = (long)(TICKS_PER_90_LEFT * 1.5f);
    gyro.resetHeading();
    resetEncoders();
    while (true) {
        float heading = gyro.updateHeading();
        if (heading >= 90.0f)               break;
        if (getRightPulses() >= tickWatchdog) break;
        analogWrite(leftRpwm,  0);       analogWrite(leftLpwm,  basePwm);
        analogWrite(rightRpwm, basePwm); analogWrite(rightLpwm, 0);
        delay(5);
    }
    stop();
}

void MotorDriver::turn180Gyro(Gyroscope& gyro, int basePwm) {
    const long tickWatchdog = (long)(TICKS_PER_180_DEG * 1.5f);
    gyro.resetHeading();
    resetEncoders();
    while (true) {
        float heading = gyro.updateHeading();
        if (heading >= 180.0f)             break;
        if (getLeftPulses() >= tickWatchdog) break;
        analogWrite(leftRpwm,  basePwm); analogWrite(leftLpwm,  0);
        analogWrite(rightRpwm, 0);       analogWrite(rightLpwm, basePwm);
        delay(5);
    }
    stop();
}

// ============================================================================
//  PID closed-loop turn — arbitrary angle, gyro primary, encoder safety limit
//  Ported from the main-branch procedural turnDegreesWith(), adapted to use
//  the OOP Gyroscope interface instead of the old gyroYaw() free functions.
// ============================================================================

// Internal helper: balanced pivot — corrects for left/right wheel speed diff
static void spinBalancedClass(MotorDriver& md,
                              float direction, int duty, int maxDuty,
                              long lPulses, long rPulses,
                              uint8_t lR, uint8_t lL,
                              uint8_t rR, uint8_t rL) {
    float diff = TURN_BALANCE_GAIN * (labs(lPulses) - labs(rPulses));
    int dl = constrain((int)(duty - diff), 0, maxDuty + 5);
    int dr = constrain((int)(duty + diff), 0, maxDuty + 5);
    if (direction > 0) { analogWrite(lR, dl); analogWrite(lL, 0);
                         analogWrite(rR, 0);  analogWrite(rL, dr); }
    else               { analogWrite(lR, 0);  analogWrite(lL, dl);
                         analogWrite(rR, dr); analogWrite(rL, 0);  }
}

TurnResult MotorDriver::turnDegreesWith(Gyroscope& gyro, float degrees,
                                        float kp, int minDuty, int maxDuty) {
    TurnResult res = {0.0f, 0.0f, 0, false};
    unsigned long start = millis();

    // Encoder-only fallback if gyro is unavailable
    if (!gyro.isReady()) {
        if (degrees > 0) turnRight90(maxDuty);
        else             turnLeft90(maxDuty);
        res.timeMs = millis() - start;
        return res;
    }

    float ticks90  = (degrees > 0) ? (float)TICKS_PER_90_DEG : (float)TICKS_PER_90_LEFT;
    long tickLimit = (long)(1.5f * ticks90 * fabs(degrees) / 90.0f);
    float dir      = (degrees > 0) ? 1.0f : -1.0f;
    resetEncoders();
    gyro.resetHeading();

    while (true) {
        if (millis() - start >= 6000 || getLeftPulses() >= tickLimit) {
            res.timedOut = true;
            break;
        }

        float yaw  = dir * gyro.updateHeading();     // positive = towards target
        float past = yaw - fabs(degrees);
        if (past > res.overshoot) res.overshoot = past;

        float err = degrees - (dir * gyro.getHeading());
        if (fabs(err) <= TURN_TOLERANCE_DEG) {
            stop();
            unsigned long t = millis();
            while (millis() - t < 150) gyro.updateHeading();
            if (fabs(degrees - (dir * gyro.getHeading())) <= TURN_TOLERANCE_DEG) break;
            continue;
        }

        int duty = constrain((int)(kp * fabs(err)), minDuty, maxDuty);
        if (turnLogHook) turnLogHook(dir * gyro.getHeading(), err, duty);
        spinBalancedClass(*this, err, duty, maxDuty,
                          getLeftPulses(), getRightPulses(),
                          leftRpwm, leftLpwm, rightRpwm, rightLpwm);
    }
    stop();
    res.timeMs = millis() - start;

    unsigned long t = millis();
    while (millis() - t < 300) gyro.updateHeading();
    res.finalError = degrees - (dir * gyro.getHeading());
    return res;
}

// ============================================================================
//  Procedural / backward-compatible free functions
//  These are kept so existing maze-exploration code compiles unchanged.
//  They use the bare-metal PWM + global encoder counters directly.
// ============================================================================

// 4x quadrature decoding table
static const int8_t QUAD_DELTA[16] = {
     0, +1, -1,  0,
    -1,  0,  0, +1,
    +1,  0,  0, -1,
     0, -1, +1,  0
};
static volatile uint8_t leftPrevState  = 0;
static volatile uint8_t rightPrevState = 0;

static inline uint8_t readLeftState() {
    return (uint8_t)((digitalRead(PIN_LEFT_ENC_A) << 1) | digitalRead(PIN_LEFT_ENC_B));
}
static inline uint8_t readRightState() {
    return (uint8_t)((digitalRead(PIN_RIGHT_ENC_A) << 1) | digitalRead(PIN_RIGHT_ENC_B));
}

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

void resetEncoders() {
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        leftEncoderCount  = 0;
        rightEncoderCount = 0;
    }
}

void setMotors(int leftSpeed, int rightSpeed) {
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

void stopMotors() {
    setPWM(PIN_LEFT_RPWM,  20000, 0);
    setPWM(PIN_LEFT_LPWM,  20000, 0);
    setPWM(PIN_RIGHT_RPWM, 20000, 0);
    setPWM(PIN_RIGHT_LPWM, 20000, 0);
}

void moveDistanceCM(float distanceCM, int speed) {
    if (isMovingForward || isTurning) return;
    isMovingForward = true;
    integralError = 0.0f;
    lastError     = 0.0f;
    long targetCounts = (long)(distanceCM / CM_PER_COUNT);
    resetEncoders();
    while (abs(leftEncoderCount) < targetCounts || abs(rightEncoderCount) < targetCounts) {
        float cf = readUltrasonic(PIN_ULTRA_FRONT_TRIG, PIN_ULTRA_FRONT_ECHO);
        if (cf < FRONT_OBSTACLE) { stopMotors(); isMovingForward = false; return; }
        setMotors(speed, speed);
        delay(10);
    }
    stopMotors();
    delay(200);
    isMovingForward = false;
}

static long procEncoderTicks() {
    long l, r;
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { l = leftEncoderCount; r = rightEncoderCount; }
    return (labs(l) + labs(r)) / 2;
}

static void spinProc(float direction, int duty) {
    if (direction > 0) setMotors(duty, -duty);
    else               setMotors(-duty, duty);
}

static void spinBalancedProc(float direction, int duty, int maxDuty) {
    long l, r;
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { l = leftEncoderCount; r = rightEncoderCount; }
    float diff = TURN_BALANCE_GAIN * (labs(l) - labs(r));
    int dl = constrain((int)(duty - diff), 0, maxDuty + 5);
    int dr = constrain((int)(duty + diff), 0, maxDuty + 5);
    if (direction > 0) setMotors(dl, -dr);
    else               setMotors(-dl, dr);
}

static void turnByEncoders(float degrees) {
    long target = (degrees > 0) ? (long)TICKS_PER_90_DEG : (long)TICKS_PER_90_LEFT;
    resetEncoders();
    spinProc(degrees, TURN_SPEED);
    unsigned long start = millis();
    while (procEncoderTicks() < target && millis() - start < 5000) {}
    stopMotors();
    delay(300);
}

TurnResult turnDegreesWith(float degrees, float kp, int minDuty, int maxDuty) {
    TurnResult res = {0.0f, 0.0f, 0, false};
    unsigned long start = millis();

    if (!gyroReady()) {
        turnByEncoders(degrees);
        res.timeMs = millis() - start;
        return res;
    }

    float ticks90  = (degrees > 0) ? (float)TICKS_PER_90_DEG : (float)TICKS_PER_90_LEFT;
    long tickLimit = (long)(1.5f * ticks90 * fabs(degrees) / 90.0f);
    float dir      = (degrees > 0) ? 1.0f : -1.0f;
    resetEncoders();
    float startYaw = gyroYaw();

    while (true) {
        if (millis() - start >= 6000 || procEncoderTicks() >= tickLimit) {
            res.timedOut = true; break;
        }
        gyroUpdate();
        float yaw  = gyroYaw() - startYaw;
        float past = dir * (yaw - degrees);
        if (past > res.overshoot) res.overshoot = past;

        float err = degrees - yaw;
        if (fabs(err) <= TURN_TOLERANCE_DEG) {
            stopMotors();
            unsigned long t = millis();
            while (millis() - t < 150) gyroUpdate();
            if (fabs(degrees - (gyroYaw() - startYaw)) <= TURN_TOLERANCE_DEG) break;
            continue;
        }
        int duty = constrain((int)(kp * fabs(err)), minDuty, maxDuty);
        if (turnLogHook) turnLogHook(yaw, err, duty);
        spinBalancedProc(err, duty, maxDuty);
    }
    stopMotors();
    res.timeMs = millis() - start;
    unsigned long t = millis();
    while (millis() - t < 300) gyroUpdate();
    res.finalError = degrees - (gyroYaw() - startYaw);
    return res;
}

void turnDegrees(float degrees) {
    turnDegreesWith(degrees, TURN_KP, TURN_MIN_DUTY, TURN_SPEED);
}

void turnLeft()   { turnDegrees(-90.0f); currentHeading = (currentHeading + 3) % 4; }
void turnRight()  { turnDegrees( 90.0f); currentHeading = (currentHeading + 1) % 4; }
void turnAround() { turnDegrees(180.0f); currentHeading = (currentHeading + 2) % 4; }

// ============================================================================
//  Bare-metal PWM — Timer 3 (pin 5) and Timer 4 (pins 6, 7, 8) on Mega 2560
// ============================================================================
void setPWM(uint8_t pin, uint32_t freq, uint8_t dutyCycle) {
    if (freq == 0) return;
    if (dutyCycle > 100) dutyCycle = 100;

    static uint32_t timer3Freq = 0, timer4Freq = 0;
    static uint16_t timer3Top  = 0, timer4Top  = 0;

    if (pin == 5) {
        if (timer3Freq != freq) {
            uint8_t  pb  = (freq > 30000) ? 1 : (freq > 4000) ? 2 : 3;
            uint16_t psc = (pb == 1) ? 1 : (pb == 2) ? 8 : 64;
            uint32_t top = (F_CPU / ((uint32_t)psc * freq)) - 1;
            if (top > 65535) top = 65535;
            timer3Top = (uint16_t)top; timer3Freq = freq;
            TCCR3A = (1 << WGM31);
            TCCR3B = (1 << WGM33) | (1 << WGM32) | pb;
            ICR3   = timer3Top;
        }
        if (dutyCycle == 0) { TCCR3A &= ~(1 << COM3A1); PORTE &= ~(1 << PE3); }
        else { OCR3A = (uint32_t)timer3Top * dutyCycle / 100; TCCR3A |= (1 << COM3A1); }
    }
    else if (pin == 6 || pin == 7 || pin == 8) {
        if (timer4Freq != freq) {
            uint8_t  pb  = (freq > 30000) ? 1 : (freq > 4000) ? 2 : 3;
            uint16_t psc = (pb == 1) ? 1 : (pb == 2) ? 8 : 64;
            uint32_t top = (F_CPU / ((uint32_t)psc * freq)) - 1;
            if (top > 65535) top = 65535;
            timer4Top = (uint16_t)top; timer4Freq = freq;
            TCCR4A = (1 << WGM41);
            TCCR4B = (1 << WGM43) | (1 << WGM42) | pb;
            ICR4   = timer4Top;
        }
        uint16_t ocr = (uint32_t)timer4Top * dutyCycle / 100;
        if (pin == 6) {
            if (dutyCycle == 0) { TCCR4A &= ~(1 << COM4A1); PORTH &= ~(1 << PH3); }
            else { OCR4A = ocr; TCCR4A |= (1 << COM4A1); }
        } else if (pin == 7) {
            if (dutyCycle == 0) { TCCR4A &= ~(1 << COM4B1); PORTH &= ~(1 << PH4); }
            else { OCR4B = ocr; TCCR4A |= (1 << COM4B1); }
        } else if (pin == 8) {
            if (dutyCycle == 0) { TCCR4A &= ~(1 << COM4C1); PORTH &= ~(1 << PH5); }
            else { OCR4C = ocr; TCCR4A |= (1 << COM4C1); }
        }
    }
}
