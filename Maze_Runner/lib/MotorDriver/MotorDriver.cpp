#include "MotorDriver.h"
#include "config.h"

// Global pulse counters required for ISR access
volatile long global_left_pulses = 0;
volatile long global_right_pulses = 0;
static uint8_t global_left_enc_b = 0;
static uint8_t global_right_enc_b = 0;

// Interrupt Service Routines
void MotorDriver::handleLeftEncoder() {
    if (digitalRead(global_left_enc_b) == HIGH) {
        global_left_pulses++;
    } else {
        global_left_pulses--;
    }
}

void MotorDriver::handleRightEncoder() {
    if (digitalRead(global_right_enc_b) == HIGH) {
        global_right_pulses--;
    } else {
        global_right_pulses++;
    }
}

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
    // Configure Driver Pins
    pinMode(leftRpwm, OUTPUT);
    pinMode(leftLpwm, OUTPUT);
    pinMode(rightRpwm, OUTPUT);
    pinMode(rightLpwm, OUTPUT);
    pinMode(enablePin, OUTPUT);

    // Enable drivers
    digitalWrite(enablePin, HIGH);

    // Configure Encoder Pins
    pinMode(leftEncA, INPUT_PULLUP);
    pinMode(leftEncB, INPUT_PULLUP);
    pinMode(rightEncA, INPUT_PULLUP);
    pinMode(rightEncB, INPUT_PULLUP);

    // Bind B pins for ISR direction checking
    global_left_enc_b = leftEncB;
    global_right_enc_b = rightEncB;

    // Attach Hardware Interrupts
    attachInterrupt(digitalPinToInterrupt(leftEncA), MotorDriver::handleLeftEncoder, RISING);
    attachInterrupt(digitalPinToInterrupt(rightEncA), MotorDriver::handleRightEncoder, RISING);

    stop();
}

void MotorDriver::resetEncoders() {
    noInterrupts();
    global_left_pulses = 0;
    global_right_pulses = 0;
    interrupts();
}

long MotorDriver::getLeftPulses() {
    noInterrupts();
    long pulses = global_left_pulses;
    interrupts();
    return pulses;
}

long MotorDriver::getRightPulses() {
    noInterrupts();
    long pulses = global_right_pulses;
    interrupts();
    return pulses;
}

void MotorDriver::setSpeeds(int leftSpeed, int rightSpeed) {
    // Left Motor Direction
    if (leftSpeed >= 0) {
        analogWrite(leftRpwm, constrain(leftSpeed, 0, 255));
        analogWrite(leftLpwm, 0);
    } else {
        analogWrite(leftRpwm, 0);
        analogWrite(leftLpwm, constrain(-leftSpeed, 0, 255));
    }

    // Right Motor Direction
    if (rightSpeed >= 0) {
        analogWrite(rightRpwm, constrain(rightSpeed, 0, 255));
        analogWrite(rightLpwm, 0);
    } else {
        analogWrite(rightRpwm, 0);
        analogWrite(rightLpwm, constrain(-rightSpeed, 0, 255));
    }
}

void MotorDriver::stop() {
    analogWrite(leftRpwm, 0);
    analogWrite(leftLpwm, 0);
    analogWrite(rightRpwm, 0);
    analogWrite(rightLpwm, 0);
}

void MotorDriver::moveCells(float numCells, int basePwm) {
    resetEncoders();

    long target_ticks = numCells * TICKS_PER_CELL_25CM;
    double kp = 1.5;
    double ki = 0.05;
    double integral_error = 0;

    while (true) {
        long current_left = getLeftPulses();
        long current_right = getRightPulses();

        if ((current_left + current_right) / 2 >= target_ticks) {
            break;
        }

        double error = current_left - current_right;
        integral_error += error;
        integral_error = constrain(integral_error, -300, 300);

        double correction = (kp * error) + (ki * integral_error);

        int left_pwm = constrain(basePwm - correction, 0, 255);
        int right_pwm = constrain(basePwm + correction, 0, 255);

        setSpeeds(left_pwm, right_pwm);
        delay(10);
    }

    stop();
}

void MotorDriver::turnRight90(int basePwm) {
    resetEncoders();

    while (true) {
        if (getLeftPulses() >= TICKS_PER_90_DEG) {
            break;
        }

        // Pivot turn right (Left motor forward, Right motor reverse)
        analogWrite(leftRpwm, basePwm);
        analogWrite(leftLpwm, 0);
        analogWrite(rightRpwm, 0);
        analogWrite(rightLpwm, basePwm);

        delay(10);
    }

    stop();
}

void MotorDriver::turn180(int basePwm) {
    resetEncoders();

    while (true) {
        if (getLeftPulses() >= TICKS_PER_180_DEG) {
            break;
        }

        // Pivot turn 180 deg
        analogWrite(leftRpwm, basePwm);
        analogWrite(leftLpwm, 0);
        analogWrite(rightRpwm, 0);
        analogWrite(rightLpwm, basePwm);

        delay(10);
    }

    stop();
}