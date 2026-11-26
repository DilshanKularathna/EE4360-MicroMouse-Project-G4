#ifndef MOTOR_DRIVER_H
#define MOTOR_DRIVER_H

#include <Arduino.h>
#include "Gyroscope.h"

// ============================================================================
//  TurnResult — returned by closed-loop gyro turns for diagnostics / tuning
// ============================================================================
struct TurnResult {
    float finalError;       // deg remaining after the robot has stopped
    float overshoot;        // deg past the target (0 if none)
    unsigned long timeMs;   // total turn duration
    bool timedOut;          // true if the 6 s safety timeout fired
};

// Optional debug hook: called every loop iteration of a PID turn.
// Set to nullptr (default) to disable. Useful for Serial plotting.
extern void (*turnLogHook)(float yaw, float err, int duty);

// ============================================================================
//  MotorDriver — OOP interface (preferred, encapsulates all motor / encoder
//  state so multiple instances could coexist in future).
// ============================================================================
class MotorDriver {
private:
    uint8_t leftRpwm,  leftLpwm;
    uint8_t rightRpwm, rightLpwm;
    uint8_t enablePin;
    uint8_t leftEncA,  leftEncB;
    uint8_t rightEncA, rightEncB;

public:
    MotorDriver(uint8_t lRpwm, uint8_t lLpwm,
                uint8_t rRpwm, uint8_t rLpwm,
                uint8_t enPin,
                uint8_t lEncA, uint8_t lEncB,
                uint8_t rEncA, uint8_t rEncB);

    void begin();

    // ---- Raw speed outputs (-255 to +255) ----
    void setSpeeds(int leftSpeed, int rightSpeed);
    void stop();

    // ---- Encoder management ----
    void resetEncoders();
    long getLeftPulses();
    long getRightPulses();

    // ---- Straight-line movement ----
    // moveCells: PI-corrected forward movement (encoder closed-loop)
    void moveCells(float numCells, int basePwm = 70);
    // moveDistanceCM: ultrasonic-guarded forward movement (from main)
    void moveDistanceCM(float distanceCM, int speed = 70);

    // ---- Encoder-only turns (fallback / no gyro) ----
    void turnRight90(int basePwm = 70);
    void turnLeft90 (int basePwm = 70);
    void turn180    (int basePwm = 70);

    // ---- Gyroscope-fused turns (preferred — gyro primary, encoder watchdog) ----
    // @param gyro     Reference to an initialised, calibrated Gyroscope object.
    // @param basePwm  Motor drive strength (0-255, default 70).
    void turnRight90Gyro(Gyroscope& gyro, int basePwm = 70);
    void turnLeft90Gyro (Gyroscope& gyro, int basePwm = 70);
    void turn180Gyro    (Gyroscope& gyro, int basePwm = 70);

    // ---- PID closed-loop turn (arbitrary angle, gyro + encoder safety) ----
    // degrees: + = right (CW), - = left (CCW)
    TurnResult turnDegreesWith(Gyroscope& gyro, float degrees,
                               float kp      = TURN_KP,
                               int   minDuty = TURN_MIN_DUTY,
                               int   maxDuty = TURN_SPEED);

    // ---- Static ISR callbacks (attached in begin()) ----
    static void handleLeftEncoder();
    static void handleRightEncoder();
};

// ============================================================================
//  Bare-metal PWM helper (Timer 3 / Timer 4 on Mega 2560)
//  Used by the procedural layer below.  Also callable directly.
// ============================================================================
void setPWM(uint8_t pin, uint32_t freq, uint8_t dutyCycle);

// ============================================================================
//  Procedural / global-function layer — kept for backward compatibility with
//  existing maze-exploration code that calls these free functions directly.
//  Internally they delegate to the robot's singleton MotorDriver instance.
// ============================================================================
void initEncoders();                            // call once in setup()
void resetEncoders();
void setMotors(int leftSpeed, int rightSpeed);  // bare PWM, no class needed
void stopMotors();
void moveDistanceCM(float distanceCM, int speed);
void turnLeft();
void turnRight();
void turnAround();
void turnDegrees(float degrees);
TurnResult turnDegreesWith(float degrees, float kp, int minDuty, int maxDuty);
void leftEncoderISR();
void rightEncoderISR();

// Global sensor / state mirrors (read by maze-exploration code)
extern volatile long leftEncoderCount;
extern volatile long rightEncoderCount;
extern bool   isMovingForward;
extern bool   isTurning;
extern float  frontDistance;
extern float  leftDistance;
extern float  rightDistance;
extern int    currentHeading;

#endif // MOTOR_DRIVER_H
