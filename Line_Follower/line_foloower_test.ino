#include <Arduino.h>

// ==========================================
// PIN DEFINITIONS
// ==========================================
// IR Sensors (8-Channel Array)
#define IR0 A15
#define IR1 A14
#define IR2 A13
#define IR3 A12
#define IR4 A11
#define IR5 A10
#define IR6 A9
#define IR7 A8
#define IR_EN 49

// Left Driver & Encoder (IBT-2 / BTS7960)
const int LEFT_RPWM = 4;
const int LEFT_LPWM = 6;
const int LEFT_ENC_A = 2; // Interrupt 0
const int LEFT_ENC_B = 3; // Interrupt 1

// Right Driver & Encoder (IBT-2 / BTS7960)
const int RIGHT_EN = 7;
const int RIGHT_RPWM = 9;
const int RIGHT_LPWM = 8;
const int RIGHT_ENC_A = 18; // Interrupt 5
const int RIGHT_ENC_B = 19; // Interrupt 4

#define NUM_SENSORS 8

// ==========================================
// CALIBRATION VALUES (From Calibrated Test)
// ==========================================
const int whiteValues[NUM_SENSORS] = {288, 158, 195, 324, 382, 516, 492, 460};
const int blackValues[NUM_SENSORS] = {1023, 1020, 1021, 1021, 1017, 1023, 1018, 1021};

// Global Arrays & Variables
int rawSensors[NUM_SENSORS];
int normalizedSensors[NUM_SENSORS];

// ==========================================
// SPEED & PID TUNING PARAMETERS
// ==========================================
const int BASE_PWM = 30; // Forward cruising speed
const int MAX_PWM = 120; // Overall Motor PWM limit

// REDUCED PID GAINS FOR SMOOTHER, SLOWER TURNS
float Kp = 0.030; // Proportional gain (Lower = gentler steering, Try 0.015 to 0.025)
float Ki = 0.000; // Integral gain
float Kd = 0.100; // Derivative gain (Dampens oscillation, Try 0.05 to 0.10)

int lastLineError = 0;
float lineIntegral = 0;

// Global Pulse Counters (Encoders)
volatile long left_pulses = 0;
volatile long right_pulses = 0;

// ==========================================
// INTERRUPT SERVICE ROUTINES (Encoders)
// ==========================================
void ISR_left_encoder()
{
    if (digitalRead(LEFT_ENC_B) == HIGH)
    {
        left_pulses++;
    }
    else
    {
        left_pulses--;
    }
}

void ISR_right_encoder()
{
    if (digitalRead(RIGHT_ENC_B) == HIGH)
    {
        right_pulses--;
    }
    else
    {
        right_pulses++;
    }
}

// ==========================================
// MOTOR DRIVER CONTROL FUNCTIONS
// ==========================================
void stopMotors()
{
    analogWrite(LEFT_RPWM, 0);
    analogWrite(LEFT_LPWM, 0);
    analogWrite(RIGHT_RPWM, 0);
    analogWrite(RIGHT_LPWM, 0);
}

void setMotorSpeeds(int leftSpeed, int rightSpeed)
{
    // Left Motor Direction & Speed
    if (leftSpeed >= 0)
    {
        analogWrite(LEFT_RPWM, constrain(leftSpeed, 0, MAX_PWM));
        analogWrite(LEFT_LPWM, 0);
    }
    else
    {
        analogWrite(LEFT_RPWM, 0);
        analogWrite(LEFT_LPWM, constrain(-leftSpeed, 0, MAX_PWM));
    }

    // Right Motor Direction & Speed
    if (rightSpeed >= 0)
    {
        analogWrite(RIGHT_RPWM, constrain(rightSpeed, 0, MAX_PWM));
        analogWrite(RIGHT_LPWM, 0);
    }
    else
    {
        analogWrite(RIGHT_RPWM, 0);
        analogWrite(RIGHT_LPWM, constrain(-rightSpeed, 0, MAX_PWM));
    }
}

// ==========================================
// SENSOR READING & PID COMPUTATION
// ==========================================
void readSensors()
{
    const int sensorPins[NUM_SENSORS] = {IR0, IR1, IR2, IR3, IR4, IR5, IR6, IR7};

    for (int i = 0; i < NUM_SENSORS; i++)
    {
        rawSensors[i] = analogRead(sensorPins[i]);
        int clamped = constrain(rawSensors[i], whiteValues[i], blackValues[i]);
        normalizedSensors[i] = map(clamped, whiteValues[i], blackValues[i], 0, 1000);
    }
}

int getLinePosition()
{
    readSensors();

    long weightedSum = 0;
    long totalValue = 0;

    for (int i = 0; i < NUM_SENSORS; i++)
    {
        weightedSum += (long)normalizedSensors[i] * (i * 1000);
        totalValue += normalizedSensors[i];
    }

    if (totalValue < 200)
    {
        return (lastLineError < 0) ? 0 : 7000;
    }

    return weightedSum / totalValue;
}

void followLinePID()
{
    int position = getLinePosition();

    // Setpoint is center (3500)
    int error = position - 3500;

    lineIntegral += error;
    lineIntegral = constrain(lineIntegral, -3000, 3000);

    int derivative = error - lastLineError;
    lastLineError = error;

    // Steering Adjustment Value
    float adjustment = (Kp * error) + (Ki * lineIntegral) + (Kd * derivative);

    // FIXED DIRECTION: Subtracted from Left, Added to Right
    int leftMotorSpeed = BASE_PWM - adjustment;
    int rightMotorSpeed = BASE_PWM + adjustment;

    setMotorSpeeds(leftMotorSpeed, rightMotorSpeed);
}

// ==========================================
// SETUP & MAIN LOOP
// ==========================================
void setup()
{
    Serial.begin(115200);

    // Driver Pins Setup
    pinMode(LEFT_RPWM, OUTPUT);
    pinMode(LEFT_LPWM, OUTPUT);
    pinMode(RIGHT_EN, OUTPUT);
    pinMode(RIGHT_RPWM, OUTPUT);
    pinMode(RIGHT_LPWM, OUTPUT);

    digitalWrite(RIGHT_EN, HIGH);

    // Encoder Pins Setup
    pinMode(LEFT_ENC_A, INPUT_PULLUP);
    pinMode(LEFT_ENC_B, INPUT_PULLUP);
    pinMode(RIGHT_ENC_A, INPUT_PULLUP);
    pinMode(RIGHT_ENC_B, INPUT_PULLUP);

    // Enable IR Array Power Pin
    pinMode(IR_EN, OUTPUT);
    digitalWrite(IR_EN, HIGH);

    // Encoder Interrupt Attachments
    attachInterrupt(digitalPinToInterrupt(LEFT_ENC_A), ISR_left_encoder, RISING);
    attachInterrupt(digitalPinToInterrupt(RIGHT_ENC_A), ISR_right_encoder, RISING);

    delay(2000);
}

void loop()
{
    followLinePID();
    delay(5);
}