#include <Arduino.h>
#include "config.h"
#include "MotorDriver.h"
// Our working File

// Global Arrays & Variables
int rawSensors[NUM_IR_SENSORS];
int normalizedSensors[NUM_IR_SENSORS];

// ==========================================
// SPEED & PID TUNING PARAMETERS
// ==========================================
const int BASE_IR_PWM = 30; // Forward cruising speed
const int MAX_PWM = 100; // Overall Motor PWM limit

// PID Gains for smooth steering
float Kp = 0.025; // Proportional gain (Adjust between 0.015 - 0.025)
float Ki = 0.000; // Integral gain
float Kd = 0.053; // Derivative gain (Adjust between 0.050 - 0.100)

// ==========================================
// LINE-LOSS RECOVERY
// ==========================================
const int LOST_THRESHOLD = 200;          // totalValue below this = line lost
const int DIR_DEADBAND = 1000;           // |error| must exceed this to update memory
const int RECOVERY_PWM = 40;             // Turn speed while searching (must overcome friction)
const unsigned long LOST_STOP_MS = 3000; // Stop if line not found within this time

int lastSeenDir = 0; // -1 = line was on left, +1 = right, 0 = unknown
bool lineLost = false;
unsigned long lineLostSince = 0;

int lastLineError = 0;
float lineIntegral = 0;

// Global Pulse Counters (Encoders)
volatile long left_pulses = 0;
volatile long right_pulses = 0;

// ==========================================
// MOTOR DRIVER CONTROL FUNCTIONS
// ==========================================
void stopMotors()
{
  analogWrite(PIN_LEFT_RPWM, 0);
  analogWrite(PIN_LEFT_LPWM, 0);
  analogWrite(PIN_RIGHT_RPWM, 0);
  analogWrite(PIN_RIGHT_LPWM, 0);
}

void setMotorSpeeds(int leftSpeed, int rightSpeed)
{
  // Left Motor Direction & Speed
  if (leftSpeed >= 0)
  {
    analogWrite(PIN_LEFT_RPWM, constrain(leftSpeed, 0, MAX_PWM));
    analogWrite(PIN_LEFT_LPWM, 0);
  }
  else
  {
    analogWrite(PIN_LEFT_RPWM, 0);
    analogWrite(PIN_LEFT_LPWM, constrain(-leftSpeed, 0, MAX_PWM));
  }

  // Right Motor Direction & Speed
  if (rightSpeed >= 0)
  {
    analogWrite(PIN_RIGHT_RPWM, constrain(rightSpeed, 0, MAX_PWM));
    analogWrite(PIN_RIGHT_LPWM, 0);
  }
  else
  {
    analogWrite(PIN_RIGHT_RPWM, 0);
    analogWrite(PIN_RIGHT_LPWM, constrain(-rightSpeed, 0, MAX_PWM));
  }
}

// ==========================================
// SENSOR READING & PID COMPUTATION
// ==========================================
const float SENSOR_EXPONENT = 2.5;

void readSensors()
{
  const int sensorPins[NUM_IR_SENSORS] = {IR0, IR1, IR2, IR3, IR4, IR5, IR6, IR7};

  for (int i = 0; i < NUM_IR_SENSORS; i++)
  {
    rawSensors[i] = analogRead(sensorPins[i]);
    int clamped = constrain(rawSensors[i], whiteValues[i], blackValues[i]);
    float ratio = (float)(clamped - whiteValues[i]) / (float)(blackValues[i] - whiteValues[i]);
    float curvedRatio = pow(ratio, SENSOR_EXPONENT);
    // normalizedSensors[i] = map(clamped, whiteValues[i], blackValues[i], 0, 1000);
    normalizedSensors[i] = (int)(curvedRatio * 1000.0);
  }
}

// Returns true if the line is visible; position is filled in
bool getLinePosition(int &position)
{
  readSensors();

  long weightedSum = 0;
  long totalValue = 0;

  for (int i = 0; i < NUM_IR_SENSORS; i++)
  {
    weightedSum += (long)normalizedSensors[i] * (i * 1000);
    totalValue += normalizedSensors[i];
  }

  if (totalValue < LOST_THRESHOLD)
  {
    return false; // line lost
  }

  position = weightedSum / totalValue;
  return true;
}

void handleLostLine()
{
  if (!lineLost)
  {
    lineLost = true;
    lineLostSince = millis();
    lineIntegral = 0;
  }

  // Give up after timeout
  if (millis() - lineLostSince > LOST_STOP_MS)
  {
    stopMotors();
    return;
  }

  if (lastSeenDir < 0)
  {
    // Line was last on the LEFT -> turn left
    setMotorSpeeds(-RECOVERY_PWM, RECOVERY_PWM);
  }
  else if (lastSeenDir > 0)
  {
    // Line was last on the RIGHT -> turn right
    setMotorSpeeds(RECOVERY_PWM, -RECOVERY_PWM);
  }
  else
  {
    // No info yet (e.g. just started) -> go straight
    setMotorSpeeds(BASE_IR_PWM, BASE_IR_PWM);
  }
}

void followLinePID()
{
  int position;

  if (!getLinePosition(position))
  {
    handleLostLine();
    return; // skip PID, don't touch lastSeenDir
  }

  int error = position - 3500;

  // Line just re-acquired: reset PID state to avoid a derivative kick
  if (lineLost)
  {
    lineLost = false;
    lineIntegral = 0;
    lastLineError = error;
  }

  // Update direction memory only when line is clearly off-center
  if (error < -DIR_DEADBAND)
    lastSeenDir = -1;
  else if (error > DIR_DEADBAND)
    lastSeenDir = +1;

  lineIntegral += error;
  lineIntegral = constrain(lineIntegral, -3000, 3000);

  int derivative = error - lastLineError;
  lastLineError = error;

  float adjustment = (Kp * error) + (Ki * lineIntegral) + (Kd * derivative);

  int leftMotorSpeed = BASE_IR_PWM + adjustment;
  int rightMotorSpeed = BASE_IR_PWM - adjustment;

  setMotorSpeeds(leftMotorSpeed, rightMotorSpeed);
}

// void followLinePID() {
//   int position = getLinePosition();

//   // Setpoint is center (3500)
//   int error = position - 3500;

//   lineIntegral += error;
//   lineIntegral = constrain(lineIntegral, -3000, 3000);

//   int derivative = error - lastLineError;
//   lastLineError  = error;

//   // Steering Adjustment Value
//   float adjustment = (Kp * error) + (Ki * lineIntegral) + (Kd * derivative);

//   // Correct motor control assignment:
//   // When line moves RIGHT (position > 3500, error > 0), left motor speeds up & right slows down to turn RIGHT.
//   int leftMotorSpeed  = BASE_IR_PWM + adjustment;
//   int rightMotorSpeed = BASE_IR_PWM - adjustment;

//   setMotorSpeeds(leftMotorSpeed, rightMotorSpeed);
// }

// ==========================================
// SETUP & MAIN LOOP
// ==========================================
void setup()
{
  Serial.begin(115200);

  // Motor Driver Pins Setup
  pinMode(PIN_RIGHT_RPWM, OUTPUT);
  pinMode(PIN_RIGHT_LPWM, OUTPUT);
  pinMode(PIN_LEFT_RPWM, OUTPUT);
  pinMode(PIN_LEFT_LPWM, OUTPUT);

  // Encoder Pins Setup
  pinMode(PIN_LEFT_ENC_A, INPUT_PULLUP);
  pinMode(PIN_LEFT_ENC_B, INPUT_PULLUP);
  pinMode(PIN_RIGHT_ENC_A, INPUT_PULLUP);
  pinMode(PIN_RIGHT_ENC_B, INPUT_PULLUP);

  // Enable IR Array Power Pin
  pinMode(IR_EN, OUTPUT);
  digitalWrite(IR_EN, HIGH);

  // Encoder Interrupt Attachments (Updated Pins 18 & 2)
  attachInterrupt(digitalPinToInterrupt(PIN_LEFT_ENC_A), leftEncoderISR, RISING);
  attachInterrupt(digitalPinToInterrupt(PIN_RIGHT_ENC_A), rightEncoderISR, RISING);

  delay(2000); // 2-second startup safety delay
}

void loop()
{
  followLinePID();
  delay(5);
}