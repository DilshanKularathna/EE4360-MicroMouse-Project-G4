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

// RIGHT MOTOR (IBT-2 / BTS7960)
const int RIGHT_RPWM  = 4;
const int RIGHT_LPWM  = 6;
const int RIGHT_ENC_A = 2; // Interrupt 0
const int RIGHT_ENC_B = 3; // Interrupt 1

// LEFT MOTOR (IBT-2 / BTS7960)
const int LEFT_RPWM   = 9;
const int LEFT_LPWM   = 8;
const int LEFT_ENC_A  = 18; // Interrupt 5
const int LEFT_ENC_B  = 19; // Interrupt 4

#define NUM_SENSORS 8

// ==========================================
// NEW CALIBRATION VALUES FROM IMAGE
// ==========================================
const int whiteValues[NUM_SENSORS] = {788, 776, 747, 734, 742, 741, 728, 802};
const int blackValues[NUM_SENSORS] = {976, 975, 974, 971, 974, 980, 978, 978};

// Global Arrays & Variables
int rawSensors[NUM_SENSORS];
int normalizedSensors[NUM_SENSORS];

// ==========================================
// SPEED & PID TUNING PARAMETERS
// ==========================================
const int BASE_PWM = 30;    // Forward cruising speed
const int MAX_PWM  = 100;   // Overall Motor PWM limit

// PID Gains for smooth steering
float Kp = 0.025; // Proportional gain (Adjust between 0.015 - 0.025)
float Ki = 0.000; // Integral gain
float Kd = 0.053; // Derivative gain (Adjust between 0.050 - 0.100)

int lastLineError = 0;
float lineIntegral = 0;

// Global Pulse Counters (Encoders)
volatile long left_pulses  = 0;
volatile long right_pulses = 0;

// ==========================================
// INTERRUPT SERVICE ROUTINES (Encoders)
// ==========================================
void ISR_left_encoder() {
  if (digitalRead(LEFT_ENC_B) == HIGH) {
    left_pulses++;
  } else {
    left_pulses--;
  }
}

void ISR_right_encoder() {
  if (digitalRead(RIGHT_ENC_B) == HIGH) {
    right_pulses--; 
  } else {
    right_pulses++; 
  }
}

// ==========================================
// MOTOR DRIVER CONTROL FUNCTIONS
// ==========================================
void stopMotors() {
  analogWrite(LEFT_RPWM, 0);
  analogWrite(LEFT_LPWM, 0);
  analogWrite(RIGHT_RPWM, 0);
  analogWrite(RIGHT_LPWM, 0);
}

void setMotorSpeeds(int leftSpeed, int rightSpeed) {
  // Left Motor Direction & Speed
  if (leftSpeed >= 0) {
    analogWrite(LEFT_RPWM, constrain(leftSpeed, 0, MAX_PWM));
    analogWrite(LEFT_LPWM, 0);
  } else {
    analogWrite(LEFT_RPWM, 0);
    analogWrite(LEFT_LPWM, constrain(-leftSpeed, 0, MAX_PWM));
  }

  // Right Motor Direction & Speed
  if (rightSpeed >= 0) {
    analogWrite(RIGHT_RPWM, constrain(rightSpeed, 0, MAX_PWM));
    analogWrite(RIGHT_LPWM, 0);
  } else {
    analogWrite(RIGHT_RPWM, 0);
    analogWrite(RIGHT_LPWM, constrain(-rightSpeed, 0, MAX_PWM));
  }
}

// ==========================================
// SENSOR READING & PID COMPUTATION
// ==========================================
const float SENSOR_EXPONENT = 2.5;

void readSensors() {
  const int sensorPins[NUM_SENSORS] = {IR0, IR1, IR2, IR3, IR4, IR5, IR6, IR7};
  
  for (int i = 0; i < NUM_SENSORS; i++) {
    rawSensors[i] = analogRead(sensorPins[i]);
    int clamped = constrain(rawSensors[i], whiteValues[i], blackValues[i]);
    float ratio = (float)(clamped - whiteValues[i]) / (float)(blackValues[i] - whiteValues[i]);
    float curvedRatio = pow(ratio, SENSOR_EXPONENT);
    // normalizedSensors[i] = map(clamped, whiteValues[i], blackValues[i], 0, 1000);
    normalizedSensors[i] = (int)(curvedRatio * 1000.0);
  }
}

int getLinePosition() {
  readSensors();
  
  long weightedSum = 0;
  long totalValue  = 0;

  for (int i = 0; i < NUM_SENSORS; i++) {
    weightedSum += (long)normalizedSensors[i] * (i * 1000);
    totalValue += normalizedSensors[i];
  }

  if (totalValue < 200) {
    return (lastLineError < 0) ? 0 : 7000;
  }

  return weightedSum / totalValue;
}

void followLinePID() {
  int position = getLinePosition();
  
  // Setpoint is center (3500)
  int error = position - 3500;

  lineIntegral += error;
  lineIntegral = constrain(lineIntegral, -3000, 3000);
  
  int derivative = error - lastLineError;
  lastLineError  = error;

  // Steering Adjustment Value
  float adjustment = (Kp * error) + (Ki * lineIntegral) + (Kd * derivative);

  // Correct motor control assignment:
  // When line moves RIGHT (position > 3500, error > 0), left motor speeds up & right slows down to turn RIGHT.
  int leftMotorSpeed  = BASE_PWM - adjustment; // change to change the steering direction
  int rightMotorSpeed = BASE_PWM + adjustment;

  setMotorSpeeds(leftMotorSpeed, rightMotorSpeed);
}

// ==========================================
// SETUP & MAIN LOOP
// ==========================================
void setup() {
  Serial.begin(115200);

  // Motor Driver Pins Setup
  pinMode(RIGHT_RPWM, OUTPUT);
  pinMode(RIGHT_LPWM, OUTPUT);
  pinMode(LEFT_RPWM, OUTPUT);
  pinMode(LEFT_LPWM, OUTPUT);

  // Encoder Pins Setup
  pinMode(LEFT_ENC_A, INPUT_PULLUP);
  pinMode(LEFT_ENC_B, INPUT_PULLUP);
  pinMode(RIGHT_ENC_A, INPUT_PULLUP);
  pinMode(RIGHT_ENC_B, INPUT_PULLUP);

  // Enable IR Array Power Pin
  pinMode(IR_EN, OUTPUT);
  digitalWrite(IR_EN, HIGH);

  // Encoder Interrupt Attachments (Updated Pins 18 & 2)
  attachInterrupt(digitalPinToInterrupt(LEFT_ENC_A), ISR_left_encoder, RISING);
  attachInterrupt(digitalPinToInterrupt(RIGHT_ENC_A), ISR_right_encoder, RISING);

  delay(2000); // 2-second startup safety delay
}

void loop() {
  followLinePID();
  delay(5);
}