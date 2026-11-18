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

// Battery Sensor Pin
#define BATTERY_PIN A7 

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
// VOLTAGE COMPENSATION & BATTERY SAFETY
// ==========================================
// IMPORTANT: Configure these 4 values based on your exact Battery/Resistor choice
const float V_MAX = 12.0;             // Max voltage of your pack fully charged (e.g., 12.6V for 3S, 16.8V for 4S)
const float CRITICAL_LOW_V = 9.9;    // Safety cutoff voltage threshold (e.g., 3.3V per cell minimum)
const float R1 = 10000.0;            // TBD:10k Ohm (Top resistor connected to Battery +) (Planning to use Potentiometer)
const float R2 = 4700.0;             // TBD:4.7k Ohm (Bottom resistor connected to GND)
const float V_REF = 5.0;             // Regulated 5V supplying your Arduino

float readBatteryVoltage() {
  int rawADC = analogRead(BATTERY_PIN);
  float pinVoltage = (rawADC * V_REF) / 1023.0;
  float dividerFactor = (R1 + R2) / R2;
  return pinVoltage * dividerFactor;
}

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
  // [SAFETY CUTOFF] Read live voltage and halt system if low
  float vCurrent = readBatteryVoltage();
  if (vCurrent < CRITICAL_LOW_V) {
    stopMotors();
    while (1) {
      // Endless safety trap to protect Li-ion from cell degradation
      // Add optional alert buzzer or flashing LED routine here
    }
  }

  // Calculate scaling multiplier based on voltage ratio
  // Safety guard prevents division-by-zero or massive values if wire disconnects
  if (vCurrent < 5.0) vCurrent = 5.0; 
  float compensationRatio = V_MAX / vCurrent;

  int position = getLinePosition();
  int error = position - 3500;

  lineIntegral += error;
  lineIntegral = constrain(lineIntegral, -3000, 3000);
  
  int derivative = error - lastLineError;
  lastLineError  = error;

  // Steering Adjustment Value
  float adjustment = (Kp * error) + (Ki * lineIntegral) + (Kd * derivative);

  // Apply raw intended base speed changes + PID math adjustments
  int rawLeftSpeed  = BASE_PWM + adjustment;
  int rawRightSpeed = BASE_PWM - adjustment;

  // [VOLTAGE COMPENSATION STEP] Scale up the PWM dynamically before sending to drivers
  int compensatedLeftSpeed  = (int)(rawLeftSpeed * compensationRatio);
  int compensatedRightSpeed = (int)(rawRightSpeed * compensationRatio);

  setMotorSpeeds(compensatedLeftSpeed, compensatedRightSpeed);
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

  // Encoder Interrupt Attachments
  attachInterrupt(digitalPinToInterrupt(LEFT_ENC_A), ISR_left_encoder, RISING);
  attachInterrupt(digitalPinToInterrupt(RIGHT_ENC_A), ISR_right_encoder, RISING);

  delay(2000); // 2-second startup safety delay
}

void loop() {
  followLinePID();
  delay(5);
}
