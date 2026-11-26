#include <Arduino.h>
#include "config.h"

// ==========================================
// PIN DEFINITIONS -> all taken from config.h
// ==========================================
// IR sensors : IR_PINS[0..7] (index 0 = robot's right most), PIN_IR_EN
// Motors     : PIN_RIGHT_RPWM, PIN_RIGHT_LPWM, PIN_LEFT_RPWM, PIN_LEFT_LPWM
//
// Serial commands (type a letter + ENTER, 115200 baud):
//   c  calibrate: slide the robot by hand across the line for 6 s (motors off)
//   r  live view: raw / normalised / position / motor speeds (motors off)
//   g  run the line follower (any key stops)
//   o  go back to the original calibration values below
//   p  print the calibration values in use

#define NUM_SENSORS NUM_IR_SENSORS

// ==========================================
// NEW CALIBRATION VALUES FROM IMAGE
// ==========================================
const int ORIGINAL_WHITE[NUM_SENSORS] = {805, 722, 731, 733, 720, 727, 743, 740};
const int ORIGINAL_BLACK[NUM_SENSORS] = {1022, 1017, 1020, 1019, 1017, 1016, 1018, 1017};
int whiteValues[NUM_SENSORS];   // values in use (start as the originals, 'c' replaces them)
int blackValues[NUM_SENSORS];

// Global Arrays & Variables
int rawSensors[NUM_SENSORS];
int normalizedSensors[NUM_SENSORS];

// ==========================================
// SPEED & PID TUNING PARAMETERS
// ==========================================
const int BASE_PWM = 95;    // Forward cruising speed
const int MAX_PWM  = 100;   // Overall Motor PWM limit

// PID Gains for smooth steering
float Kp = 0.025; // Proportional gain (Adjust between 0.015 - 0.025)
float Ki = 0.000; // Integral gain
float Kd = 0.053; // Derivative gain (Adjust between 0.050 - 0.100)

int lastLineError = 0;
float lineIntegral = 0;

// Last values computed by followLinePID (for the live view)
int lastPosition = 0;
int lastLeftSpeed = 0;
int lastRightSpeed = 0;

// ==========================================
// MOTOR DRIVER CONTROL FUNCTIONS
// ==========================================
void stopMotors() {
  analogWrite(PIN_LEFT_RPWM, 0);
  analogWrite(PIN_LEFT_LPWM, 0);
  analogWrite(PIN_RIGHT_RPWM, 0);
  analogWrite(PIN_RIGHT_LPWM, 0);
}

void setMotorSpeeds(int leftSpeed, int rightSpeed) {
  // Left Motor Direction & Speed
  if (leftSpeed >= 0) {
    analogWrite(PIN_LEFT_RPWM, constrain(leftSpeed, 0, MAX_PWM));
    analogWrite(PIN_LEFT_LPWM, 0);
  } else {
    analogWrite(PIN_LEFT_RPWM, 0);
    analogWrite(PIN_LEFT_LPWM, constrain(-leftSpeed, 0, MAX_PWM));
  }

  // Right Motor Direction & Speed
  if (rightSpeed >= 0) {
    analogWrite(PIN_RIGHT_RPWM, constrain(rightSpeed, 0, MAX_PWM));
    analogWrite(PIN_RIGHT_LPWM, 0);
  } else {
    analogWrite(PIN_RIGHT_RPWM, 0);
    analogWrite(PIN_RIGHT_LPWM, constrain(-rightSpeed, 0, MAX_PWM));
  }
}

// ==========================================
// SENSOR READING & PID COMPUTATION
// ==========================================
const float SENSOR_EXPONENT = 2.5;

void readSensors() {
  for (int i = 0; i < NUM_SENSORS; i++) {
    rawSensors[i] = analogRead(IR_PINS[i]);
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

// driveMotors = false computes everything but leaves the motors off (live view)
void followLinePID(bool driveMotors = true) {
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

  lastPosition   = position;
  lastLeftSpeed  = leftMotorSpeed;
  lastRightSpeed = rightMotorSpeed;

  if (driveMotors) setMotorSpeeds(leftMotorSpeed, rightMotorSpeed);
}

// ==========================================
// CALIBRATION / TEST HELPERS
// ==========================================
void flushSerial() {
  delay(20);
  while (Serial.available()) Serial.read();
}

bool keyPressed() {
  if (!Serial.available()) return false;
  flushSerial();
  return true;
}

void printArray(const char *name, const int *v) {
  Serial.print("const int ");
  Serial.print(name);
  Serial.print("[8] = {");
  for (int i = 0; i < NUM_SENSORS; i++) {
    Serial.print(v[i]);
    if (i < NUM_SENSORS - 1) Serial.print(", ");
  }
  Serial.println("};");
}

void useOriginalCalibration() {
  for (int i = 0; i < NUM_SENSORS; i++) {
    whiteValues[i] = ORIGINAL_WHITE[i];
    blackValues[i] = ORIGINAL_BLACK[i];
  }
}

void printCalibration() {
  Serial.println();
  printArray("whiteValues", whiteValues);
  printArray("blackValues", blackValues);
  Serial.println();
}

void calibrate() {
  stopMotors();
  int newWhite[NUM_SENSORS], newBlack[NUM_SENSORS];
  for (int i = 0; i < NUM_SENSORS; i++) { newWhite[i] = 1023; newBlack[i] = 0; }

  Serial.println();
  Serial.println("CALIBRATION: slide the robot slowly side to side so EVERY sensor");
  Serial.println("passes over both the white floor and the black line. Starting in 2 s...");
  delay(2000);
  Serial.print("Sampling 6 s ");

  unsigned long start = millis();
  int n = 0;
  while (millis() - start < 6000) {
    for (int i = 0; i < NUM_SENSORS; i++) {
      int v = analogRead(IR_PINS[i]);
      if (v < newWhite[i]) newWhite[i] = v;
      if (v > newBlack[i]) newBlack[i] = v;
    }
    if (++n % 60 == 0) Serial.print('.');
    delay(5);
  }
  Serial.println(" done");
  Serial.println();

  Serial.println("Sensor | White | Black | Range | Original W/B | Check");
  Serial.println("-------+-------+-------+-------+--------------+------");
  for (int i = 0; i < NUM_SENSORS; i++) {
    int range = newBlack[i] - newWhite[i];
    int dW = newWhite[i] - ORIGINAL_WHITE[i];
    int dB = newBlack[i] - ORIGINAL_BLACK[i];
    char line[80];
    snprintf(line, sizeof(line), "  %d    | %5d | %5d | %5d | %4d / %4d  | ",
             i, newWhite[i], newBlack[i], range, ORIGINAL_WHITE[i], ORIGINAL_BLACK[i]);
    Serial.print(line);
    if (range < 200)                       Serial.println("WEAK: not swept over line, or bad sensor");
    else if (abs(dW) > 80 || abs(dB) > 80) Serial.println("original value is OUTDATED");
    else                                   Serial.println("ok");
  }

  for (int i = 0; i < NUM_SENSORS; i++) {
    whiteValues[i] = newWhite[i];
    blackValues[i] = newBlack[i];
  }
  Serial.println();
  Serial.println("Now using the NEW values ('o' = back to original). Paste these to keep them:");
  printCalibration();
}

void liveView() {
  stopMotors();
  lastLineError = 0;
  lineIntegral = 0;
  Serial.println();
  Serial.println("LIVE VIEW (motors off, any key stops). Sensor 0 (right most) printed first.");
  Serial.println("Put the line under each side: the slower wheel must be on the line's side.");
  while (!keyPressed()) {
    followLinePID(false);

    Serial.print("raw ");
    for (int i = 0; i < NUM_SENSORS; i++) { Serial.print(rawSensors[i]); Serial.print(' '); }
    Serial.print("| norm ");
    for (int i = 0; i < NUM_SENSORS; i++) { Serial.print(normalizedSensors[i]); Serial.print(' '); }
    Serial.print("| ");
    for (int i = 0; i < NUM_SENSORS; i++) Serial.print(normalizedSensors[i] > 500 ? '#' : '.');
    Serial.print(" | pos "); Serial.print(lastPosition);
    Serial.print(" | L "); Serial.print(lastLeftSpeed);
    Serial.print(" R "); Serial.print(lastRightSpeed);
    if (lastLeftSpeed < lastRightSpeed - 2)      Serial.println("  -> turns LEFT");
    else if (lastRightSpeed < lastLeftSpeed - 2) Serial.println("  -> turns RIGHT");
    else                                         Serial.println("  -> straight");
    delay(150);
  }
  Serial.println("stopped");
}

void runFollower() {
  Serial.println();
  Serial.println("LINE FOLLOW: place the robot on the line. Starting in 2 s (any key stops).");
  delay(2000); // 2-second startup safety delay
  flushSerial();
  lastLineError = 0;
  lineIntegral = 0;
  while (!keyPressed()) {
    followLinePID();
    delay(5);
  }
  stopMotors();
  Serial.println("stopped");
}

void printMenu() {
  Serial.println("Commands: c=calibrate  r=live view  g=run follower  o=original cal  p=print cal");
}

// ==========================================
// SETUP & MAIN LOOP
// ==========================================
void setup() {
  Serial.begin(115200);

  // Motor Driver Pins Setup
  pinMode(PIN_RIGHT_RPWM, OUTPUT);
  pinMode(PIN_RIGHT_LPWM, OUTPUT);
  pinMode(PIN_LEFT_RPWM, OUTPUT);
  pinMode(PIN_LEFT_LPWM, OUTPUT);
  stopMotors();

  // Enable IR Array Power Pin
  pinMode(PIN_IR_EN, OUTPUT);
  digitalWrite(PIN_IR_EN, HIGH);

  useOriginalCalibration();
  delay(500);

  Serial.println();
  Serial.println("=== IR array test ===");
  printCalibration();
  printMenu();
}

void loop() {
  if (!Serial.available()) return;
  char cmd = Serial.read();
  flushSerial();

  switch (cmd) {
    case 'c': calibrate(); break;
    case 'r': liveView(); break;
    case 'g': runFollower(); break;
    case 'o': useOriginalCalibration(); printCalibration(); break;
    case 'p': printCalibration(); break;
    default: break;
  }
  printMenu();
}
