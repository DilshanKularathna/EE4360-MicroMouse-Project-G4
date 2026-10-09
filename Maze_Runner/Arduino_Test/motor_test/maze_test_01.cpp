#include <Arduino.h>
#include "config.h"

// ==========================================
// GLOBALS & PID TUNING
// ==========================================
const int BASE_PWM = 30;    // Forward cruising speed
const int MAX_PWM  = 100;   // Overall Motor PWM limit
const int TURN_PWM = 45;    // Speed for turning

// PID Gains for straight-line encoder syncing
float Kp = 0.5;  
float Kd = 0.1;  
int lastEncoderError = 0;

volatile long left_pulses  = 0;
volatile long right_pulses = 0;

// ==========================================
// INTERRUPT SERVICE ROUTINES
// ==========================================
void ISR_left_encoder() {
  if (digitalRead(PIN_LEFT_ENC_B) == HIGH) left_pulses++;
  else left_pulses--;
}

void ISR_right_encoder() {
  if (digitalRead(PIN_RIGHT_ENC_B) == HIGH) right_pulses--; 
  else right_pulses++; 
}

// ==========================================
// MOTOR DRIVER CONTROL 
// ==========================================
void stopMotors() {
  analogWrite(PIN_LEFT_RPWM, 0);
  analogWrite(PIN_LEFT_LPWM, 0);
  analogWrite(PIN_RIGHT_RPWM, 0);
  analogWrite(PIN_RIGHT_LPWM, 0);
}

void setMotorSpeeds(int leftSpeed, int rightSpeed) {
  // Left Motor
  if (leftSpeed >= 0) {
    analogWrite(PIN_LEFT_RPWM, constrain(leftSpeed, 0, MAX_PWM));
    analogWrite(PIN_LEFT_LPWM, 0);
  } else {
    analogWrite(PIN_LEFT_RPWM, 0);
    analogWrite(PIN_LEFT_LPWM, constrain(-leftSpeed, 0, MAX_PWM));
  }

  // Right Motor
  if (rightSpeed >= 0) {
    analogWrite(PIN_RIGHT_RPWM, constrain(rightSpeed, 0, MAX_PWM));
    analogWrite(PIN_RIGHT_LPWM, 0);
  } else {
    analogWrite(PIN_RIGHT_RPWM, 0);
    analogWrite(PIN_RIGHT_LPWM, constrain(-rightSpeed, 0, MAX_PWM));
  }
}

// ==========================================
// MOVEMENT & SENSORS
// ==========================================
void moveForward25cm() {
  left_pulses = 0;
  right_pulses = 0;
  lastEncoderError = 0;
  
  // 288 ticks = 25 cm[cite: 1]
  while (((abs(left_pulses) + abs(right_pulses)) / 2) < TICKS_PER_CELL_25CM) {
    // Error is the difference between left and right wheel travel distance
    int error = abs(left_pulses) - abs(right_pulses);
    
    int derivative = error - lastEncoderError;
    lastEncoderError = error;

    // Calculate adjustment
    float adjustment = (Kp * error) + (Kd * derivative);

    // If left is ahead (positive error), left motor slows down and right speeds up
    int leftMotorSpeed  = BASE_PWM - adjustment;
    int rightMotorSpeed = BASE_PWM + adjustment;

    setMotorSpeeds(leftMotorSpeed, rightMotorSpeed);
    delay(5);
  }
  
  stopMotors();
}

void turn90(bool turnRight) {
  left_pulses = 0;
  right_pulses = 0;
  
  // Target is ~116 ticks for a 90-degree turn[cite: 1]
  int targetTicks = TICKS_PER_90_DEG; 
  
  while ((abs(left_pulses) + abs(right_pulses)) / 2 < targetTicks) {
    if (turnRight) {
      setMotorSpeeds(TURN_PWM, -TURN_PWM); // Left forward, Right backward
    } else {
      setMotorSpeeds(-TURN_PWM, TURN_PWM); // Left backward, Right forward
    }
    delay(5);
  }
  
  stopMotors();
}

long getUltrasonicDistance(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  
  // 30ms timeout (approx 5 meters)
  long duration = pulseIn(echoPin, HIGH, 30000); 
  
  if (duration == 0) return 999; // No echo (too far)
  return duration * 0.034 / 2;   // Convert to cm
}

void scanAndDecide() {
  // Allow robot to settle before pinging
  delay(200); 
  
  long distFront = getUltrasonicDistance(PIN_ULTRA_FRONT_TRIG, PIN_ULTRA_FRONT_ECHO);
  long distLeft  = getUltrasonicDistance(PIN_ULTRA_LEFT_TRIG, PIN_ULTRA_LEFT_ECHO);
  long distRight = getUltrasonicDistance(PIN_ULTRA_RIGHT_TRIG, PIN_ULTRA_RIGHT_ECHO);

  Serial.print("Distances -> L: "); Serial.print(distLeft);
  Serial.print(" F: "); Serial.print(distFront);
  Serial.print(" R: "); Serial.println(distRight);

  // Threshold to determine if a wall is present (e.g., 20 cm)
  const int WALL_THRESHOLD = 20; 

  if (distLeft > WALL_THRESHOLD) {
    Serial.println("Turning Left (No Wall)");
    turn90(false);
  } 
  else if (distRight > WALL_THRESHOLD) {
    Serial.println("Turning Right (No Wall)");
    turn90(true);
  }
  else if (distFront > WALL_THRESHOLD) {
    Serial.println("Path Clear Forward (No Turn Needed)");
  }
  else {
    Serial.println("Dead End! (Turning 180)");
    turn90(true); 
    turn90(true);
  }
}

// ==========================================
// SETUP & MAIN LOOP
// ==========================================
void setup() {
  Serial.begin(115200);

  // Motor Pins
  pinMode(PIN_LEFT_RPWM, OUTPUT);
  pinMode(PIN_LEFT_LPWM, OUTPUT);
  pinMode(PIN_RIGHT_RPWM, OUTPUT);
  pinMode(PIN_RIGHT_LPWM, OUTPUT);

  // Encoder Pins
  pinMode(PIN_LEFT_ENC_A, INPUT_PULLUP);
  pinMode(PIN_LEFT_ENC_B, INPUT_PULLUP);
  pinMode(PIN_RIGHT_ENC_A, INPUT_PULLUP);
  pinMode(PIN_RIGHT_ENC_B, INPUT_PULLUP);

  // Ultrasonic Pins
  pinMode(PIN_ULTRA_FRONT_TRIG, OUTPUT);
  pinMode(PIN_ULTRA_FRONT_ECHO, INPUT);
  pinMode(PIN_ULTRA_LEFT_TRIG, OUTPUT);
  pinMode(PIN_ULTRA_LEFT_ECHO, INPUT);
  pinMode(PIN_ULTRA_RIGHT_TRIG, OUTPUT);
  pinMode(PIN_ULTRA_RIGHT_ECHO, INPUT);

  // Attach Interrupts
  attachInterrupt(digitalPinToInterrupt(PIN_LEFT_ENC_A), ISR_left_encoder, RISING);
  attachInterrupt(digitalPinToInterrupt(PIN_RIGHT_ENC_A), ISR_right_encoder, RISING);

  delay(2000); 
}

void loop() {
  moveForward25cm();
  scanAndDecide();
  
  // Pause to prevent continuous running during testing
  while(true) { delay(100); } 
}