// --- PIN DEFINITIONS ---
// Left Motor Driver (Driver 1)
const int L_RPWM = 2; // Forward PWM pin
const int L_LPWM = 3; // Reverse PWM pin

// Right Motor Driver (Driver 2)
const int R_RPWM = 6; // Forward PWM pin
const int R_LPWM = 7; // Reverse PWM pin

// Enable Pins (Assumes R_EN and L_EN are shorted together per driver)
const int L_EN = 4;   // Left Driver Enable pin
const int R_EN = 5;   // Right Driver Enable pin

void setup() {
  // Set all motor pins as outputs
  pinMode(L_RPWM, OUTPUT);
  pinMode(L_LPWM, OUTPUT);
  pinMode(R_RPWM, OUTPUT);
  pinMode(R_LPWM, OUTPUT);
  pinMode(L_EN,   OUTPUT);
  pinMode(R_EN,   OUTPUT);

  // Enable both motor drivers
  digitalWrite(L_EN, HIGH);
  digitalWrite(R_EN, HIGH);
}

// Helper function to drive individual motors
// Speed range: -255 (Full Reverse) to 255 (Full Forward)
void setMotors(int leftSpeed, int rightSpeed) {
  // Left Motor Control
  if (leftSpeed >= 0) {
    analogWrite(L_RPWM, leftSpeed);
    analogWrite(L_LPWM, 0);
  } else {
    analogWrite(L_RPWM, 0);
    analogWrite(L_LPWM, -leftSpeed);
  }

  // Right Motor Control
  if (rightSpeed >= 0) {
    analogWrite(R_RPWM, rightSpeed);
    analogWrite(R_LPWM, 0);
  } else {
    analogWrite(R_RPWM, 0);
    analogWrite(R_LPWM, -rightSpeed);
  }
}

void loop() {
  // 1. Move Forward (Moderate Speed)
  setMotors(150, 150);
  delay(2000);

  // 2. Full Stop
  setMotors(0, 0);
  delay(1000);

  // 3. Move Reverse
  setMotors(-150, -150);
  delay(2000);

  // 4. Full Stop
  setMotors(0, 0);
  delay(1000);

  // 5. Spin Turn Right (Left forward, Right reverse)
  setMotors(120, -120);
  delay(1500);

  // 6. Full Stop
  setMotors(0, 0);
  delay(1000);

  // 7. Spin Turn Left (Left reverse, Right forward)
  setMotors(-120, 120);
  delay(1500);

  // 8. Full Stop before repeating loop
  setMotors(0, 0);
  delay(2000);
}