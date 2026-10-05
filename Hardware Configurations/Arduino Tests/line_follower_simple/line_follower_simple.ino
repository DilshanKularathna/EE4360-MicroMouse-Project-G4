// ============================================================
//  Simple Line Follower: BLACK line on WHITE floor
//  Sensor reading:  black surface = 0,  white surface = 1
// ============================================================

// ---------------- Motor Pins ----------------
const int LEFT_RPWM  = 4;
const int LEFT_LPWM  = 6;
const int RIGHT_EN   = 7;
const int RIGHT_RPWM = 8;
const int RIGHT_LPWM = 9;

// If your LEFT driver has its own enable pin on the Mega, put the pin here.
// Leave as -1 if it is tied to 5V.
const int LEFT_EN = -1;

// ---------------- IR Sensor Array ----------------
const int IR_ENABLE_PIN = 49;
const int NUM_SENSORS = 7;
const int irPins[NUM_SENSORS] = {47, 45, 43, 41, 39, 37, 35};  // index 0 = LEFTMOST

int s[NUM_SENSORS];   // 0 = black, 1 = white

// ---------------- Settings ----------------
const int BASE_PWM = 90;     // forward speed
const int MAX_PWM  = 150;    // safety limit
const float Kp = 25.0;       // steering strength
const float Kd = 12.0;       // damping (reduces wobble)

const unsigned long PRINT_INTERVAL_MS = 100;
unsigned long lastPrint = 0;

float lastError = 0;
int lastSide = 0;            // -1 = line was last on left, +1 = on right

// Values shared with the printer
int leftCmd = 0, rightCmd = 0;
const char* action = "STOP";

// ---------------- Motors ----------------
void setMotors(int leftSpeed, int rightSpeed) {
  leftSpeed  = constrain(leftSpeed,  -MAX_PWM, MAX_PWM);
  rightSpeed = constrain(rightSpeed, -MAX_PWM, MAX_PWM);
  leftCmd = leftSpeed;
  rightCmd = rightSpeed;

  if (leftSpeed >= 0) {
    analogWrite(LEFT_RPWM, leftSpeed);
    analogWrite(LEFT_LPWM, 0);
  } else {
    analogWrite(LEFT_RPWM, 0);
    analogWrite(LEFT_LPWM, -leftSpeed);
  }

  if (rightSpeed >= 0) {
    analogWrite(RIGHT_RPWM, rightSpeed);
    analogWrite(RIGHT_LPWM, 0);
  } else {
    analogWrite(RIGHT_RPWM, 0);
    analogWrite(RIGHT_LPWM, -rightSpeed);
  }
}

void stopMotors() { setMotors(0, 0); }

// ---------------- Serial Print ----------------
const char* wheelText(int speed) {
  if (speed > 0) return "FWD";
  if (speed < 0) return "REV";
  return "STOP";
}

void printStatus() {
  if (millis() - lastPrint < PRINT_INTERVAL_MS) return;
  lastPrint = millis();

  // IR sensors: 0 = black, 1 = white  (left -> right)
  Serial.print("IR: ");
  for (int i = 0; i < NUM_SENSORS; i++) {
    Serial.print(s[i]);
    Serial.print(" ");
  }

  // What the robot is doing
  Serial.print("| ");
  Serial.print(action);

  // Wheels: direction and PWM value
  Serial.print(" | L: ");
  Serial.print(wheelText(leftCmd));
  Serial.print(" ");
  Serial.print(abs(leftCmd));

  Serial.print(" | R: ");
  Serial.print(wheelText(rightCmd));
  Serial.print(" ");
  Serial.println(abs(rightCmd));
}

// ---------------- Setup ----------------
void setup() {
  Serial.begin(115200);

  pinMode(LEFT_RPWM, OUTPUT);
  pinMode(LEFT_LPWM, OUTPUT);
  pinMode(RIGHT_EN, OUTPUT);
  pinMode(RIGHT_RPWM, OUTPUT);
  pinMode(RIGHT_LPWM, OUTPUT);
  digitalWrite(RIGHT_EN, HIGH);

  if (LEFT_EN >= 0) {
    pinMode(LEFT_EN, OUTPUT);
    digitalWrite(LEFT_EN, HIGH);
  }

  pinMode(IR_ENABLE_PIN, OUTPUT);
  digitalWrite(IR_ENABLE_PIN, HIGH);   // turn IR emitters on

  for (int i = 0; i < NUM_SENSORS; i++) {
    pinMode(irPins[i], INPUT);
  }

  stopMotors();
  Serial.println("Place robot on the black line. Starting in 3 s...");
  Serial.println("IR: 0 = black, 1 = white  (left -> right)");
  delay(3000);
}

// ---------------- Main Loop ----------------
void loop() {
  // 1. Read sensors (0 = black, 1 = white)
  for (int i = 0; i < NUM_SENSORS; i++) {
    s[i] = digitalRead(irPins[i]);
  }

  // 2. Find where the black line is (weighted average of sensors seeing black)
  const float center = (NUM_SENSORS - 1) / 2.0;
  float sum = 0;
  int count = 0;
  for (int i = 0; i < NUM_SENSORS; i++) {
    if (s[i] == 0) {                 // black detected
      sum += (i - center);           // -3 .. +3
      count++;
    }
  }

  // 3. Line lost (all white): spin toward the side where it was last seen
  if (count == 0) {
    if (lastSide <= 0) {
      action = "SEARCH LEFT";
      setMotors(-BASE_PWM, BASE_PWM);
    } else {
      action = "SEARCH RIGHT";
      setMotors(BASE_PWM, -BASE_PWM);
    }
    printStatus();
    return;
  }

  // 4. Steer with PD control
  float error = sum / count;         // negative = line on left, positive = line on right
  float derivative = error - lastError;
  lastError = error;
  if (error < 0) lastSide = -1;
  else if (error > 0) lastSide = 1;

  float correction = Kp * error + Kd * derivative;

  // Line on right (error > 0) -> left wheel faster, right wheel slower
  int leftSpeed  = BASE_PWM + correction;
  int rightSpeed = BASE_PWM - correction;

  if (error > -0.5 && error < 0.5) action = "FORWARD";
  else if (error < 0)              action = "TURN LEFT";
  else                             action = "TURN RIGHT";

  setMotors(leftSpeed, rightSpeed);
  printStatus();
}
