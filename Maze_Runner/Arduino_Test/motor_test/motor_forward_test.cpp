// Motor Forward and Rotate 90 Degrees Test
// Pin Definitions for Left Driver & Encoder
const int LEFT_RPWM = 4;
const int LEFT_LPWM = 6;
const int LEFT_ENC_A = 2; // Interrupt 0
const int LEFT_ENC_B = 3; // Interrupt 1

// Pin Definitions for Right Driver & Encoder
const int RIGHT_EN = 7;
const int RIGHT_RPWM = 9; //chnaged in the code as wheels rotate oposite
const int RIGHT_LPWM = 8;
const int RIGHT_ENC_A = 18; // Interrupt 5
const int RIGHT_ENC_B = 19; // Interrupt 4

// Global Pulse Counters
volatile long left_pulses = 0;
volatile long right_pulses = 0;

// Motor Speed
const int BASE_PWM = 70;

// Calibrated Calibration Values
// 288 ticks = 25 cm (1 cell) -> 11.52 ticks/cm
const float TICKS_PER_CELL_25CM = 288.0;
const float TICKS_PER_90_DEG = 116.5;  // Average of 116-117 ticks
const float TICKS_PER_180_DEG = 232.5; // Average of 232-233 ticks

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

void setup()
{
    Serial.begin(115200);

    // Driver Control Pins
    pinMode(LEFT_RPWM, OUTPUT);
    pinMode(LEFT_LPWM, OUTPUT);
    pinMode(RIGHT_EN, OUTPUT);
    pinMode(RIGHT_RPWM, OUTPUT);
    pinMode(RIGHT_LPWM, OUTPUT);

    digitalWrite(RIGHT_EN, HIGH);

    // Encoder Pins
    pinMode(LEFT_ENC_A, INPUT_PULLUP);
    pinMode(LEFT_ENC_B, INPUT_PULLUP);
    pinMode(RIGHT_ENC_A, INPUT_PULLUP);
    pinMode(RIGHT_ENC_B, INPUT_PULLUP);

    // Attach Interrupts
    attachInterrupt(digitalPinToInterrupt(LEFT_ENC_A), ISR_left_encoder, RISING);
    attachInterrupt(digitalPinToInterrupt(RIGHT_ENC_A), ISR_right_encoder, RISING);

    delay(2000); // 2-second startup safety delay
}

void loop()
{
    Serial.println("Moving 1 Cell Forward (25 cm)...");
    moveCells(1.0); // Move 1 maze cell (288 ticks)

    delay(500);

    Serial.println("Turning Right 90 Degrees...");
    turnRight90(); // Turn right 90 deg (~116.5 ticks)

    delay(500);

    Serial.println("Moving 1 Cell Forward (25 cm)...");
    moveCells(1.0);

    delay(500);

    Serial.println("Performing 180 Degree U-Turn...");
    turn180(); // Turn 180 deg (~232.5 ticks)

    // Halt execution after test run
    stopMotors();
    Serial.println("Sequence Finished!");
    while (1)
        ;
}

// Resets encoder counters atomically
void resetEncoders()
{
    noInterrupts();
    left_pulses = 0;
    right_pulses = 0;
    interrupts();
}

// Halts motor outputs
void stopMotors()
{
    analogWrite(LEFT_RPWM, 0);
    analogWrite(LEFT_LPWM, 0);
    analogWrite(RIGHT_RPWM, 0);
    analogWrite(RIGHT_LPWM, 0);
}

// Straight-line cell movement using PI controller
void moveCells(float num_cells)
{
    resetEncoders();

    long target_ticks = num_cells * TICKS_PER_CELL_25CM;

    double kp = 1.5;
    double ki = 0.05;
    double integral_error = 0;

    while (true)
    {
        noInterrupts();
        long current_left = left_pulses;
        long current_right = right_pulses;
        interrupts();

        // Check if average pulse count reaches target
        if ((current_left + current_right) / 2 >= target_ticks)
        {
            break;
        }

        double error = current_left - current_right;
        integral_error += error;
        integral_error = constrain(integral_error, -300, 300);

        double correction = (kp * error) + (ki * integral_error);

        int left_pwm = constrain(BASE_PWM - correction, 0, 255);
        int right_pwm = constrain(BASE_PWM + correction, 0, 255);

        analogWrite(LEFT_RPWM, left_pwm);
        analogWrite(LEFT_LPWM, 0);
        analogWrite(RIGHT_RPWM, right_pwm);
        analogWrite(RIGHT_LPWM, 0);

        delay(10);
    }

    stopMotors();
}

// Pivot Turn Right 90° (Left wheel forward, Right wheel reverse)
void turnRight90()
{
    resetEncoders();

    while (true)
    {
        noInterrupts();
        long current_left = left_pulses;
        interrupts();

        if (current_left >= TICKS_PER_90_DEG)
        {
            break;
        }

        analogWrite(LEFT_RPWM, BASE_PWM);
        analogWrite(LEFT_LPWM, 0);

        analogWrite(RIGHT_RPWM, 0);
        analogWrite(RIGHT_LPWM, BASE_PWM);

        delay(10);
    }

    stopMotors();
}

// Pivot Turn 180° U-Turn
void turn180()
{
    resetEncoders();

    while (true)
    {
        noInterrupts();
        long current_left = left_pulses;
        interrupts();

        if (current_left >= TICKS_PER_180_DEG)
        {
            break;
        }

        analogWrite(LEFT_RPWM, BASE_PWM);
        analogWrite(LEFT_LPWM, 0);

        analogWrite(RIGHT_RPWM, 0);
        analogWrite(RIGHT_LPWM, BASE_PWM);

        delay(10);
    }

    stopMotors();
}