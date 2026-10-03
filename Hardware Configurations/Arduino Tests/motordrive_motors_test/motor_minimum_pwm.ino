#include <Arduino.h>
// Test the minimum speed input for wheel rotation (adjust forward, left, right)
// Motor Driver Pins
const int RPWM = 5;
const int LPWM = 6;
const int L_EN = 7;
const int R_EN = 8;

// Encoder Pin (Must be an interrupt pin: Pin 2 or Pin 3 on Uno/Nano)
const int ENCODER_A = 2;

// Volatile variable for ISR (Interrupt Service Routine)
volatile long encoderTicks = 0;

// Interrupt Service Routine: increments on every encoder pulse
void countEncoder() {
    encoderTicks++;
}

void setup() {
    Serial.begin(9600);

    // Motor driver pin outputs
    pinMode(RPWM, OUTPUT);
    pinMode(LPWM, OUTPUT);
    pinMode(R_EN, OUTPUT);
    pinMode(L_EN, OUTPUT);

    // Enable motor driver outputs
    digitalWrite(R_EN, HIGH);
    digitalWrite(L_EN, HIGH);

    // Set PWM channels to 0 initially
    analogWrite(RPWM, 0);
    analogWrite(LPWM, 0);

    // Configure Encoder Pin with internal pullup
    pinMode(ENCODER_A, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(ENCODER_A), countEncoder, RISING);

    delay(2000);
    Serial.println("--- Starting Minimum PWM Dead-Zone Test ---");
}

void loop() {
    int minPwmFound = -1;

    for (int i = 0; i <= 255; i++) {
        // Reset encoder tick count prior to sampling interval
        noInterrupts();
        encoderTicks = 0;
        interrupts();

        // Apply test PWM value
        analogWrite(RPWM, i);
        analogWrite(LPWM, 0);

        // Wait 1 second as requested to allow full response
        delay(1000);

        // Read recorded encoder counts over the 1-second interval
        noInterrupts();
        long currentTicks = encoderTicks;
        interrupts();

        // Print progress to Serial Monitor
        Serial.print("PWM Value: ");
        Serial.print(i);
        Serial.print("  |  Encoder Ticks in 1s: ");
        Serial.println(currentTicks);

        // Check if movement detected (ticks > 0)
        if (currentTicks > 0) {
            minPwmFound = i;
            break; // Stop ramping up PWM
        }
    }

    // Stop motor immediately after finding threshold
    analogWrite(RPWM, 0);
    analogWrite(LPWM, 0);

    // Display Result
    Serial.println("-------------------------------------------");
    if (minPwmFound != -1) {
        Serial.print("SUCCESS: Minimum PWM required to rotate motor = ");
        Serial.println(minPwmFound);
    } else {
        Serial.println("ERROR: No movement detected across full PWM range (0-255).");
    }
    Serial.println("-------------------------------------------");

    // Halt program execution inside infinite loop
    while (true) {
        delay(1000);
    }
}