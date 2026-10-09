#include <Arduino.h>
#include <util/atomic.h>
#include "config.h"

volatile long testLeftTicks = 0;
volatile long testRightTicks = 0;

void testLeftEncoderISR() {
  if (digitalRead(PIN_LEFT_ENC_B) == HIGH) {
    --testLeftTicks;
  } else {
    ++testLeftTicks;
  }
}

void testRightEncoderISR() {
  if (digitalRead(PIN_RIGHT_ENC_B) == HIGH) {
    ++testRightTicks;
  } else {
    --testRightTicks;
  }
}

void printEncoderCounts() {
  long leftTicks;
  long rightTicks;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    leftTicks = testLeftTicks;
    rightTicks = testRightTicks;
  }

  Serial.print(F("left_ticks="));
  Serial.print(leftTicks);
  Serial.print(F(" right_ticks="));
  Serial.print(rightTicks);
  Serial.print(F(" configured_counts_per_rev="));
  Serial.println(ENCODER_COUNTS_PER_REV);
}

void resetEncoderCounts() {
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    testLeftTicks = 0;
    testRightTicks = 0;
  }
  Serial.println(F("Counts reset. Rotate one wheel exactly one full revolution, then send P."));
}

void setup() {
  Serial.begin(115200);

  pinMode(PIN_LEFT_ENC_A, INPUT_PULLUP);
  pinMode(PIN_LEFT_ENC_B, INPUT_PULLUP);
  pinMode(PIN_RIGHT_ENC_A, INPUT_PULLUP);
  pinMode(PIN_RIGHT_ENC_B, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(PIN_LEFT_ENC_A), testLeftEncoderISR, RISING);
  attachInterrupt(digitalPinToInterrupt(PIN_RIGHT_ENC_A), testRightEncoderISR, RISING);

  Serial.println(F("Encoder tick test (motors are not driven)."));
  Serial.println(F("Lift the robot so wheels can turn freely."));
  Serial.println(F("Send Z to zero counts, rotate one wheel by hand exactly 1 revolution, send P to print."));
  Serial.println(F("Test left and right wheels separately. P prints signed counts; compare absolute count with configured 360."));
  resetEncoderCounts();
}

void loop() {
  if (!Serial.available()) return;

  const char command = (char)Serial.read();
  if (command == 'z' || command == 'Z') {
    resetEncoderCounts();
  } else if (command == 'p' || command == 'P') {
    printEncoderCounts();
  }
}
