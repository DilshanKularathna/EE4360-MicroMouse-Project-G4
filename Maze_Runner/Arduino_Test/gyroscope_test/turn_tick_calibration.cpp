#include <Arduino.h>
#include <EEPROM.h>
#include <Wire.h>
#include <util/atomic.h>
#include "config.h"

// Mounting: MPU6050 +X points forward, +Y right, +Z down, so Z is the yaw axis.
const uint8_t MPU6050_ADDRESS = 0x68;
const uint8_t REG_PWR_MGMT_1 = 0x6B;
const uint8_t REG_GYRO_CONFIG = 0x1B;
const uint8_t REG_ACCEL_XOUT_H = 0x3B;
const float GYRO_LSB_PER_DPS = 131.0f;
const uint8_t GYRO_YAW_AXIS = MPU6050_YAW_AXIS_INDEX;
const unsigned long BIAS_CALIBRATION_MS = 10000UL;
const unsigned long SAMPLE_INTERVAL_US = 10000UL; // 100 Hz
const unsigned long START_DELAY_MS = 5000UL;
const float TURN_TOLERANCE_DEG = 15.0f;
const float TICK_TOLERANCE_PERCENT = 15.0f;
const int TURN_DUTY_FAST = 25;
const int TURN_DUTY_SLOW = 18;
const int EEPROM_ADDRESS = 0;
const uint16_t EEPROM_MAGIC = 0x4D54;
// Bump this when the yaw axis or calibration behavior changes so old EEPROM
// results from the X-axis experiment cannot be mistaken for Z-yaw results.
const uint8_t RECORD_VERSION = 5;

enum RunStatus : uint8_t { STATUS_EMPTY = 0, STATUS_RUNNING = 1, STATUS_COMPLETE = 2, STATUS_FAILED = 3 };

struct TurnCalibrationRecord {
  uint16_t magic;
  uint8_t version;
  uint8_t status;
  uint8_t failureStage; // 1=bias calibration, 2..5=the four turns
  int16_t gyroOffsetRaw[3];
  float requestedDegrees[4];
  float targetCorrectionDegrees[4];
  float targetDegrees[4];
  float measuredDegrees[4];
  long leftTicks[4];
  long rightTicks[4];
  uint8_t turnPassed[4];
};

volatile long leftTicks = 0;
volatile long rightTicks = 0;
int16_t gyroOffsets[3] = {0, 0, 0};
TurnCalibrationRecord resultRecord;

void leftEncoderTestISR() {
  if (digitalRead(PIN_LEFT_ENC_B) == HIGH) --leftTicks;
  else ++leftTicks;
}

void rightEncoderTestISR() {
  if (digitalRead(PIN_RIGHT_ENC_B) == HIGH) ++rightTicks;
  else --rightTicks;
}

bool writeRegister(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(MPU6050_ADDRESS);
  Wire.write(reg);
  Wire.write(value);
  return Wire.endTransmission() == 0;
}

bool readImu(int16_t values[7]) {
  for (uint8_t attempt = 0; attempt < 3; ++attempt) {
    Wire.beginTransmission(MPU6050_ADDRESS);
    Wire.write(REG_ACCEL_XOUT_H);
    if (Wire.endTransmission(false) == 0 &&
        Wire.requestFrom(MPU6050_ADDRESS, (uint8_t)14) == 14) {
      for (uint8_t i = 0; i < 7; ++i) {
        values[i] = (int16_t)((Wire.read() << 8) | Wire.read());
      }
      return true;
    }
    while (Wire.available()) Wire.read();
    delay(2);
  }
  return false;
}

void setWheelDuty(int leftPercent, int rightPercent) {
  leftPercent = constrain(leftPercent, -100, 100);
  rightPercent = constrain(rightPercent, -100, 100);
  const uint8_t leftPwm = (uint8_t)(abs(leftPercent) * 255L / 100L);
  const uint8_t rightPwm = (uint8_t)(abs(rightPercent) * 255L / 100L);

  analogWrite(PIN_LEFT_RPWM, leftPercent > 0 ? leftPwm : 0);
  analogWrite(PIN_LEFT_LPWM, leftPercent < 0 ? leftPwm : 0);
  analogWrite(PIN_RIGHT_RPWM, rightPercent > 0 ? rightPwm : 0);
  analogWrite(PIN_RIGHT_LPWM, rightPercent < 0 ? rightPwm : 0);
}

void stopMotors() {
  analogWrite(PIN_LEFT_RPWM, 0);
  analogWrite(PIN_LEFT_LPWM, 0);
  analogWrite(PIN_RIGHT_RPWM, 0);
  analogWrite(PIN_RIGHT_LPWM, 0);
}

void resetTicks() {
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    leftTicks = 0;
    rightTicks = 0;
  }
}

void copyTicks(long &left, long &right) {
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    left = leftTicks;
    right = rightTicks;
  }
}

bool calibrateGyroBias() {
  int32_t sums[3] = {0, 0, 0};
  uint16_t samples = 0;
  int16_t values[7];
  const unsigned long startMs = millis();
  unsigned long nextSampleUs = micros();

  Serial.println(F("Keep robot still for 10 seconds: calibrating gyro bias."));
  while (millis() - startMs < BIAS_CALIBRATION_MS) {
    if ((long)(micros() - nextSampleUs) >= 0) {
      if (!readImu(values)) return false;
      for (uint8_t axis = 0; axis < 3; ++axis) sums[axis] += values[4 + axis];
      ++samples;
      nextSampleUs += SAMPLE_INTERVAL_US;
    }
  }
  if (samples == 0) return false;
  for (uint8_t axis = 0; axis < 3; ++axis) {
    gyroOffsets[axis] = (int16_t)(sums[axis] / samples);
    resultRecord.gyroOffsetRaw[axis] = gyroOffsets[axis];
  }
  Serial.print(F("Gyro bias calibration complete; samples="));
  Serial.println(samples);
  return true;
}

bool runTurn(uint8_t index, float targetDegrees) {
  int16_t values[7];
  float angle = 0.0f;
  unsigned long lastSampleUs = micros();
  unsigned long nextSampleUs = lastSampleUs;
  const unsigned long startMs = millis();
  const unsigned long timeoutMs = (fabs(targetDegrees) > 90.0f) ? 12000UL : 8000UL;
  const float stopToleranceDegrees = 3.0f;
  int motorYawSign = 1;
  bool motorDirectionVerified = false;

  resetTicks();
  Serial.print(F("Starting turn "));
  Serial.print(index + 1);
  Serial.print(F(" target="));
  Serial.print(targetDegrees, 0);
  Serial.println(F(" degrees"));

  while (fabs(targetDegrees - angle) > stopToleranceDegrees &&
         millis() - startMs < timeoutMs) {
    if ((long)(micros() - nextSampleUs) < 0) continue;
    if (!readImu(values)) {
      stopMotors();
      copyTicks(resultRecord.leftTicks[index], resultRecord.rightTicks[index]);
      resultRecord.targetDegrees[index] = targetDegrees;
      resultRecord.measuredDegrees[index] = angle;
      return false;
    }

    const unsigned long sampleUs = micros();
    const float dt = (sampleUs - lastSampleUs) / 1000000.0f;
    lastSampleUs = sampleUs;
    const float rate = (values[4 + GYRO_YAW_AXIS] - gyroOffsets[GYRO_YAW_AXIS]) / GYRO_LSB_PER_DPS;
    angle += rate * dt;

    // Check the real motor-to-gyro sign after a small initial movement. If
    // wiring makes the first command turn the wrong way, reverse the command.
    if (!motorDirectionVerified && millis() - startMs >= 200UL && fabs(angle) >= 2.0f) {
      motorYawSign = (angle * targetDegrees > 0.0f) ? 1 : -1;
      motorDirectionVerified = true;
    }

    const float error = targetDegrees - angle;
    const int direction = ((error > 0.0f) ? 1 : -1) * motorYawSign;
    const int duty = (fabs(error) < 20.0f) ? TURN_DUTY_SLOW : TURN_DUTY_FAST;
    // +Z down makes positive yaw clockwise. Drive toward the signed angle;
    // this also corrects for swapped motor direction wiring.
    setWheelDuty(direction * duty, -direction * duty);
    nextSampleUs += SAMPLE_INTERVAL_US;
  }

  const bool reachedTarget = fabs(targetDegrees - angle) <= stopToleranceDegrees;
  stopMotors();

  // Include coast-down motion in the final gyro angle and encoder counts.
  const unsigned long settleStart = millis();
  while (millis() - settleStart < 300UL) {
    if (!readImu(values)) {
      copyTicks(resultRecord.leftTicks[index], resultRecord.rightTicks[index]);
      resultRecord.targetDegrees[index] = targetDegrees;
      resultRecord.measuredDegrees[index] = angle;
      return false;
    }
    const unsigned long sampleUs = micros();
    const float dt = (sampleUs - lastSampleUs) / 1000000.0f;
    lastSampleUs = sampleUs;
    const float rate = (values[4 + GYRO_YAW_AXIS] - gyroOffsets[GYRO_YAW_AXIS]) / GYRO_LSB_PER_DPS;
    angle += rate * dt;
    delay(5);
  }

  copyTicks(resultRecord.leftTicks[index], resultRecord.rightTicks[index]);
  resultRecord.targetDegrees[index] = targetDegrees;
  resultRecord.measuredDegrees[index] = angle;
  resultRecord.turnPassed[index] = (reachedTarget &&
      fabs(angle - targetDegrees) <= TURN_TOLERANCE_DEG) ? 1 : 0;

  Serial.print(F("Turn result angle="));
  Serial.print(angle, 1);
  Serial.print(F(" left_ticks="));
  Serial.print(resultRecord.leftTicks[index]);
  Serial.print(F(" right_ticks="));
  Serial.println(resultRecord.rightTicks[index]);
  return true;
}

float expectedTicksFor(uint8_t index) {
  return (index < 2) ? TICKS_PER_90_DEG : TICKS_PER_180_DEG;
}

void printSavedResults(const TurnCalibrationRecord &record) {
  if (record.magic != EEPROM_MAGIC || record.version != RECORD_VERSION) {
    Serial.println(F("No saved turn calibration is available."));
    return;
  }

  Serial.println();
  Serial.println(F("===== SAVED TURN / ENCODER CALIBRATION ====="));
  Serial.print(F("Status: "));
  if (record.status == STATUS_COMPLETE) Serial.println(F("COMPLETE"));
  else if (record.status == STATUS_FAILED) Serial.println(F("FAILED"));
  else if (record.status == STATUS_RUNNING) Serial.println(F("INTERRUPTED while running"));
  else Serial.println(F("UNKNOWN"));
  if (record.status == STATUS_FAILED) {
    Serial.print(F("Failure stage: "));
    if (record.failureStage == 1) Serial.println(F("gyro bias calibration; no motor turn should have started"));
    else if (record.failureStage >= 2 && record.failureStage <= 5) {
      const char *failedTurns[4] = {"90 CW", "90 CCW", "180 CW", "180 CCW"};
      Serial.print(failedTurns[record.failureStage - 2]);
      Serial.println(F(" gyro read; inspect MPU6050/I2C"));
    } else Serial.println(F("unknown"));
  }
  Serial.print(F("Stationary gyro offsets raw X/Y/Z: "));
  Serial.print(record.gyroOffsetRaw[0]);
  Serial.print(',');
  Serial.print(record.gyroOffsetRaw[1]);
  Serial.print(',');
  Serial.println(record.gyroOffsetRaw[2]);
  Serial.print(F("Stationary bias dps X/Y/Z: "));
  Serial.print(record.gyroOffsetRaw[0] / GYRO_LSB_PER_DPS, 3);
  Serial.print(',');
  Serial.print(record.gyroOffsetRaw[1] / GYRO_LSB_PER_DPS, 3);
  Serial.print(',');
  Serial.println(record.gyroOffsetRaw[2] / GYRO_LSB_PER_DPS, 3);
  Serial.println(F("Mount mapping used: +X=front, +Y=right, +Z=down; yaw gyro axis=Z."));

  const char *labels[4] = {"90 CW", "90 CCW", "180 CW", "180 CCW"};
  float normalizedCounts[4][2] = {{0, 0}, {0, 0}, {0, 0}, {0, 0}};
  for (uint8_t i = 0; i < 4; ++i) {
    const float expected = expectedTicksFor(i);
    const float leftCount = fabs((float)record.leftTicks[i]);
    const float rightCount = fabs((float)record.rightTicks[i]);
    const float measuredMagnitude = fabs(record.measuredDegrees[i]);
    const float requestedMagnitude = fabs(record.requestedDegrees[i]);
    if (measuredMagnitude > 0.1f) {
      normalizedCounts[i][0] = leftCount * requestedMagnitude / measuredMagnitude;
      normalizedCounts[i][1] = rightCount * requestedMagnitude / measuredMagnitude;
    }
    const float leftDeviation = (expected > 0.0f) ? (normalizedCounts[i][0] - expected) * 100.0f / expected : 0.0f;
    const float rightDeviation = (expected > 0.0f) ? (normalizedCounts[i][1] - expected) * 100.0f / expected : 0.0f;

    Serial.println();
    Serial.print(labels[i]);
    Serial.print(F(": requested_deg=")); Serial.print(record.requestedDegrees[i], 1);
    Serial.print(F(" applied_target_deg=")); Serial.print(record.targetDegrees[i], 1);
    Serial.print(F(" measured_deg=")); Serial.print(record.measuredDegrees[i], 1);
    Serial.print(F(" left_ticks=")); Serial.print(record.leftTicks[i]);
    Serial.print(F(" right_ticks=")); Serial.println(record.rightTicks[i]);
    Serial.print(F(" normalized_ticks_per_wheel="));
    Serial.print(normalizedCounts[i][0], 1);
    Serial.print(',');
    Serial.println(normalizedCounts[i][1], 1);
    Serial.print(F(" expected_ticks_per_wheel=")); Serial.print(expected, 1);
    Serial.print(F(" left_deviation_pct=")); Serial.print(leftDeviation, 1);
    Serial.print(F(" right_deviation_pct=")); Serial.println(rightDeviation, 1);
    Serial.print(F("Turn check: "));
    Serial.println(record.turnPassed[i] ? F("PASS") : F("CHECK ANGLE / TIMEOUT"));
  }

  const float avg90 = (normalizedCounts[0][0] + normalizedCounts[0][1] +
      normalizedCounts[1][0] + normalizedCounts[1][1]) / 4.0f;
  const float avg180 = (normalizedCounts[2][0] + normalizedCounts[2][1] +
      normalizedCounts[3][0] + normalizedCounts[3][1]) / 4.0f;
  Serial.println();
  Serial.print(F("Measured average TICKS_PER_90_DEG: ")); Serial.println(avg90, 1);
  Serial.print(F("Measured average TICKS_PER_180_DEG: ")); Serial.println(avg180, 1);
  Serial.println(F("Compare these with config.h. Values are measurements; the test does not edit config.h."));
  Serial.println(F("Next-run angle corrections (degrees):"));
  for (uint8_t i = 0; i < 4; ++i) {
    Serial.print(labels[i]); Serial.print('=');
    Serial.print(record.targetCorrectionDegrees[i], 2);
    if (i < 3) Serial.print(F(", "));
  }
  Serial.println();
  Serial.println(F("Enter T then Enter to repeat with an autotuned target correction."));
  Serial.println(F("============================================"));
}

void indicateComplete() {
  pinMode(LED_BUILTIN, OUTPUT);
  for (uint8_t i = 0; i < 3; ++i) {
    digitalWrite(LED_BUILTIN, HIGH); delay(250);
    digitalWrite(LED_BUILTIN, LOW); delay(250);
  }
}

void runCalibrationSequence(bool allowAutoTuneRepeat) {
  TurnCalibrationRecord previousRecord;
  EEPROM.get(EEPROM_ADDRESS, previousRecord);
  const bool havePreviousRun = previousRecord.magic == EEPROM_MAGIC &&
      previousRecord.version == RECORD_VERSION && previousRecord.status == STATUS_COMPLETE;
  const float requestedTargets[4] = {90.0f, -90.0f, 180.0f, -180.0f};
  float corrections[4] = {0.0f, 0.0f, 0.0f, 0.0f};

  // One-step proportional autotune. Learn from the previous run's final
  // angle error, but cap each target adjustment to avoid large jumps.
  if (havePreviousRun) {
    for (uint8_t i = 0; i < 4; ++i) {
      const float error = requestedTargets[i] - previousRecord.measuredDegrees[i];
      corrections[i] = constrain(previousRecord.targetCorrectionDegrees[i] + 0.7f * error,
                                  -6.0f, 6.0f);
    }
  }

  memset(&resultRecord, 0, sizeof(resultRecord));
  resultRecord.magic = EEPROM_MAGIC;
  resultRecord.version = RECORD_VERSION;
  resultRecord.status = STATUS_RUNNING;
  resultRecord.failureStage = 1;
  for (uint8_t i = 0; i < 4; ++i) {
    resultRecord.requestedDegrees[i] = requestedTargets[i];
    resultRecord.targetCorrectionDegrees[i] = corrections[i];
  }
  EEPROM.put(EEPROM_ADDRESS, resultRecord);

  Serial.println(F("Turn tick calibration starts in 5 seconds. Place robot on a level floor with clear space."));
  if (havePreviousRun) Serial.println(F("Using autotuned corrections learned from the previous completed run."));
  else Serial.println(F("Initial run: no previous turn results available for autotuning."));
  pinMode(LED_BUILTIN, OUTPUT);
  for (uint8_t i = 0; i < 5; ++i) {
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    delay(1000);
  }
  digitalWrite(LED_BUILTIN, LOW);

  if (!calibrateGyroBias()) {
    stopMotors();
    resultRecord.status = STATUS_FAILED;
    EEPROM.put(EEPROM_ADDRESS, resultRecord);
    Serial.println(F("FAILED: MPU6050 read error during bias calibration; motors were never commanded."));
    indicateComplete();
    return;
  }

  for (uint8_t i = 0; i < 4; ++i) {
    resultRecord.failureStage = 2 + i;
    EEPROM.put(EEPROM_ADDRESS, resultRecord);
    const float appliedTarget = requestedTargets[i] + corrections[i];
    if (!runTurn(i, appliedTarget)) {
      stopMotors();
      resultRecord.status = STATUS_FAILED;
      EEPROM.put(EEPROM_ADDRESS, resultRecord);
      Serial.println(F("FAILED: MPU6050 read error during turn. Partial results saved."));
      indicateComplete();
      return;
    }
    resultRecord.turnPassed[i] =
        (fabs(resultRecord.measuredDegrees[i] - requestedTargets[i]) <= TURN_TOLERANCE_DEG) ? 1 : 0;
    resultRecord.failureStage = 0;
    EEPROM.put(EEPROM_ADDRESS, resultRecord);
    delay(1500);
  }

  resultRecord.status = STATUS_COMPLETE;
  resultRecord.failureStage = 0;
  EEPROM.put(EEPROM_ADDRESS, resultRecord);
  Serial.println(F("Calibration run saved to EEPROM."));
  printSavedResults(resultRecord);

  if (allowAutoTuneRepeat && !havePreviousRun) {
    Serial.println(F("Starting one autotuned verification run using the measured angle errors."));
    delay(2000);
    runCalibrationSequence(false);
    return;
  }

  indicateComplete();
}

void handleSerialCommands() {
  static char command = 0;
  while (Serial.available()) {
    const char c = (char)Serial.read();
    if (c == 'r' || c == 'R' || c == 't' || c == 'T') command = c;
    if (c == '\n' || c == '\r') {
      if (command == 'r' || command == 'R') {
        TurnCalibrationRecord saved;
        EEPROM.get(EEPROM_ADDRESS, saved);
        printSavedResults(saved);
      } else if (command == 't' || command == 'T') {
        runCalibrationSequence(false);
      }
      command = 0;
    }
  }
}

void setup() {
  Serial.begin(115200);
  Wire.begin(); // Mega 2560 SDA=20, SCL=21
  Wire.setClock(100000);

  pinMode(PIN_LEFT_ENC_A, INPUT_PULLUP);
  pinMode(PIN_LEFT_ENC_B, INPUT_PULLUP);
  pinMode(PIN_RIGHT_ENC_A, INPUT_PULLUP);
  pinMode(PIN_RIGHT_ENC_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_LEFT_ENC_A), leftEncoderTestISR, RISING);
  attachInterrupt(digitalPinToInterrupt(PIN_RIGHT_ENC_A), rightEncoderTestISR, RISING);

  pinMode(PIN_LEFT_RPWM, OUTPUT);
  pinMode(PIN_LEFT_LPWM, OUTPUT);
  pinMode(PIN_RIGHT_RPWM, OUTPUT);
  pinMode(PIN_RIGHT_LPWM, OUTPUT);
  stopMotors();

  if (!writeRegister(REG_PWR_MGMT_1, 0x00) || !writeRegister(REG_GYRO_CONFIG, 0x00)) {
    Serial.println(F("MPU6050 setup failed. Motors remain stopped."));
    return;
  }

  TurnCalibrationRecord saved;
  EEPROM.get(EEPROM_ADDRESS, saved);
  if (saved.magic == EEPROM_MAGIC && saved.version == RECORD_VERSION &&
      (saved.status == STATUS_COMPLETE || saved.status == STATUS_FAILED || saved.status == STATUS_RUNNING)) {
    if (saved.status == STATUS_RUNNING) {
      saved.status = STATUS_FAILED;
      EEPROM.put(EEPROM_ADDRESS, saved);
      Serial.println(F("Previous run was interrupted. Motors will not restart automatically."));
    } else {
      Serial.println(F("Saved calibration found. Send R then Enter to display it, or T then Enter to run again."));
    }
    return;
  }

  runCalibrationSequence(true);
}

void loop() {
  handleSerialCommands();
}
