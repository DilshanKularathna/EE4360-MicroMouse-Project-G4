#include <Arduino.h>
#include <Wire.h>
#include "config.h"

// MPU6050 default I2C address. Change to 0x69 if AD0 is tied HIGH.
const uint8_t MPU6050_ADDRESS = 0x68;
const uint8_t REG_PWR_MGMT_1 = 0x6B;
const uint8_t REG_GYRO_CONFIG = 0x1B;
const uint8_t REG_ACCEL_XOUT_H = 0x3B;

const unsigned long CALIBRATION_TIME_MS = 10000UL;
const unsigned long TURN_TEST_TIME_MS = 10000UL;
const unsigned long SAMPLE_INTERVAL_MS = 20UL; // 50 Hz output
const float GYRO_LSB_PER_DPS = 131.0f; // ±250 degrees/second range
const float MIN_TURN_ANGLE_DEG = 30.0f;
const float MIN_DOMINANCE_RATIO = 1.5f;

int16_t gyroOffsetX = 0;
int16_t gyroOffsetY = 0;
int16_t gyroOffsetZ = 0;

bool writeRegister(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(MPU6050_ADDRESS);
  Wire.write(reg);
  Wire.write(value);
  return Wire.endTransmission() == 0;
}

bool readRaw(int16_t values[7]) {
  Wire.beginTransmission(MPU6050_ADDRESS);
  Wire.write(REG_ACCEL_XOUT_H);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(MPU6050_ADDRESS, (uint8_t)14) != 14) return false;

  for (uint8_t i = 0; i < 7; ++i) {
    values[i] = (int16_t)((Wire.read() << 8) | Wire.read());
  }
  return true;
}

void waitForEnter(const __FlashStringHelper *message) {
  Serial.println(message);
  Serial.println(F("Type any character and press Enter."));

  while (!Serial.available()) delay(5);
  // Wait for the end of the user's line; accept either CR or LF.
  while (true) {
    if (Serial.available()) {
      const char c = (char)Serial.read();
      if (c == '\n' || c == '\r') break;
    }
  }
  delay(10);
  while (Serial.available()) Serial.read();
}

bool calibrateGyro() {
  int32_t sumX = 0;
  int32_t sumY = 0;
  int32_t sumZ = 0;
  uint16_t samples = 0;
  const unsigned long startMs = millis();
  unsigned long nextSampleMs = startMs;
  int16_t values[7];

  Serial.println(F("Calibration started. Keep the robot completely still for 10 seconds."));
  while (millis() - startMs < CALIBRATION_TIME_MS) {
    if (millis() >= nextSampleMs) {
      if (!readRaw(values)) return false;
      sumX += values[4];
      sumY += values[5];
      sumZ += values[6];
      ++samples;
      nextSampleMs += SAMPLE_INTERVAL_MS;
    }
  }

  if (samples == 0) return false;
  gyroOffsetX = (int16_t)(sumX / samples);
  gyroOffsetY = (int16_t)(sumY / samples);
  gyroOffsetZ = (int16_t)(sumZ / samples);

  Serial.print(F("Calibration complete; samples="));
  Serial.print(samples);
  Serial.print(F(" offsets_raw="));
  Serial.print(gyroOffsetX);
  Serial.print(',');
  Serial.print(gyroOffsetY);
  Serial.print(',');
  Serial.println(gyroOffsetZ);
  return true;
}

bool recordTurn(const __FlashStringHelper *label, float integratedAngles[3]) {
  int16_t values[7];
  const unsigned long startMs = millis();
  unsigned long lastSampleUs = micros();
  unsigned long nextSampleMs = startMs;
  integratedAngles[0] = 0.0f;
  integratedAngles[1] = 0.0f;
  integratedAngles[2] = 0.0f;

  Serial.println(label);
  Serial.println(F("elapsed_ms,accel_x_g,accel_y_g,accel_z_g,gyro_x_dps,gyro_y_dps,gyro_z_dps"));

  while (millis() - startMs < TURN_TEST_TIME_MS) {
    if (millis() >= nextSampleMs) {
      if (!readRaw(values)) {
        Serial.println(F("I2C_READ_ERROR"));
        return false;
      }

      const unsigned long sampleUs = micros();
      const float deltaSeconds = (sampleUs - lastSampleUs) / 1000000.0f;
      lastSampleUs = sampleUs;
      const float gyroRates[3] = {
        (values[4] - gyroOffsetX) / GYRO_LSB_PER_DPS,
        (values[5] - gyroOffsetY) / GYRO_LSB_PER_DPS,
        (values[6] - gyroOffsetZ) / GYRO_LSB_PER_DPS
      };
      for (uint8_t axis = 0; axis < 3; ++axis) {
        integratedAngles[axis] += gyroRates[axis] * deltaSeconds;
      }

      Serial.print(millis() - startMs);
      Serial.print(','); Serial.print(values[0] / 16384.0f, 3);
      Serial.print(','); Serial.print(values[1] / 16384.0f, 3);
      Serial.print(','); Serial.print(values[2] / 16384.0f, 3);
      Serial.print(','); Serial.print(gyroRates[0], 2);
      Serial.print(','); Serial.print(gyroRates[1], 2);
      Serial.print(','); Serial.println(gyroRates[2], 2);

      nextSampleMs += SAMPLE_INTERVAL_MS;
    }
  }
  Serial.println(F("Turn capture complete."));
  return true;
}

void reportAxisComparison(const float clockwise[3], const float counterclockwise[3]) {
  const char *axisNames[3] = {"X", "Y", "Z"};
  int bestAxis = -1;
  float bestScore = 0.0f;
  float secondBestScore = 0.0f;

  Serial.println();
  Serial.println(F("===== AUTOMATIC GYRO AXIS RESULT ====="));
  Serial.println(F("Integrated gyro angles (degrees):"));
  for (uint8_t axis = 0; axis < 3; ++axis) {
    Serial.print(axisNames[axis]);
    Serial.print(F(" axis: CW="));
    Serial.print(clockwise[axis], 1);
    Serial.print(F("  CCW="));
    Serial.print(counterclockwise[axis], 1);
    Serial.println();

    // A valid yaw candidate must reverse sign between the two commanded turns.
    if (clockwise[axis] * counterclockwise[axis] < 0.0f) {
      const float score = min(fabs(clockwise[axis]), fabs(counterclockwise[axis]));
      if (score > bestScore) {
        secondBestScore = bestScore;
        bestScore = score;
        bestAxis = axis;
      } else if (score > secondBestScore) {
        secondBestScore = score;
      }
    }
  }

  if (bestAxis < 0 || bestScore < MIN_TURN_ANGLE_DEG) {
    Serial.println(F("RESULT: No reliable yaw axis found. Repeat with clear, opposite in-place turns."));
    return;
  }

  Serial.print(F("Likely yaw axis: "));
  Serial.println(axisNames[bestAxis]);
  if (bestAxis == MPU6050_YAW_AXIS_INDEX) {
    Serial.println(F("Axis agrees with mounting map: +X=front, +Y=right, +Z=down."));
  } else {
    Serial.println(F("MOUNTING MISMATCH: stated map expects Z yaw (+Z down). Repeat level in-place turns and check axis labels."));
  }
  Serial.print(F("Clockwise sign on that axis: "));
  Serial.println(clockwise[bestAxis] > 0.0f ? F("positive") : F("negative"));

  if (secondBestScore > 0.0f && bestScore < secondBestScore * MIN_DOMINANCE_RATIO) {
    Serial.println(F("RESULT: AMBIGUOUS - multiple axes moved. Keep the robot level and turn in place."));
  } else {
    Serial.println(F("RESULT: Axis is dominant and reverses sign as expected."));
  }
  Serial.println(F("======================================"));
}

void setup() {
  Serial.begin(115200);
  Wire.begin(); // Mega 2560: SDA=20, SCL=21
  Wire.setClock(100000);
  delay(100);

  if (!writeRegister(REG_PWR_MGMT_1, 0x00) ||
      !writeRegister(REG_GYRO_CONFIG, 0x00)) {
    Serial.println(F("MPU6050 I2C setup failed. Check power, GND, SDA/SCL and address."));
    while (true) delay(1000);
  }

  int16_t values[7];
  if (!readRaw(values)) {
    Serial.println(F("No MPU6050 response at 0x68. Check wiring and AD0/address."));
    while (true) delay(1000);
  }

  Serial.println(F("MPU6050 axis identification test ready."));
  waitForEnter(F("Place the robot still on a level surface to estimate gyro offsets."));
  if (!calibrateGyro()) {
    Serial.println(F("I2C read failed during calibration."));
    while (true) delay(1000);
  }

  float clockwiseAngles[3];
  float counterclockwiseAngles[3];
  waitForEnter(F("Place robot level. Start a slow clockwise in-place turn when prompted."));
  if (!recordTurn(F("CLOCKWISE TURN - rotate in place for 10 seconds"), clockwiseAngles)) {
    while (true) delay(1000);
  }

  waitForEnter(F("Reset the robot to its starting heading. Then turn counter-clockwise."));
  if (!recordTurn(F("COUNTERCLOCKWISE TURN - rotate in place for 10 seconds"), counterclockwiseAngles)) {
    while (true) delay(1000);
  }

  reportAxisComparison(clockwiseAngles, counterclockwiseAngles);
}

void loop() {
  // This calibration/axis test runs once after reset.
}
