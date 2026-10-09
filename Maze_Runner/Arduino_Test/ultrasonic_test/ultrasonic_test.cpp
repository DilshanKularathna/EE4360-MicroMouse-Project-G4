#include <Arduino.h>
#include "config.h"

// HC-SR04-style sensors need a short quiet period between pings to reduce
// echoes from one sensor being picked up by another.
const unsigned long ECHO_TIMEOUT_US = 30000UL; // about 5 m maximum range
const unsigned long SENSOR_SETTLE_MS = 60UL;
const unsigned long SAMPLE_INTERVAL_MS = 250UL;

struct UltrasonicSensor {
  const char *name;
  uint8_t trigPin;
  uint8_t echoPin;
};

const UltrasonicSensor sensors[] = {
  {"front", PIN_ULTRA_FRONT_TRIG, PIN_ULTRA_FRONT_ECHO},
  {"left",  PIN_ULTRA_LEFT_TRIG,  PIN_ULTRA_LEFT_ECHO},
  {"right", PIN_ULTRA_RIGHT_TRIG, PIN_ULTRA_RIGHT_ECHO}
};

const size_t SENSOR_COUNT = sizeof(sensors) / sizeof(sensors[0]);

// Returns centimeters; -1 means no echo was received before the timeout.
float readDistanceCm(const UltrasonicSensor &sensor) {
  digitalWrite(sensor.trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(sensor.trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(sensor.trigPin, LOW);

  const unsigned long duration = pulseIn(sensor.echoPin, HIGH, ECHO_TIMEOUT_US);
  if (duration == 0) {
    return -1.0f;
  }

  // Sound travels to the target and back. 0.0343 cm/us is the nominal
  // speed of sound at room temperature.
  return (duration * 0.0343f) / 2.0f;
}

void printDistance(float distanceCm) {
  if (distanceCm < 0.0f) {
    Serial.print("NO_ECHO");
  } else {
    Serial.print(distanceCm, 2);
  }
}

void setup() {
  Serial.begin(115200);

  for (size_t i = 0; i < SENSOR_COUNT; ++i) {
    pinMode(sensors[i].trigPin, OUTPUT);
    pinMode(sensors[i].echoPin, INPUT);
    digitalWrite(sensors[i].trigPin, LOW);
  }

  // CSV output can be copied directly from the Serial Monitor for analysis.
  Serial.println("time_ms,front_cm,left_cm,right_cm");
}

void loop() {
  float distances[SENSOR_COUNT];

  // Read sequentially, allowing echoes to die out between different sensors.
  for (size_t i = 0; i < SENSOR_COUNT; ++i) {
    distances[i] = readDistanceCm(sensors[i]);
    if (i + 1 < SENSOR_COUNT) {
      delay(SENSOR_SETTLE_MS);
    }
  }

  Serial.print(millis());
  for (size_t i = 0; i < SENSOR_COUNT; ++i) {
    Serial.print(',');
    printDistance(distances[i]);
  }
  Serial.println();

  delay(SAMPLE_INTERVAL_MS);
}
