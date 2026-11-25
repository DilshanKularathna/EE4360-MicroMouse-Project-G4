#include <QTRSensors.h>

QTRSensors qtr;

// Define the 8 analog input pins you connected the sensor to
const uint8_t SensorCount = 8;
uint8_t sensorPins[SensorCount] = {A15, A14, A13, A12, A11, A10, A9, A8};

void setup() {
  Serial.begin(9600);
  
  // Configure the library for Analog mode
  qtr.setTypeAnalog();
  qtr.setSensorPins(sensorPins, SensorCount);
  
  // Optional: Set how many samples to average per sensor to smooth out noise
  qtr.setSamplesPerSensor(4); 
  
  // If your cheap board has a LEDON / CTRL pin, wire it to an Arduino pin (e.g. Pin 10)
  // If it doesn't have one, or you left it disconnected, leave this line commented out.
  qtr.setEmitterPin(49); 

  Serial.println("--- QTR-8A Raw Test Started ---");
}

void loop() {
  uint16_t sensorValues[SensorCount];
  
  // Read raw values with the IR emitters turned ON
  qtr.read(sensorValues, QTRReadMode::On);

  // Print the values side-by-side to the Serial Monitor
  for (uint8_t i = 0; i < SensorCount; i++) {
    Serial.print(sensorValues[i]);
    Serial.print('\t'); // Tab spaced
  }
  Serial.println(); // New line

  delay(250); // Read 4 times a second
}
