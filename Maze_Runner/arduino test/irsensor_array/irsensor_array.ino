// Define the IR enable pin
const int irEnablePin = 30;

const int sensorPins[8] = {22, 24, 26, 28, 30, 32, 34, 36};
int sensorValues[8];

void setup() {
  Serial.begin(9600);
  
  // Set the IR Enable pin as an output and turn it ON
  pinMode(irEnablePin, OUTPUT);
  digitalWrite(irEnablePin, HIGH); // Turns on the infrared LEDs
  
  for (int i = 0; i < 8; i++) {
    pinMode(sensorPins[i], INPUT);
  }
  
  Serial.println("IR Sensor Array Test Started...");
  delay(1000);
}

void loop() {
  for (int i = 0; i < 8; i++) {
    sensorValues[i] = digitalRead(sensorPins[i]);
  }
  
  for (int i = 0; i < 8; i++) {
    Serial.print("S");
    Serial.print(i + 1); 
    Serial.print(": ");
    Serial.print(sensorValues[i]); 
    Serial.print("  |  "); 
  }
  
  Serial.println();
  delay(200);
}