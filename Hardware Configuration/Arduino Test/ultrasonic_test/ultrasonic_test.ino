// Define the pins connected to the sensor on the Mega 2560
const int trigPin = 12;
const int echoPin = 13;

// Variables to store the duration of the sound wave and the calculated distance
long duration;
int distance;

void setup() {
  // Set up the Serial Monitor
  Serial.begin(9600);
  
  // Set the pin modes
  pinMode(trigPin, OUTPUT); 
  pinMode(echoPin, INPUT);  
}

void loop() {
  // 1. Clear the Trig pin by setting it LOW for 2 microseconds
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  
  // 2. Trigger the sensor by setting the Trig pin HIGH for 10 microseconds
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  
  // 3. Read the Echo pin; pulseIn returns the time it took for the wave to travel (in microseconds)
  duration = pulseIn(echoPin, HIGH);
  
  // 4. Calculate the distance in centimeters
  // Speed of sound is ~0.034 cm/microsecond. 
  // We divide by 2 because the wave travels to the object AND back.
  distance = duration * 0.034 / 2;
  
  // 5. Print the distance to the Serial Monitor
  Serial.print("Distance: ");
  Serial.print(distance);
  Serial.println(" cm");
  
  // Wait a brief moment before taking the next reading
  delay(100);
}