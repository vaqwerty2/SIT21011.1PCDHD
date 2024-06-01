const int trigPin = 9;   // Trig pin of ultrasonic sensor
const int echoPin = 10;  // Echo pin of ultrasonic sensor
const int buzzerPin = 8; // Pin for buzzer

long duration;
int distance;
const int threshold = 200; // Threshold distance in cm

void setup() {
  pinMode(trigPin, OUTPUT);  // Set the trigPin as an output
  pinMode(echoPin, INPUT);   // Set the echoPin as an input
  pinMode(buzzerPin, OUTPUT); // Set the buzzerPin as an output

  Serial.begin(9600); // Start serial communication at 9600 baud
}

void loop() {
  // Clear the trigPin
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);

  // Set the trigPin HIGH for 10 microseconds
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  // Read the echoPin, returns the sound wave travel time in microseconds
  duration = pulseIn(echoPin, HIGH);

  // Calculate the distance
  distance = duration * 0.034 / 2; // Speed of sound wave divided by 2 (go and back)

  // Print the distance to the Serial Monitor
  Serial.print("Distance: ");
  Serial.print(distance);
  Serial.println(" cm");

  // If distance is less than the threshold, sound the buzzer
  if (distance < threshold) {
    digitalWrite(buzzerPin, HIGH);
  } else {
    digitalWrite(buzzerPin, LOW);
  }

  delay(500); // Wait for half a second before the next measurement
}
