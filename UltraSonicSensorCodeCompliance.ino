const int trigPin = 10;
const int echoPin = 9;
int ledPin = 13;
// defines variables
long duration;
int distance;
void setup() {
  pinMode(trigPin, OUTPUT); // Sets the trigPin as an Output
  pinMode(echoPin, INPUT); // Sets the echoPin as an Input
  pinMode(ledPin, OUTPUT);
  Serial.begin(9600); // Starts the serial communication
}
void loop() {
  // Clears the trigPin
  digitalWrite(trigPin, LOW);
  delay(200);
  // Sets the trigPin on HIGH state for 10 micro seconds
  digitalWrite(trigPin, HIGH);
  delay(200);
  digitalWrite(trigPin, LOW);
  // Reads the echoPin, returns the sound wave travel time in microseconds
  duration = pulseIn(echoPin, HIGH);
  // Calculating the distance
  distance = duration * 0.034 / 2;
  Serial.print("Distance: ");
  Serial.println(distance);

  int blinkDelay;
  //declare var for blinkDelay
  if (distance <= 10) {
    blinkDelay = 100;
  }
  else {
    blinkDelay = 600;
  }

  digitalWrite(ledPin, HIGH);
  delay(blinkDelay);
  digitalWrite(ledPin, LOW);
  delay(blinkDelay);
  digitalWrite(ledPin, HIGH);

  
  // if (distance < 10.5) {
  //   Serial.print("Distance: ");
  //   Serial.println(distance);

  //   //make the in built led blink
  //   digitalWrite(ledPin, HIGH);
  //   delay(200);
  //   digitalWrite(ledPin, LOW);
  //   delay(200);
  //   digitalWrite(ledPin, HIGH);
  // } 
  //   Serial.print("Distance: ");
  //   Serial.println(distance);
  //   digitalWrite(ledPin, HIGH);
  //   delay(500);
  //   digitalWrite(ledPin, LOW);
  //   delay(500);
  //   digitalWrite(ledPin, HIGH);
  
}
