#include <Pixy2.h>


Pixy2 pixy;

const int trigPin = 10;
const int echoPin = 9;
int ledPin = 13;
// defines variables
long duration;
int distance;

//signaturs 
int blueBallSig = 4;
int greenBallSig = 3;

void setup() {
  pinMode(trigPin, OUTPUT); // Sets the trigPin as an Output
  pinMode(echoPin, INPUT); // Sets the echoPin as an Input
  pinMode(ledPin, OUTPUT);
  Serial.begin(9600); // Starts the serial communication
}
void loop() {
  //-----------------------------------Ultrasonic Sensor---------------------
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

//----------------------------PixyCam-----------------------------------------
  bool ballDetected = false;
  pixy.ccc.getBlocks();
  //if any objects are detected
  if (pixy.ccc.numBlocks) {
    for (int i = 0; i < pixy.ccc.numBlocks; i++) {
      int sig = pixy.ccc.blocks[i].m_signature;
      if (sig == blueBallSig) {
        ballDetected = true;
        Serial.print("Blue ball detected");
        //produce output 
      }
      if (sig == greenBallSig) {
        ballDetected = true;
        Serial.print("Green ball detected");
        //produce output 
      }
    }
  }

// -------------------Obstacle Detection---------------------------------------
  //if an object that is not a green/blue ball is within 10cm 
  if (!ballDetected && distance > 0 && distance <= 10.0) {
    int blinkDelay = 300;
    Serial.print("Distance: ");
    digitalWrite(ledPin, HIGH);
    delay(blinkDelay);
    digitalWrite(ledPin, LOW);
    delay(blinkDelay);
    digitalWrite(ledPin, HIGH);
  }
  delay(200);

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
