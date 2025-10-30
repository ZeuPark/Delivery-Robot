#include <SPI.h>
#include <Pixy2.h>

/*
 Vision System Test for Delivery Robot
 Dual LED control based on object detection
 - Blue ball: LED1 on
 - Green ball: LED2 on
 - Base: Both LEDs on
 - Obstacle (≤10cm): Both LEDs blink
*/

Pixy2 pixy;

/* User Settings */

// Serial
constexpr unsigned long SERIAL_BAUD = 115200;

// PixyMon signatures
constexpr uint8_t SIG_BASE  = 1; // Base
constexpr uint8_t SIG_RED   = 2; // Forbidden ball
constexpr uint8_t SIG_GREEN = 3; // Allowed ball
constexpr uint8_t SIG_BLUE  = 4; // Allowed ball

// External status LEDs
constexpr uint8_t LED1_PIN = 6;  // LED for Blue ball
constexpr uint8_t LED2_PIN = 7;   // LED for Green ball

// Ultrasonic sensor
constexpr uint8_t TRIG_PIN = 10;
constexpr uint8_t ECHO_PIN = 9;
constexpr float OBSTACLE_CM_THRESHOLD = 10.0f;
constexpr float OBSTACLE_TOLERANCE = 0.5f; // Detection range: 9.5cm to 10.5cm

// LED blink timing for obstacle detection
constexpr unsigned int BLINK_INTERVAL_MS = 200;

/* State variables */
enum DetectionState : uint8_t {
  STATE_NONE = 0,
  STATE_BLUE,
  STATE_GREEN,
  STATE_BASE,
  STATE_OBSTACLE
};

static DetectionState currentState = STATE_NONE;
static unsigned long lastBlinkTime = 0;
static bool blinkState = false;

/* Ultrasonic distance in centimeters */

long readUltrasonicCm() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);

  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  unsigned long duration = pulseIn(ECHO_PIN, HIGH, 30000UL); // up to about 5 meters
  if (duration == 0) return -1;

  long distanceCm = static_cast<long>(duration * 0.0343f / 2.0f);
  return distanceCm;
}

/* LED Control Functions */

void setLEDState(DetectionState state) {
  currentState = state;
  
  switch (state) {
    case STATE_BLUE:
      // Blue ball detected: LED1 on, LED2 off
      digitalWrite(LED1_PIN, HIGH);
      digitalWrite(LED2_PIN, LOW);
      break;
      
    case STATE_GREEN:
      // Green ball detected: LED1 off, LED2 on
      digitalWrite(LED1_PIN, LOW);
      digitalWrite(LED2_PIN, HIGH);
      break;
      
    case STATE_BASE:
      // Base detected: Both LEDs on
      digitalWrite(LED1_PIN, HIGH);
      digitalWrite(LED2_PIN, HIGH);
      break;
      
    case STATE_OBSTACLE:
      // Obstacle detected: Both LEDs will blink (handled in updateLEDs)
      break;
      
    case STATE_NONE:
    default:
      // Nothing detected: Both LEDs off
      digitalWrite(LED1_PIN, LOW);
      digitalWrite(LED2_PIN, LOW);
      break;
  }
}

void updateLEDs() {
  if (currentState == STATE_OBSTACLE) {
    // Blink both LEDs when obstacle is detected
    unsigned long now = millis();
    if (now - lastBlinkTime >= BLINK_INTERVAL_MS) {
      blinkState = !blinkState;
      digitalWrite(LED1_PIN, blinkState ? HIGH : LOW);
      digitalWrite(LED2_PIN, blinkState ? HIGH : LOW);
      lastBlinkTime = now;
    }
  }
}

/* Main */

void setup() {
  Serial.begin(SERIAL_BAUD);
  unsigned long t0 = millis();
  while (!Serial && (millis() - t0) < 2000) {}

  // Keep SPI master mode stable on Uno
  pinMode(10, OUTPUT); // SS must be output on master

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  pinMode(LED1_PIN, OUTPUT);
  pinMode(LED2_PIN, OUTPUT);
  
  digitalWrite(TRIG_PIN, LOW);
  digitalWrite(LED1_PIN, LOW);
  digitalWrite(LED2_PIN, LOW);

  Serial.println("=== Vision System with Dual LED Control ===");
  Serial.println("LED1 (Pin 6): Blue ball");
  Serial.println("LED2 (Pin 7): Green ball");
  Serial.println("Both LEDs: Base or Obstacle (blinking)");

  pixy.init();
}

void loop() {
  // Ultrasonic obstacle check (highest priority)
  long distance = readUltrasonicCm();
  // Only detect obstacle if distance is within tolerance range around 10cm
  if (distance > 0 && 
      distance >= static_cast<long>(OBSTACLE_CM_THRESHOLD - OBSTACLE_TOLERANCE) && 
      distance <= static_cast<long>(OBSTACLE_CM_THRESHOLD + OBSTACLE_TOLERANCE)) {
    if (currentState != STATE_OBSTACLE) {
      setLEDState(STATE_OBSTACLE);
      Serial.print("OBSTACLE DETECTED at ~10cm! Distance: ");
      Serial.print(distance);
      Serial.println(" cm");
    }
    updateLEDs();
    return;
  }

  // Vision blocks
  pixy.ccc.getBlocks();

  if (pixy.ccc.numBlocks == 0) {
    if (currentState != STATE_NONE) {
      setLEDState(STATE_NONE);
    }
    return;
  }

  bool sawBase = false;
  bool sawBlue = false;
  bool sawGreen = false;

  for (int i = 0; i < pixy.ccc.numBlocks; i++) {
    Block& block = pixy.ccc.blocks[i];
    long area = static_cast<long>(block.m_width) * static_cast<long>(block.m_height);

    if (block.m_signature == SIG_BASE) {
      Serial.print("Base detected (S1) area: ");
      Serial.println(area);
      sawBase = true;

    } else if (block.m_signature == SIG_GREEN) {
      Serial.print("Green ball detected (S3) area: ");
      Serial.println(area);
      sawGreen = true;

    } else if (block.m_signature == SIG_BLUE) {
      Serial.print("Blue ball detected (S4) area: ");
      Serial.println(area);
      sawBlue = true;

    } else if (block.m_signature == SIG_RED) {
      Serial.print("Red ball ignored (S2) area: ");
      Serial.println(area);
    }
  }

  // Apply detection state based on priority
  if (sawBase) {
    if (currentState != STATE_BASE) {
      setLEDState(STATE_BASE);
    }
  } else if (sawBlue) {
    if (currentState != STATE_BLUE) {
      setLEDState(STATE_BLUE);
    }
  } else if (sawGreen) {
    if (currentState != STATE_GREEN) {
      setLEDState(STATE_GREEN);
    }
  } else {
    if (currentState != STATE_NONE) {
      setLEDState(STATE_NONE);
    }
  }
}