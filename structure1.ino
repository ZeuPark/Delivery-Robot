#include <SPI.h>
#include <Pixy2.h>

/*
 Vision System Test for Delivery Robot
 Simplified sensor test focused on Pixy2 vision and ultrasonic obstacle detection
 Feedback via Serial Monitor and external LED on pin 7
 Robot motion logic removed for stable sensor testing on Arduino Uno
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

// External status LED
constexpr uint8_t EXT_LED_PIN = 13;  // Use an external LED with series resistor

// Ultrasonic sensor
constexpr uint8_t TRIG_PIN = 10;
constexpr uint8_t ECHO_PIN = 9;
constexpr float OBSTACLE_CM_THRESHOLD = 10.0f;

// LED pattern timings
constexpr unsigned int LED_FAST_ON_MS   = 100;
constexpr unsigned int LED_FAST_OFF_MS  = 100;
constexpr unsigned int LED_BURST_GAP_MS = 300;

/* LED Pattern Engine */

enum LedPatternType : uint8_t {
  LED_NONE = 0,
  LED_GREEN2,
  LED_BLUE3,
  LED_BASE4,
  LED_OBSTACLE_FAST
};

static LedPatternType currentPattern = LED_NONE;
static bool ledIsOn = false;
static uint8_t blinksCompletedInBurst = 0;
static uint8_t targetBlinksInBurst = 0;
static unsigned long nextLedEventAt = 0;
static bool inBurstGap = false;

static uint8_t patternPriority(LedPatternType p) {
  switch (p) {
    case LED_OBSTACLE_FAST: return 4;
    case LED_BASE4:         return 3;
    case LED_BLUE3:         return 2;
    case LED_GREEN2:        return 1;
    default:                return 0;
  }
}

static void setPattern(LedPatternType newPattern) {
  if (patternPriority(newPattern) < patternPriority(currentPattern)) return;
  if (newPattern == currentPattern) return;

  currentPattern = newPattern;

  ledIsOn = false;
  blinksCompletedInBurst = 0;
  inBurstGap = false;

  switch (currentPattern) {
    case LED_GREEN2:        targetBlinksInBurst = 2; break;
    case LED_BLUE3:         targetBlinksInBurst = 3; break;
    case LED_BASE4:         targetBlinksInBurst = 4; break;
    case LED_OBSTACLE_FAST: targetBlinksInBurst = 0; break; // continuous
    default:                targetBlinksInBurst = 0; break;
  }
  nextLedEventAt = millis();
}

static void updateLedPattern() {
  unsigned long now = millis();

  if (currentPattern == LED_NONE) {
    if (ledIsOn) {
      digitalWrite(EXT_LED_PIN, LOW);
      ledIsOn = false;
    }
    return;
  }

  if (currentPattern == LED_OBSTACLE_FAST) {
    if (now >= nextLedEventAt) {
      ledIsOn = !ledIsOn;
      digitalWrite(EXT_LED_PIN, ledIsOn ? HIGH : LOW);
      nextLedEventAt = now + (ledIsOn ? LED_FAST_ON_MS : LED_FAST_OFF_MS);
    }
    return;
  }

  if (inBurstGap) {
    if (now >= nextLedEventAt) {
      inBurstGap = false;
      blinksCompletedInBurst = 0;
    }
    return;
  }

  if (now < nextLedEventAt) return;

  if (!ledIsOn) {
    digitalWrite(EXT_LED_PIN, HIGH);
    ledIsOn = true;
    nextLedEventAt = now + LED_FAST_ON_MS;
  } else {
    digitalWrite(EXT_LED_PIN, LOW);
    ledIsOn = false;
    blinksCompletedInBurst++;
    if (blinksCompletedInBurst >= targetBlinksInBurst) {
      inBurstGap = true;
      nextLedEventAt = now + LED_BURST_GAP_MS;
    } else {
      nextLedEventAt = now + LED_FAST_OFF_MS;
    }
  }
}

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

/* Main */

void setup() {
  Serial.begin(SERIAL_BAUD);
  unsigned long t0 = millis();
  while (!Serial && (millis() - t0) < 2000) {}

  // Keep SPI master mode stable on Uno
  pinMode(10, OUTPUT); // SS must be output on master

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  pinMode(EXT_LED_PIN, OUTPUT);
  digitalWrite(TRIG_PIN, LOW);
  digitalWrite(EXT_LED_PIN, LOW);

  Serial.println("=== Vision System Test initialized ===");
  Serial.println("Show objects to the camera to test detection.");

  pixy.init();
}

void loop() {
  // Ultrasonic obstacle check
  long distance = readUltrasonicCm();
  if (distance > 0 && distance <= static_cast<long>(OBSTACLE_CM_THRESHOLD)) {
    setPattern(LED_OBSTACLE_FAST);
    Serial.print("Obstacle ultrasonic within ");
    Serial.print(distance);
    Serial.println(" cm");
  }

  // Vision blocks
  pixy.ccc.getBlocks();

  if (pixy.ccc.numBlocks == 0) {
    updateLedPattern();
    return;
  }

  bool sawBase = false;
  bool sawBlue = false;
  bool sawGreen = false;

  for (int i = 0; i < pixy.ccc.numBlocks; i++) {
    Block& block = pixy.ccc.blocks[i];
    long area = static_cast<long>(block.m_width) * static_cast<long>(block.m_height);

    if (block.m_signature == SIG_BASE) {
      Serial.print("Base detected S1 area ");
      Serial.println(area);
      sawBase = true;

    } else if (block.m_signature == SIG_GREEN) {
      Serial.print("Green ball detected S3 area ");
      Serial.println(area);
      sawGreen = true;

    } else if (block.m_signature == SIG_BLUE) {
      Serial.print("Blue ball detected S4 area ");
      Serial.println(area);
      sawBlue = true;

    } else if (block.m_signature == SIG_RED) {
      Serial.print("Red ball ignored S2 area ");
      Serial.println(area);
    } else {
      // Unknown signature not used
    }
  }

  // Apply vision pattern only if obstacle pattern is not active
  if (patternPriority(currentPattern) < patternPriority(LED_OBSTACLE_FAST)) {
    if (sawBase) {
      setPattern(LED_BASE4);
    } else if (sawBlue) {
      setPattern(LED_BLUE3);
    } else if (sawGreen) {
      setPattern(LED_GREEN2);
    } else {
      setPattern(LED_NONE);
    }
  }

  updateLedPattern();
}
