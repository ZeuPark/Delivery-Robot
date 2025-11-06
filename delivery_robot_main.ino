#include <SPI.h>
#include <Pixy2.h>
#include <Servo.h>

/*
 * Delivery Robot Main Code
 * Integrates Pixy2 vision, motor control, and servo/gripper control
 * 
 * Hardware:
 * - Pixy2 Camera (SPI)
 * - Ultrasonic Sensor (Trig: A0, Echo: A1)
 * - Motor Driver (PWMA: 5, DIRA: 4, PWMB: 6, DIRB: 7)
 * - Encoders (ENCA_A: 2, ENCA_B: 8, ENCB_A: 3, ENCB_B: 9)
 * - Dual Servo Motors for Gripper (Left: Pin 10, Right: Pin 11)
 * - Status LEDs (optional)
 */

/* ==================== Pixy2 Configuration ==================== */
Pixy2 pixy;

// PixyMon signatures
constexpr uint8_t SIG_BASE  = 1; // Base
constexpr uint8_t SIG_RED   = 2; // Forbidden ball
constexpr uint8_t SIG_GREEN = 3; // Allowed ball
constexpr uint8_t SIG_BLUE  = 4; // Allowed ball

/* ==================== Motor Control Pins ==================== */
#define PWMA 5      // Left motor PWM
#define DIRA 4      // Left motor direction (LOW=forward, HIGH=backward)
#define PWMB 6      // Right motor PWM
#define DIRB 7      // Right motor direction (LOW=forward, HIGH=backward)

/* ==================== Encoder Pins ==================== */
#define ENCA_A 2    // Left encoder channel A
#define ENCA_B 8    // Left encoder channel B
#define ENCB_A 3    // Right encoder channel A
#define ENCB_B 9    // Right encoder channel B

// Encoder counts
volatile long countA = 0;  // Left encoder count
volatile long countB = 0;   // Right encoder count

/* ==================== Ultrasonic Sensor ==================== */
// Using analog pins A0/A1 as digital pins to avoid conflicts with encoders
constexpr uint8_t TRIG_PIN = A0;  // Pin 14 (A0 as digital)
constexpr uint8_t ECHO_PIN = A1;   // Pin 15 (A1 as digital)
constexpr float OBSTACLE_CM_THRESHOLD = 10.0f;
constexpr float OBSTACLE_TOLERANCE = 0.5f; // Detection range: 9.5cm to 10.5cm

// Motor speed (0-255 for PWM)
constexpr uint8_t MOTOR_SPEED_NORMAL = 150;
constexpr uint8_t MOTOR_SPEED_SLOW = 100;
constexpr uint8_t MOTOR_SPEED_TURN = 120;

/* ==================== Dual Servo/Gripper Control ==================== */
Servo leftServo;
Servo rightServo;
constexpr uint8_t SERVO_LEFT_PIN = 10;
constexpr uint8_t SERVO_RIGHT_PIN = 11;

// Servo positions
constexpr uint8_t SERVO_LEFT_CLOSED = 0;      // Closed position: Left 0°
constexpr uint8_t SERVO_RIGHT_CLOSED = 180;  // Closed position: Right 180°
constexpr uint8_t SERVO_LEFT_COLLECT = 40;    // Collect position: Left 40°
constexpr uint8_t SERVO_RIGHT_COLLECT = 140;   // Collect position: Right 140°
constexpr uint8_t SERVO_LEFT_FLICK = 100;     // Flick position: Left 100°
constexpr uint8_t SERVO_RIGHT_FLICK = 80;     // Flick position: Right 80°
constexpr uint8_t SERVO_LEFT_OPEN = 100;      // Open position (same as flick)
constexpr uint8_t SERVO_RIGHT_OPEN = 80;      // Open position (same as flick)

/* ==================== Status LEDs (Optional) ==================== */
constexpr uint8_t LED1_PIN = 12;  // Status LED 1
constexpr uint8_t LED2_PIN = 13;  // Status LED 2

/* ==================== Serial Configuration ==================== */
constexpr unsigned long SERIAL_BAUD = 115200;

/* ==================== Timing Constants ==================== */
constexpr unsigned long SERIAL_PRINT_INTERVAL = 500; // ms between serial prints
constexpr unsigned long MOTOR_COMMAND_DELAY = 50;    // ms between motor commands

/* ==================== State Machine ==================== */
enum RobotState : uint8_t {
  STATE_INIT = 0,
  STATE_SEARCH_BALL,
  STATE_APPROACH_BALL,
  STATE_PICKUP_BALL,
  STATE_RETURN_BASE,
  STATE_DEPOSIT_BALL,
  STATE_OBSTACLE_AVOID,
  STATE_IDLE
};

static RobotState currentState = STATE_INIT;
static unsigned long lastSerialPrint = 0;
static unsigned long lastObstacleCheck = 0;

/* ==================== Detection Variables ==================== */
struct DetectedObject {
  bool found;
  uint8_t signature;
  int16_t x;
  int16_t y;
  int16_t width;
  int16_t height;
  long area;
};

static DetectedObject lastDetectedBall = {false, 0, 0, 0, 0, 0, 0};
static DetectedObject lastDetectedBase = {false, 0, 0, 0, 0, 0, 0};

/* ==================== Utility Functions ==================== */

long readUltrasonicCm() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  unsigned long duration = pulseIn(ECHO_PIN, HIGH, 30000UL);
  if (duration == 0) return -1;

  long distanceCm = static_cast<long>(duration * 0.0343f / 2.0f);
  return distanceCm;
}

bool isObstacleDetected() {
  long distance = readUltrasonicCm();
  return (distance > 0 && 
          distance >= static_cast<long>(OBSTACLE_CM_THRESHOLD - OBSTACLE_TOLERANCE) && 
          distance <= static_cast<long>(OBSTACLE_CM_THRESHOLD + OBSTACLE_TOLERANCE));
}

void scanForObjects() {
  pixy.ccc.getBlocks();
  
  // Reset detection flags
  lastDetectedBall.found = false;
  lastDetectedBase.found = false;
  
  if (pixy.ccc.numBlocks == 0) {
    return;
  }
  
  // Find the largest ball and base
  long largestBallArea = 0;
  long largestBaseArea = 0;
  
  for (int i = 0; i < pixy.ccc.numBlocks; i++) {
    Block& block = pixy.ccc.blocks[i];
    long area = static_cast<long>(block.m_width) * static_cast<long>(block.m_height);
    
    if (block.m_signature == SIG_BASE) {
      if (area > largestBaseArea) {
        largestBaseArea = area;
        lastDetectedBase.found = true;
        lastDetectedBase.signature = block.m_signature;
        lastDetectedBase.x = block.m_x;
        lastDetectedBase.y = block.m_y;
        lastDetectedBase.width = block.m_width;
        lastDetectedBase.height = block.m_height;
        lastDetectedBase.area = area;
      }
    } else if (block.m_signature == SIG_GREEN || block.m_signature == SIG_BLUE) {
      if (area > largestBallArea) {
        largestBallArea = area;
        lastDetectedBall.found = true;
        lastDetectedBall.signature = block.m_signature;
        lastDetectedBall.x = block.m_x;
        lastDetectedBall.y = block.m_y;
        lastDetectedBall.width = block.m_width;
        lastDetectedBall.height = block.m_height;
        lastDetectedBall.area = area;
      }
    }
    // Red ball (SIG_RED) is ignored
  }
}

void printStatus(const char* message) {
  unsigned long now = millis();
  if (now - lastSerialPrint >= SERIAL_PRINT_INTERVAL) {
    Serial.println(message);
    lastSerialPrint = now;
  }
}

/* ==================== Encoder Interrupt Handlers ==================== */

void encoderLeftISR() {
  // Read channel B to determine direction
  if (digitalRead(ENCA_B) == HIGH) {
    countA++;  // Forward
  } else {
    countA--;  // Backward
  }
}

void encoderRightISR() {
  // Read channel B to determine direction
  if (digitalRead(ENCB_B) == HIGH) {
    countB++;  // Forward
  } else {
    countB--;  // Backward
  }
}

/* ==================== Motor Control Functions ==================== */

void motorStop() {
  // Stop both motors
  analogWrite(PWMA, 0);
  analogWrite(PWMB, 0);
}

void motorForward(uint8_t speed = MOTOR_SPEED_NORMAL) {
  // Both motors forward (DIRA LOW = forward, DIRB LOW = forward)
  digitalWrite(DIRA, LOW);   // Left motor forward
  digitalWrite(DIRB, LOW);   // Right motor forward
  analogWrite(PWMA, speed);
  analogWrite(PWMB, speed);
}

void motorBackward(uint8_t speed = MOTOR_SPEED_NORMAL) {
  // Both motors backward (DIRA HIGH = backward, DIRB HIGH = backward)
  digitalWrite(DIRA, HIGH);  // Left motor backward
  digitalWrite(DIRB, HIGH);  // Right motor backward
  analogWrite(PWMA, speed);
  analogWrite(PWMB, speed);
}

void motorTurnLeft(uint8_t speed = MOTOR_SPEED_TURN) {
  // Left motor backward, right motor forward
  digitalWrite(DIRA, HIGH);  // Left motor backward
  digitalWrite(DIRB, LOW);   // Right motor forward
  analogWrite(PWMA, speed);
  analogWrite(PWMB, speed);
}

void motorTurnRight(uint8_t speed = MOTOR_SPEED_TURN) {
  // Left motor forward, right motor backward
  digitalWrite(DIRA, LOW);   // Left motor forward
  digitalWrite(DIRB, HIGH);   // Right motor backward
  analogWrite(PWMA, speed);
  analogWrite(PWMB, speed);
}

void motorTurnLeftSlow(uint8_t speed = MOTOR_SPEED_SLOW) {
  // Left motor stop, right motor forward (pivot turn)
  digitalWrite(DIRA, LOW);   // Direction doesn't matter when stopped
  digitalWrite(DIRB, LOW);   // Right motor forward
  analogWrite(PWMA, 0);      // Left motor stopped
  analogWrite(PWMB, speed);
}

void motorTurnRightSlow(uint8_t speed = MOTOR_SPEED_SLOW) {
  // Left motor forward, right motor stop (pivot turn)
  digitalWrite(DIRA, LOW);   // Left motor forward
  digitalWrite(DIRB, LOW);   // Direction doesn't matter when stopped
  analogWrite(PWMA, speed);
  analogWrite(PWMB, 0);      // Right motor stopped
}

/* ==================== Dual Servo/Gripper Control ==================== */

void gripperOpen() {
  // Open position (flick position)
  leftServo.write(SERVO_LEFT_OPEN);
  rightServo.write(SERVO_RIGHT_OPEN);
  delay(1000); // Wait for servos to move
}

void gripperClose() {
  // Closed position
  leftServo.write(SERVO_LEFT_CLOSED);
  rightServo.write(SERVO_RIGHT_CLOSED);
  delay(1000); // Wait for servos to move
}

void gripperCollect() {
  // Collect position (ready to pick up)
  leftServo.write(SERVO_LEFT_COLLECT);
  rightServo.write(SERVO_RIGHT_COLLECT);
  delay(1000); // Wait for servos to move
}

void gripperFlick() {
  // Flick position (for depositing)
  leftServo.write(SERVO_LEFT_FLICK);
  rightServo.write(SERVO_RIGHT_FLICK);
  delay(1000); // Wait for servos to move
}

/* ==================== Navigation Functions ==================== */

void centerOnObject(DetectedObject& obj) {
  // Pixy2 resolution: 316x208, center X is around 158
  const int16_t CENTER_X = 158;
  const int16_t TOLERANCE = 15; // Pixels tolerance for centering
  
  int16_t errorX = obj.x - CENTER_X;
  
  if (abs(errorX) <= TOLERANCE) {
    // Object is centered, move forward slowly
    motorForward(MOTOR_SPEED_SLOW);
  } else if (errorX > 0) {
    // Object is to the right, turn right
    motorTurnRightSlow();
  } else {
    // Object is to the left, turn left
    motorTurnLeftSlow();
  }
}

bool isObjectCloseEnough(DetectedObject& obj, long minArea = 8000) {
  // Larger area means object is closer
  return obj.area >= minArea;
}

void avoidObstacle() {
  // Stop, backup, turn away
  motorStop();
  delay(200);
  motorBackward(MOTOR_SPEED_SLOW);
  delay(500);
  motorStop();
  delay(200);
  motorTurnRight();
  delay(300);
  motorStop();
}

/* ==================== State Machine Logic ==================== */

void executeState() {
  switch (currentState) {
    case STATE_INIT:
      printStatus("[STATE] Initializing...");
      gripperOpen(); // Start with gripper open
      delay(1000);
      currentState = STATE_SEARCH_BALL;
      break;
      
    case STATE_SEARCH_BALL:
      scanForObjects();
      
      // Check for obstacles first
      if (isObstacleDetected()) {
        currentState = STATE_OBSTACLE_AVOID;
        break;
      }
      
      if (lastDetectedBall.found) {
        printStatus("[STATE] Ball found! Approaching...");
        currentState = STATE_APPROACH_BALL;
      } else {
        // Search pattern: move forward, turn periodically
        motorForward();
      }
      break;
      
    case STATE_APPROACH_BALL:
      scanForObjects();
      
      // Check for obstacles
      if (isObstacleDetected()) {
        currentState = STATE_OBSTACLE_AVOID;
        break;
      }
      
      if (!lastDetectedBall.found) {
        // Lost ball, go back to search
        currentState = STATE_SEARCH_BALL;
        break;
      }
      
      if (isObjectCloseEnough(lastDetectedBall)) {
        // Close enough to pick up
        motorStop();
        printStatus("[STATE] Close enough for pickup!");
        currentState = STATE_PICKUP_BALL;
      } else {
        // Center and approach
        centerOnObject(lastDetectedBall);
      }
      break;
      
    case STATE_PICKUP_BALL:
      motorStop();
      gripperCollect();  // Move to collect position
      delay(500);
      gripperClose();     // Close to grab ball
      delay(500);
      printStatus("[STATE] Ball picked up! Returning to base...");
      currentState = STATE_RETURN_BASE;
      break;
      
    case STATE_RETURN_BASE:
      scanForObjects();
      
      // Check for obstacles
      if (isObstacleDetected()) {
        currentState = STATE_OBSTACLE_AVOID;
        break;
      }
      
      if (lastDetectedBase.found) {
        if (isObjectCloseEnough(lastDetectedBase, 10000)) {
          // Close enough to deposit
          motorStop();
          printStatus("[STATE] At base! Depositing ball...");
          currentState = STATE_DEPOSIT_BALL;
        } else {
          // Center and approach base
          centerOnObject(lastDetectedBase);
        }
      } else {
        // Search for base (rotate in place)
        motorTurnLeft();
      }
      break;
      
    case STATE_DEPOSIT_BALL:
      motorStop();
      gripperFlick();     // Flick to deposit ball
      delay(1000);
      gripperOpen();      // Return to open position
      delay(500);
      motorBackward(MOTOR_SPEED_SLOW);
      delay(500);
      motorStop();
      printStatus("[STATE] Ball deposited! Searching for next ball...");
      delay(1000);
      currentState = STATE_SEARCH_BALL;
      break;
      
    case STATE_OBSTACLE_AVOID:
      avoidObstacle();
      delay(500);
      currentState = STATE_SEARCH_BALL;
      break;
      
    case STATE_IDLE:
    default:
      motorStop();
      break;
  }
}

/* ==================== Setup ==================== */

void setup() {
  Serial.begin(SERIAL_BAUD);
  unsigned long t0 = millis();
  while (!Serial && (millis() - t0) < 2000) {}
  
  Serial.println("=== Delivery Robot Initializing ===");
  
  // Initialize SPI (required for Pixy2 on Uno)
  // Note: Pin 10 is used for leftServo, but SPI SS is handled automatically
  // If you have issues, you may need to use a different pin for the servo
  
  // Initialize ultrasonic sensor
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  digitalWrite(TRIG_PIN, LOW);
  
  // Initialize motor pins
  pinMode(PWMA, OUTPUT);
  pinMode(DIRA, OUTPUT);
  pinMode(PWMB, OUTPUT);
  pinMode(DIRB, OUTPUT);
  motorStop();
  
  // Initialize encoder pins
  pinMode(ENCA_A, INPUT_PULLUP);
  pinMode(ENCA_B, INPUT_PULLUP);
  pinMode(ENCB_A, INPUT_PULLUP);
  pinMode(ENCB_B, INPUT_PULLUP);
  
  // Attach encoder interrupts
  attachInterrupt(digitalPinToInterrupt(ENCA_A), encoderLeftISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCB_A), encoderRightISR, CHANGE);
  
  // Reset encoder counts
  countA = 0;
  countB = 0;
  
  // Initialize dual servos
  leftServo.attach(SERVO_LEFT_PIN);
  rightServo.attach(SERVO_RIGHT_PIN);
  gripperOpen(); // Start with gripper open
  
  // Initialize LEDs (optional)
  pinMode(LED1_PIN, OUTPUT);
  pinMode(LED2_PIN, OUTPUT);
  digitalWrite(LED1_PIN, LOW);
  digitalWrite(LED2_PIN, LOW);
  
  // Initialize Pixy2
  pixy.init();
  Serial.println("Pixy2 initialized");
  Serial.println("=== Ready to start ===");
  
  delay(1000);
  currentState = STATE_INIT;
}

/* ==================== Main Loop ==================== */

void loop() {
  executeState();
  delay(MOTOR_COMMAND_DELAY);
}

