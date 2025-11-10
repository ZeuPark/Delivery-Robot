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

/* ==================== TUNING CONSTANTS TABLE ==================== */
// All tuning values in one place - adjust according to competition environment

// Motor speeds (0-255 PWM)
constexpr uint8_t MOTOR_SPEED_NORMAL = 150;
constexpr uint8_t MOTOR_SPEED_SLOW = 100;
constexpr uint8_t MOTOR_SPEED_TURN = 120;
constexpr uint8_t MOTOR_BASE_SPEED = 120;  // Proportional control base speed

// Pixy2 Vision
constexpr int16_t PIXY_CENTER_X = 158;     // Pixy2 center X coordinate
constexpr int16_t PIXY_CENTER_TOLERANCE = 15;  // Pixels tolerance for centering
constexpr long BALL_MIN_AREA = 800;        // Minimum ball area to detect
constexpr long BALL_PICKUP_AREA = 8000;    // Area threshold for pickup
constexpr long BASE_MIN_AREA = 1500;       // Minimum base area to detect
constexpr long BASE_DEPOSIT_AREA = 10000;  // Area threshold for deposit
constexpr float ASPECT_RATIO_MIN = 0.5f;   // Minimum width/height ratio
constexpr float ASPECT_RATIO_MAX = 2.0f;   // Maximum width/height ratio
constexpr int PIXY_BRIGHTNESS = 117;        // Camera brightness (0-255)

// Ultrasonic Sensor
constexpr uint8_t TRIG_PIN = A0;           // Pin 14 (A0 as digital)
constexpr uint8_t ECHO_PIN = A1;           // Pin 15 (A1 as digital)
constexpr float OBSTACLE_CM_THRESHOLD = 10.0f;
constexpr float OBSTACLE_TOLERANCE = 0.5f; // Detection range: 9.5cm to 10.5cm
constexpr int SONAR_MEDIAN_SIZE = 5;       // Median filter buffer size

// Encoder Calibration
constexpr float WHEEL_DIAMETER_CM = 6.5f;   // Wheel diameter (cm) - needs measurement
constexpr float GEAR_RATIO = 1.0f;         // Gear ratio - needs measurement
constexpr int ENCODER_PPR = 20;            // Encoder pulses per rotation - needs measurement
constexpr float COUNTS_PER_CM = (float)(ENCODER_PPR * GEAR_RATIO) / (WHEEL_DIAMETER_CM * 3.14159f);  // Counts per cm

static_assert(SONAR_MEDIAN_SIZE == 5 || SONAR_MEDIAN_SIZE == 7, "median size sanity");

// State Machine Timeouts (ms)
constexpr unsigned long TIMEOUT_SEARCH_BALL = 30000;   // 30 seconds
constexpr unsigned long TIMEOUT_APPROACH_BALL = 5000;  // 5 seconds
constexpr unsigned long TIMEOUT_PICKUP_BALL = 3000;    // 3 seconds
constexpr unsigned long TIMEOUT_RETURN_BASE = 30000;   // 30 seconds
constexpr unsigned long TIMEOUT_DEPOSIT_BALL = 3000;   // 3 seconds

// Navigation
constexpr float BACKUP_DISTANCE_CM = 10.0f;    // Obstacle avoidance backup distance
constexpr float TURN_ANGLE_DEG = 90.0f;         // Obstacle avoidance turn angle
constexpr float TRACK_WIDTH_CM = 14.0f;        // Track width (cm) - measure actual value

// Safety
constexpr uint8_t KILL_PIN = A2;                // Emergency stop pin (pull-up to GND)

// Kill switch debouncing
bool killAsserted() {
  static unsigned long t = 0;
  if (digitalRead(KILL_PIN) == LOW) {
    if (millis() - t > 30) return true;  // Debounce: 30ms
  } else {
    t = millis();
  }
  return false;
}

// Telemetry
constexpr unsigned long TELEMETRY_INTERVAL = 200;  // ms between telemetry logs

// Tick-based scheduler
constexpr unsigned long MAIN_TICK_MS = 20;
unsigned long lastTick = 0;

/* ==================== Pixy2 Configuration ==================== */
Pixy2 pixy;

// PixyMon signatures
constexpr uint8_t SIG_BASE  = 1; // Base
constexpr uint8_t SIG_RED   = 2; // Forbidden ball
constexpr uint8_t SIG_GREEN = 3; // Allowed ball
constexpr uint8_t SIG_BLUE  = 4; // Allowed ball

uint8_t myBaseSig = 0;  // Detected base signature (0 = not detected yet)

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

// Encoder debouncing
volatile unsigned long lastEdgeA = 0;
volatile unsigned long lastEdgeB = 0;
constexpr unsigned long ENC_DEBOUNCE_US = 400;  // Increased for stability (200->400us)

// Encoder counts
volatile long countA = 0;  // Left encoder count
volatile long countB = 0;   // Right encoder count

/* ==================== Ultrasonic Sensor ==================== */
// Using analog pins A0/A1 as digital pins to avoid conflicts with encoders

/* ==================== Dual Servo/Gripper Control ==================== */
Servo leftServo;
Servo rightServo;
constexpr uint8_t SERVO_LEFT_PIN = 11;
constexpr uint8_t SERVO_RIGHT_PIN = A3;  // Use analog pin as digital (avoid SPI SS conflict)

// Servo positions (microseconds) - needs calibration
constexpr int L_CLOSED = 1000;     // Left closed (example value)
constexpr int L_COLLECT = 1500;     // Left collect (example value)
constexpr int L_OPEN = 1900;       // Left open (example value)
constexpr int R_CLOSED = 2000;     // Right closed (example value)
constexpr int R_COLLECT = 1500;    // Right collect (example value)
constexpr int R_OPEN = 1100;       // Right open (example value)

struct ServoJob {
  int l_us, r_us;
  unsigned long t0;
  unsigned long holdMs;
  bool busy;
} sjob = {0, 0, 0, 0, false};

/* ==================== Status LEDs (Optional) ==================== */
constexpr uint8_t LED1_PIN = 12;  // Status LED 1
constexpr uint8_t LED2_PIN = 13;  // Status LED 2

/* ==================== Serial Configuration ==================== */
constexpr unsigned long SERIAL_BAUD = 115200;

/* ==================== Timing Constants ==================== */
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
static unsigned long stateEnterAt = 0;  // State entry timestamp for timeout

// Gripper substeps (fully non-blocking)
enum PickupStep { PK_COLLECT, PK_CLOSE, PK_VERIFY };
enum DepositStep { DP_FLICK, DP_OPEN, DP_BACKUP };
PickupStep pkStep = PK_COLLECT;
DepositStep dpStep = DP_FLICK;

// State recovery tracking
static unsigned long searchStart = 0;

// Motor command tracking for telemetry
int lastPWM_L = 0;
int lastPWM_R = 0;

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

// Pixy smoothing (EMA)
struct PixySMA {
  float x = -1;
  float area = -1;
} ballS, baseS;

float ema(float prev, float cur, float alpha) {
  if (prev < 0) return cur;
  return prev + alpha * (cur - prev);
}

// Ultrasonic overflow-safe comparison
inline bool afterOrEqual(unsigned long a, unsigned long b) {
  return (long)(a - b) >= 0;
}

/* ==================== Ultrasonic Sensor (Non-blocking with Median Filter) ==================== */

// Periodic asynchronous ultrasonic measurement
struct Sonar {
  enum {IDLE, TRIG_HIGH, WAIT_ECHO} phase = IDLE;
  unsigned long t0 = 0, echoStart = 0, echoEnd = 0;
  long dist = -1;
} sonar;

// Ultrasonic noise suppression - median filter
const int N = SONAR_MEDIAN_SIZE;
long sonarBuf[N];
int idx = 0;

void pushSonar(long v) {
  sonarBuf[idx++ % N] = v;
}

long medianN() {
  long tmp[N];
  int m = 0;
  // Filter out negative values
  for (int i = 0; i < N; i++) {
    if (sonarBuf[i] >= 0) tmp[m++] = sonarBuf[i];
  }
  if (m == 0) return -1;
  
  // Insertion sort
  for (int i = 1; i < m; i++) {
    long v = tmp[i];
    int j = i - 1;
    while (j >= 0 && tmp[j] > v) {
      tmp[j + 1] = tmp[j];
      j--;
    }
    tmp[j + 1] = v;
  }
  return tmp[m / 2];  // Median value
}

void serviceSonar() {
  unsigned long now = millis();
  static unsigned long lastShot = 0;
  if (now - lastShot < 30) return;

  switch (sonar.phase) {
    case Sonar::IDLE:
      digitalWrite(TRIG_PIN, LOW);
      delayMicroseconds(2);
      digitalWrite(TRIG_PIN, HIGH);
      delayMicroseconds(10);
      digitalWrite(TRIG_PIN, LOW);
      sonar.t0 = micros();
      sonar.phase = Sonar::WAIT_ECHO;
      lastShot = now;
      break;
    case Sonar::WAIT_ECHO: {
      unsigned long m = micros();
      if (digitalRead(ECHO_PIN) == HIGH && sonar.echoStart == 0) sonar.echoStart = m;
      if (digitalRead(ECHO_PIN) == LOW && sonar.echoStart != 0 && sonar.echoEnd == 0) {
        unsigned long dur = afterOrEqual(m, sonar.echoStart) ? (m - sonar.echoStart) : 0;
        long rawDist = dur > 0 ? (long)(dur * 0.0343f / 2.0f) : -1;
        pushSonar(rawDist);  // Add to median filter buffer
        sonar.dist = medianN();  // Use median value (filters negatives)
        sonar.echoStart = sonar.echoEnd = 0;
        sonar.phase = Sonar::IDLE;
      }
      if (afterOrEqual(m, sonar.t0 + 30000UL)) {
        sonar.dist = -1;
        pushSonar(-1);  // Clear buffer residue to prevent false obstacle detection
        sonar.phase = Sonar::IDLE;
      }
    } break;
  }
}

bool isObstacleDetectedCached() {
  static uint8_t hit = 0;
  bool ok = sonar.dist > 0 && sonar.dist <= (long)OBSTACLE_CM_THRESHOLD;  // Detect if distance is below threshold
  
  if (ok) {
    if (hit < 3) hit++;
  } else if (hit > 0) {
    hit--;
  }
  
  return hit >= 2;  // Hysteresis: need 2 consecutive hits
}

/* ==================== Encoder Reading (Atomic) ==================== */

// Ensure encoder reading accuracy and atomicity
long getCountsA() {
  noInterrupts();
  long v = countA;
  interrupts();
  return v;
}

long getCountsB() {
  noInterrupts();
  long v = countB;
  interrupts();
  return v;
}

/* ==================== Pixy2 Configuration ==================== */

void configurePixy() {
  pixy.setLamp(1, 1);  // LED on
  pixy.setCameraBrightness(PIXY_BRIGHTNESS);  // Adjust according to field lighting
  pixy.changeProg("color_connected_components");
}

bool passBallFilter(const Block& b) {
  long area = (long)b.m_width * b.m_height;
  if (area < BALL_MIN_AREA) return false;
  float ar = (float)b.m_width / (float)b.m_height;
  return ar > ASPECT_RATIO_MIN && ar < ASPECT_RATIO_MAX;
}

bool passBaseFilter(const Block& b) {
  long area = (long)b.m_width * b.m_height;
  return area >= BASE_MIN_AREA;  // Base has relaxed shape constraints
}

/* ==================== Base Detection ==================== */

void detectMyBaseOnce() {
  if (myBaseSig != 0) return;  // Already detected
  
  static uint8_t votes[5] = {0};
  
  // Sample 5 frames for voting
  for (int k = 0; k < 5; k++) {
    pixy.ccc.getBlocks();
    int bestIdx = -1;
    long bestArea = 0;
    
    for (int i = 0; i < pixy.ccc.numBlocks; i++) {
      Block& b = pixy.ccc.blocks[i];
      long area = (long)b.m_width * b.m_height;
      
      if (b.m_signature <= 4 && passBaseFilter(b) && area > bestArea) {
        bestArea = area;
        bestIdx = i;
      }
    }
    
    if (bestIdx >= 0) {
      votes[pixy.ccc.blocks[bestIdx].m_signature]++;
    }
    delay(20);
  }
  
  // Find signature with most votes
  uint8_t bestSig = 0;
  uint8_t bestVote = 0;
  for (uint8_t s = 1; s <= 4; s++) {
    if (votes[s] > bestVote) {
      bestVote = votes[s];
      bestSig = s;
    }
  }
  
  // Require at least 3 votes out of 5
  if (bestVote >= 3) {
    myBaseSig = bestSig;
    Serial.print("Base sig=");
    Serial.println(myBaseSig);
  }
}

/* ==================== Utility Functions ==================== */

void scanForObjects() {
  static unsigned long last = 0;
  if (millis() - last < 40) return;  // 25 Hz frame rate limit
  last = millis();
  
  pixy.ccc.getBlocks();
  
  // Reset detection flags
  lastDetectedBall.found = false;
  lastDetectedBase.found = false;
  
  if (pixy.ccc.numBlocks == 0) {
    return;
  }
  
  // Find the largest ball and base (with shape filter)
  long largestBallArea = 0;
  long largestBaseArea = 0;
  
  for (int i = 0; i < pixy.ccc.numBlocks; i++) {
    Block& block = pixy.ccc.blocks[i];
    long area = (long)block.m_width * block.m_height;
    
    // Base detection (using myBaseSig with separate filter)
    if (myBaseSig != 0 && block.m_signature == myBaseSig && passBaseFilter(block)) {
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
    } else if ((block.m_signature == SIG_GREEN || block.m_signature == SIG_BLUE) && passBallFilter(block)) {
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
  
  // Reset area to 0 when object is not detected to prevent stale values from affecting decisions
  if (!lastDetectedBall.found) lastDetectedBall.area = 0;
  if (!lastDetectedBase.found) lastDetectedBase.area = 0;
}

void smoothDetections() {
  if (lastDetectedBall.found) {
    ballS.x = ema(ballS.x, lastDetectedBall.x, 0.3f);
    ballS.area = ema(ballS.area, lastDetectedBall.area, 0.3f);
  } else {
    // Reset EMA when ball is not detected to prevent stale values
    ballS.x = -1;
    ballS.area = -1;
  }
  if (lastDetectedBase.found) {
    baseS.x = ema(baseS.x, lastDetectedBase.x, 0.3f);
    baseS.area = ema(baseS.area, lastDetectedBase.area, 0.3f);
  } else {
    // Reset EMA when base is not detected to prevent stale values
    baseS.x = -1;
    baseS.area = -1;
  }
}

/* ==================== State Management ==================== */

void enterState(RobotState s) {
  currentState = s;
  stateEnterAt = millis();
  
  // Initialize substeps
  if (s == STATE_PICKUP_BALL) {
    pkStep = PK_COLLECT;
  }
  if (s == STATE_DEPOSIT_BALL) {
    dpStep = DP_FLICK;
  }
  if (s == STATE_RETURN_BASE) {
    searchStart = 0;  // Reset search tracking
  }
  
  // State transition telemetry
  Serial.print("#STATE,");
  Serial.print(stateName(s));
  Serial.print(",t=");
  Serial.println(stateEnterAt);
}

bool expired(unsigned long ms) {
  return millis() - stateEnterAt > ms;
}

/* ==================== Encoder Interrupt Handlers ==================== */

void encoderLeftISR() {
  unsigned long t = micros();
  if (afterOrEqual(t, lastEdgeA + ENC_DEBOUNCE_US)) {
    lastEdgeA = t;
    countA += digitalRead(ENCA_B) ? 1 : -1;
  }
}

void encoderRightISR() {
  unsigned long t = micros();
  if (afterOrEqual(t, lastEdgeB + ENC_DEBOUNCE_US)) {
    lastEdgeB = t;
    countB += digitalRead(ENCB_B) ? 1 : -1;
  }
}

/* ==================== Motor Control Functions ==================== */

void motorStop() {
  // Stop both motors
  analogWrite(PWMA, 0);
  analogWrite(PWMB, 0);
  lastPWM_L = 0;
  lastPWM_R = 0;
}

void motorForward(uint8_t speed = MOTOR_SPEED_NORMAL) {
  // Both motors forward (DIRA LOW = forward, DIRB LOW = forward)
  digitalWrite(DIRA, LOW);   // Left motor forward
  digitalWrite(DIRB, LOW);   // Right motor forward
  analogWrite(PWMA, speed);
  analogWrite(PWMB, speed);
  lastPWM_L = speed;
  lastPWM_R = speed;
}

void motorBackward(uint8_t speed = MOTOR_SPEED_NORMAL) {
  // Both motors backward (DIRA HIGH = backward, DIRB HIGH = backward)
  digitalWrite(DIRA, HIGH);  // Left motor backward
  digitalWrite(DIRB, HIGH);  // Right motor backward
  analogWrite(PWMA, speed);
  analogWrite(PWMB, speed);
  lastPWM_L = speed;
  lastPWM_R = speed;
}

void motorTurnLeft(uint8_t speed = MOTOR_SPEED_TURN) {
  // Left motor backward, right motor forward
  digitalWrite(DIRA, HIGH);  // Left motor backward
  digitalWrite(DIRB, LOW);   // Right motor forward
  analogWrite(PWMA, speed);
  analogWrite(PWMB, speed);
  lastPWM_L = speed;
  lastPWM_R = speed;
}

void motorTurnRight(uint8_t speed = MOTOR_SPEED_TURN) {
  // Left motor forward, right motor backward
  digitalWrite(DIRA, LOW);   // Left motor forward
  digitalWrite(DIRB, HIGH);   // Right motor backward
  analogWrite(PWMA, speed);
  analogWrite(PWMB, speed);
  lastPWM_L = speed;
  lastPWM_R = speed;
}

void motorTurnLeftSlow(uint8_t speed = MOTOR_SPEED_SLOW) {
  // Left motor stop, right motor forward (pivot turn)
  digitalWrite(DIRA, LOW);   // Direction doesn't matter when stopped
  digitalWrite(DIRB, LOW);   // Right motor forward
  analogWrite(PWMA, 0);      // Left motor stopped
  analogWrite(PWMB, speed);
  lastPWM_L = 0;
  lastPWM_R = speed;
}

void motorTurnRightSlow(uint8_t speed = MOTOR_SPEED_SLOW) {
  // Left motor forward, right motor stop (pivot turn)
  digitalWrite(DIRA, LOW);   // Left motor forward
  digitalWrite(DIRB, LOW);   // Direction doesn't matter when stopped
  analogWrite(PWMA, speed);
  analogWrite(PWMB, 0);      // Right motor stopped
  lastPWM_L = speed;
  lastPWM_R = 0;
}

/* ==================== Dual Servo/Gripper Control (Non-blocking with Microseconds) ==================== */

// Gripper control calibrated to microseconds - move non-blocking
void gripperMove(int l_us, int r_us, unsigned long hold) {
  leftServo.writeMicroseconds(l_us);
  rightServo.writeMicroseconds(r_us);
  sjob.l_us = l_us;
  sjob.r_us = r_us;
  sjob.t0 = millis();
  sjob.holdMs = hold;
  sjob.busy = true;
}

void serviceGripper() {
  if (sjob.busy && millis() - sjob.t0 > sjob.holdMs) {
    sjob.busy = false;
  }
}

bool gripperIdle() {
  return !sjob.busy;
}

void gripperOpen() {
  gripperMove(L_OPEN, R_OPEN, 1000);
}

void gripperClose() {
  gripperMove(L_CLOSED, R_CLOSED, 1000);
}

void gripperCollect() {
  gripperMove(L_COLLECT, R_COLLECT, 1000);
}

void gripperFlick() {
  gripperMove(L_OPEN, R_OPEN, 1000);  // Flick uses open position
}

/* ==================== Navigation Functions ==================== */

// Replace rotation control with proportional control - differential PWM distribution based on Pixy X error
void driveToCenterX(int16_t x, int baseSpeed = MOTOR_BASE_SPEED) {
  const int16_t CX = PIXY_CENTER_X;
  int16_t err = x - CX;
  
  // Deadband: go straight if within tolerance range
  if (abs(err) <= PIXY_CENTER_TOLERANCE) {
    digitalWrite(DIRA, LOW);
    digitalWrite(DIRB, LOW);
    analogWrite(PWMA, baseSpeed);
    analogWrite(PWMB, baseSpeed);
    lastPWM_L = baseSpeed;
    lastPWM_R = baseSpeed;
    return;
  }
  
  // Proportional gain with constrained output
  const float Kp = 0.6f;
  int u = constrain((int)(Kp * err), -120, 120);
  int left = constrain(baseSpeed - u, 0, 255);
  int right = constrain(baseSpeed + u, 0, 255);
  digitalWrite(DIRA, LOW);
  digitalWrite(DIRB, LOW);
  analogWrite(PWMA, left);
  analogWrite(PWMB, right);
  lastPWM_L = left;
  lastPWM_R = right;
}

void approachWithArea(const DetectedObject& obj) {
  // Use smoothed area for speed calculation
  // As distance decreases, area increases, so speed should decrease (slower when closer)
  // Check if smoothed values are valid (not -1)
  if (ballS.area < 0 || ballS.x < 0) {
    // Fallback to raw detection values if smoothing not initialized
    driveToCenterX(obj.x, MOTOR_SPEED_SLOW);
    return;
  }
  long a = max(1L, (long)ballS.area);
  float ratio = min(1.0f, (float)a / (float)BALL_PICKUP_AREA);
  int v = MOTOR_SPEED_SLOW + (int)((1.0f - ratio) * 80);  // Slower when closer
  driveToCenterX((int)ballS.x, v);
}

void centerOnObject(const DetectedObject& obj) {
  driveToCenterX(obj.x, MOTOR_BASE_SPEED);  // Use parameter instead of baseS
}

bool isObjectCloseEnough(DetectedObject& obj, long minArea = BALL_PICKUP_AREA) {
  // Use smoothed area for comparison
  if (obj.signature == SIG_GREEN || obj.signature == SIG_BLUE) {
    return ballS.area >= minArea;
  } else {
    return baseS.area >= minArea;
  }
}

bool pickupConfirmed() {
  static unsigned long tFirst = 0;
  if (tFirst == 0) tFirst = millis();
  if (millis() - tFirst < 250) return false;  // Minimum wait time (gripper closing time)
  
  bool ballVisible = false;
  pixy.ccc.getBlocks();
  for (int i = 0; i < pixy.ccc.numBlocks; i++) {
    auto& b = pixy.ccc.blocks[i];
    if ((b.m_signature == SIG_GREEN || b.m_signature == SIG_BLUE) &&
        ((long)b.m_width * b.m_height) > BALL_MIN_AREA) {
      ballVisible = true;
      break;
    }
  }
  
  // Check for proximity signal: if ball is in gripper, close-range reflection reduces distance
  bool nearObstacleRise = (sonar.dist > 0 && sonar.dist < 12);
  
  if (!ballVisible || nearObstacleRise) {
    tFirst = 0;  // Reset for next pickup
    return true;
  }
  return false;
}

const char* stateName(RobotState s) {
  switch (s) {
    case STATE_INIT: return "INIT";
    case STATE_SEARCH_BALL: return "SEARCH";
    case STATE_APPROACH_BALL: return "APPROACH";
    case STATE_PICKUP_BALL: return "PICKUP";
    case STATE_RETURN_BASE: return "RETURN";
    case STATE_DEPOSIT_BALL: return "DEPOSIT";
    case STATE_OBSTACLE_AVOID: return "AVOID";
    case STATE_IDLE:
    default: return "IDLE";
  }
}

/* ==================== Encoder-Based Movement ==================== */

// Heading correction for straight driving
void driveStraightCounts(long target, int base) {
  long a0 = getCountsA();
  long b0 = getCountsB();
  
  while (true) {
    long da = getCountsA() - a0;
    long db = getCountsB() - b0;
    if (abs(da) >= target && abs(db) >= target) break;
    
    int err = (int)(da - db);  // Left-right difference
    int corr = constrain(err, -30, 30);  // Simple proportional correction
    int la = constrain(base - corr, 0, 255);
    int rb = constrain(base + corr, 0, 255);
    
    digitalWrite(DIRA, LOW);
    digitalWrite(DIRB, LOW);
    analogWrite(PWMA, la);
    analogWrite(PWMB, rb);
    lastPWM_L = la;
    lastPWM_R = rb;
    
    serviceSonar();
    serviceGripper();
    if (isObstacleDetectedCached()) {
      motorStop();
      return;
    }
  }
  motorStop();
}

bool moveForwardCm_blocking(float cm, int sp, unsigned long timeoutMs = 2000) {
  long target = (long)(cm * COUNTS_PER_CM);
  long startA = getCountsA();
  long startB = getCountsB();
  unsigned long t0 = millis();
  
  digitalWrite(DIRA, LOW);
  digitalWrite(DIRB, LOW);
  
  while (true) {
    long da = abs(getCountsA() - startA);
    long db = abs(getCountsB() - startB);
    if (da >= target && db >= target) break;
    
    if (millis() - t0 > timeoutMs) {
      motorStop();
      return false;
    }
    
    if (isObstacleDetectedCached()) {
      motorStop();
      return false;
    }
    
    analogWrite(PWMA, sp);
    analogWrite(PWMB, sp);
    lastPWM_L = sp;
    lastPWM_R = sp;
    
    serviceSonar();
    serviceGripper();
  }
  motorStop();
  return true;
}

bool moveBackwardCm_blocking(float cm, int sp, unsigned long timeoutMs = 2000) {
  long target = (long)(cm * COUNTS_PER_CM);
  long startA = getCountsA();
  long startB = getCountsB();
  unsigned long t0 = millis();
  
  digitalWrite(DIRA, HIGH);
  digitalWrite(DIRB, HIGH);
  
  while (true) {
    long da = abs(getCountsA() - startA);
    long db = abs(getCountsB() - startB);
    if (da >= target && db >= target) break;
    
    if (millis() - t0 > timeoutMs) {
      motorStop();
      return false;
    }
    
    analogWrite(PWMA, sp);
    analogWrite(PWMB, sp);
    lastPWM_L = sp;
    lastPWM_R = sp;
    
    serviceSonar();
    serviceGripper();
  }
  motorStop();
  return true;
}

void moveForwardCm_precise(float cm, int sp) {
  long target = (long)(cm * COUNTS_PER_CM);
  driveStraightCounts(target, sp);
}

void turnDegrees_precise(float deg, int sp) {
  float arc = 3.14159f * TRACK_WIDTH_CM * deg / 360.0f;
  long target = (long)(arc * COUNTS_PER_CM);
  long a0 = getCountsA();
  long b0 = getCountsB();
  
  digitalWrite(DIRA, LOW);   // Left forward
  digitalWrite(DIRB, HIGH);  // Right backward
  
  while (true) {
    long da = abs(getCountsA() - a0);
    long db = abs(getCountsB() - b0);
    if (da >= target && db >= target) break;
    
    analogWrite(PWMA, sp);
    analogWrite(PWMB, sp);
    lastPWM_L = sp;
    lastPWM_R = sp;
    
    serviceSonar();
    serviceGripper();
  }
  motorStop();
}

// Change obstacle avoidance from delay-based to distance-based
void avoidObstacle() {
  motorStop();
  if (!moveBackwardCm_blocking(BACKUP_DISTANCE_CM, MOTOR_SPEED_SLOW)) return;
  turnDegrees_precise(TURN_ANGLE_DEG, MOTOR_SPEED_TURN);
}

/* ==================== State Machine Logic ==================== */

// Initialization helper
static bool initDone = false;

void executeState() {
  switch (currentState) {
    case STATE_INIT:
      if (!initDone) {
        detectMyBaseOnce();  // Detect base signature
        gripperOpen(); // Start with gripper open
        initDone = true;
      }
      // Wait for gripper to finish opening before proceeding
      if (gripperIdle() && (millis() - stateEnterAt) >= 1000) {
        enterState(STATE_SEARCH_BALL);
      }
      break;
      
    case STATE_SEARCH_BALL: {
      scanForObjects();
      smoothDetections();
      
      // State timeout check
      if (expired(TIMEOUT_SEARCH_BALL)) {
        motorStop();
        enterState(STATE_SEARCH_BALL);  // Reset timeout
        break;
      }
      
      // Check for obstacles first
      if (isObstacleDetectedCached()) {
        enterState(STATE_OBSTACLE_AVOID);
        break;
      }
      
      if (lastDetectedBall.found) {
        enterState(STATE_APPROACH_BALL);
        break;
      }
      
      // Enhanced search pattern: sweep (forward + turn)
      static unsigned long sweepT0 = 0;
      static bool turning = false;
      static unsigned long lastStateEntry = 0;
      
      // Initialize sweep timer on state entry (detect state change)
      if (lastStateEntry != stateEnterAt) {
        sweepT0 = stateEnterAt;  // Use state entry time
        lastStateEntry = stateEnterAt;
        turning = false;
      }
      
      unsigned long now = millis();
      if (!turning) {
        motorForward(MOTOR_SPEED_SLOW);
        if (now - sweepT0 > 1200) {  // Forward for 1.2s
          turning = true;
          sweepT0 = now;
          motorStop();
        }
      } else {
        motorTurnLeft(MOTOR_SPEED_TURN);
        if (now - sweepT0 > 800) {  // Turn for 0.8s
          turning = false;
          sweepT0 = now;
          motorStop();
        }
      }
    } break;
      
    case STATE_APPROACH_BALL:
      scanForObjects();
      smoothDetections();
      
      // State timeout check
      if (expired(TIMEOUT_APPROACH_BALL)) {
        motorStop();
        enterState(STATE_SEARCH_BALL);
        break;
      }
      
      // Check for obstacles
      if (isObstacleDetectedCached()) {
        enterState(STATE_OBSTACLE_AVOID);
        break;
      }
      
      if (!lastDetectedBall.found) {
        // Lost ball, go back to search
        enterState(STATE_SEARCH_BALL);
        break;
      }
      
      // Use smoothed values for decision (check if valid)
      if (ballS.area > 0 && ballS.area >= BALL_PICKUP_AREA) {
        motorStop();
        enterState(STATE_PICKUP_BALL);
      } else {
        // Center and approach with area-based speed control
        approachWithArea(lastDetectedBall);
      }
      break;
      
    case STATE_PICKUP_BALL:
      motorStop();
      
      // State timeout check
      if (expired(TIMEOUT_PICKUP_BALL)) {
        enterState(STATE_SEARCH_BALL);
        break;
      }
      
      // Non-blocking substep sequence
      if (!gripperIdle()) break;
      
      if (pkStep == PK_COLLECT) {
        gripperCollect();
        pkStep = PK_CLOSE;
        break;
      }
      
      if (pkStep == PK_CLOSE) {
        gripperClose();
        pkStep = PK_VERIFY;
        break;
      }
      
      if (pkStep == PK_VERIFY) {
        if (pickupConfirmed()) {
          enterState(STATE_RETURN_BASE);
        } else {
          // Pickup failed, retry approach
          enterState(STATE_APPROACH_BALL);
        }
      }
      break;
      
    case STATE_RETURN_BASE:
      scanForObjects();
      smoothDetections();
      
      // State timeout check
      if (expired(TIMEOUT_RETURN_BASE)) {
        motorStop();
        enterState(STATE_SEARCH_BALL);
        break;
      }
      
      // Check for obstacles
      if (isObstacleDetectedCached()) {
        enterState(STATE_OBSTACLE_AVOID);
        break;
      }
      
      // Recover base detection if lost
      if (myBaseSig == 0) {
        detectMyBaseOnce();
      }
      
      if (lastDetectedBase.found) {
        searchStart = 0;  // Reset search timer
        // Check if smoothed area is valid before comparing
        if (baseS.area > 0 && baseS.area >= BASE_DEPOSIT_AREA) {
          motorStop();
          enterState(STATE_DEPOSIT_BALL);
        } else {
          // Center and approach base using smoothed values
          centerOnObject(lastDetectedBase);
        }
      } else {
        // Enhanced search pattern
        if (searchStart == 0) searchStart = millis();
        
        motorTurnLeft();
        
        // After 1.5s of rotation, move forward then continue searching
        if (millis() - searchStart > 1500) {
          motorStop();
          if (!moveForwardCm_blocking(6.0f, MOTOR_SPEED_SLOW)) {
            enterState(STATE_SEARCH_BALL);  // Failed movement, reset
            break;
          }
          searchStart = millis();  // Reset for next cycle
        }
      }
      break;
      
    case STATE_DEPOSIT_BALL:
      motorStop();
      
      // State timeout check
      if (expired(TIMEOUT_DEPOSIT_BALL)) {
        enterState(STATE_SEARCH_BALL);
        break;
      }
      
      // Non-blocking substep sequence
      if (!gripperIdle()) break;
      
      if (dpStep == DP_FLICK) {
        gripperFlick();
        dpStep = DP_OPEN;
        break;
      }
      
      if (dpStep == DP_OPEN) {
        gripperOpen();
        dpStep = DP_BACKUP;
        break;
      }
      
      if (dpStep == DP_BACKUP) {
        if (!moveBackwardCm_blocking(5.0f, MOTOR_SPEED_SLOW)) {
          // Movement failed, proceed anyway
        }
        enterState(STATE_SEARCH_BALL);
      }
      break;
      
    case STATE_OBSTACLE_AVOID:
      avoidObstacle();
      // Use timestamp-based wait instead of delay
      if (expired(500)) {
        enterState(STATE_SEARCH_BALL);
      }
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
  pinMode(10, OUTPUT);  // SPI SS stabilization (servos moved to pin 11 and A3 to avoid conflict)
  digitalWrite(10, HIGH);
  
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
  
  // Initialize safety kill switch
  pinMode(KILL_PIN, INPUT_PULLUP);
  
  // Initialize Pixy2
  pixy.init();
  configurePixy();  // Configure LED, brightness, etc.
  Serial.println("Pixy2 initialized and configured");
  
  // Initialize sonar buffer
  for (int i = 0; i < N; i++) {
    sonarBuf[i] = -1;
  }
  
  // Warm up sonar with initial samples
  for (int k = 0; k < 8; k++) {
    serviceSonar();
    delay(35);
  }
  
  Serial.println("=== Ready to start ===");
  Serial.println("time,state,dist,ball_x,ball_area,base_x,base_area,countA,countB,pwm_L,pwm_R");  // CSV header
  
  enterState(STATE_INIT);
  initDone = false;
}

/* ==================== Telemetry Logging ==================== */

void logTelemetry() {
  static unsigned long last = 0;
  if (millis() - last < TELEMETRY_INTERVAL) return;
  last = millis();
  
  Serial.print(millis());
  Serial.print(',');
  Serial.print(stateName(currentState));
  Serial.print(',');
  Serial.print(sonar.dist);
  Serial.print(',');
  Serial.print(lastDetectedBall.found ? lastDetectedBall.x : -1);
  Serial.print(',');
  Serial.print(lastDetectedBall.area);
  Serial.print(',');
  Serial.print(lastDetectedBase.found ? lastDetectedBase.x : -1);
  Serial.print(',');
  Serial.print(lastDetectedBase.area);
  Serial.print(',');
  Serial.print(getCountsA());
  Serial.print(',');
  Serial.print(getCountsB());
  Serial.print(',');
  Serial.print(lastPWM_L);
  Serial.print(',');
  Serial.println(lastPWM_R);
}

/* ==================== Main Loop ==================== */

void loop() {
  // Safety check - emergency stop (with debouncing)
  if (killAsserted()) {
    motorStop();
    return;
  }
  
  // Serial command handling
  if (Serial.available()) {
    char c = Serial.read();
    if (c == 's') {
      motorStop();
    }
    if (c == 'r') {
      enterState(STATE_SEARCH_BALL);
    }
  }
  
  // Tick-based scheduler
  unsigned long now = millis();
  if (now - lastTick >= MAIN_TICK_MS) {
    lastTick = now;
    serviceSonar();      // Asynchronous ultrasonic service
    serviceGripper();   // Asynchronous gripper service
    logTelemetry();      // Telemetry logging
    executeState();      // State machine execution (decisions and motor commands only)
  }
}

