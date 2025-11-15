/*
 * Project purpose: autonomous delivery robot that grabs the nearest allowed ball with Pixy2 vision and deposits it at a base in a continuous loop.
 * Hardware map: Pixy2 on SPI (10-13 reserved), motors (M_A_PWM=5, M_A_DIR=4, M_B_PWM=6, M_B_DIR=14), dual gripper servos (SERVO_LEFT_PIN=11, SERVO_RIGHT_PIN=A3), LEDs on 2/7, dual ultrasonic trig/echo (8/9 and A1/A2), kill hardware not used here.
 * Pixy2 mode: uses "color_connected_components" program with signatures SIG_GREEN=1, SIG_BLUE=2 for balls and SIG_BASE=3 for the scoring base.
 * State machine summary: INIT primes hardware; SEEK_BALL spins to find a ball; APPROACH_BALL centers/alines; GRAB_BALL closes gripper; SEEK_BASE searches for base; APPROACH_BASE aligns/approaches; SCORE drives forward to deposit; RESET_TURN spins 180° and restarts cycle.
 * Tuning constants: CENTER_TOL_PIX keeps Pixy aim tolerance; TURN_MS_PER_DEG controls open-loop turning (calibrate by timing a known rotation); BALL_CLOSE_AREA tunes how near the ball must appear (adjust via observed blob size); BASE_LARGE_AREA determines when the base fills enough pixels; SCORE_MS sets drive duration into the base (tune to ensure scoring without stalling).
 */
#include <Pixy2.h>
#include <Servo.h>

// Fixed hardware constants (do not rename)
constexpr uint8_t LED1_PIN = 2;  // status LED 1 away from SPI pins
constexpr uint8_t LED2_PIN = 7;  // status LED 2 indicator
constexpr uint8_t US_TRIG  = 8;  // primary ultrasonic trigger
constexpr uint8_t US_ECHO  = 9;  // primary ultrasonic echo
// second ultrasonic
constexpr uint8_t US2_TRIG = A1;  // secondary ultrasonic trigger
constexpr uint8_t US2_ECHO = A2;  // secondary ultrasonic echo
constexpr uint8_t SIG_GREEN = 1;  // Pixy signature for allowed ball (green)
constexpr uint8_t SIG_BLUE  = 2;  // Pixy signature for allowed ball (blue)

// Additional signatures
constexpr uint8_t SIG_BASE = 3;  // Pixy signature for base target

// Motor / servo pins (values adjustable as needed)
constexpr uint8_t M_A_PWM = 5;        // left motor PWM output
constexpr uint8_t M_A_DIR = 4;        // left motor direction control
constexpr uint8_t M_B_PWM = 6;        // right motor PWM output
constexpr uint8_t M_B_DIR = 14;       // right motor direction control on A0
constexpr uint8_t SERVO_LEFT_PIN = 11;   // left gripper servo signal
constexpr uint8_t SERVO_RIGHT_PIN = A3;  // right gripper servo signal (analog pin as digital)

// Tuning constants
constexpr int CENTER_TOL_PIX = 20;             // allowable Pixy pixel offset for alignment
constexpr int BALL_CLOSE_AREA = 2500;          // minimum blob area to consider ball close
constexpr int BASE_LARGE_AREA = 9000;          // minimum base blob area to consider "arrived"
constexpr uint16_t NUDGE_MS = 120;             // short movement duration for nudges
constexpr uint16_t SCORE_MS = 2000;            // drive time to push ball into base
constexpr uint8_t TURN_SPEED_PWM = 130;        // PWM level for turning maneuvers
constexpr uint8_t DRIVE_SPEED_PWM = 160;       // PWM level for forward motion
constexpr int PIXY_CENTER_X = 158;             // Pixy horizontal center reference
constexpr float TURN_MS_PER_DEG = 6.0f;        // ms per degree for timing-based turns
constexpr int L_CLOSED = 1000;                 // left servo closed pulse
constexpr int L_COLLECT = 1500;                // left servo collect pulse
constexpr int L_OPEN = 1900;                   // left servo open pulse
constexpr int R_CLOSED = 2000;                 // right servo closed pulse
constexpr int R_COLLECT = 1500;                // right servo collect pulse
constexpr int R_OPEN = 1100;                   // right servo open pulse

enum class State {
  INIT,
  SEEK_BALL,
  APPROACH_BALL,
  GRAB_BALL,
  SEEK_BASE,
  APPROACH_BASE,
  SCORE,
  RESET_TURN
};

struct SystemState {
  State st = State::INIT;              // current finite state
  unsigned long stateTs = 0;           // timestamp of last transition
  int lastBallCX = -1;                 // last seen ball center X
  int lastBallCY = -1;                 // last seen ball center Y
  uint32_t lastBallArea = 0;           // last seen ball area heuristic
  int lastBaseCX = -1;                 // last seen base center X
  uint32_t lastBaseArea = 0;           // last seen base area
  uint8_t baseScanSteps = 0;           // incremental scan counter for base search
};

Pixy2 pixy;
Servo leftServo;   // left gripper servo instance
Servo rightServo;  // right gripper servo instance
SystemState S;

constexpr bool LED1_AVAILABLE =
    LED1_PIN != M_A_PWM && LED1_PIN != M_A_DIR && LED1_PIN != M_B_PWM && LED1_PIN != M_B_DIR;  // ensure LED1 pin free from motor overlap
constexpr bool LED2_AVAILABLE =
    LED2_PIN != M_A_PWM && LED2_PIN != M_A_DIR && LED2_PIN != M_B_PWM && LED2_PIN != M_B_DIR;  // ensure LED2 pin free from motor overlap

// Forward declarations of required functions
void setLEDs(bool on1, bool on2);
int getNearestBallSigAndXY(int &cx, int &cy);
bool isBallCentered(int cx);
bool isBallClose();
void nudgeTowardBall(int cx);
void shortForward();
void servoOpen();
void servoClose();
bool findBase();
bool isBaseLarge();
void adjustTowardBase();
void driveForwardMax(unsigned long ms);
void turnDegrees(int deg);
void stopMotors();

// Utility helpers
/*
 * Purpose: convert a State enum to its textual label for logging.
 * Inputs: st - current finite state.
 * Outputs: returns const char* with readable name.
 * Side effects: none.
 */
static const char *stateName(State st) {
  switch (st) {
    case State::INIT: return "INIT";
    case State::SEEK_BALL: return "SEEK_BALL";
    case State::APPROACH_BALL: return "APPROACH_BALL";
    case State::GRAB_BALL: return "GRAB_BALL";
    case State::SEEK_BASE: return "SEEK_BASE";
    case State::APPROACH_BASE: return "APPROACH_BASE";
    case State::SCORE: return "SCORE";
    case State::RESET_TURN: return "RESET_TURN";
  }
  return "?";
}

/*
 * Purpose: switch the robot to a new state and timestamp it for timing logic.
 * Inputs: next - state to enter.
 * Outputs: none.
 * Side effects: updates S.st, S.stateTs, prints to Serial.
 */
static void transitionTo(State next) {
  S.st = next;
  S.stateTs = millis();
  Serial.println(stateName(next));
}

/*
 * Purpose: clamp a PWM value into the 0-255 range expected by analogWrite.
 * Inputs: v - signed integer PWM request.
 * Outputs: saturated uint8_t value.
 * Side effects: none.
 */
static uint8_t clampPWM(int v) {
  if (v < 0) v = 0;
  if (v > 255) v = 255;
  return static_cast<uint8_t>(v);
}

/*
 * Purpose: drive both motors with raw PWM and direction signals.
 * Inputs: lpwm/rdir combos for left and right motor commands.
 * Outputs: none.
 * Side effects: writes analog/digital outputs to the motor driver pins.
 */
static void motorRaw(int lpwm, bool ldir, int rpwm, bool rdir) {
  analogWrite(M_A_PWM, clampPWM(lpwm));
  digitalWrite(M_A_DIR, ldir ? HIGH : LOW);
  analogWrite(M_B_PWM, clampPWM(rpwm));
  digitalWrite(M_B_DIR, rdir ? HIGH : LOW);
}

/*
 * Purpose: capture the largest SIG_BASE block currently detected by Pixy.
 * Inputs: none.
 * Outputs: returns true if any base block found, false otherwise, updates S.lastBase*.
 * Side effects: reads Pixy data via SPI.
 */
static bool captureBaseBlock() {
  int count = pixy.ccc.getBlocks();
  S.lastBaseArea = 0;
  S.lastBaseCX = -1;

  bool found = false;
  for (int i = 0; i < count; ++i) {
    auto &b = pixy.ccc.blocks[i];
    if (b.m_signature != SIG_BASE) continue;
    uint32_t area = static_cast<uint32_t>(b.m_width) * b.m_height;
    if (area > S.lastBaseArea) {
      S.lastBaseArea = area;
      S.lastBaseCX = b.m_x;
      found = true;
    }
  }
  return found;
}

/*
 * Purpose: control the two status LEDs if their pins are free.
 * Inputs: on1/on2 booleans for LED1/LED2.
 * Outputs: none.
 * Side effects: digitalWrite on LED pins when available.
 */
void setLEDs(bool on1, bool on2) {
  if (LED1_AVAILABLE) {
    digitalWrite(LED1_PIN, on1 ? HIGH : LOW);
  }
  if (LED2_AVAILABLE) {
    digitalWrite(LED2_PIN, on2 ? HIGH : LOW);
  }
}

/*
 * Purpose: stop both drive motors.
 * Inputs: none.
 * Outputs: none.
 * Side effects: sets motor PWM to zero.
 */
void stopMotors() {
  motorRaw(0, LOW, 0, LOW);
}

/*
 * Purpose: drive forward at maximum configured speed for a fixed time.
 * Inputs: ms - duration in milliseconds.
 * Outputs: none.
 * Side effects: continuous motor command, blocking delay loop.
 */
void driveForwardMax(unsigned long ms) {
  motorRaw(DRIVE_SPEED_PWM, HIGH, DRIVE_SPEED_PWM, HIGH);
  unsigned long start = millis();
  while (millis() - start < ms) {
    delay(5);
  }
  stopMotors();
}

/*
 * Purpose: perform a short forward nudge used for fine approach steps.
 * Inputs: none.
 * Outputs: none.
 * Side effects: blocking delay while motors run briefly.
 */
void shortForward() {
  motorRaw(DRIVE_SPEED_PWM, HIGH, DRIVE_SPEED_PWM, HIGH);
  delay(NUDGE_MS);
  stopMotors();
}

/*
 * Purpose: rotate the robot in place by an approximate number of degrees.
 * Inputs: deg - signed degrees (positive clockwise).
 * Outputs: none.
 * Side effects: blocking motor command based on open-loop timing.
 */
void turnDegrees(int deg) {
  if (deg == 0) {
    stopMotors();
    return;
  }
  bool clockwise = deg > 0;
  unsigned long duration = static_cast<unsigned long>(abs(deg) * TURN_MS_PER_DEG);  // open-loop timing scaled by TURN_MS_PER_DEG tuning
  motorRaw(TURN_SPEED_PWM, clockwise, TURN_SPEED_PWM, !clockwise);
  unsigned long start = millis();
  while (millis() - start < duration) {
    delay(5);
  }
  stopMotors();
}

/*
 * Purpose: gently align the robot toward the detected ball or creep forward if centered.
 * Inputs: cx - horizontal coordinate of ball.
 * Outputs: none.
 * Side effects: short motor movement and blocking delay.
 */
void nudgeTowardBall(int cx) {
  int dx = cx - PIXY_CENTER_X;
  if (abs(dx) <= CENTER_TOL_PIX) {
    shortForward();
    return;
  }
  bool clockwise = dx > 0;
  motorRaw(TURN_SPEED_PWM, clockwise, TURN_SPEED_PWM, !clockwise);
  delay(NUDGE_MS);
  stopMotors();
}

/*
 * Purpose: make incremental adjustments to align or advance toward the base blob.
 * Inputs: none (uses cached S.lastBase*).
 * Outputs: none.
 * Side effects: runs motors briefly depending on alignment error.
 */
void adjustTowardBase() {
  if (!captureBaseBlock()) {
    stopMotors();
    return;
  }
  int dx = S.lastBaseCX - PIXY_CENTER_X;  // horizontal error relative to camera center
  if (abs(dx) > CENTER_TOL_PIX) {
    // If base is off-center, rotate proportionally (direction from dx sign)
    bool clockwise = dx > 0;
    motorRaw(TURN_SPEED_PWM, clockwise, TURN_SPEED_PWM, !clockwise);
    delay(NUDGE_MS);
  } else {
    // Once within tolerance, move forward slightly to close the distance
    motorRaw(DRIVE_SPEED_PWM, HIGH, DRIVE_SPEED_PWM, HIGH);
    delay(NUDGE_MS);
  }
  stopMotors();
}

/*
 * Purpose: open both gripper servos to the release position.
 * Inputs: none.
 * Outputs: none.
 * Side effects: writes microsecond pulses to both servos.
 */
void servoOpen() {
  leftServo.writeMicroseconds(L_OPEN);
  rightServo.writeMicroseconds(R_OPEN);
}

/*
 * Purpose: close both gripper servos to hold the ball securely.
 * Inputs: none.
 * Outputs: none.
 * Side effects: writes microsecond pulses to both servos.
 */
void servoClose() {
  leftServo.writeMicroseconds(L_CLOSED);
  rightServo.writeMicroseconds(R_CLOSED);
}

/*
 * Purpose: find the nearest allowed ball by choosing the largest allowed signature block.
 * Inputs: references to store cx/cy.
 * Outputs: returns signature id (0 if none).
 * Side effects: updates cached last ball metrics.
 */
int getNearestBallSigAndXY(int &cx, int &cy) {
  int n = pixy.ccc.getBlocks();
  S.lastBallArea = 0;
  S.lastBallCX = -1;
  S.lastBallCY = -1;

  uint8_t bestSig = 0;
  for (int i = 0; i < n; ++i) {
    auto &b = pixy.ccc.blocks[i];
    if (b.m_signature != SIG_GREEN && b.m_signature != SIG_BLUE) continue;  // ignore other signatures
    uint32_t area = static_cast<uint32_t>(b.m_width) * b.m_height;          // area as proxy for distance (larger = closer)
    if (area > S.lastBallArea) {                                            // keep the largest area to target nearest ball
      S.lastBallArea = area;
      S.lastBallCX = b.m_x;
      S.lastBallCY = b.m_y;
      bestSig = b.m_signature;
    }
  }

  if (bestSig == 0) {
    cx = cy = -1;
    return 0;
  }

  cx = S.lastBallCX;
  cy = S.lastBallCY;
  return bestSig;
}

/*
 * Purpose: check if the ball is centered enough in the Pixy frame.
 * Inputs: cx - horizontal coordinate.
 * Outputs: true if within tolerance.
 * Side effects: none.
 */
bool isBallCentered(int cx) {
  return abs(cx - PIXY_CENTER_X) <= CENTER_TOL_PIX;
}

/*
 * Purpose: decide whether the ball is close enough based on area.
 * Inputs: none (uses cached S.lastBallArea).
 * Outputs: boolean close flag.
 * Side effects: none.
 */
bool isBallClose() {
  return S.lastBallArea >= static_cast<uint32_t>(BALL_CLOSE_AREA);
}

/*
 * Purpose: scan for the base by reading Pixy blocks and spinning if needed.
 * Inputs: none.
 * Outputs: true when base detected.
 * Side effects: issues incremental turns and reads Pixy.
 */
bool findBase() {
  if (captureBaseBlock()) {
    S.baseScanSteps = 0;
    return true;
  }

  // incremental spin search
  turnDegrees(15);
  if (++S.baseScanSteps >= 24) {
    S.baseScanSteps = 0;
  }

  return captureBaseBlock();
}

/*
 * Purpose: evaluate whether the last seen base block is large enough to assume proximity.
 * Inputs: none (uses cached S.lastBaseArea).
 * Outputs: boolean threshold test.
 * Side effects: none.
 */
bool isBaseLarge() {
  return S.lastBaseArea >= static_cast<uint32_t>(BASE_LARGE_AREA);
}

/*
 * Purpose: set initial LED/servo/motor states to a safe ready condition.
 * Inputs: none.
 * Outputs: none.
 * Side effects: drives LEDs, servo, and stops motors.
 */
void setReadyState() {
  setLEDs(true, false);
  servoOpen();
  stopMotors();
}

/*
 * Purpose: initialize serial, I/O pins, Pixy, and enter the first state.
 * Inputs: none.
 * Outputs: none.
 * Side effects: hardware setup, serial logging.
 */
void setup() {
  Serial.begin(115200);

  pinMode(LED1_PIN, OUTPUT);
  pinMode(LED2_PIN, OUTPUT);
  pinMode(M_A_PWM, OUTPUT);
  pinMode(M_A_DIR, OUTPUT);
  pinMode(M_B_PWM, OUTPUT);
  pinMode(M_B_DIR, OUTPUT);
  pinMode(US_TRIG, OUTPUT);
  pinMode(US_ECHO, INPUT);
  digitalWrite(US_TRIG, LOW);
  pinMode(US2_TRIG, OUTPUT);
  pinMode(US2_ECHO, INPUT);
  digitalWrite(US2_TRIG, LOW);

  leftServo.attach(SERVO_LEFT_PIN);
  rightServo.attach(SERVO_RIGHT_PIN);
  setReadyState();

  pixy.init();
  pixy.setLamp(1, 1);
  pixy.changeProg("color_connected_components");

  transitionTo(State::INIT);
}

/*
 * Purpose: run the state machine to seek, grab, and score balls continuously.
 * Inputs: none.
 * Outputs: none.
 * Side effects: drives motors/servo and reads sensors.
 */
void loop() {
  switch (S.st) {
    case State::INIT:
      transitionTo(State::SEEK_BALL);  // immediately start searching once initialized
      break;

    case State::SEEK_BALL: {
      int cx, cy;
      if (getNearestBallSigAndXY(cx, cy) == 0) {
        turnDegrees(10);  // no ball found, continue scanning
      } else {
        transitionTo(State::APPROACH_BALL);  // ball detected, begin approach routine
      }
    } break;

    case State::APPROACH_BALL: {
      int cx, cy;
      if (getNearestBallSigAndXY(cx, cy) == 0) {
        transitionTo(State::SEEK_BALL);  // lost sight of ball, resume search
        break;
      }
      if (!isBallCentered(cx)) {
        nudgeTowardBall(cx);  // rotate slightly until centered
      } else if (!isBallClose()) {
        shortForward();  // advance to close distance after centering
      } else {
        transitionTo(State::GRAB_BALL);  // ball centered and near, ready to grab
      }
    } break;

    case State::GRAB_BALL:
      shortForward();
      servoClose();
      delay(150);
      transitionTo(State::SEEK_BASE);  // after grabbing, start looking for the base
      break;

    case State::SEEK_BASE:
      if (findBase()) {
        transitionTo(State::APPROACH_BASE);  // base acquired, switch to alignment phase
      }
      break;

    case State::APPROACH_BASE:
      if (!isBaseLarge()) {
        adjustTowardBase();  // continue nudging toward base until it appears large
      } else {
        transitionTo(State::SCORE);  // base is sufficiently close, go score
      }
      break;

    case State::SCORE:
      driveForwardMax(SCORE_MS);
      servoOpen();
      transitionTo(State::RESET_TURN);  // after depositing, prepare to reorient
      break;

    case State::RESET_TURN:
      turnDegrees(180);
      transitionTo(State::SEEK_BALL);  // face away to resume hunting the next ball
      break;
  }
}
