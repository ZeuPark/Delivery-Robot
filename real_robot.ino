#include <Pixy2.h>
#include <Servo.h>

Pixy2 pixy;

// motor pins
constexpr uint8_t M_A_PWM = 5;
constexpr uint8_t M_A_DIR = 4;
constexpr uint8_t M_B_PWM = 6;
constexpr uint8_t M_B_DIR = 7;

// ultrasonic 1 pins (ball distance)
constexpr uint8_t US_TRIG = 8;
constexpr uint8_t US_ECHO = 9;

// servo pins
constexpr uint8_t SERVO_LEFT_PIN  = A4;
constexpr uint8_t SERVO_RIGHT_PIN = A3;

// Pixy signatures
// base 1, ball 3 4
constexpr uint8_t SIG_BASE  = 1;
constexpr uint8_t SIG_GREEN = 3;
constexpr uint8_t SIG_BLUE  = 4;

// center
const int CENTER_X = 158;

// ball align
const int TOL_ALIGN_BALL      = 20;
const int TOL_GRAB_ALIGN_BALL = 15;
const int BIG_MISALIGN_BALL   = 40;

// base align
const int TOL_ALIGN_BASE = 20;
const int BIG_MISALIGN_BASE = 40;

// motor speed for ball phase
const int TURN_PWM_BALL = 140;
const int SEEK_PWM_BALL = 120;
const int FWD_PWM_BALL  = 150;

// motor speed for base phase (같이 씀)
const int TURN_PWM_BASE = 140;
const int SEEK_PWM_BASE = 120;
const int FWD_PWM_BASE  = 150;

// distance for grab
const int TARGET_GRAB_CM = 6;

// base area conditions
const uint32_t BASE_AREA_MIN     = 800;
const uint32_t BASE_AREA_RELEASE = 20000;

// smoothing for log
const float AREA_SMOOTH_ALPHA = 0.7f;

// servo positions
int leftHold     = 1250;
int rightHold    = 1450;
int leftGrab     = 1000;
int rightGrab    = 1800;
int leftRelease  = 1900;
int rightRelease = 900;

Servo leftServo;
Servo rightServo;

// top level phase
enum Phase {
  PHASE_FIND_BALL,
  PHASE_FIND_BASE,
  PHASE_DONE
};

Phase phase = PHASE_FIND_BALL;

// ball state machine (테스트 코드 그대로 사용)
enum GrabState {
  SEEK_ALIGN,
  APPROACH_US,
  GRAB_DONE
};

GrabState gState = SEEK_ALIGN;

// base state machine (base area 코드 그대로 사용)
enum BaseState {
  SEEK_BASE,
  APPROACH_BASE,
  RELEASE_DONE
};

BaseState bState = SEEK_BASE;

float baseAreaSmooth = 0.0f;
bool alreadyReleased = false;

// motor helpers

uint8_t clampPWM(int v) {
  if (v < 0) v = 0;
  if (v > 255) v = 255;
  return (uint8_t)v;
}

void motorRaw(int lpwm, bool ldir, int rpwm, bool rdir) {
  analogWrite(M_A_PWM, clampPWM(lpwm));
  digitalWrite(M_A_DIR, ldir ? HIGH : LOW);
  analogWrite(M_B_PWM, clampPWM(rpwm));
  digitalWrite(M_B_DIR, rdir ? HIGH : LOW);
}

void motorsStop() {
  motorRaw(0, LOW, 0, LOW);
}

// ball phase low level motions

void turnLeftBall() {
  motorRaw(TURN_PWM_BALL, LOW, TURN_PWM_BALL, HIGH);
}

void turnRightBall() {
  motorRaw(TURN_PWM_BALL, HIGH, TURN_PWM_BALL, LOW);
}

void slowSeekSpinBall() {
  motorRaw(SEEK_PWM_BALL, LOW, SEEK_PWM_BALL, HIGH);
}

void forwardContBall() {
  motorRaw(FWD_PWM_BALL, LOW, FWD_PWM_BALL, LOW);
}

// base phase low level motions

void turnLeftBase() {
  motorRaw(TURN_PWM_BASE, LOW, TURN_PWM_BASE, HIGH);
}

void turnRightBase() {
  motorRaw(TURN_PWM_BASE, HIGH, TURN_PWM_BASE, LOW);
}

void slowSeekSpinBase() {
  motorRaw(SEEK_PWM_BASE, LOW, SEEK_PWM_BASE, HIGH);
}

void forwardContBase() {
  motorRaw(FWD_PWM_BASE, LOW, FWD_PWM_BASE, LOW);
}

// ultrasonic

long readUltrasonicCM(uint8_t trig, uint8_t echo) {
  digitalWrite(trig, LOW);
  delayMicroseconds(3);
  digitalWrite(trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig, LOW);
  unsigned long dur = pulseIn(echo, HIGH, 30000UL);
  if (dur == 0) return -1;
  long cm = dur / 58;
  return cm;
}

// servos

void servoHold() {
  leftServo.writeMicroseconds(leftHold);
  rightServo.writeMicroseconds(rightHold);
}

void servoGrab() {
  leftServo.writeMicroseconds(leftGrab);
  rightServo.writeMicroseconds(rightGrab);
}

void servoRelease() {
  leftServo.writeMicroseconds(leftRelease);
  rightServo.writeMicroseconds(rightRelease);
}

// slow open and stay open

void servoReleaseSlow() {
  int l0 = leftGrab;
  int r0 = rightGrab;

  for (int step = 0; step <= 100; step++) {
    int lv = l0 + (leftRelease - l0) * step / 100;
    int rv = r0 + (rightRelease - r0) * step / 100;

    leftServo.writeMicroseconds(lv);
    rightServo.writeMicroseconds(rv);
    delay(12);
  }
}

void servoReleaseSlowHold() {
  leftServo.writeMicroseconds(leftRelease);
  rightServo.writeMicroseconds(rightRelease);
}

// select largest green or blue ball (공 테스트 코드 그대로)

bool selectBall(int &xOut) {
  int n = pixy.ccc.getBlocks();
  uint32_t bestArea = 0;
  int bestX = -1;

  for (int i = 0; i < n; i++) {
    auto &b = pixy.ccc.blocks[i];

    if (b.m_signature == SIG_GREEN || b.m_signature == SIG_BLUE) {
      uint32_t area = (uint32_t)b.m_width * b.m_height;
      if (area > bestArea) {
        bestArea = area;
        bestX = b.m_x;
      }
    }
  }

  if (bestArea == 0) return false;

  xOut = bestX;
  return true;
}

// base detection (sig 1, area 최대, 작은 건 버림)

bool selectBaseBlock(int &xOut, uint32_t &areaOut) {
  int n = pixy.ccc.getBlocks();
  uint32_t bestArea = 0;
  int bestX = -1;

  for (int i = 0; i < n; i++) {
    auto &b = pixy.ccc.blocks[i];
    if (b.m_signature == SIG_BASE) {
      uint32_t area = (uint32_t)b.m_width * b.m_height;
      if (area < BASE_AREA_MIN) continue;

      if (area > bestArea) {
        bestArea = area;
        bestX = b.m_x;
      }
    }
  }

  if (bestArea == 0) return false;

  xOut = bestX;
  areaOut = bestArea;
  return true;
}

// ball phase one step (테스트에서 잘 됐던 loop 그대로 함수로 감싼 것)

void ballLoopOneStep() {
  int ballX = 0;
  bool hasBall = selectBall(ballX);
  long dist = readUltrasonicCM(US_TRIG, US_ECHO);

  Serial.print("[BALL] st=");
  Serial.print((int)gState);
  Serial.print(" hasBall=");
  Serial.print(hasBall);
  Serial.print(" dist=");
  Serial.print(dist);
  Serial.print("cm ");

  int error = 0;
  if (hasBall) {
    error = ballX - CENTER_X;
    Serial.print(" x=");
    Serial.print(ballX);
    Serial.print(" err=");
    Serial.print(error);
  }
  Serial.println();

  switch (gState) {

    case SEEK_ALIGN: {
      if (!hasBall) {
        Serial.println("[BALL] no ball  seek spin");
        slowSeekSpinBall();
        delay(60);
        return;
      }

      if (abs(error) > TOL_ALIGN_BALL) {
        if (error < 0) {
          Serial.println("[BALL] SEEK  turn left");
          turnLeftBall();
        } else {
          Serial.println("[BALL] SEEK  turn right");
          turnRightBall();
        }
        delay(60);
        motorsStop();
      } else {
        Serial.println("[BALL] SEEK  aligned  -> APPROACH_US");
        motorsStop();
        gState = APPROACH_US;
      }
      break;
    }

    case APPROACH_US: {
      if (!hasBall) {
        Serial.println("[BALL] APPROACH  lost ball  -> SEEK_ALIGN");
        motorsStop();
        gState = SEEK_ALIGN;
        break;
      }

      if (dist > 0 && dist <= TARGET_GRAB_CM && abs(error) <= TOL_GRAB_ALIGN_BALL) {
        Serial.println("[BALL] APPROACH  grab distance and aligned  GRAB");
        motorsStop();
        servoGrab();
        delay(400);
        gState = GRAB_DONE;
        break;
      }

      if (abs(error) > BIG_MISALIGN_BALL) {
        Serial.println("[BALL] APPROACH  big misalign  turn only");
        if (error < 0) turnLeftBall();
        else           turnRightBall();
        delay(50);
        motorsStop();
        break;
      }

      if (dist < 0 || dist > TARGET_GRAB_CM) {
        Serial.println("[BALL] APPROACH  forward smooth");
        forwardContBall();
        delay(40);
      } else {
        motorsStop();
      }
      break;
    }

    case GRAB_DONE: {
      motorsStop();
      delay(100);
      break;
    }
  }
}

// base phase one step (base area 코드 loop를 그대로 함수로 옮김)

void baseLoopOneStep() {
  if (alreadyReleased) {
    motorsStop();
    delay(50);
    return;
  }

  int baseX = 0;
  uint32_t baseAreaRaw = 0;
  bool hasBase = selectBaseBlock(baseX, baseAreaRaw);

  if (hasBase) {
    if (baseAreaSmooth <= 0.0f) baseAreaSmooth = baseAreaRaw;
    else baseAreaSmooth = AREA_SMOOTH_ALPHA * baseAreaSmooth
                        + (1.0f - AREA_SMOOTH_ALPHA) * baseAreaRaw;
  } else {
    baseAreaSmooth *= 0.9f;
    if (baseAreaSmooth < 1.0f) baseAreaSmooth = 0.0f;
  }

  Serial.print("[BASE] st=");
  Serial.print((int)bState);
  Serial.print(" has=");
  Serial.print(hasBase);
  if (hasBase) {
    int e = baseX - CENTER_X;
    Serial.print(" x=");
    Serial.print(baseX);
    Serial.print(" err=");
    Serial.print(e);
    Serial.print(" areaRaw=");
    Serial.print(baseAreaRaw);
    Serial.print(" areaSmooth=");
    Serial.print(baseAreaSmooth);
  }
  Serial.println();

  switch (bState) {

    case SEEK_BASE: {
      if (!hasBase) {
        slowSeekSpinBase();
        delay(60);
        return;
      }

      int error = baseX - CENTER_X;

      if (abs(error) > TOL_ALIGN_BASE) {
        if (error < 0) turnLeftBase();
        else           turnRightBase();
        delay(60);
        motorsStop();
      } else {
        motorsStop();
        bState = APPROACH_BASE;
      }
      break;
    }

    case APPROACH_BASE: {
      if (!hasBase) {
        motorsStop();
        bState = SEEK_BASE;
        break;
      }

      int error = baseX - CENTER_X;

      if (!alreadyReleased &&
          baseAreaRaw >= BASE_AREA_RELEASE &&
          abs(error) <= TOL_ALIGN_BASE) {

        motorsStop();

        servoReleaseSlow();
        servoReleaseSlowHold();

        baseAreaSmooth = 0.0f;
        alreadyReleased = true;
        bState = RELEASE_DONE;
        break;
      }

      if (abs(error) > BIG_MISALIGN_BASE) {
        if (error < 0) turnLeftBase();
        else           turnRightBase();
        delay(50);
        motorsStop();
        break;
      }

      forwardContBase();
      delay(40);
      break;
    }

    case RELEASE_DONE: {
      motorsStop();
      delay(100);
      break;
    }
  }
}

// setup

void setup() {
  Serial.begin(115200);
  while (!Serial) {}

  pinMode(M_A_PWM, OUTPUT);
  pinMode(M_A_DIR, OUTPUT);
  pinMode(M_B_PWM, OUTPUT);
  pinMode(M_B_DIR, OUTPUT);
  motorsStop();

  pinMode(US_TRIG, OUTPUT);
  pinMode(US_ECHO, INPUT);
  digitalWrite(US_TRIG, LOW);

  leftServo.attach(SERVO_LEFT_PIN);
  rightServo.attach(SERVO_RIGHT_PIN);
  servoHold();

  pixy.init();
  pixy.changeProg("color_connected_components");
  pixy.setLamp(0, 0);

  phase = PHASE_FIND_BALL;
  gState = SEEK_ALIGN;
  bState = SEEK_BASE;
  baseAreaSmooth = 0.0f;
  alreadyReleased = false;

  Serial.println("integrated ball grab plus base area release");
}

// loop

void loop() {
  switch (phase) {

    case PHASE_FIND_BALL: {
      ballLoopOneStep();
      if (gState == GRAB_DONE) {
        phase = PHASE_FIND_BASE;
        bState = SEEK_BASE;
        baseAreaSmooth = 0.0f;
        alreadyReleased = false;
        delay(200);
      }
      break;
    }

    case PHASE_FIND_BASE: {
      baseLoopOneStep();
      if (bState == RELEASE_DONE && alreadyReleased) {
        phase = PHASE_DONE;
      }
      break;
    }

    case PHASE_DONE: {
      motorsStop();
      delay(100);
      break;
    }
  }
}
