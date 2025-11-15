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
constexpr uint8_t SIG_GREEN = 3;   // ball
constexpr uint8_t SIG_BLUE  = 4;   // ball
constexpr uint8_t SIG_BASE  = 1;   // base

// Pixy center and tolerances
const int CENTER_X        = 158;

// ball align
const int TOL_ALIGN_BALL      = 20;
const int TOL_GRAB_ALIGN_BALL = 15;
const int BIG_MISALIGN_BALL   = 40;

// base align
const int TOL_ALIGN_BASE = 20;
const int BIG_MISALIGN_BASE = 40;

// speeds
const int TURN_PWM  = 140;
const int SEEK_PWM  = 120;
const int FWD_PWM   = 150;

// ultrasonic distance for grab (cm)
const int TARGET_GRAB_CM = 6;

// base area thresholds
const uint32_t BASE_AREA_MIN     = 800;
const uint32_t BASE_AREA_RELEASE = 20000;
const uint32_t MIN_BALL_AREA     = 1500;  // tune this later

// smoothing for debug log only
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

// ball state machine
enum GrabState {
  G_SEEK_ALIGN,
  G_APPROACH_US,
  G_GRAB_DONE
};

GrabState gState = G_SEEK_ALIGN;

// base state machine
enum BaseState {
  B_SEEK_BASE,
  B_APPROACH_BASE,
  B_RELEASE_DONE
};

BaseState bState = B_SEEK_BASE;

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

void turnLeft() {
  motorRaw(TURN_PWM, LOW, TURN_PWM, HIGH);
}

void turnRight() {
  motorRaw(TURN_PWM, HIGH, TURN_PWM, LOW);
}

void slowSeekSpin() {
  motorRaw(SEEK_PWM, LOW, SEEK_PWM, HIGH);
}

void forwardCont() {
  motorRaw(FWD_PWM, LOW, FWD_PWM, LOW);
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

// select largest green or blue ball
bool selectBall(int &xOut) {
  int n = pixy.ccc.getBlocks();
  uint32_t bestArea = 0;
  int bestX = -1;

  for (int i = 0; i < n; i++) {
    auto &b = pixy.ccc.blocks[i];
    if (b.m_signature == SIG_GREEN || b.m_signature == SIG_BLUE) {
      uint32_t area = (uint32_t)b.m_width * b.m_height;
      if (area < MIN_BALL_AREA) continue;
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

// select largest base block (sig 1), ignore tiny noise
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

// servo helpers
void servoHold() {
  leftServo.writeMicroseconds(leftHold);
  rightServo.writeMicroseconds(rightHold);
}

void servoGrab() {
  leftServo.writeMicroseconds(leftGrab);
  rightServo.writeMicroseconds(rightGrab);
}

void servoReleaseInstant() {
  leftServo.writeMicroseconds(leftRelease);
  rightServo.writeMicroseconds(rightRelease);
}

// slow open from grab to release and stay open
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
  leftServo.writeMicroseconds(leftRelease);
  rightServo.writeMicroseconds(rightRelease);
}

// ball phase state machine
void updateBallPhase() {
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
  if (hasBall) {
    int err = ballX - CENTER_X;
    Serial.print(" x=");
    Serial.print(ballX);
    Serial.print(" err=");
    Serial.print(err);
  }
  Serial.println();

  switch (gState) {

    case G_SEEK_ALIGN: {
      if (!hasBall) {
        slowSeekSpin();
        delay(60);
        return;
      }

      int error = ballX - CENTER_X;
      if (abs(error) > TOL_ALIGN_BALL) {
        if (error < 0) turnLeft();
        else           turnRight();
        delay(60);
        motorsStop();
      } else {
        motorsStop();
        gState = G_APPROACH_US;
      }
      break;
    }

    case G_APPROACH_US: {
      if (!hasBall) {
        motorsStop();
        gState = G_SEEK_ALIGN;
        break;
      }

      int error = ballX - CENTER_X;

      // grab 조건  거리와 정렬 둘 다
      if (dist > 0 && dist <= TARGET_GRAB_CM &&
          abs(error) <= TOL_GRAB_ALIGN_BALL) {

        motorsStop();
        servoGrab();
        delay(400);
        gState = G_GRAB_DONE;
        break;
      }

      // 심하게 틀어졌으면 회전 먼저
      if (abs(error) > BIG_MISALIGN_BALL) {
        if (error < 0) turnLeft();
        else           turnRight();
        delay(50);
        motorsStop();
        break;
      }

      // 여기에서는 정렬은 어느 정도 유지된 상태
      if (dist < 0 || dist > TARGET_GRAB_CM) {
        forwardCont();
        delay(40);
      } else {
        motorsStop();
      }
      break;
    }

    case G_GRAB_DONE: {
      motorsStop();
      // phase 전환은 loop에서 처리
      break;
    }
  }
}

// base phase state machine
void updateBasePhase() {
  if (alreadyReleased) {
    motorsStop();
    delay(50);
    return;
  }

  int baseX = 0;
  uint32_t baseAreaRaw = 0;
  bool hasBase = selectBaseBlock(baseX, baseAreaRaw);

  Serial.print("[BASE] st=");
  Serial.print((int)bState);
  Serial.print(" hasBase=");
  Serial.print(hasBase);
  if (hasBase) {
    int e = baseX - CENTER_X;
    Serial.print(" x=");
    Serial.print(baseX);
    Serial.print(" err=");
    Serial.print(e);
    Serial.print(" areaRaw=");
    Serial.print(baseAreaRaw);
  }
  Serial.println();

  switch (bState) {

    case B_SEEK_BASE: {
      if (!hasBase) {
        slowSeekSpin();
        delay(60);
        return;
      }

      int error = baseX - CENTER_X;

      if (abs(error) > TOL_ALIGN_BASE) {
        if (error < 0) turnLeft();
        else           turnRight();
        delay(60);
        motorsStop();
      } else {
        motorsStop();
        bState = B_APPROACH_BASE;
      }
      break;
    }

    case B_APPROACH_BASE: {
      if (!hasBase) {
        motorsStop();
        bState = B_SEEK_BASE;
        break;
      }

      int error = baseX - CENTER_X;

      // release 조건  raw area 기준, 아직 안 열린 상태
      if (!alreadyReleased &&
          baseAreaRaw >= BASE_AREA_RELEASE &&
          abs(error) <= TOL_ALIGN_BASE) {

        motorsStop();

        servoReleaseSlow();

        alreadyReleased = true;
        bState = B_RELEASE_DONE;
        break;
      }

      if (abs(error) > BIG_MISALIGN_BASE) {
        if (error < 0) turnLeft();
        else           turnRight();
        delay(50);
        motorsStop();
        break;
      }

      forwardCont();
      delay(40);
      break;
    }

    case B_RELEASE_DONE: {
      motorsStop();
      break;
    }
  }
}

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
  gState = G_SEEK_ALIGN;
  bState = B_SEEK_BASE;
  alreadyReleased = false;

  Serial.println("Full robot  ball grab with ultrasonic, base score with Pixy");
}

void loop() {
  switch (phase) {

    case PHASE_FIND_BALL: {
      updateBallPhase();
      if (gState == G_GRAB_DONE) {
        phase = PHASE_FIND_BASE;
        bState = B_SEEK_BASE;
        alreadyReleased = false;
        delay(200);
      }
      break;
    }

    case PHASE_FIND_BASE: {
      updateBasePhase();
      if (bState == B_RELEASE_DONE && alreadyReleased) {
        phase = PHASE_DONE;
      }
      break;
    }

    case PHASE_DONE: {
      motorsStop();
      // 여기서 다시 루프 돌고 싶으면 아래 코드로 초기화
      // phase = PHASE_FIND_BALL;
      // gState = G_SEEK_ALIGN;
      // bState = B_SEEK_BASE;
      // alreadyReleased = false;
      // servoHold();
      delay(100);
      break;
    }
  }
}
