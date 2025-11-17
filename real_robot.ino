#include <Pixy2.h>
#include <Servo.h>

Pixy2 pixy;

// motor pins
constexpr uint8_t M_A_PWM = 5;
constexpr uint8_t M_A_DIR = 4;
constexpr uint8_t M_B_PWM = 6;
constexpr uint8_t M_B_DIR = 7;

// ultrasonic pins
constexpr uint8_t US_TRIG  = 8;
constexpr uint8_t US_ECHO  = 9;
constexpr uint8_t US2_TRIG = A1;
constexpr uint8_t US2_ECHO = A2;

// servo pins
constexpr uint8_t SERVO_LEFT_PIN  = A4;
constexpr uint8_t SERVO_RIGHT_PIN = A3;

// Pixy signatures
constexpr uint8_t SIG_GREEN = 3;  // ball
constexpr uint8_t SIG_BLUE  = 4;  // ball
constexpr uint8_t SIG_BASE  = 1;  // base

// alignment
const int CENTER_X        = 158;

// ball align
const int TOL_ALIGN       = 20;
const int TOL_GRAB_ALIGN  = 15;
const int BIG_MISALIGN    = 40;

// base align
const int TOL_ALIGN_BASE  = 20;

// speed
const int TURN_PWM = 140;
const int SEEK_PWM = 120;
const int FWD_PWM  = 150;

// distance thresholds
const int TARGET_GRAB_CM  = 6;   // ball
const int TARGET_BASE_CM  = 30;  // base wall ultrasonic threshold

// base area
const uint32_t BASE_AREA_MIN = 800;

// smoothing log
const float AREA_SMOOTH_ALPHA = 0.7f;

// servo positions
int leftHold     = 1250;
int rightHold    = 1450;
int leftGrab     = 800;
int rightGrab    = 1970;
int leftRelease  = 1900;
int rightRelease = 900;

Servo leftServo;
Servo rightServo;

// top level state
enum TopState {
  STATE_GRAB_BALL,
  STATE_SEEK_AND_APPROACH_BASE
};

TopState topState = STATE_GRAB_BALL;

// ball grab state
enum GrabState {
  SEEK_ALIGN,
  APPROACH_US,
  GRAB_DONE
};

GrabState gState = SEEK_ALIGN;

// base state
enum BaseState {
  SEEK_BASE,
  APPROACH_BASE,
  RELEASE_DONE
};

BaseState bState = SEEK_BASE;

float baseAreaSmooth = 0.0f;
bool alreadyReleased = false;

// helpers
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

// ultrasonic generic
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

// largest green or blue ball
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

// base detection sig 1 and area
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

void servoRelease() {
  leftServo.writeMicroseconds(leftRelease);
  rightServo.writeMicroseconds(rightRelease);
}

// ball grab step
bool runGrabBallStep() {
  int ballX = 0;
  bool hasBall = selectBall(ballX);
  long dist = readUltrasonicCM(US_TRIG, US_ECHO);

  Serial.print("TOP=GRAB  st=");
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
        Serial.println("no ball  seek spin");
        slowSeekSpin();
        delay(60);
        return false;
      }

      if (abs(error) > TOL_ALIGN) {
        if (error < 0) {
          Serial.println("SEEK  turn left");
          turnLeft();
        } else {
          Serial.println("SEEK  turn right");
          turnRight();
        }
        delay(60);
        motorsStop();
      } else {
        Serial.println("SEEK  aligned  -> APPROACH_US");
        motorsStop();
        gState = APPROACH_US;
      }
      break;
    }

    case APPROACH_US: {
      if (!hasBall) {
        Serial.println("APPROACH  lost ball  -> SEEK_ALIGN");
        motorsStop();
        gState = SEEK_ALIGN;
        break;
      }

      if (dist > 0 && dist <= TARGET_GRAB_CM && abs(error) <= TOL_GRAB_ALIGN) {
        Serial.println("APPROACH  grab distance and aligned  GRAB");
        motorsStop();
        servoGrab();
        delay(400);
        gState = GRAB_DONE;
        break;
      }

      if (abs(error) > BIG_MISALIGN) {
        Serial.println("APPROACH  big misalign  turn only");
        if (error < 0) turnLeft();
        else           turnRight();
        delay(50);
        motorsStop();
        break;
      }

      if (dist < 0 || dist > TARGET_GRAB_CM) {
        Serial.println("APPROACH  forward smooth");
        forwardCont();
        delay(40);
      } else {
        motorsStop();
      }
      break;
    }

    case GRAB_DONE: {
      motorsStop();
      Serial.println("GRAB_DONE");
      return true;
    }
  }

  return (gState == GRAB_DONE);
}

// base step with ultrasonic 2 and instant release
bool runBaseStep() {
  if (alreadyReleased) {
    motorsStop();
    bState = RELEASE_DONE;
    Serial.println("Base released  cycle end");
    delay(50);
    return true;
  }

  int baseX = 0;
  uint32_t baseAreaRaw = 0;
  bool hasBase = selectBaseBlock(baseX, baseAreaRaw);
  long dist2 = readUltrasonicCM(US2_TRIG, US2_ECHO);

  if (hasBase) {
    if (baseAreaSmooth <= 0.0f) baseAreaSmooth = baseAreaRaw;
    else baseAreaSmooth = AREA_SMOOTH_ALPHA * baseAreaSmooth
                        + (1.0f - AREA_SMOOTH_ALPHA) * baseAreaRaw;
  } else {
    baseAreaSmooth *= 0.9f;
    if (baseAreaSmooth < 1.0f) baseAreaSmooth = 0.0f;
  }

  Serial.print("TOP=BASE st=");
  Serial.print((int)bState);
  Serial.print(" has=");
  Serial.print(hasBase);
  Serial.print(" dist2=");
  Serial.print(dist2);
  Serial.print("cm ");
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
        slowSeekSpin();
        delay(60);
        return false;
      }

      int error = baseX - CENTER_X;

      if (abs(error) > TOL_ALIGN_BASE) {
        if (error < 0) turnLeft();
        else           turnRight();
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

      // release condition  base seen  centered  wall close on US2
      if (!alreadyReleased &&
          dist2 > 0 &&
          dist2 <= TARGET_BASE_CM &&
          abs(error) <= TOL_ALIGN_BASE) {

        Serial.println("BASE  close enough  instant release");
        motorsStop();

        // instant release no delay
        servoRelease();

        alreadyReleased = true;
        bState = RELEASE_DONE;
        break;
      }

      if (abs(error) > BIG_MISALIGN) {
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

    case RELEASE_DONE: {
      motorsStop();
      Serial.println("RELEASE_DONE state");
      return true;
    }
  }

  return (bState == RELEASE_DONE && alreadyReleased);
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

  pinMode(US2_TRIG, OUTPUT);
  pinMode(US2_ECHO, INPUT);
  digitalWrite(US2_TRIG, LOW);

  leftServo.attach(SERVO_LEFT_PIN);
  rightServo.attach(SERVO_RIGHT_PIN);
  servoHold();

  pixy.init();
  pixy.changeProg("color_connected_components");
  pixy.setLamp(0, 0);

  gState = SEEK_ALIGN;
  bState = SEEK_BASE;
  topState = STATE_GRAB_BALL;
  baseAreaSmooth = 0.0f;
  alreadyReleased = false;

  Serial.println("Full cycle  ball via US1  base via Pixy SIG1 plus US2 instant release");
}

void loop() {
  switch (topState) {

    case STATE_GRAB_BALL: {
      bool grabbed = runGrabBallStep();
      if (grabbed) {
        Serial.println("Top  ball grabbed  switching to base");
        delay(200);
        bState = SEEK_BASE;
        baseAreaSmooth = 0.0f;
        alreadyReleased = false;
        topState = STATE_SEEK_AND_APPROACH_BASE;
      }
      break;
    }

    case STATE_SEEK_AND_APPROACH_BASE: {
      bool finishedBase = runBaseStep();
      if (finishedBase) {
        Serial.println("Top  base done  restart cycle");
        motorsStop();
        delay(500);
        servoHold();
        gState = SEEK_ALIGN;
        topState = STATE_GRAB_BALL;
      }
      break;
    }
  }
}
