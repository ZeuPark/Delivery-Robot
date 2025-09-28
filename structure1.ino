#include <SPI.h>
#include <Pixy2.h>

/**
 * Robot core logic (Pixy2 + Arduino Uno) with obstacle detection (Pixy-only)
 * - Transport: SPI (default) → avoids Serial1/UART on Uno.
 * - Vision-driven FSM; actuators are mocked (Serial logs).
 * - Train signatures in PixyMon (see mapping below).
 *
 * Mapping (recommended):
 *   S1 = BASE
 *   S2 = BALL_RED   (forbidden)
 *   S3 = BALL_GREEN (allowed)
 *   S4 = BALL_BLUE  (allowed)
 *   S5 = SCORING_LINE
 */

Pixy2 pixy;

/* --------------------------- Config --------------------------- */
constexpr uint8_t SIG_BASE       = 1;
constexpr uint8_t SIG_RED        = 2;
constexpr uint8_t SIG_GREEN      = 3;
constexpr uint8_t SIG_BLUE       = 4;
constexpr uint8_t SIG_SCORE_LINE = 5;

constexpr int X_CENTER = 158;   // Pixy2: 316x208
constexpr int X_TOL    = 18;

constexpr long BALL_NEAR_AREA = 2500;
constexpr long BASE_NEAR_AREA = 6000;
constexpr int  LINE_Y_BAND    = 180;

/* Obstacle heuristics (Pixy-only) */
constexpr long      OBS_AREA_GATE      = 3000;
constexpr long      OBS_RATE_GATE      = 1500;     // px^2/s
constexpr uint32_t  OBS_DT_MS          = 100;
constexpr int       ROI_X_MIN          = 316*3/10; // ~94
constexpr int       ROI_X_MAX          = 316*7/10; // ~221
constexpr int       ROI_Y_MIN          = 208*6/10; // ~125
constexpr uint8_t   OBS_PERSIST_REQ    = 2;

/* -------------------------- Utilities ------------------------- */
#define LOG(x)        do { Serial.println(x); } while (0)
#define LOG2(a,b)     do { Serial.print(a); Serial.println(b); } while (0)
#define LOG3(a,b,c)   do { Serial.print(a); Serial.print(b); Serial.println(c); } while (0)

struct Det {
  bool     found;
  int      idx;
  uint8_t  sig;
  int      x, y, w, h;
  long     area;
  Det(): found(false), idx(-1), sig(0), x(0), y(0), w(0), h(0), area(0) {}
};

inline int xError(const Det& d){ return d.x - X_CENTER; }

/* Fill Det from a Pixy block */
static void fillDetFromBlock(Det &d, int i, const Block &b) {
  d.found = true;
  d.idx   = i;
  d.sig   = (uint8_t)b.m_signature;
  d.x     = b.m_x;
  d.y     = b.m_y;
  d.w     = b.m_width;
  d.h     = b.m_height;
  d.area  = (long)b.m_width * (long)b.m_height;
}

Det largestBySig(uint8_t sig){
  Det d; long best=-1;
  for(int i=0;i<pixy.ccc.numBlocks;i++){
    const auto &b = pixy.ccc.blocks[i];
    if(b.m_signature==sig){
      long a = (long)b.m_width * (long)b.m_height;
      if(a>best){ best=a; fillDetFromBlock(d, i, b); }
    }
  }
  return d;
}

Det largestAllowedBall(){
  Det d; long best=-1;
  for(int i=0;i<pixy.ccc.numBlocks;i++){
    const auto &b = pixy.ccc.blocks[i];
    if(b.m_signature==SIG_RED) continue;
    if(b.m_signature!=SIG_GREEN && b.m_signature!=SIG_BLUE) continue;
    long a = (long)b.m_width * (long)b.m_height;
    if(a>best){ best=a; fillDetFromBlock(d, i, b); }
  }
  return d;
}

/* ---------------------- Actuator Abstraction ------------------- */
void driveForward(){ LOG("[ACT] Forward"); }
void driveBackoff(){ LOG("[ACT] Backoff"); }
void turnLeft()    { LOG("[ACT] Turn Left"); }
void turnRight()   { LOG("[ACT] Turn Right"); }
void halt()        { LOG("[ACT] Stop"); }
void gripperOpen() { LOG("[ACT] Gripper Open"); }
void gripperClose(){ LOG("[ACT] Gripper Close"); }

/* ----------------------- Obstacle (Pixy-only) ------------------ */
struct ObsState {
  uint32_t lastMs;
  long     lastArea;
  uint8_t  persist;
  ObsState(): lastMs(0), lastArea(0), persist(0) {}
} obs;

bool isKnownSig(uint8_t s) {
  return (s==SIG_BASE || s==SIG_RED || s==SIG_GREEN || s==SIG_BLUE || s==SIG_SCORE_LINE);
}

bool inDangerROI(int x, int y) {
  return (x >= ROI_X_MIN && x <= ROI_X_MAX && y >= ROI_Y_MIN);
}

bool obstacleTriggered() {
  long maxArea = 0; int selX = -1, selY = -1;

  for (int i=0;i<pixy.ccc.numBlocks;i++){
    const auto &b = pixy.ccc.blocks[i];
    if (isKnownSig(b.m_signature)) continue;
    if (!inDangerROI(b.m_x, b.m_y)) continue;
    long a = (long)b.m_width * (long)b.m_height;
    if (a > maxArea) { maxArea = a; selX = b.m_x; selY = b.m_y; }
  }

  if (maxArea == 0) {
    if (obs.persist) obs.persist--;
    obs.lastArea = 0; obs.lastMs = millis();
    return false;
  }

  uint32_t now = millis();
  long rate = 0;
  if (obs.lastMs > 0 && (now - obs.lastMs) > 0) {
    rate = (maxArea - obs.lastArea) * 1000L / (long)(now - obs.lastMs);
  }
  obs.lastArea = maxArea; obs.lastMs = now;

  bool areaHit = (maxArea >= OBS_AREA_GATE);
  bool rateHit = (rate    >= OBS_RATE_GATE);

  Serial.print("[OBS] ROI x="); Serial.print(selX);
  Serial.print(" y="); Serial.print(selY);
  Serial.print(" area="); Serial.print(maxArea);
  Serial.print(" dA/dt="); Serial.print(rate);
  Serial.print(" gates: area>="); Serial.print(OBS_AREA_GATE);
  Serial.print(" or rate>="); Serial.println(OBS_RATE_GATE);

  bool hit = (areaHit || rateHit);
  if (hit) {
    if (++obs.persist >= OBS_PERSIST_REQ) {
      LOG("[OBS] CONFIRMED obstacle -> AVOID");
      return true;
    } else {
      LOG("[OBS] pre-confirm (persistence building)");
    }
  } else {
    if (obs.persist) obs.persist--;
  }
  return false;
}

/* ------------------------------ FSM ---------------------------- */
enum class State {
  INIT, BASE_DETECT, LEAVE_BASE, SEARCH_BALL, APPROACH_BALL,
  PICKUP, RETURN_HOME, ALIGN_AT_BASE, DEPOSIT, AVOID, DONE
};
State st = State::INIT;

void setup(){
  Serial.begin(115200);
  while(!Serial){}  // wait if needed
  LOG("=== Robot core start (SPI link, obstacle detection) ===");
  pixy.init();      // SPI init
  LOG("Pixy OK. Train S1..S5 in PixyMon and lock WB/Exposure (manual).");
  st = State::BASE_DETECT;
}

void loop(){
  pixy.ccc.getBlocks();

  if (st!=State::AVOID && st!=State::DONE) {
    if (obstacleTriggered()) st = State::AVOID;
  }

  switch(st){

    case State::INIT:
      st = State::BASE_DETECT;
      break;

    case State::BASE_DETECT: {
      Det base = largestBySig(SIG_BASE);
      if(!base.found){ LOG("[BASE_DETECT] scanning for base..."); turnLeft(); }
      else { LOG("[BASE_DETECT] base found -> LEAVE_BASE"); st=State::LEAVE_BASE; }
    } break;

    case State::LEAVE_BASE: {
      Det base = largestBySig(SIG_BASE);
      if(!base.found){ LOG("[LEAVE_BASE] left base -> SEARCH_BALL"); st=State::SEARCH_BALL; }
      else { driveForward(); }
    } break;

    case State::SEARCH_BALL: {
      Det ball = largestAllowedBall();
      if(!ball.found){ LOG("[SEARCH_BALL] rotate to find allowed ball..."); turnRight(); }
      else { LOG3("[SEARCH_BALL] allowed ball seen (sig=", ball.sig, ") -> APPROACH_BALL"); st=State::APPROACH_BALL; }
    } break;

    case State::APPROACH_BALL: {
      Det ball = largestAllowedBall();
      if(!ball.found){ LOG("[APPROACH_BALL] lost ball -> SEARCH_BALL"); st=State::SEARCH_BALL; break; }
      int err = xError(ball);
      Serial.print("   ball x_err="); Serial.print(err);
      Serial.print("  area="); Serial.println(ball.area);

      if (abs(err) > X_TOL) {
        if (err > 0) turnRight(); else turnLeft();
      } else {
        if (ball.area < BALL_NEAR_AREA) driveForward();
        else { LOG3("[PICKUP] close, picking ball sig=", ball.sig, ""); gripperClose(); st=State::RETURN_HOME; }
      }
    } break;

    case State::RETURN_HOME: {
      Det base = largestBySig(SIG_BASE);
      if(!base.found){ LOG("[RETURN_HOME] searching base..."); turnLeft(); }
      else{
        int err=xError(base);
        Serial.print("   base x_err="); Serial.print(err);
        Serial.print("  base area="); Serial.println(base.area);
        if(abs(err)>X_TOL){ if(err>0) turnRight(); else turnLeft(); }
        else{ driveForward(); }
        if(base.area>BASE_NEAR_AREA){ LOG("[RETURN_HOME] near base -> ALIGN_AT_BASE"); st=State::ALIGN_AT_BASE; }
      }
    } break;

    case State::ALIGN_AT_BASE: {
      Det line = largestBySig(SIG_SCORE_LINE);
      if(line.found){
        Serial.print("   line y="); Serial.println(line.y);
        if(line.y > LINE_Y_BAND){
          st = State::DEPOSIT;
        } else {
          driveForward();
        }
      } else {
        turnRight();
      }
    } break;

    case State::DEPOSIT: {
      LOG("[DEPOSIT] releasing ball beyond scoring line");
      gripperOpen();
      halt();
      st = State::DONE;
    } break;

    case State::AVOID: {
      LOG("[AVOID] stop + backoff + replan");
      halt(); delay(350);
      driveBackoff(); delay(400);
      halt(); delay(150);
      st = State::SEARCH_BALL; // resume
    } break;

    case State::DONE:
      halt();
      delay(300);
      break;
  }
}

 