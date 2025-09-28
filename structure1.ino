/**
 * Robot core logic (Pixy2 + Arduino Uno)
 * - Vision-driven FSM; actuators are abstracted (Serial logs now).
 * - Train signatures in PixyMon beforehand (see mapping below).
 *
 * Required PixyMon mapping (recommended):
 *   S1 = BASE marker color (or ColorCode marker if you prefer)
 *   S2 = BALL_RED   (forbidden: must NOT pick)
 *   S3 = BALL_GREEN (allowed)
 *   S4 = BALL_BLUE  (allowed)
 *   S5 = SCORING_LINE (tape/marker inside base)
 *
 * You can remap IDs by changing constants below.
 */

 #include <Pixy2.h>
 Pixy2 pixy;
 
 /* --------------------------- Config --------------------------- */
 // Signature IDs
 constexpr uint8_t SIG_BASE       = 1;
 constexpr uint8_t SIG_RED        = 2;  // forbidden ball
 constexpr uint8_t SIG_GREEN      = 3;  // allowed ball
 constexpr uint8_t SIG_BLUE       = 4;  // allowed ball
 constexpr uint8_t SIG_SCORE_LINE = 5;  // deposit line/marker
 
 // Pixy2 image geometry
 constexpr int   X_CENTER = 158;         // 316x208 → center x
 constexpr int   X_TOL    = 18;          // acceptable horizontal error in px
 
 // Heuristics (tune after a few runs)
 constexpr long  BALL_NEAR_AREA = 2500;  // when to "pickup"
 constexpr long  BASE_NEAR_AREA = 6000;  // when "close to base"
 constexpr int   LINE_Y_BAND    = 180;   // line near bottom of frame
 
 // Obstacle heuristics (unknown big/rapid object)
 constexpr long  OBS_MAX_UNKNOWN_AREA  = 4000;
 constexpr int   OBS_DA_DT_THRESHOLD   = 1500; // px^2 per second
 constexpr uint32_t OBS_DT_MS          = 100;
 
 /* -------------------------- Utilities ------------------------- */
 #define LOG(x)        do { Serial.println(x); } while (0)
 #define LOG2(a,b)     do { Serial.print(a); Serial.println(b); } while (0)
 #define LOG3(a,b,c)   do { Serial.print(a); Serial.print(b); Serial.println(c); } while (0)
 
 struct Det {
   bool found=false; int idx=-1;
   uint8_t sig=0; int x=0,y=0,w=0,h=0; long area=0;
 };
 
 inline int xError(const Det& d){ return d.x - X_CENTER; }
 
 Det largestBySig(uint8_t sig){
   Det d; long best=-1;
   for(int i=0;i<pixy.ccc.numBlocks;i++){
     auto &b = pixy.ccc.blocks[i];
     if(b.m_signature==sig){
       long a = (long)b.m_width*b.m_height;
       if(a>best){ best=a; d={true,i,(uint8_t)b.m_signature,b.m_x,b.m_y,b.m_width,b.m_height,a}; }
     }
   }
   return d;
 }
 
 Det largestAllowedBall(){
   Det d; long best=-1;
   for(int i=0;i<pixy.ccc.numBlocks;i++){
     auto &b=pixy.ccc.blocks[i];
     if(b.m_signature==SIG_RED) continue;                // reject red
     if(b.m_signature!=SIG_GREEN && b.m_signature!=SIG_BLUE) continue;
     long a=(long)b.m_width*b.m_height;
     if(a>best){ best=a; d={true,i,(uint8_t)b.m_signature,b.m_x,b.m_y,b.m_width,b.m_height,a}; }
   }
   return d;
 }
 
 /* ---------------------- Actuator Abstraction ------------------- */
 /* Replace these with real drivers later (L298N, servos, etc.) */
 void driveForward(){ LOG("[ACT] Forward"); }
 void driveBackoff(){ LOG("[ACT] Backoff"); }
 void turnLeft()    { LOG("[ACT] Turn Left"); }
 void turnRight()   { LOG("[ACT] Turn Right"); }
 void halt()        { LOG("[ACT] Stop"); }
 void gripperOpen() { LOG("[ACT] Gripper Open"); }
 void gripperClose(){ LOG("[ACT] Gripper Close"); }
 
 /* ------------------------------ FSM ---------------------------- */
 enum class State {
   INIT, BASE_DETECT, LEAVE_BASE, SEARCH_BALL, APPROACH_BALL,
   PICKUP, RETURN_HOME, ALIGN_AT_BASE, DEPOSIT, AVOID, DONE
 };
 State st = State::INIT;
 
 // For obstacle heuristic
 uint32_t lastObsMs=0; long lastUnknownArea=0;
 
 void setup(){
   Serial.begin(115200);
   while(!Serial){}  // wait if needed
   LOG("=== Robot core start ===");
   pixy.init();
   LOG("Pixy initialized. Ensure signatures S1..S5 are trained.");
   st = State::BASE_DETECT;
 }
 
 bool obstacleTriggered(){
   long maxUnknown=0;
   for(int i=0;i<pixy.ccc.numBlocks;i++){
     auto &b=pixy.ccc.blocks[i];
     bool known = (b.m_signature==SIG_BASE || b.m_signature==SIG_RED ||
                   b.m_signature==SIG_GREEN|| b.m_signature==SIG_BLUE ||
                   b.m_signature==SIG_SCORE_LINE);
     if(!known){
       long a=(long)b.m_width*b.m_height;
       if(a>maxUnknown) maxUnknown=a;
     }
   }
   uint32_t now=millis();
   if(now-lastObsMs>=OBS_DT_MS){
     long dA = maxUnknown - lastUnknownArea;
     long rate = dA * 1000L / (long)(now-lastObsMs+1);
     lastUnknownArea=maxUnknown; lastObsMs=now;
     if(maxUnknown>OBS_MAX_UNKNOWN_AREA || rate>OBS_DA_DT_THRESHOLD){
       LOG("[OBS] Obstacle heuristic triggered");
       return true;
     }
   }
   return false;
 }
 
 void loop(){
   pixy.ccc.getBlocks();
 
   if(obstacleTriggered() && st!=State::AVOID && st!=State::DONE){
     st = State::AVOID;
   }
 
   switch(st){
 
     case State::INIT:
       st = State::BASE_DETECT;
       break;
 
     case State::BASE_DETECT: {
       Det base = largestBySig(SIG_BASE);
       if(!base.found){ LOG("[BASE_DETECT] scanning..."); turnLeft(); }
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
       else { LOG3("[SEARCH_BALL] ball seen (sig=", ball.sig, ") -> APPROACH_BALL"); st=State::APPROACH_BALL; }
     } break;
 
     case State::APPROACH_BALL: {
       Det ball = largestAllowedBall();
       if(!ball.found){ LOG("[APPROACH_BALL] lost ball -> SEARCH_BALL"); st=State::SEARCH_BALL; break; }
       int err = xError(ball);
       LOG3("   x_err=", err, " px");
       LOG2("   area=", ball.area);
       if(abs(err)>X_TOL){
         if(err>0) turnRight(); else turnLeft();
       }else{
         if(ball.area < BALL_NEAR_AREA) driveForward();
         else { LOG3("[PICKUP] close, picking ball sig=", ball.sig, ""); gripperClose(); st=State::RETURN_HOME; }
       }
     } break;
 
     case State::RETURN_HOME: {
       Det base = largestBySig(SIG_BASE);
       if(!base.found){ LOG("[RETURN_HOME] searching base..."); turnLeft(); }
       else{
         int err=xError(base);
         LOG3("   base x_err=", err, " px");
         if(abs(err)>X_TOL){ if(err>0) turnRight(); else turnLeft(); }
         else{ driveForward(); }
         if(base.area>BASE_NEAR_AREA){ LOG("[RETURN_HOME] near base -> ALIGN_AT_BASE"); st=State::ALIGN_AT_BASE; }
       }
     } break;
 
     case State::ALIGN_AT_BASE: {
       // Optional extra alignment step if you want to slow down before deposit
       Det line = largestBySig(SIG_SCORE_LINE);
       if(line.found){
         LOG2("   line y=", line.y);
         if(line.y > LINE_Y_BAND){
           st = State::DEPOSIT;
         } else {
           driveForward();
         }
       } else {
         // keep searching line while near base
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
       halt(); delay(400);
       driveBackoff(); delay(400);
       halt(); delay(150);
       // After avoidance, resume the most reasonable state:
       st = State::SEARCH_BALL;
     } break;
 
     case State::DONE:
       halt();
       // keep idle; you can reset state if multiple runs are required
       delay(300);
       break;
   }
 }
 