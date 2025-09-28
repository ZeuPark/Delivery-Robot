#include <Pixy2.h>
#include <Servo.h>

//set an instance of the camera in arduino environment
Pix2 pixy;

int baseSignature;
int redBallSignature;
int blueBallSignature;
int greenBallSignature;

//Detect base colour when starting off 

void setup() {
  Serial.begin(9600);
  pixy.init();
  delay(500);

  // Assume you manually teach the base as a color signature in PixyMon
   baseSignature = 1; // e.g., signature #1 corresponds to the base
   redBallSignature = 2;
   blueBallSignature = 3;
   greenBallSignature = 4;
}

void loop() {
    detectBall();
    moveToBall();
    pickUpBall();
    returnToBase();
    releaseBall();
}


//Start moving towards arena

//Encounter a ball
void detectBall() {
  pixy.ccc.getBlocks();

  if (pixy.ccc.numBlocks) {
    for (int i = 0; i < pixy.ccc.numBlocks; i++) {
      int sig = pixy.ccc.blocks[i].m_signature;

      if (sig == 2) {
        // Red ball detected, skip it
        continue;
      } else {
        // Non-red ball detected
        // moveToBall(pixy.ccc.blocks[i]);
        // pickUpBall();
        // returnToBase();

        return;
      }
    }
  } else {
    explore(); // move randomly or follow a search pattern
  }   
}

void moveToBall(Pixy2CCC::Block &block) {
    //move forward to ball until close enough

    //center camera/robot 
    return;
}

void pickUpBall() {
    //depends if we use a gripper/leg or suck the ball in 
    return;
}

void returnToBase() {
  while (true) {
    //fetch whatever objects detected so far, this should include walls, balls, base
    pixy.ccc.getBlocks();

    bool foundBase = false;
    for (int i = 0; i < pixy.ccc.numBlocks; i++) {
      if (pixy.ccc.blocks[i].m_signature == baseSignature) {
        baseBlock = pixy.ccc.blocks[i];
        foundBase = true;
        break;
      }
    }

    if (foundBase) {
        //return to base
        //center the camera and robot using centerX and errorX accordingly
        int centerX = 160;
        int errorX = block.m_x - centerX
        //if errorX > 0, the object is to the right, robot needs to turn right
        //if errorX < 0, the object is to the left, robot needs to turn left
        //if errorX = 0, the object is centered, move forward 
    } else {
      explore(); // search pattern if base not visible
    }
  }
  return;
}

void releaseBall() {
    //release the ball 
}

void explore() {
  moveForward(200);
  turnLeft(150);
  //see if any obstacles/balls were detected
  pixy.ccc.getBlocks();
  delay(200);
}

//don't know how to make it stop (maybe set a 20min timer where the robot just automatically stops moving)
void stopRobot() {
    //stop motors 
}
