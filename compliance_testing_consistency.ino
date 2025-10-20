//import libraryies
//#include <Pixy2.h>

//create instance
Pixy2 pixy;

//bring all colour signatures saved on pixyCam
int sigBlueBall = 1;
int sigGreenBall = 2;

void setup() {
    pixy.init();
}


void loop() {
    //CASE 1: detect colour for differently rotated blue ball do same for green ball 
    //fetch blocks 
    pixy.ccc.getBlocks();
    //if an object is detected 
    if (pixy.ccc.numBlocks) {
        for (int i = 0; i < pixy.ccc.numBlocks; i++) {
            //blue ball
            if (pixy.ccc.blocks[i].m_signature == 1) {
                //print output for blueball
                Serial.println("Blue ball detected");
                //light up led
            }
            //greenball
            else if (pixy.ccc.blocks[i].m_signature == 2) {
                //print output for greenball
                Serial.println("Green ball detected");
            }
        }
    }

}




