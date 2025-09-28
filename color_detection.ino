// https://docs.pixycam.com/wiki/doku.php?id=wiki:v2:hooking_up_pixy_to_a_microcontroller_-28like_an_arduino-29

  
#include <Pixy2.h>

// This is the main Pixy object 
Pixy2 pixy;

void setup()
{
  Serial.begin(115200);
  Serial.print("Starting...\n");
  
  pixy.init();
}

void loop()
{ 
  int i; 
  // grab blocks!
  pixy.ccc.getBlocks();
  
  // If there are detect blocks, print them!
  if (pixy.ccc.numBlocks)
  {
    Serial.print("Detected ");
    Serial.println(pixy.ccc.numBlocks);
    for (i=0; i<pixy.ccc.numBlocks; i++)
    {
      Serial.print("  block ");
      Serial.print(i);
      Serial.print(": ");
      // Print signature (color)
      Serial.print("color: ");
      switch(pixy.ccc.blocks[i].m_signature)
      {
        case 1:
          Serial.print("Red");
          break;
        case 2:
          Serial.print("Green");
          break;
        case 3:
          Serial.print("Blue");
          break;
        // Add more cases for other signatures you've trained
        default:
          Serial.print("Signature ");
          Serial.print(pixy.ccc.blocks[i].m_signature);
          break;
      }

      // Print range estimate (based on object size)
      // A larger area means the object is closer.
      int32_t area = (int32_t)pixy.ccc.blocks[i].m_width * (int32_t)pixy.ccc.blocks[i].m_height;
      Serial.print(", range estimate (area): ");
      Serial.println(area);
    }
  }  
}
