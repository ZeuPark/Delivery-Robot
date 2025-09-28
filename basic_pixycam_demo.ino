/**
 * Pixy2 4-signature detection logger
 * - Use after training colors into S1~S4 with PixyMon
 * - Prints signature, position (x,y), size (w,h), and area to Serial
 */

 #include <Pixy2.h>
 Pixy2 pixy;
 
 // Human-readable names for each signature ID
 const char* sigName(uint8_t sig) {
   switch (sig) {
     case 1: return "S1 Base";
     case 2: return "S2 RedBall";
     case 3: return "S3 BlueBall";
     case 4: return "S4 Line";
     default: return "S? Unknown";
   }
 }
 
 void setup() {
   Serial.begin(115200);
   while (!Serial) { /* wait for Serial on some boards */ }
   Serial.println(F("=== Pixy2 4-signature Logger ==="));
   pixy.init();
   Serial.println(F("Pixy init done. Make sure S1~S4 are trained in PixyMon."));
   delay(200);
 }
 
 unsigned long lastPrint = 0;
 const unsigned long PRINT_INTERVAL_MS = 200; // print interval (avoid flooding serial)
 
 void loop() {
   // Get block data from Pixy
   pixy.ccc.getBlocks();
 
   unsigned long now = millis();
   // Skip printing if interval not elapsed
   if (now - lastPrint < PRINT_INTERVAL_MS) return;
   lastPrint = now;
 
   int n = pixy.ccc.numBlocks;
   if (n <= 0) {
     Serial.println(F("[INFO] No blocks detected"));
     return;
   }
 
   // Print header
   Serial.print(F("Detected blocks: "));
   Serial.println(n);
 
   // Print all detected blocks
   for (int i = 0; i < n; i++) {
     Pixy2::CCC::Block &b = pixy.ccc.blocks[i];
     uint8_t sig = b.m_signature;
     int x = b.m_x;
     int y = b.m_y;
     int w = b.m_width;
     int h = b.m_height;
     long area = (long)w * (long)h;
 
     // Main log line
     Serial.print(" - ");
     Serial.print(sigName(sig)); Serial.print(" (S"); Serial.print(sig); Serial.print(")");
     Serial.print("  | X:"); Serial.print(x);
     Serial.print(" Y:"); Serial.print(y);
     Serial.print(" W:"); Serial.print(w);
     Serial.print(" H:"); Serial.print(h);
     Serial.print(" area:"); Serial.println(area);
 
     // Extra hint: center alignment and proximity
     int centerX = 158; // Pixy2 center X (316x208 resolution)
     int errX = x - centerX;
     Serial.print("    -> x_error: "); Serial.print(errX);
     if (abs(errX) <= 20) Serial.print(" (CENTER)");
     else if (errX > 0) Serial.print(" (RIGHT)");
     else Serial.print(" (LEFT)");
 
     if (area >= 6000) Serial.print("  | NEAR");
     else if (area >= 2500) Serial.print("  | APPROACH");
     else Serial.print("  | FAR");
     Serial.println();
   }
 
   Serial.println(F("-------------------------------------"));
 }
 