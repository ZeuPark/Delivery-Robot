# Vision-Guided Delivery Robot

This project controls a simple autonomous robot using an Arduino Uno and a Pixy2 smart camera. The robot's goal is to find colored balls, pick them up, and return them to a home base.

---

### What This Robot Can Do

For team members who are new to programming, here’s a simple summary of the robot's mission:

1.  **Find its Home Base:** The robot starts by looking for its designated "home base" to know where its starting (and ending) point is.
2.  **Search for Target Objects:** Once it leaves the base, it roams around to find specific colored balls (Green or Blue).
3.  **Pick Up the Correct Balls:** It knows which balls it's allowed to pick up and which ones to ignore (Red).
4.  **Return and Deliver:** After grabbing an allowed ball, it finds its way back to the home base to drop it off.
5.  **Avoid Unexpected Obstacles:** If something unknown suddenly appears in its path, the robot will stop, back up, and try to continue its mission.

---

### How It Works: The Robot's "Brain"

The robot's logic is based on a **Finite State Machine (FSM)**, which is like a simple checklist or a set of steps the robot follows. It can only be in one "state" (or step) at a time. For example, it can't search for a ball and return home at the same time.

The main states are:

-   `BASE_DETECT`: Find the home base.
-   `LEAVE_BASE`: Move away from the base to start the search.
-   `SEARCH_BALL`: Look for a Green or Blue ball.
-   `APPROACH_BALL`: Center the ball in its view and move towards it.
-   `RETURN_HOME`: After picking up a ball, find the base again.
-   `DEPOSIT`: Drop the ball at the base.
-   `AVOID`: Handle unexpected obstacles.

All of these decisions are made based on what the **Pixy2 camera** sees.

### Hardware Required

To build this robot, you will need:

-   **Controller:** Arduino Uno (or a compatible board)
-   **Vision Sensor:** Pixy2 Camera
-   **Chassis:** A robot body with 2 or 4 wheels and motors.
-   **Motor Driver:** A module to control the motors (e.g., L298N).
-   **Gripper:** A mechanism to pick up balls (e.g., a simple claw controlled by a servo motor).
-   **Power:** A battery pack for the Arduino and motors.

### Project Setup & Configuration

Follow these steps to get the robot up and running.

#### Step 1: Assemble the Robot

Build the robot chassis, mount the motors, motor driver, Arduino, and Pixy2 camera. The camera should be facing forward.

#### Step 2: Train the Pixy2 Camera (Very Important!)

The robot is only as smart as its eyes! You must teach the Pixy2 what to look for. Connect the Pixy2 to your computer via USB and use the **PixyMon** software.

You need to teach it **5 color signatures** in this exact order:

-   **Signature 1: `BASE`**
    -   This is the home base. Use a large, uniquely colored object (e.g., a big piece of blue construction paper).
-   **Signature 2: `BALL_RED`**
    -   The forbidden ball. The robot will see this but ignore it.
-   **Signature 3: `BALL_GREEN`**
    -   An allowed ball that the robot should pick up.
-   **Signature 4: `BALL_BLUE`**
    -   Another allowed ball.
-   **Signature 5: `SCORING_LINE`**
    -   A line or marker inside the base where the robot should drop the ball (e.g., a strip of yellow tape).

**⭐ Pro Tip for Reliability:** In PixyMon's settings (`Configure -> Camera`), **turn OFF "Auto Exposure Correction" and "Auto White Balance"**. Manually adjust the sliders until the colors look stable and correct in your robot's environment. This prevents the robot from getting confused by changing lights.

#### Step 3: Upload the Code

1.  Install the [Arduino IDE](https://www.arduino.cc/en/software).
2.  Install the Pixy2 library. In the IDE, go to `Sketch > Include Library > Manage Libraries...` and search for `Pixy2`.
3.  Open the `structure1.ino` file in the Arduino IDE.
4.  Connect the Arduino to your computer and upload the sketch.

### From Logic to Movement: The Final Step

The current `structure1.ino` code contains the complete logic, but the movement commands are just text printouts (e.g., `Serial.println("[ACT] Forward")`). This allows you to test the robot's brain using the **Serial Monitor** without any hardware moving.

To make the robot move, a programmer needs to edit the **`Actuator Abstraction`** section of the code. Each function (`driveForward()`, `turnLeft()`, etc.) must be filled in with code that controls your specific motor driver and servo.

**Example (for an L298N motor driver):**

```cpp
// This is just an example!
// Your code will depend on your specific motor driver library.

void driveForward(){
  // Code to make both motors spin forward
}

void turnLeft(){
  // Code to make the left motor go backward and the right motor go forward
}
```

