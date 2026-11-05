
/*
  TME 441 - AC-Team 9 - Milestone 2
  HuskyLens AprilTag Navigation

  Find Tag 1, then Tag 2, then Tag 3.
  Stop near each tag using camera measurements.
  Use motor encoders and PID for wheel movements.

  No ultrasonic sensor.
  No dead reckoning extra labor.
*/

#include <Encoder.h>
#include "motor_control.h"
#include <HUSKYLENS.h>
#include <math.h>

// HuskyLens uses Serial3 on the Arduino Mega.
HUSKYLENS cam;

bool cameraReady = false;

// Store range and heading from ONE camera image.
struct TagPose {
  int status;       // 1 = found, 0 = not seen, -2 = camera error
  float range;      // Distance from camera lens to tag, cm
  float heading;    // Positive = left, negative = right
  int xCenter;      // Horizontal position in the image
  int height;       // Apparent tag height in pixels
};

// Functions in the other Arduino tabs.
TagPose rangeAndHeadingToTag(int tagID);
float distanceFromFront(TagPose tag);
int driveToTag(int tagID);

bool driveDistance(float centimeters);
bool rotateCCW(float degrees);

// Keep false until camera distance and angle
// measurements have been calibrated and tested.
const bool CAMERA_CALIBRATED = false;


void setup() {

  // Start computer and HuskyLens communication.
  Serial.begin(9600);
  Serial3.begin(9600);

  // Set up the motor pins.
  initMotors();

  // Keep both motors stopped during startup.
  motorHalt(LMOTOR);
  motorHalt(RMOTOR);

  delay(700);

  // Check that HuskyLens is connected.
  // Camera TX -> Mega pin 15
  // Camera RX -> Mega pin 14
  if (!cam.begin(Serial3)) {

    Serial.println("HuskyLens connection failed");
    return;
  }

  // Tell the camera to recognize learned AprilTags.
  cam.writeAlgorithm(ALGORITHM_TAG_RECOGNITION);

  cameraReady = true;
  delay(700);

  // Prevent autonomous motion before calibration.
  if (!CAMERA_CALIBRATED) {

    Serial.println("Calibration mode: robot will not move");
    return;
  }

  // Visit tags 1, 2, and 3 in order.
  for (int tagID = 1; tagID <= 3; tagID++) {

    Serial.print("Searching for Tag ");
    Serial.println(tagID);

    // Search, aim, approach, and stop.
    int result = driveToTag(tagID);

    // Make sure both motors stop.
    motorHalt(LMOTOR);
    motorHalt(RMOTOR);

    // Stop the mission if navigation fails.
    if (result != 1) {

      Serial.print("Navigation failed: ");
      Serial.println(result);

      return;
    }

    Serial.print("Estimated arrival at Tag ");
    Serial.println(tagID);

    delay(400);

    // After Tags 1 and 2, back away before turning
    // to find the next sign.
    //
    // IMPORTANT: Confirm space behind the robot is clear.
    if (tagID < 3) {

      if (!driveDistance(-9.0f)) {

        Serial.println("Reverse movement failed");
        return;
      }
    }
  }

  // Stop permanently after Tag 3.
  motorHalt(LMOTOR);
  motorHalt(RMOTOR);

  Serial.println("Milestone 2 complete!");
}


void loop() {

  // Never repeat the mission.
  motorHalt(LMOTOR);
  motorHalt(RMOTOR);

  // When calibration is disabled, print the
  // camera measurements without moving the robot.
  if (!CAMERA_CALIBRATED && cameraReady) {

    static unsigned long lastPrint = 0;

    if (millis() - lastPrint >= 500) {

      lastPrint = millis();

      TagPose p = rangeAndHeadingToTag(1);

      if (p.status == 1) {

        Serial.print("x=");
        Serial.print(p.xCenter);

        Serial.print(" height=");
        Serial.print(p.height);

        Serial.print(" lens_cm=");
        Serial.print(p.range);

        Serial.print(" front_cm=");
        Serial.print(distanceFromFront(p));

        Serial.print(" heading_deg=");
        Serial.println(p.heading);

      } else if (p.status == 0) {

        Serial.println("Tag 1 not visible");

      } else {

        Serial.println("HuskyLens communication error");
      }
    }
  }

  delay(100);
}
