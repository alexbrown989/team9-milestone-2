/* TME 441
*  Milestone 2 Solution
*
*  Navigate to a series of waypoints using camera input.
*
*  This library does not include driving and motor control functions. You must
*  copy in your own driving.ino and motor_control.h files.
*
*  @author Matthew Ford
*/

//hello world!
#include <Encoder.h>
#include "motor_control.h"
#include "HUSKYLENS.h"

HUSKYLENS cam;


int main() {
  init();

  // Initialize communication interfaces
  Serial.begin(9600);
  Serial3.begin(9600);  // HuskyLens

  initMotors();

  // Configure HuskyLens camera
  while (!cam.begin(Serial3)) {
    Serial.println(F("Begin failed!"));
    Serial.println(F("1.Please recheck the \"Protocol Type\" in HUSKYLENS (General Settings>>Protocol Type>>Serial 9600)"));
    Serial.println(F("2.Please recheck the connection."));
    delay(100);
  }

  Serial.println("HuskyLens Connected!");

  // Set the HuskyLens to tag recognition mode
  cam.writeAlgorithm(ALGORITHM_TAG_RECOGNITION);
  delay(1000);

  // Navigate to each April tag in sequence
  int current_tag = 1;
  while (true) {
    delay(1000);

    Serial.print("Navigating to tag: ");
    Serial.println(current_tag);

    int result = driveToTag(current_tag);

    if (result == 1) {
      current_tag = (current_tag < 3) ? current_tag+1 : 1;
    }
  }
}
