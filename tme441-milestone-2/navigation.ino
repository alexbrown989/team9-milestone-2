
/*
  navigation.ino

  Find and drive toward a specific AprilTag.

  Steps:
  1. Search for the tag.
  2. Calculate its heading and distance.
  3. Turn toward the tag.
  4. Check the camera again.
  5. Drive forward in short movements.
  6. Stop near the tag.

  No ultrasonic sensor is used.
*/

// Search by rotating 15 degrees at a time.
const float SEARCH_STEP_DEG = 15.0f;

// Maximum angle correction per movement.
const float MAX_ALIGN_TURN_DEG = 8.0f;

// Desired heading accuracy.
const float HEADING_OK_DEG = 3.0f;

// Allow slightly more error when close to a sign.
const float NEAR_HEADING_OK_DEG = 6.0f;

// Desired stopping distance from robot's front.
const float STOP_FRONT_CM = 8.5f;

// Assignment requires stopping within 10 cm.
const float ACCEPT_FRONT_CM = 10.0f;

// Reject suspiciously close measurements.
const float TOO_CLOSE_CM = 4.5f;

// Avoid pivoting very close to the sign.
const float PIVOT_MIN_GAP_CM = 14.0f;

// Begin final short movement from this distance.
const float FINAL_MOVE_FROM_CM = 17.0f;

// Maximum time allowed for each tag.
const unsigned long TAG_TIMEOUT_MS = 120000UL;


// Stop both motors if something goes wrong.
int stopTagWithError(int errorCode) {

  stopDriving();

  return errorCode;
}


// Find and approach the requested tag.
//
// Return values:
//  1 = Reached estimated stopping position
// -1 = Tag not found or search timed out
// -2 = Camera communication error
// -3 = Driving or range error
int driveToTag(int tagID) {

  // Remember when the search began.
  const unsigned long startTime = millis();

  // Track search rotation.
  float searchDegrees = 0.0f;

  // Remember whether we started approaching.
  bool startedApproach = false;

  // Count missed camera frames.
  int missedFrames = 0;


  while (millis() - startTime < TAG_TIMEOUT_MS) {

    // STEP 1: Read range and heading together.
    TagPose p = rangeAndHeadingToTag(tagID);

    // Stop if communication with the camera fails.
    if (p.status == -2) {
      return stopTagWithError(-2);
    }


    // ===================================
    // STEP 2: SEARCH FOR THE TAG
    // ===================================

    if (p.status == 0) {

      // If the robot was already approaching,
      // do not continue driving without the tag.
      if (startedApproach) {

        missedFrames++;

        // Allow a few camera retries.
        if (missedFrames >= 5) {
          return stopTagWithError(-1);
        }

        delay(80);
        continue;
      }

      // Stop after searching a full revolution.
      if (searchDegrees >= 360.0f) {
        return stopTagWithError(-1);
      }

      // Turn 15 degrees counterclockwise.
      if (!rotateCCW(SEARCH_STEP_DEG)) {
        return stopTagWithError(-3);
      }

      searchDegrees += SEARCH_STEP_DEG;

      // Allow camera image to settle.
      delay(80);

      continue;
    }


    // ===================================
    // STEP 3: CHECK DISTANCE
    // ===================================

    missedFrames = 0;

    // Convert lens distance to front distance.
    float front1 = distanceFromFront(p);

    // Reject unsafe or unreasonable readings.
    if (front1 <= TOO_CLOSE_CM || front1 > 250.0f) {
      return stopTagWithError(-3);
    }

    // Use a slightly larger heading tolerance
    // when very close to a sign.
    float headingTolerance =
        (front1 < 20.0f)
        ? NEAR_HEADING_OK_DEG
        : HEADING_OK_DEG;


    // ===================================
    // STEP 4: ALIGN WITH THE TAG
    // ===================================

    // If the tag is not centered, turn toward it.
    if (fabsf(p.heading) > headingTolerance) {

      // Avoid turning while extremely close.
      if (front1 < PIVOT_MIN_GAP_CM) {
        return stopTagWithError(-3);
      }

      // Limit the turn to prevent overshooting.
      float angle = constrain(
          p.heading,
          -MAX_ALIGN_TURN_DEG,
          MAX_ALIGN_TURN_DEG
      );

      // Rotate toward the tag.
      if (!rotateCCW(angle)) {
        return stopTagWithError(-3);
      }

      delay(80);

      // Take another camera reading after turning.
      continue;
    }


    // ===================================
    // STEP 5: CONFIRM CAMERA MEASUREMENT
    // ===================================

    // Take another measurement for reliability.
    delay(45);

    TagPose q = rangeAndHeadingToTag(tagID);

    if (q.status == -2) {
      return stopTagWithError(-2);
    }

    // Do not move without seeing the correct tag.
    if (q.status != 1) {
      continue;
    }

    // Confirm the tag is still centered.
    if (fabsf(q.heading) > headingTolerance) {
      continue;
    }

    float front2 = distanceFromFront(q);

    // Reject invalid distances.
    if (front2 <= TOO_CLOSE_CM || front2 > 250.0f) {
      return stopTagWithError(-3);
    }

    // Ignore large changes between measurements.
    if (fabsf(front2 - front1) >
        max(5.0f, 0.20f * front1)) {

      delay(80);
      continue;
    }

    // Average the two measurements.
    float gap = (front1 + front2) / 2.0f;

    // Print the estimated position.
    Serial.print("Tag ");
    Serial.print(tagID);

    Serial.print(" heading=");
    Serial.print(q.heading);

    Serial.print(" front_gap_estimate=");
    Serial.println(gap);


    // ===================================
    // STEP 6: CHECK STOPPING DISTANCE
    // ===================================

    // Stop if calibrated camera measurements
    // estimate that we are within 10 cm.
    if (gap <= ACCEPT_FRONT_CM) {

      stopDriving();

      return 1;
    }


    // ===================================
    // STEP 7: FINAL CLOSE-RANGE MOVEMENT
    // ===================================

    // When very close, the tag may become too
    // large for the camera to recognize reliably.
    //
    // Use the last confirmed distance and heading
    // for one short encoder-controlled movement.
    //
    // The actual gap must be checked during testing.
    if (gap <= FINAL_MOVE_FROM_CM) {

      float finalCm = gap - STOP_FRONT_CM;

      // Reject an unexpected movement distance.
      if (finalCm <= 0.0f || finalCm > 8.5f) {
        return stopTagWithError(-3);
      }

      // Drive toward estimated final stopping point.
      if (!driveDistance(finalCm)) {
        return stopTagWithError(-3);
      }

      // Stop after final encoder movement.
      stopDriving();

      Serial.println(
          "Final camera-based stop: verify gap with ruler"
      );

      return 1;
    }


    // ===================================
    // STEP 8: NORMAL APPROACH
    // ===================================

    // Use shorter movements near the target.
    float stepCm = 6.0f;

    if (gap < 45.0f) {
      stepCm = 3.0f;
    }

    if (gap < 25.0f) {
      stepCm = 1.5f;
    }

    // Stop before entering the final approach zone
    // so that we can take another camera reading.
    stepCm = min(
        stepCm,
        gap - FINAL_MOVE_FROM_CM
    );

    // Reject an invalid movement.
    if (stepCm <= 0.0f) {
      return stopTagWithError(-3);
    }

    startedApproach = true;

    // Drive using wheel encoder feedback.
    if (!driveDistance(stepCm)) {
      return stopTagWithError(-3);
    }

    // After moving, start the loop again and
    // get updated camera range and heading.
  }

  // Stop if the tag takes too long to reach.
  return stopTagWithError(-1);
}
