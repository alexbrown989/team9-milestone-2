
/*
  camera.ino

  Estimate distance AND heading to an AprilTag.

  X position tells us whether the tag is left or right.
  Tag HEIGHT estimates how far away it is.

  We use height instead of width because width
  can change significantly when viewing a tag
  from an angle.

  Camera settings must be calibrated experimentally.
*/

// Horizontal center of the camera image.
const float CAMERA_CENTER_X = 160.0f;

// Approximate focal length in pixels.
// Replace with your measured camera calibration.
const float CAMERA_FOCAL_PIXELS = 320.0f;

// Adjust if the camera is mounted slightly
// left or right of the robot's forward direction.
const float CAMERA_YAW_OFFSET_DEG = 0.0f;

// Range equation from Dr. Ford's original code.
// These values need to be checked experimentally.
const float RANGE_SCALE = 3489.0f;
const float RANGE_OFFSET_CM = 3.0f;

// Distance between camera lens and robot's front.
//
// Positive = lens is behind the front of robot.
// Replace 0 with your measured distance.
const float CAMERA_TO_FRONT_CM = 0.0f;


// Find one specific AprilTag.
// Return its range and heading from the same image.
TagPose rangeAndHeadingToTag(int tagID) {

  // Default: tag not visible.
  TagPose pose = {0, -1.0f, 0.0f, 0, 0};

  bool received = false;

  // Ask HuskyLens for the requested tag only.
  // Retry if a camera request fails.
  for (int attempt = 0; attempt < 3; attempt++) {

    if (cam.requestBlocks(tagID)) {

      received = true;
      break;
    }

    delay(35);
  }

  // Camera communication failed.
  if (!received) {

    pose.status = -2;
    return pose;
  }

  // Check the detected objects.
  while (cam.available()) {

    HUSKYLENSResult block = cam.read();

    // Ignore incorrect tags or invalid readings.
    if (block.command != COMMAND_RETURN_BLOCK ||
        block.ID != tagID ||
        block.height <= 0) {

      continue;
    }

    // Correct tag found.
    pose.status = 1;

    // Save the image measurements.
    pose.xCenter = block.xCenter;
    pose.height = block.height;

    // Estimate distance from apparent tag height.
    pose.range =
        RANGE_SCALE / block.height - RANGE_OFFSET_CM;

    // Calculate angle from the tag's horizontal position.
    //
    // Left of center  = positive angle (CCW).
    // Right of center = negative angle (CW).
    pose.heading =
        atan2f(
          CAMERA_CENTER_X - block.xCenter,
          CAMERA_FOCAL_PIXELS
        ) * 180.0f / PI;

    // Correct for camera mounting alignment.
    pose.heading += CAMERA_YAW_OFFSET_DEG;

    return pose;
  }

  // Requested tag was not found.
  return pose;
}


// Convert camera distance to distance from
// the robot's front reference point.
float distanceFromFront(TagPose tag) {

  return tag.range - CAMERA_TO_FRONT_CM;
}
