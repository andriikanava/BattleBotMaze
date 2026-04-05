#include "MazeLogic.h"
#include "Config.h"
#include "ServoControl.h"
#include "Movement.h"
#include "Sensors.h"

// Waits until an obstacle is detected in front of the robot several times in a row.
// This acts as a stable start condition and avoids reacting to random sensor noise.
// After confirmation, the robot pauses briefly and performs the initial maze setup.
void waitForTurn() {
  int validCount = 0;

  while (true) {
    refreshServo();

    float frontDist = readDistanceCm(trigPin, echoPin);

    Serial.print("WAIT front = ");
    Serial.println(frontDist);

    // Count only consistent short-range detections.
    // If the reading becomes invalid or the obstacle disappears, restart the check.
    if (frontDist > 0 && frontDist < 30) {
      validCount++;
    } else {
      validCount = 0;
    }

    // Start only after several consecutive confirmations.
    if (validCount >= 3) {
      waitWithServo(3000);
      initMaze();
      break;
    }

    delay(30);
  }
}

// Performs the initial sequence required before normal maze navigation.
// move a little forward, close the gripper, turn left,
// and reposition the robot for the wall-following phase.
void initMaze() {
  moveForward(0.28);
  pauseAfterAction();

  grip();
  waitWithServo(1000);

  turnLeft(75);
  pauseAfterAction();

  moveForward(0.07);
  pauseAfterAction();
}

// Handles the situation when the robot is too close to the left wall.
// If the robot is trapped in a corner, it performs a stronger recovery:
// move back, rotate right, and step forward slightly.
// Otherwise, it performs a lighter correction to restore safe spacing.
void escapeFromLeftWall() {
  float frontDist = readDistanceCm(trigPin, echoPin);
  float leftDist  = readDistanceCm(leftTrigPin, leftEchoPin);

  Serial.print("ESCAPE | front = ");
  Serial.print(frontDist);
  Serial.print(" | left = ");
  Serial.println(leftDist);

  // Corner case: the robot is blocked in front and also pressed too close to the left wall.
  if (leftCritical(leftDist) && hasFrontWall(frontDist)) {
    Serial.println("ESCAPE: corner detected -> stronger recovery");

    moveBackward(0.05);
    pauseAfterAction();

    correctRight(18);
    pauseAfterAction();

    microForward(0.02);
    pauseAfterAction();

    return;
  }

  // Standard recovery when the left wall is too close but the robot is not fully trapped.
  Serial.println("ESCAPE: too close to left wall");
  moveBackward(0.02);
  pauseAfterAction();

  correctRightAndAdvance(10);
}

// Restores contact with the left wall when the robot starts drifting away from it.
// The function first checks that the front path is still clear.
// If the front is blocked, correction is aborted to avoid steering into an obstacle.
void reacquireLeftWall() {
  Serial.println("REACQUIRE: left wall drifting away");

  float frontDist = readDistanceCm(trigPin, echoPin);
  if (hasFrontWall(frontDist)) {
    Serial.println("REACQUIRE aborted: front blocked");
    return;
  }

  correctLeftAndAdvance(8);
}

// Fine-tunes the robot position relative to the left wall.
// The function tries several times to bring the robot back to the target distance.
// - if the wall is too close, shift right;
// - if the wall is too far, shift left;
// - if the front is blocked, avoid unsafe left corrections;
// - if the distance is acceptable, stop aligning.
void alignToLeftWall() {
  for (int i = 0; i < 3; i++) {
    float leftDist = readDistanceCm(leftTrigPin, leftEchoPin);
    float frontDist = readDistanceCm(trigPin, echoPin);

    Serial.print("ALIGN left = ");
    Serial.print(leftDist);
    Serial.print(" | front = ");
    Serial.println(frontDist);

    // Abort alignment if the side reading is invalid.
    if (leftDist < 0) {
      return;
    }

    // Strong correction when the robot is dangerously close to the wall.
    if (leftDist < LEFT_CRITICAL) {
      Serial.println("ALIGN: LEFT CRITICAL -> hard right + advance");
      correctRightAndAdvance(12);
      continue;
    }

    // Moderate correction when the robot is slightly too close.
    if (leftDist < LEFT_TARGET - LEFT_TOL) {
      Serial.println("ALIGN: too close -> right + advance");
      correctRightAndAdvance(7);
      continue;
    }

    // Stronger left correction when the wall is almost lost.
    if (leftAlmostLost(leftDist)) {
      Serial.println("ALIGN: almost lost -> stronger left + advance");
      if (!hasFrontWall(frontDist)) {
        correctLeftAndAdvance(8);
      }
      continue;
    }

    // Small left correction when the robot is simply too far from the wall.
    if (leftTooFar(leftDist)) {
      Serial.println("ALIGN: too far -> left + advance");
      if (!hasFrontWall(frontDist)) {
        correctLeftAndAdvance(4);
      }
      continue;
    }

    // Alignment is acceptable; no further correction is needed.
    Serial.println("ALIGN: OK");
    return;
  }
}

// Moves the robot forward in very small increments while continuously checking safety.
// This function is the main protected forward-motion routine.
// During motion it stops immediately if:
// - a wall appears in front,
// - the robot gets too close to the left wall,
// - the left wall is lost,
// - the robot drifts too far from the target corridor position.
// Instead of forcing forward motion, it switches to the proper correction routine.
void safeStepForward() {
  float moved = 0.0;

  while (moved < STEP_METERS) {
    float frontDist = readDistanceCm(trigPin, echoPin);
    float leftDist  = readDistanceCm(leftTrigPin, leftEchoPin);

    Serial.print("SAFE STEP | front = ");
    Serial.print(frontDist);
    Serial.print(" | left = ");
    Serial.println(leftDist);

    // Stop advancing if the front path is no longer free.
    if (hasFrontWall(frontDist)) {
      Serial.println("SAFE STEP: front blocked");
      break;
    }

    // Immediate escape if the robot is critically close to the left wall.
    if (leftCritical(leftDist)) {
      Serial.println("SAFE STEP: left critical");
      escapeFromLeftWall();
      break;
    }

    // Small right correction if the robot is too close to the wall.
    if (leftTooClose(leftDist)) {
      Serial.println("SAFE STEP: left too close -> right correction");
      correctRightAndAdvance(6);
      break;
    }

    // Reacquire the wall if the left side is almost open.
    if (leftAlmostLost(leftDist)) {
      Serial.println("SAFE STEP: left almost lost -> reacquire");
      reacquireLeftWall();
      break;
    }

    // Small left correction if the robot drifts away from the wall.
    if (leftTooFar(leftDist)) {
      Serial.println("SAFE STEP: left too far -> slight left correction");
      correctLeftAndAdvance(4);
      break;
    }

    // Continue forward only while all safety conditions remain valid.
    microForward(MICRO_STEP);
    moved += MICRO_STEP;
  }

  pauseAfterAction();
}

// Executes one full decision step of the left-wall-following maze algorithm.
// Priority of decisions:
// 1. Escape if the left wall is dangerously close.
// 2. Slightly move right if the robot is too close to the wall.
// 3. Turn left if the left side opens up.
// 4. Turn right if the front is blocked.
// 5. Reacquire or realign if the left wall is becoming too distant.
// 6. Otherwise, continue moving forward safely.
//
// In simple terms, this function decides where the robot should go next
// based on the front and left distance sensors.
void followLeftWallStep() {
  float frontDist = readDistanceCm(trigPin, echoPin);
  float leftDist  = readDistanceCm(leftTrigPin, leftEchoPin);

  Serial.print("front = ");
  Serial.print(frontDist);
  Serial.print(" | left = ");
  Serial.println(leftDist);

  // Highest-priority safety case: the robot is critically close to the left wall.
  if (leftCritical(leftDist)) {
    Serial.println("Decision: LEFT CRITICAL -> escape right");
    escapeFromLeftWall();
    return;
  }

  // Mild correction when the robot is too close but not yet in a critical state.
  if (leftTooClose(leftDist)) {
    Serial.println("Decision: LEFT TOO CLOSE -> slight right");
    correctRightAndAdvance(6);
    return;
  }

  // Left side is open: according to the left-wall rule, prefer turning left.
  if (leftWallLost(leftDist)) {
    Serial.println("Decision: LEFT OPEN -> move forward then turn left");

    // Make a short forward move before turning, but only if the front remains clear.
    float frontCheck = readDistanceCm(trigPin, echoPin);
    if (hasFrontWall(frontCheck)) {
      Serial.println("Left open but front blocked too soon, cannot advance");
    } else {
      moveForward(0.15);
      pauseAfterAction();
    }

    correctLeft(LEFT_TURN_DEG);
    moveForward(0.1);

    float frontAfterLeft = readDistanceCm(trigPin, echoPin);
    float leftAfterLeft  = readDistanceCm(leftTrigPin, leftEchoPin);

    Serial.print("after left turn | front = ");
    Serial.print(frontAfterLeft);
    Serial.print(" | left = ");
    Serial.println(leftAfterLeft);

    // If the new direction is free, continue normal progression.
    if (!hasFrontWall(frontAfterLeft)) {
      safeStepForward();
      alignToLeftWall();
      return;
    }

    // If the left turn leads into an obstacle, partially recover to the right.
    Serial.println("Left turn blocked, recovering...");
    correctRight(10);

    frontAfterLeft = readDistanceCm(trigPin, echoPin);
    if (!hasFrontWall(frontAfterLeft)) {
      safeStepForward();
      alignToLeftWall();
      return;
    }
  }

  // Front obstacle detected: turn right to continue navigating.
  if (hasFrontWall(frontDist)) {
    Serial.println("Decision: FRONT BLOCKED -> turn right");
    correctRight(RIGHT_TURN_DEG);

    float frontAfterRight = readDistanceCm(trigPin, echoPin);
    float leftAfterRight  = readDistanceCm(leftTrigPin, leftEchoPin);

    Serial.print("after right turn | front = ");
    Serial.print(frontAfterRight);
    Serial.print(" | left = ");
    Serial.println(leftAfterRight);

    // If the new right direction is free, proceed and then realign.
    if (!hasFrontWall(frontAfterRight)) {
      microForward(MICRO_STEP);
      pauseAfterAction();

      safeStepForward();
      alignToLeftWall();
      return;
    }

    // If right is also blocked, the robot is likely in a dead end.
    // A second right turn acts as a U-turn.
    Serial.println("Decision: DEAD END -> second right turn");
    correctRight(RIGHT_TURN_DEG);

    float frontAfterUTurn = readDistanceCm(trigPin, echoPin);
    Serial.print("front after uturn = ");
    Serial.println(frontAfterUTurn);

    if (!hasFrontWall(frontAfterUTurn)) {
      microForward(MICRO_STEP);
      pauseAfterAction();

      safeStepForward();
      alignToLeftWall();
      return;
    }

    // Final fallback in case even the U-turn direction remains blocked.
    Serial.println("Emergency recovery");
    moveBackward(0.03);
    pauseAfterAction();
    correctRightAndAdvance(20);
    return;
  }

  // Left wall is almost gone: restore wall contact before continuing.
  if (leftAlmostLost(leftDist)) {
    Serial.println("Decision: LEFT ALMOST LOST -> reacquire first");
    reacquireLeftWall();
    alignToLeftWall();
    return;
  }

  // Left wall still exists but the robot is drifting too far from it.
  if (leftTooFar(leftDist)) {
    Serial.println("Decision: LEFT TOO FAR -> align left");
    alignToLeftWall();
    return;
  }

  // Default case: path is stable, so continue forward under safety control.
  Serial.println("Decision: FORWARD");
  safeStepForward();
  alignToLeftWall();
}