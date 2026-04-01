#include "MazeLogic.h"
#include "Config.h"
#include "ServoControl.h"
#include "Movement.h"
#include "Sensors.h"

void waitForTurn() {
  int validCount = 0;

  while (true) {
    refreshServo();

    float frontDist = readDistanceCm(trigPin, echoPin);

    Serial.print("WAIT front = ");
    Serial.println(frontDist);

    if (frontDist > 0 && frontDist < 30) {
      validCount++;
    } else {
      validCount = 0;
    }

    if (validCount >= 3) {
      waitWithServo(3000);
      initMaze();
      break;
    }

    delay(30);
  }
}

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

void escapeFromLeftWall() {
  float frontDist = readDistanceCm(trigPin, echoPin);
  float leftDist  = readDistanceCm(leftTrigPin, leftEchoPin);

  Serial.print("ESCAPE | front = ");
  Serial.print(frontDist);
  Serial.print(" | left = ");
  Serial.println(leftDist);

  // Если реально зажат в углу: слева слишком близко и спереди стена
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

  Serial.println("ESCAPE: too close to left wall");
  moveBackward(0.02);
  pauseAfterAction();

  correctRightAndAdvance(10);
}

void reacquireLeftWall() {
  Serial.println("REACQUIRE: left wall drifting away");

  float frontDist = readDistanceCm(trigPin, echoPin);
  if (hasFrontWall(frontDist)) {
    Serial.println("REACQUIRE aborted: front blocked");
    return;
  }

  correctLeftAndAdvance(8);
}

void alignToLeftWall() {
  for (int i = 0; i < 3; i++) {
    float leftDist = readDistanceCm(leftTrigPin, leftEchoPin);
    float frontDist = readDistanceCm(trigPin, echoPin);

    Serial.print("ALIGN left = ");
    Serial.print(leftDist);
    Serial.print(" | front = ");
    Serial.println(frontDist);

    if (leftDist < 0) {
      return;
    }

    if (leftDist < LEFT_CRITICAL) {
      Serial.println("ALIGN: LEFT CRITICAL -> hard right + advance");
      correctRightAndAdvance(12);
      continue;
    }

    if (leftDist < LEFT_TARGET - LEFT_TOL) {
      Serial.println("ALIGN: too close -> right + advance");
      correctRightAndAdvance(7);
      continue;
    }

    if (leftAlmostLost(leftDist)) {
      Serial.println("ALIGN: almost lost -> stronger left + advance");
      if (!hasFrontWall(frontDist)) {
        correctLeftAndAdvance(8);
      }
      continue;
    }

    if (leftTooFar(leftDist)) {
      Serial.println("ALIGN: too far -> left + advance");
      if (!hasFrontWall(frontDist)) {
        correctLeftAndAdvance(4);
      }
      continue;
    }

    Serial.println("ALIGN: OK");
    return;
  }
}

void safeStepForward() {
  float moved = 0.0;

  while (moved < STEP_METERS) {
    float frontDist = readDistanceCm(trigPin, echoPin);
    float leftDist  = readDistanceCm(leftTrigPin, leftEchoPin);

    Serial.print("SAFE STEP | front = ");
    Serial.print(frontDist);
    Serial.print(" | left = ");
    Serial.println(leftDist);

    if (hasFrontWall(frontDist)) {
      Serial.println("SAFE STEP: front blocked");
      break;
    }

    if (leftCritical(leftDist)) {
      Serial.println("SAFE STEP: left critical");
      escapeFromLeftWall();
      break;
    }

    if (leftTooClose(leftDist)) {
      Serial.println("SAFE STEP: left too close -> right correction");
      correctRightAndAdvance(6);
      break;
    }

    if (leftAlmostLost(leftDist)) {
      Serial.println("SAFE STEP: left almost lost -> reacquire");
      reacquireLeftWall();
      break;
    }

    if (leftTooFar(leftDist)) {
      Serial.println("SAFE STEP: left too far -> slight left correction");
      correctLeftAndAdvance(4);
      break;
    }

    microForward(MICRO_STEP);
    moved += MICRO_STEP;
  }

  pauseAfterAction();
}

void followLeftWallStep() {
  float frontDist = readDistanceCm(trigPin, echoPin);
  float leftDist  = readDistanceCm(leftTrigPin, leftEchoPin);

  Serial.print("front = ");
  Serial.print(frontDist);
  Serial.print(" | left = ");
  Serial.println(leftDist);

  if (leftCritical(leftDist)) {
    Serial.println("Decision: LEFT CRITICAL -> escape right");
    escapeFromLeftWall();
    return;
  }

  if (leftTooClose(leftDist)) {
    Serial.println("Decision: LEFT TOO CLOSE -> slight right");
    correctRightAndAdvance(6);
    return;
  }

  if (leftWallLost(leftDist)) {
    Serial.println("Decision: LEFT OPEN -> move forward then turn left");

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

    if (!hasFrontWall(frontAfterLeft)) {
      safeStepForward();
      alignToLeftWall();
      return;
    }

    Serial.println("Left turn blocked, recovering...");
    correctRight(10);

    frontAfterLeft = readDistanceCm(trigPin, echoPin);
    if (!hasFrontWall(frontAfterLeft)) {
      safeStepForward();
      alignToLeftWall();
      return;
    }
  }

  if (hasFrontWall(frontDist)) {
    Serial.println("Decision: FRONT BLOCKED -> turn right");
    correctRight(RIGHT_TURN_DEG);

    float frontAfterRight = readDistanceCm(trigPin, echoPin);
    float leftAfterRight  = readDistanceCm(leftTrigPin, leftEchoPin);

    Serial.print("after right turn | front = ");
    Serial.print(frontAfterRight);
    Serial.print(" | left = ");
    Serial.println(leftAfterRight);

    if (!hasFrontWall(frontAfterRight)) {
      microForward(MICRO_STEP);
      pauseAfterAction();

      safeStepForward();
      alignToLeftWall();
      return;
    }

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

    Serial.println("Emergency recovery");
    moveBackward(0.03);
    pauseAfterAction();
    correctRightAndAdvance(20);
    return;
  }

  if (leftAlmostLost(leftDist)) {
    Serial.println("Decision: LEFT ALMOST LOST -> reacquire first");
    reacquireLeftWall();
    alignToLeftWall();
    return;
  }

  if (leftTooFar(leftDist)) {
    Serial.println("Decision: LEFT TOO FAR -> align left");
    alignToLeftWall();
    return;
  }

  Serial.println("Decision: FORWARD");
  safeStepForward();
  alignToLeftWall();
}
