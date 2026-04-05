#include "Movement.h"
#include "Config.h"
#include "ServoControl.h"
#include "Sensors.h"
#include "NeoPixelStatus.h"

// Immediately disables all motor control signals.
// This function is the basic safety stop used throughout the motion subsystem.
void stopMotors() {
  digitalWrite(LEFT_FORWARD, LOW);
  digitalWrite(LEFT_REVERSE, LOW);
  digitalWrite(RIGHT_FORWARD, LOW);
  digitalWrite(RIGHT_REVERSE, LOW);
}

// Stops the robot and inserts a short stabilization pause after an action.
// The pause helps prevent command overlap between consecutive movements.
void pauseAfterAction() {
  stopMotors();
  waitWithServo(ACTION_PAUSE_MS);
}

// Moves the robot forward for a distance estimated from time calibration.
// During motion, the servo subsystem continues to update in the background.
void moveForward(float meters) {
  setRobotLightsMoving();

  analogWrite(LEFT_FORWARD, LEFT_SPEED_FORWARD);
  digitalWrite(LEFT_REVERSE, LOW);

  analogWrite(RIGHT_FORWARD, RIGHT_SPEED_FORWARD);
  digitalWrite(RIGHT_REVERSE, LOW);

  int ms = (int)(meters * MOVE_MS_PER_METER);

  unsigned long endTime = millis() + ms;
  while (millis() < endTime) {
    refreshServo();
  }

  stopMotors();
}

// Moves the robot backward for a calibrated time corresponding to the requested distance.
// Servo refresh is preserved during the movement interval.
void moveBackward(float meters) {
  setRobotLightsMoving();

  digitalWrite(LEFT_FORWARD, LOW);
  analogWrite(LEFT_REVERSE, LEFT_SPEED_BACKWARD);

  digitalWrite(RIGHT_FORWARD, LOW);
  analogWrite(RIGHT_REVERSE, RIGHT_SPEED_BACKWARD);

  int ms = (int)(meters * MOVE_MS_PER_METER);

  unsigned long endTime = millis() + ms;
  while (millis() < endTime) {
    refreshServo();
  }

  stopMotors();
}

// Performs a short forward movement using the same timed-motion model as moveForward().
// In practice, this is used for fine positioning and small corrective advances.
void microForward(float meters) {
  setRobotLightsMoving();

  analogWrite(LEFT_FORWARD, LEFT_SPEED_FORWARD);
  digitalWrite(LEFT_REVERSE, LOW);

  analogWrite(RIGHT_FORWARD, RIGHT_SPEED_FORWARD);
  digitalWrite(RIGHT_REVERSE, LOW);

  int ms = (int)(meters * MOVE_MS_PER_METER);

  unsigned long endTime = millis() + ms;
  while (millis() < endTime) {
    refreshServo();
  }

  stopMotors();
}

// Rotates the robot to the right using differential wheel motion.
// The turn angle is approximated through a calibrated time-per-degree constant.
void turnRight(int degree) {
  setRobotLightsTurningRight();

  analogWrite(LEFT_FORWARD, LEFT_SPEED_FORWARD);
  digitalWrite(LEFT_REVERSE, LOW);

  digitalWrite(RIGHT_FORWARD, LOW);
  analogWrite(RIGHT_REVERSE, RIGHT_SPEED_BACKWARD);

  int ms = degree * TURN_MS_PER_DEGREE;

  unsigned long endTime = millis() + ms;
  while (millis() < endTime) {
    refreshServo();
  }

  stopMotors();
}

// Rotates the robot to the left using differential wheel motion.
// As in right turns, the rotation is time-based rather than encoder-based.
void turnLeft(int degree) {
  setRobotLightsTurningLeft();

  digitalWrite(LEFT_FORWARD, LOW);
  analogWrite(LEFT_REVERSE, LEFT_SPEED_BACKWARD);

  analogWrite(RIGHT_FORWARD, RIGHT_SPEED_FORWARD);
  digitalWrite(RIGHT_REVERSE, LOW);

  int ms = degree * TURN_MS_PER_DEGREE;

  unsigned long endTime = millis() + ms;
  while (millis() < endTime) {
    refreshServo();
  }

  stopMotors();
}

// Applies a right correction and then pauses to stabilize the robot before the next action.
void correctRight(int deg) {
  turnRight(deg);
  pauseAfterAction();
}

// Applies a left correction and then pauses to stabilize the robot before the next action.
void correctLeft(int deg) {
  turnLeft(deg);
  pauseAfterAction();
}

// Turns the robot slightly left and then advances only if the front path remains clear.
// function first checks safety, performs the correction,
// and only then makes a short forward step when no front obstacle is detected.
void correctLeftAndAdvance(int deg) {
  float frontDist = readDistanceCm(trigPin, echoPin);

  Serial.print("correctLeftAndAdvance front = ");
  Serial.println(frontDist);

  if (hasFrontWall(frontDist)) {
    Serial.println("correctLeftAndAdvance: front blocked");
    return;
  }

  turnLeft(deg);
  pauseAfterAction();

  frontDist = readDistanceCm(trigPin, echoPin);
  if (!hasFrontWall(frontDist)) {
    microForward(MICRO_STEP);
    pauseAfterAction();
  }
}

// Turns the robot slightly right and then advances only if the front path remains clear.
// This is the symmetric counterpart of correctLeftAndAdvance().
void correctRightAndAdvance(int deg) {
  float frontDist = readDistanceCm(trigPin, echoPin);

  Serial.print("correctRightAndAdvance front = ");
  Serial.println(frontDist);

  if (hasFrontWall(frontDist)) {
    Serial.println("correctRightAndAdvance: front blocked");
    return;
  }

  turnRight(deg);
  pauseAfterAction();

  frontDist = readDistanceCm(trigPin, echoPin);
  if (!hasFrontWall(frontDist)) {
    microForward(MICRO_STEP);
    pauseAfterAction();
  }
}

// Executes a short forward movement followed by a pause.
// This helper provides a compact way to request a small discrete advance.
void smallForward() {
  moveForward(0.05);
  pauseAfterAction();
}