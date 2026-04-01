#include "Movement.h"
#include "Config.h"
#include "ServoControl.h"
#include "Sensors.h"

void stopMotors() {
  digitalWrite(LEFT_FORWARD, LOW);
  digitalWrite(LEFT_REVERSE, LOW);
  digitalWrite(RIGHT_FORWARD, LOW);
  digitalWrite(RIGHT_REVERSE, LOW);
}

void pauseAfterAction() {
  stopMotors();
  waitWithServo(ACTION_PAUSE_MS);
}

void moveForward(float meters) {
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

void moveBackward(float meters) {
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

void microForward(float meters) {
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

void turnRight(int degree) {
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

void turnLeft(int degree) {
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

void correctRight(int deg) {
  turnRight(deg);
  pauseAfterAction();
}

void correctLeft(int deg) {
  turnLeft(deg);
  pauseAfterAction();
}

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

void smallForward() {
  moveForward(0.05);
  pauseAfterAction();
}
