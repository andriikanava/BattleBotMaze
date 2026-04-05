#include "LineFollower.h"
#include "Config.h"
#include "Movement.h"

// Configuration of the eight reflective line sensors and their positional weights.
// Negative weights correspond to the left side of the sensor array,
// and positive weights correspond to the right side.
static const int sensorPins[8] = {A0, A1, A2, A3, A4, A5, A6, A7};
static const int weights[8]    = {-350, -250, -150, -50, 50, 150, 250, 350};

// Proportional control parameters for line tracking.
// Kp defines how strongly the robot reacts to lateral error.
// threshold separates background from the detected line signal.
// lastError stores the previous direction of deviation and is used
// to decide where to search when the line is temporarily lost.
static float Kp = 3.0;
static int threshold = 800;
static int lastError = 0;
static const float speedMul = 1.0;

// Reads the raw analog value from a single line sensor.
static int readLineValue(int pin) {
  return analogRead(pin);
}

// Initializes all line sensor pins as inputs before line tracking begins.
void lineFollowerInit() {
  for (int i = 0; i < 8; i++) {
    pinMode(sensorPins[i], INPUT);
  }
}

// Counts how many sensors currently detect the line above the configured threshold.
// This is used to estimate whether the robot is on a narrow line or at a wider feature
// such as an intersection.
int countActiveLineSensors() {
  int active = 0;

  for (int i = 0; i < 8; i++) {
    int raw = readLineValue(sensorPins[i]);
    int v = raw - threshold;
    if (v > 0) {
      active++;
    }
  }

  return active;
}

// Returns true when at least one sensor detects the line.
bool isLineDetected() {
  return countActiveLineSensors() > 0;
}

// Computes the line position using a weighted average of all active sensors.
// In simple terms, the function estimates whether the line is centered,
// shifted to the left, or shifted to the right under the robot.
// If the total signal is too small, the line is considered lost.
int readLinePosition() {
  long sum = 0;
  long total = 0;

  for (int i = 0; i < 8; i++) {
    int raw = readLineValue(sensorPins[i]);

    int v = raw - threshold;
    if (v < 0) v = 0;

    sum += (long)v * weights[i];
    total += v;
  }

  if (total < 50) return 9999;
  return (int)(sum / total);
}

// Rotates the robot in the direction of the last known line position
// in order to reacquire the track after signal loss.
// If the previous error was positive, the robot searches to one side;
// otherwise, it searches to the opposite side.
void searchLine() {
  int spin = (lastError >= 0) ? 120 : -120;

  int left  = constrain(LEFT_SPEED_FORWARD  - spin, 0, 255);
  int right = constrain(RIGHT_SPEED_FORWARD + spin, 0, 255);

  analogWrite(LEFT_FORWARD, left);
  digitalWrite(LEFT_REVERSE, LOW);

  analogWrite(RIGHT_FORWARD, right);
  digitalWrite(RIGHT_REVERSE, LOW);

  delay(20);
}

// Executes one proportional control step for line following.
// The robot reads the current line position, computes the tracking error,
// and adjusts left and right motor speeds to steer back toward the center.
// If the line is not detected, the robot switches to search mode.
void lineFollowStep() {
  int error = readLinePosition();

  if (error == 9999) {
    searchLine();
    return;
  }

  lastError = error;

  int correction = (int)(error * Kp);

  int leftSpeed  = LEFT_SPEED_FORWARD  - correction;
  int rightSpeed = RIGHT_SPEED_FORWARD + correction;

  leftSpeed  = constrain(leftSpeed, 0, 255);
  rightSpeed = constrain(rightSpeed, 0, 255);

  leftSpeed  = (int)(leftSpeed  * speedMul);
  rightSpeed = (int)(rightSpeed * speedMul);

  analogWrite(LEFT_FORWARD, leftSpeed);
  digitalWrite(LEFT_REVERSE, LOW);

  analogWrite(RIGHT_FORWARD, rightSpeed);
  digitalWrite(RIGHT_REVERSE, LOW);

  delay(10);
}

// Follows the line for a fixed time interval and then stops the motors.
// This is useful for short controlled movements along the track.
void followLineForMs(unsigned long durationMs) {
  unsigned long endTime = millis() + durationMs;

  while (millis() < endTime) {
    lineFollowStep();
  }

  stopMotors();
}

// Continues following the line until the line is considered truly lost.
// A short loss does not stop the robot immediately; instead, several
// consecutive missing readings are required to confirm that the line ended.
void followLineUntilLost() {
  int lostCount = 0;

  while (true) {
    int error = readLinePosition();

    if (error == 9999) {
      lostCount++;
      searchLine();

      // Confirm persistent line loss rather than a single unstable reading.
      if (lostCount >= 8) {
        break;
      }
    } else {
      lostCount = 0;
      lineFollowStep();
    }
  }

  stopMotors();
}

// Follows the line until a sufficient number of sensors detect it simultaneously.
// In simple terms, this is used to recognize a wide marking or an intersection.
void followLineUntilIntersection(int minActiveSensors) {
  while (true) {
    int active = countActiveLineSensors();

    if (active >= minActiveSensors) {
      break;
    }

    lineFollowStep();
  }

  stopMotors();
}

// Checks whether the line is currently visible.
// The timeout parameter is present in the interface, but in the current implementation
// the function performs an immediate check and returns the result without waiting.
bool waitForLine(unsigned long timeoutMs = 2000) {
  return readLinePosition() != 9999;
}