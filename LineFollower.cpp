#include "LineFollower.h"
#include "Config.h"
#include "Movement.h"

// ===== line sensor config =====
static const int sensorPins[8] = {A0, A1, A2, A3, A4, A5, A6, A7};
static const int weights[8]    = {-350, -250, -150, -50, 50, 150, 250, 350};

static float Kp = 3.0;
static int threshold = 800;
static int lastError = 0;
static const float speedMul = 1.0;

// внутренний helper
static int readLineValue(int pin) {
  return analogRead(pin);
}

void lineFollowerInit() {
  for (int i = 0; i < 8; i++) {
    pinMode(sensorPins[i], INPUT);
  }
}

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

bool isLineDetected() {
  return countActiveLineSensors() > 0;
}

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

void followLineForMs(unsigned long durationMs) {
  unsigned long endTime = millis() + durationMs;

  while (millis() < endTime) {
    lineFollowStep();
  }

  stopMotors();
}

void followLineUntilLost() {
  int lostCount = 0;

  while (true) {
    int error = readLinePosition();

    if (error == 9999) {
      lostCount++;
      searchLine();

      // линия реально пропала, а не просто мигнула
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

bool waitForLine(unsigned long timeoutMs = 2000) {
  return readLinePosition() != 9999;
}
