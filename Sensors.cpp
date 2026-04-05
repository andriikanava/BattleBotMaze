#include "Sensors.h"
#include "Config.h"

// Reads distance from an ultrasonic sensor and returns the average
// of several valid measurements in centimeters.
// Invalid, out-of-range, or missing readings are ignored.
// If no valid measurement is obtained, the function returns -1.0.
float readDistanceCm(int trig, int echo) {
  float sum = 0.0;
  int count = 0;

  for (int i = 0; i < 3; i++) {
    digitalWrite(trig, LOW);
    delayMicroseconds(2);
    digitalWrite(trig, HIGH);
    delayMicroseconds(10);
    digitalWrite(trig, LOW);

    unsigned long d = pulseIn(echo, HIGH, 25000);

    if (d > 0) {
      float dist = d * 0.0343 / 2.0;

      // Accept only physically meaningful distances inside the working range.
      if (dist > 1.5 && dist < 200.0) {
        sum += dist;
        count++;
      }
    }

    delay(5);
  }

  if (count == 0) {
    return -1.0;
  }

  return sum / count;
}

// Returns true when an obstacle is detected in front of the robot
// within the configured front safety threshold.
bool hasFrontWall(float dist) {
  return (dist > 0 && dist < FRONT_LIMIT);
}

// Returns true when the left wall is no longer reliably detected.
// This includes both invalid readings and distances beyond the allowed limit.
bool leftWallLost(float dist) {
  return (dist < 0 || dist > LEFT_LOST);
}

// Returns true when the robot is closer to the left wall than desired.
bool leftTooClose(float dist) {
  return (dist > 0 && dist < LEFT_TOO_CLOSE);
}

// Returns true when the robot is dangerously close to the left wall
// and immediate corrective action may be required.
bool leftCritical(float dist) {
  return (dist > 0 && dist < LEFT_CRITICAL);
}

// Returns true when the robot is farther from the left wall
// than the target distance plus tolerance, but the wall is still detectable.
bool leftTooFar(float dist) {
  return (dist > LEFT_TARGET + LEFT_TOL && dist <= LEFT_LOST);
}

// Returns true when the left wall is still visible but is close
// to being considered lost, indicating that wall reacquisition may be needed.
bool leftAlmostLost(float dist) {
  return (dist > LEFT_LOST - 2.0 && dist <= LEFT_LOST);
}