#include "Sensors.h"
#include "Config.h"

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

bool hasFrontWall(float dist) {
  return (dist > 0 && dist < FRONT_LIMIT);
}

bool leftWallLost(float dist) {
  return (dist < 0 || dist > LEFT_LOST);
}

bool leftTooClose(float dist) {
  return (dist > 0 && dist < LEFT_TOO_CLOSE);
}

bool leftCritical(float dist) {
  return (dist > 0 && dist < LEFT_CRITICAL);
}

bool leftTooFar(float dist) {
  return (dist > LEFT_TARGET + LEFT_TOL && dist <= LEFT_LOST);
}

bool leftAlmostLost(float dist) {
  return (dist > LEFT_LOST - 2.0 && dist <= LEFT_LOST);
}
