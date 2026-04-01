#ifndef SENSORS_H
#define SENSORS_H

#include <Arduino.h>

float readDistanceCm(int trig, int echo);

bool hasFrontWall(float dist);
bool leftWallLost(float dist);
bool leftTooClose(float dist);
bool leftCritical(float dist);
bool leftTooFar(float dist);
bool leftAlmostLost(float dist);

#endif
