#ifndef MOVEMENT_H
#define MOVEMENT_H

#include <Arduino.h>

void stopMotors();
void pauseAfterAction();

void moveForward(float meters);
void moveBackward(float meters);
void microForward(float meters);

void turnRight(int degree);
void turnLeft(int degree);

void correctRight(int deg);
void correctLeft(int deg);

void correctLeftAndAdvance(int deg);
void correctRightAndAdvance(int deg);

void smallForward();

#endif
