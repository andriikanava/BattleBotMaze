#ifndef NEOPIXEL_STATUS_H
#define NEOPIXEL_STATUS_H

#include <Arduino.h>

void initRobotLights(uint8_t dataPin, uint8_t pixelCount);

void setRobotLightsWaiting();
void setRobotLightsMoving();
void setRobotLightsTurningLeft();
void setRobotLightsTurningRight();
void setRobotLightsFinished();

void updateRobotLights();

#endif