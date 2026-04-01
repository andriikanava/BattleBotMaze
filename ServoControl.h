#ifndef SERVO_CONTROL_H
#define SERVO_CONTROL_H

#include <Arduino.h>

void servoPulse(int pulseUs);
void refreshServo();
void waitWithServo(unsigned long ms);
void grip();

#endif