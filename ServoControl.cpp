#include "ServoControl.h"
#include "Config.h"

void servoPulse(int pulseUs) {
  digitalWrite(SERVO_PIN, HIGH);
  delayMicroseconds(pulseUs);
  digitalWrite(SERVO_PIN, LOW);
}

void refreshServo() {
  unsigned long now = micros();
  if (now - lastServoPulseMicros >= servoFrameUs) {
    lastServoPulseMicros = now;
    servoPulse(servoTargetUs);
  }
}

void waitWithServo(unsigned long ms) {
  unsigned long endTime = millis() + ms;
  while (millis() < endTime) {
    refreshServo();
  }
}

void grip() {
  servoTargetUs = closeUs;
}
