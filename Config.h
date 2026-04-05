#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>

// Motor pins
extern const int LEFT_FORWARD;
extern const int LEFT_REVERSE;
extern const int RIGHT_FORWARD;
extern const int RIGHT_REVERSE;

// Motor speeds
extern const int LEFT_SPEED_FORWARD;
extern const int RIGHT_SPEED_FORWARD;
extern const int LEFT_SPEED_BACKWARD;
extern const int RIGHT_SPEED_BACKWARD;

// Servo
extern const int SERVO_PIN;
extern const int openUs;
extern const int closeUs;

extern int servoTargetUs;
extern unsigned long lastServoPulseMicros;
extern const unsigned long servoFrameUs;

// Ultrasonic
extern const int trigPin;
extern const int echoPin;
extern const int leftTrigPin;
extern const int leftEchoPin;

extern const int buzzerPin;

// Movement tuning
extern const float STEP_METERS;
extern const float MICRO_STEP;
extern const int ACTION_PAUSE_MS;

// Calibration
extern const float MOVE_MS_PER_METER;
extern const int TURN_MS_PER_DEGREE;

// Wall-following parameters
extern const float FRONT_LIMIT;
extern const float LEFT_TARGET;
extern const float LEFT_TOL;
extern const float LEFT_LOST;

// Anti-crash parameters
extern const float LEFT_TOO_CLOSE;
extern const float LEFT_CRITICAL;

// Turn tuning
extern const int LEFT_TURN_DEG;
extern const int RIGHT_TURN_DEG;
extern const int UTURN_DEG;

extern const uint8_t NEOPIXEL_DATA_PIN;
extern const uint8_t NEOPIXEL_COUNT;
extern const uint8_t NEOPIXEL_UNUSED_DO_PIN;

#endif