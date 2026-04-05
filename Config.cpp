#include "Config.h"

const int LEFT_FORWARD = 3;
const int LEFT_REVERSE = 5;
const int RIGHT_FORWARD = 6;
const int RIGHT_REVERSE = 9;

const int LEFT_SPEED_FORWARD  = 235;
const int RIGHT_SPEED_FORWARD = 255;
const int LEFT_SPEED_BACKWARD = 247;
const int RIGHT_SPEED_BACKWARD = 255;

const int SERVO_PIN = 10;

const int openUs  = 1650;
const int closeUs = 1000;

const int trigPin = 12;
const int echoPin = 13;

const int leftTrigPin = 11;
const int leftEchoPin = 8;

int servoTargetUs = 1650;

unsigned long lastServoPulseMicros = 0;
const unsigned long servoFrameUs = 20000;

// ===== movement tuning =====
const float STEP_METERS = 0.05;
const float MICRO_STEP  = 0.015;
const int ACTION_PAUSE_MS = 10;

// Calibration constants
const float MOVE_MS_PER_METER = 3500.0;
const int TURN_MS_PER_DEGREE = 6;

// Wall-following parameters
const float FRONT_LIMIT = 12.0;
const float LEFT_TARGET = 8.0;
const float LEFT_TOL    = 1.0;
const float LEFT_LOST   = 20.0;

// Anti-crash parameters
const float LEFT_TOO_CLOSE = 3.8;
const float LEFT_CRITICAL  = 2.8;

// Turn tuning
const int LEFT_TURN_DEG   = 75;
const int RIGHT_TURN_DEG  = 65;
const int UTURN_DEG       = 150;

const int buzzerPin = 7;

const uint8_t NEOPIXEL_DATA_PIN = 4;
const uint8_t NEOPIXEL_COUNT = 4;
const uint8_t NEOPIXEL_UNUSED_DO_PIN = 2;