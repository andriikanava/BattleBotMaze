#include "Config.h"
#include "ServoControl.h"
#include "Movement.h"
#include "Sensors.h"
#include "MazeLogic.h"
#include "LineFollower.h"
#include "Buzzer.h"
#include "NeoPixelStatus.h"

const uint8_t NEOPIXEL_DATA_PIN = 4;
const uint8_t NEOPIXEL_COUNT = 4;
const uint8_t NEOPIXEL_UNUSED_DO_PIN = 2;

void setup() {
  Serial.begin(9600);
  Serial.println("Started");

  pinMode(LEFT_FORWARD, OUTPUT);
  pinMode(LEFT_REVERSE, OUTPUT);
  pinMode(RIGHT_FORWARD, OUTPUT);
  pinMode(RIGHT_REVERSE, OUTPUT);
  stopMotors();

  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  pinMode(leftTrigPin, OUTPUT);
  pinMode(leftEchoPin, INPUT);

  pinMode(SERVO_PIN, OUTPUT);

  pinMode(buzzerPin, OUTPUT);
  pinMode(NEOPIXEL_UNUSED_DO_PIN, INPUT);

  initRobotLights(NEOPIXEL_DATA_PIN, NEOPIXEL_COUNT);
  setRobotLightsWaiting();

  servoTargetUs = openUs;

  lineFollowerInit();

  waitForTurn();
  followLineUntilLost();
}

void loop() {
  refreshServo();
  followLeftWallStep(); 

  if (waitForLine())
  {
    followLineUntilIntersection(8);
    stopMotors();

    setRobotLightsFinished();

    while (true) {
      updateRobotLights();
      playVictoryMelody(buzzerPin);
    }
  }
}