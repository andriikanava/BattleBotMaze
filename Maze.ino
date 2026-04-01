#include "Config.h"
#include "ServoControl.h"
#include "Movement.h"
#include "Sensors.h"
#include "MazeLogic.h"
#include "LineFollower.h"
#include "Buzzer.h"

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

    while (true) {
      playVictoryMelody(buzzerPin);
    }
  }
}