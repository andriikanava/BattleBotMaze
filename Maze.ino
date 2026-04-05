#include "Config.h"
#include "ServoControl.h"
#include "Movement.h"
#include "Sensors.h"
#include "MazeLogic.h"
#include "LineFollower.h"
#include "Buzzer.h"
#include "NeoPixelStatus.h"

// NeoPixel configuration parameters.
const uint8_t NEOPIXEL_DATA_PIN = 4;
const uint8_t NEOPIXEL_COUNT = 4;
const uint8_t NEOPIXEL_UNUSED_DO_PIN = 2;

void setup() {
  // Initialize communication and hardware interfaces.
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

  // Initialize control subsystems.
  servoTargetUs = openUs;
  lineFollowerInit();

  // Execute initial positioning and line acquisition.
  waitForTurn();
  followLineUntilLost();
}

void loop() {
  // Main control loop: wall following with continuous servo update.
  refreshServo();
  followLeftWallStep(); 

  // Detect finish condition and execute termination routine.
  if (waitForLine())
  {
    followLineUntilIntersection(8);
    stopMotors();

    setRobotLightsFinished();

    // Persistent completion signaling.
    while (true) {
      updateRobotLights();
      playVictoryMelody(buzzerPin);
    }
  }
}