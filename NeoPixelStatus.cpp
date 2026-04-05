#include "NeoPixelStatus.h"
#include <Adafruit_NeoPixel.h>

namespace {
  // Internal NeoPixel strip instance and state flags.
  // stripReady ensures safe usage only after initialization.
  Adafruit_NeoPixel strip;
  bool stripReady = false;
  uint8_t totalPixels = 0;

  // Defines high-level lighting modes corresponding to robot states.
  enum LightState {
    LIGHT_WAITING,
    LIGHT_MOVING,
    LIGHT_TURNING,
    LIGHT_FINISHED
  };

  LightState currentState = LIGHT_WAITING;

  // Variables used for non-blocking flashing animation in FINISHED state.
  unsigned long lastFlashMs = 0;
  uint8_t flashStep = 0;

  const uint16_t FLASH_INTERVAL_MS = 180;
  const uint32_t TURN_YELLOW = Adafruit_NeoPixel::Color(255, 140, 0);

  // Fixed logical mapping of LEDs to robot orientation.
  // This allows directional signaling (left/right turns).
  const uint8_t FRONT_LEFT_INDEX = 0;
  const uint8_t FRONT_RIGHT_INDEX = 1;
  const uint8_t BACK_RIGHT_INDEX = 2;
  const uint8_t BACK_LEFT_INDEX = 3;

  // Sets all pixels to the same color.
  // Used as the base operation for most lighting states.
  void fillAll(uint32_t color) {
    if (!stripReady) {
      return;
    }

    for (uint8_t i = 0; i < totalPixels; i++) {
      strip.setPixelColor(i, color);
    }
    strip.show();
  }

  // Turns off all LEDs.
  void clearAll() {
    fillAll(strip.Color(0, 0, 0));
  }

  // Displays turn indication on one side of the robot.
  // If there are fewer than 4 LEDs, fallback to full strip indication.
  void setTurningSide(bool isLeftTurn) {
    if (!stripReady) {
      return;
    }

    if (totalPixels < 4) {
      fillAll(TURN_YELLOW);
      return;
    }

    clearAll();

    if (isLeftTurn) {
      strip.setPixelColor(FRONT_LEFT_INDEX, TURN_YELLOW);
      strip.setPixelColor(BACK_LEFT_INDEX, TURN_YELLOW);
    } else {
      strip.setPixelColor(FRONT_RIGHT_INDEX, TURN_YELLOW);
      strip.setPixelColor(BACK_RIGHT_INDEX, TURN_YELLOW);
    }

    strip.show();
  }
}

// Initializes the NeoPixel strip and sets the initial state.
// Must be called before any other lighting function.
void initRobotLights(uint8_t dataPin, uint8_t pixelCount) {
  totalPixels = pixelCount;
  strip = Adafruit_NeoPixel(totalPixels, dataPin, NEO_GRB + NEO_KHZ800);
  strip.begin();
  strip.show();
  stripReady = true;

  setRobotLightsWaiting();
}

// Sets the robot to waiting state (e.g., before start).
// Uses solid red to indicate idle/standby.
void setRobotLightsWaiting() {
  currentState = LIGHT_WAITING;
  fillAll(strip.Color(255, 0, 0));
}

// Sets the robot to moving state.
// Uses white light to indicate active motion.
void setRobotLightsMoving() {
  if (currentState == LIGHT_FINISHED) {
    return;
  }

  currentState = LIGHT_MOVING;
  fillAll(strip.Color(255, 255, 255));
}

// Activates left turn indication.
// Only updates if the robot is not in finished state.
void setRobotLightsTurningLeft() {
  if (currentState == LIGHT_FINISHED) {
    return;
  }

  currentState = LIGHT_TURNING;
  setTurningSide(true);
}

// Activates right turn indication.
// Only updates if the robot is not in finished state.
void setRobotLightsTurningRight() {
  if (currentState == LIGHT_FINISHED) {
    return;
  }

  currentState = LIGHT_TURNING;
  setTurningSide(false);
}

// Switches to finished state and resets flashing animation.
// From this point, other lighting updates are ignored.
void setRobotLightsFinished() {
  currentState = LIGHT_FINISHED;
  flashStep = 0;
  lastFlashMs = 0;
}

// Updates lighting in finished state.
// Implements a non-blocking RGB cycling effect.
void updateRobotLights() {
  if (!stripReady) {
    return;
  }

  if (currentState != LIGHT_FINISHED) {
    return;
  }

  unsigned long now = millis();

  // Ensures fixed timing between color changes.
  if (lastFlashMs != 0 && (now - lastFlashMs) < FLASH_INTERVAL_MS) {
    return;
  }

  lastFlashMs = now;

  uint32_t color;
  if (flashStep == 0) {
    color = strip.Color(255, 0, 0);
  } else if (flashStep == 1) {
    color = strip.Color(0, 255, 0);
  } else {
    color = strip.Color(0, 0, 255);
  }

  fillAll(color);
  flashStep = (flashStep + 1) % 3;
}