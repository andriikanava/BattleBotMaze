#include "NeoPixelStatus.h"
#include <Adafruit_NeoPixel.h>

namespace {
  Adafruit_NeoPixel strip;
  bool stripReady = false;
  uint8_t totalPixels = 0;

  enum LightState {
    LIGHT_WAITING,
    LIGHT_MOVING,
    LIGHT_TURNING,
    LIGHT_FINISHED
  };

  LightState currentState = LIGHT_WAITING;

  unsigned long lastFlashMs = 0;
  uint8_t flashStep = 0;

  const uint16_t FLASH_INTERVAL_MS = 180;
  const uint32_t TURN_YELLOW = Adafruit_NeoPixel::Color(255, 140, 0);

  // Expected order: 0=front-left, 1=front-right, 2=back-right, 3=back-left.
  const uint8_t FRONT_LEFT_INDEX = 0;
  const uint8_t FRONT_RIGHT_INDEX = 1;
  const uint8_t BACK_RIGHT_INDEX = 2;
  const uint8_t BACK_LEFT_INDEX = 3;

  void fillAll(uint32_t color) {
    if (!stripReady) {
      return;
    }

    for (uint8_t i = 0; i < totalPixels; i++) {
      strip.setPixelColor(i, color);
    }
    strip.show();
  }

  void clearAll() {
    fillAll(strip.Color(0, 0, 0));
  }

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

void initRobotLights(uint8_t dataPin, uint8_t pixelCount) {
  totalPixels = pixelCount;
  strip = Adafruit_NeoPixel(totalPixels, dataPin, NEO_GRB + NEO_KHZ800);
  strip.begin();
  strip.show();
  stripReady = true;

  setRobotLightsWaiting();
}

void setRobotLightsWaiting() {
  currentState = LIGHT_WAITING;
  fillAll(strip.Color(255, 0, 0));
}

void setRobotLightsMoving() {
  if (currentState == LIGHT_FINISHED) {
    return;
  }

  currentState = LIGHT_MOVING;
  fillAll(strip.Color(255, 255, 255));
}

void setRobotLightsTurningLeft() {
  if (currentState == LIGHT_FINISHED) {
    return;
  }

  currentState = LIGHT_TURNING;
  setTurningSide(true);
}

void setRobotLightsTurningRight() {
  if (currentState == LIGHT_FINISHED) {
    return;
  }

  currentState = LIGHT_TURNING;
  setTurningSide(false);
}

void setRobotLightsFinished() {
  currentState = LIGHT_FINISHED;
  flashStep = 0;
  lastFlashMs = 0;
}

void updateRobotLights() {
  if (!stripReady) {
    return;
  }

  if (currentState != LIGHT_FINISHED) {
    return;
  }

  unsigned long now = millis();
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