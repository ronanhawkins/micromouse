#include "ui.h"

#include <Arduino.h>

#include "config.h"
#include "tof.h"

namespace {

volatile UiColour current = UiColour::OFF;
volatile bool flashRequested = false;

struct Rgb {
  uint8_t r, g, b;
};

Rgb colourOf(UiColour c) {
  switch (c) {
    case UiColour::IDLE: return {0, 0, 20};
    case UiColour::WAITING: return {20, 12, 0};
    case UiColour::HANDS_OFF: return {40, 0, 0};
    case UiColour::SEARCH: return {0, 10, 20};
    case UiColour::RUN: return {20, 0, 20};
    case UiColour::DONE: return {0, 25, 0};
    case UiColour::ERROR: return {30, 0, 0};
    case UiColour::HELD_LONG: return {25, 25, 25};
    default: return {0, 0, 0};
  }
}

// Blink period in ms, 0 = steady.
int blinkPeriod(UiColour c) {
  if (c == UiColour::WAITING) return 600;
  if (c == UiColour::HANDS_OFF) return 150;
  return 0;
}

// The only place the LED is written, so callers never block on it.
void ledTask(void*) {
  Rgb shown{255, 255, 255};
  for (;;) {
    Rgb want = colourOf(current);
    int period = blinkPeriod(current);
    if (period && (millis() % period) >= unsigned(period / 2)) want = {0, 0, 0};
    if (flashRequested) {
      flashRequested = false;
      rgbLedWrite(PIN_STATUS_LED, 25, 25, 25);
      vTaskDelay(pdMS_TO_TICKS(40));
      shown = {255, 255, 255};  // force a rewrite
    }
    if (want.r != shown.r || want.g != shown.g || want.b != shown.b) {
      rgbLedWrite(PIN_STATUS_LED, want.r, want.g, want.b);
      shown = want;
    }
    vTaskDelay(pdMS_TO_TICKS(20));
  }
}

}  // namespace

void uiBegin() {
  pinMode(PIN_BUTTON, INPUT_PULLUP);
  current = UiColour::IDLE;
  xTaskCreate(ledTask, "led", 2048, nullptr, 2, nullptr);
}

void uiSet(UiColour c) { current = c; }

void uiBlink() { flashRequested = true; }

bool uiButtonDown() { return digitalRead(PIN_BUTTON) == LOW; }

unsigned long uiWaitPress() {
  while (!uiButtonDown()) delay(5);
  unsigned long t0 = millis();
  UiColour before = current;
  while (uiButtonDown()) {
    if (millis() - t0 > UI_LONG_PRESS_MS) uiSet(UiColour::HELD_LONG);
    delay(5);
  }
  uiSet(before);
  delay(30);  // debounce
  return millis() - t0;
}

void uiWaitStart() {
  uiSet(UiColour::WAITING);
  // Hand (or anything) close in front, or the button.
  for (;;) {
    if (uiButtonDown()) {
      uiSet(UiColour::HANDS_OFF);
      while (uiButtonDown()) delay(5);
      break;
    }
    if (tofMm(TOF_FRONT) < 40) {
      uiSet(UiColour::HANDS_OFF);
      while (tofMm(TOF_FRONT) < 80) delay(5);
      break;
    }
    delay(5);
  }
  delay(1000);  // hands off
}
