#include "ui.h"

#include <Arduino.h>

#include "config.h"
#include "tof.h"

namespace {

UiColour current = UiColour::OFF;

void show(UiColour c) {
  uint8_t r = 0, g = 0, b = 0;
  switch (c) {
    case UiColour::OFF: break;
    case UiColour::IDLE: b = 20; break;
    case UiColour::WAITING: r = 20; g = 12; break;
    case UiColour::SEARCH: g = 10; b = 20; break;
    case UiColour::RUN: r = 20; g = 0; b = 20; break;
    case UiColour::DONE: g = 25; break;
    case UiColour::ERROR: r = 30; break;
  }
  rgbLedWrite(PIN_STATUS_LED, r, g, b);
}

}  // namespace

void uiBegin() {
  pinMode(PIN_BUTTON, INPUT_PULLUP);
  uiSet(UiColour::IDLE);
}

void uiSet(UiColour c) {
  current = c;
  show(c);
}

void uiBlink() {
  rgbLedWrite(PIN_STATUS_LED, 20, 20, 20);
  delay(15);
  show(current);
}

bool uiButtonDown() { return digitalRead(PIN_BUTTON) == LOW; }

unsigned long uiWaitPress() {
  while (!uiButtonDown()) delay(5);
  unsigned long t0 = millis();
  while (uiButtonDown()) delay(5);
  delay(30);  // debounce
  return millis() - t0;
}

void uiWaitStart() {
  UiColour before = current;
  uiSet(UiColour::WAITING);
  // Hand (or anything) close in front, or the button.
  for (;;) {
    if (uiButtonDown()) {
      while (uiButtonDown()) delay(5);
      break;
    }
    if (tofMm(TOF_FRONT) < 40) {
      while (tofMm(TOF_FRONT) < 80) delay(5);
      break;
    }
    delay(5);
  }
  uiSet(before);
  delay(1000);  // hands off
}
