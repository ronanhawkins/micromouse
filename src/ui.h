#pragma once
// Onboard RGB LED and BOOT button.
//
// LED:
//   IDLE      blue, steady            ready for a command / button press
//   WAITING   amber, slow blink       waiting for the start signal
//   HANDS_OFF red, fast flash         start seen: keep still (gyro calibrating)
//   SEARCH    cyan, steady            searching (white blip per new cell)
//   RUN       magenta, steady         speed run
//   DONE      green, steady           finished OK
//   ERROR     red, steady             failed / aborted
//   HELD_LONG white, steady           BOOT held > 1.5 s: releasing clears the map

enum class UiColour { OFF, IDLE, WAITING, HANDS_OFF, SEARCH, RUN, DONE, ERROR, HELD_LONG };

constexpr unsigned long UI_LONG_PRESS_MS = 1500;

void uiBegin();
void uiSet(UiColour c);
void uiBlink();  // brief white flash (new cell mapped, button press seen)

bool uiButtonDown();
// Wait for a press; returns its length in ms. Shows HELD_LONG once the
// press passes the long-press time, then restores the previous colour.
unsigned long uiWaitPress();

// Start signal: wave a hand in front of the front sensor (or press BOOT).
// Returns once the hand is removed and the mouse has been still for a
// moment, ready for gyro calibration. Leaves the LED on HANDS_OFF.
void uiWaitStart();
