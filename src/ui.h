#pragma once
// Onboard RGB LED and BOOT button.

enum class UiColour { OFF, IDLE, WAITING, SEARCH, RUN, DONE, ERROR };

void uiBegin();
void uiSet(UiColour c);
void uiBlink();  // brief flash, e.g. when a new cell is mapped

bool uiButtonDown();
// Wait for a press; returns its length in ms.
unsigned long uiWaitPress();

// Start signal: wave a hand in front of the front sensor (or press BOOT).
// Returns once the hand is removed and the mouse has been still for a
// moment, ready for gyro calibration.
void uiWaitStart();
