#pragma once
// Direction flags for the encoders, motors and gyro. They start as the
// INVERT_* defaults in config.h, and the `check` command measures them and
// saves them to flash, overriding those defaults from then on.

struct Polarity {
  bool encL;
  bool encR;
  bool motL;
  bool motR;
  bool gyro;
};

extern Polarity polarity;

void polarityLoad();  // saved values if any, else config.h defaults
void polaritySave();
