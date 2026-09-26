#include "polarity.h"

#include <Preferences.h>

#include "config.h"

Polarity polarity{INVERT_ENC_L, INVERT_ENC_R, INVERT_MOTOR_L, INVERT_MOTOR_R, INVERT_GYRO};

namespace {
constexpr const char* NS = "mouse";
constexpr const char* KEY = "polarity";
}

void polarityLoad() {
  Preferences p;
  p.begin(NS, false);
  if (p.isKey(KEY) && p.getBytesLength(KEY) == sizeof polarity) p.getBytes(KEY, &polarity, sizeof polarity);
  p.end();
}

void polaritySave() {
  Preferences p;
  p.begin(NS, false);
  p.putBytes(KEY, &polarity, sizeof polarity);
  p.end();
}
