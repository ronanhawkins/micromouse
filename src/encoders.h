#pragma once
// Quadrature wheel encoders on the ESP32-C6 PCNT peripheral (x4 decoding).
// Counts accumulate in hardware; forward rotation counts up once the
// direction flags are right (see polarity.h).

#include <stdint.h>

bool encodersBegin();
int32_t encoderLeft();
int32_t encoderRight();
