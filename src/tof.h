#pragma once
// Three VL53L0X ToF sensors (left, front, right) on one I2C bus. They all
// power up at 0x29, so each boot holds them in reset with XSHUT and brings
// them up one at a time with a new address. A background task polls them
// without blocking and keeps the latest reading of each.

#include <stdint.h>

enum TofId { TOF_LEFT = 0, TOF_FRONT = 1, TOF_RIGHT = 2 };

bool tofBegin();  // returns false if any sensor failed; see tofOk()
bool tofOk(TofId id);

// Latest distance in mm (9999 = nothing in range / no data yet).
float tofMm(TofId id);
// Incremented every time a new reading arrives for that sensor.
uint32_t tofSeq(TofId id);

// Average the next `n` fresh readings of all three sensors (blocks ~n/50 s).
void tofAverage(int n, float out[3]);
