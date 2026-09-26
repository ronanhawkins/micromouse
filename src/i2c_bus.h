#pragma once
// One I2C bus shared by the gyro (read every control tick) and the three
// ToF sensors (polled by their own task). Hold the lock for every complete
// transaction sequence.

#include <Wire.h>

void i2cBegin();
void i2cLock();
void i2cUnlock();

struct I2CGuard {
  I2CGuard() { i2cLock(); }
  ~I2CGuard() { i2cUnlock(); }
};

// Print every responding address to Serial.
void i2cScan();
