#pragma once
// Minimal MPU-6050 driver: only the Z gyro is used (heading during turns).

#include <stdint.h>

bool imuBegin();
// Average the gyro at rest to find its bias. The mouse must be still.
void imuCalibrate(int samples = 500);
// Z rate in deg/s, bias removed, positive = turning left (CCW from above).
// Takes the I2C lock itself.
float imuReadZ();

// Number of failed gyro reads since boot (I2C noise indicator).
uint32_t imuReadErrors();
