#pragma once
// Minimal MPU-6050 driver: only the Z gyro is used (heading during turns).

bool imuBegin();
// Average the gyro at rest to find its bias. The mouse must be still.
void imuCalibrate(int samples = 500);
// Z rate in deg/s, bias removed, positive = turning left (CCW from above).
// Takes the I2C lock itself.
float imuReadZ();
