#include "imu.h"

#include <Arduino.h>

#include "config.h"
#include "i2c_bus.h"

namespace {

constexpr uint8_t REG_SMPLRT_DIV = 0x19;
constexpr uint8_t REG_CONFIG = 0x1A;
constexpr uint8_t REG_GYRO_CONFIG = 0x1B;
constexpr uint8_t REG_GYRO_ZOUT_H = 0x47;
constexpr uint8_t REG_PWR_MGMT_1 = 0x6B;
constexpr uint8_t REG_WHO_AM_I = 0x75;
constexpr float LSB_PER_DPS = 32.8f;  // +-1000 deg/s range

float bias = 0;
bool present = false;  // skip bus traffic (and its timeouts) if the IMU is missing

void writeReg(uint8_t reg, uint8_t v) {
  Wire.beginTransmission(IMU_ADDR);
  Wire.write(reg);
  Wire.write(v);
  Wire.endTransmission();
}

int16_t readRawZ() {
  Wire.beginTransmission(IMU_ADDR);
  Wire.write(REG_GYRO_ZOUT_H);
  if (Wire.endTransmission(false) != 0) return 0;
  if (Wire.requestFrom(IMU_ADDR, uint8_t(2)) != 2) return 0;
  uint8_t hi = Wire.read();
  uint8_t lo = Wire.read();
  return int16_t((hi << 8) | lo);
}

}  // namespace

bool imuBegin() {
  I2CGuard g;
  Wire.beginTransmission(IMU_ADDR);
  Wire.write(REG_WHO_AM_I);
  if (Wire.endTransmission(false) != 0 || Wire.requestFrom(IMU_ADDR, uint8_t(1)) != 1) return false;
  uint8_t who = Wire.read();
  // Genuine parts answer 0x68; common clones answer 0x70/0x72/0x98.
  if (who != 0x68 && who != 0x70 && who != 0x72 && who != 0x98) return false;

  writeReg(REG_PWR_MGMT_1, 0x01);   // wake, clock from gyro X PLL
  delay(10);
  writeReg(REG_CONFIG, 0x02);       // DLPF ~94 Hz
  writeReg(REG_SMPLRT_DIV, 0x00);   // 1 kHz sample rate
  writeReg(REG_GYRO_CONFIG, 0x10);  // +-1000 deg/s
  present = true;
  return true;
}

void imuCalibrate(int samples) {
  if (!present) return;
  float sum = 0;
  for (int i = 0; i < samples; i++) {
    {
      I2CGuard g;
      sum += readRawZ();
    }
    delay(2);
  }
  bias = sum / samples;
}

float imuReadZ() {
  if (!present) return 0;
  int16_t raw;
  {
    I2CGuard g;
    raw = readRawZ();
  }
  float dps = (raw - bias) / LSB_PER_DPS;
  return INVERT_GYRO ? -dps : dps;
}
