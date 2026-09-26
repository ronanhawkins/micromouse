#include "i2c_bus.h"

#include <Arduino.h>

#include "config.h"

namespace {
SemaphoreHandle_t mutex;
}

void i2cBegin() {
  mutex = xSemaphoreCreateMutex();
  Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL, I2C_HZ);
  Wire.setTimeOut(10);
}

void i2cLock() { xSemaphoreTake(mutex, portMAX_DELAY); }
void i2cUnlock() { xSemaphoreGive(mutex); }

void i2cScan() {
  I2CGuard g;
  int found = 0;
  for (uint8_t a = 1; a < 127; a++) {
    Wire.beginTransmission(a);
    if (Wire.endTransmission() == 0) {
      Serial.printf("  0x%02X", a);
      if (a == IMU_ADDR) Serial.print("  MPU-6050");
      if (a == 0x29) Serial.print("  VL53L0X (default address, not yet moved)");
      if (a == TOF_ADDR_L) Serial.print("  ToF left");
      if (a == TOF_ADDR_F) Serial.print("  ToF front");
      if (a == TOF_ADDR_R) Serial.print("  ToF right");
      Serial.println();
      found++;
    }
  }
  if (!found) Serial.println("  nothing found: check wiring, or swap PIN_I2C_SDA/SCL in config.h");
}
