#include "i2c_bus.h"

#include <Arduino.h>

#include "config.h"

namespace {

SemaphoreHandle_t mutex;
int failsInARow = 0;
uint32_t recoveries = 0;
constexpr int FAILS_BEFORE_RESET = 3;

void startBus() {
  Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL, I2C_HZ);
  Wire.setTimeOut(10);
}

// Tear the driver down, clock out any byte a sensor is still sending (it
// may be holding SDA low), send a STOP, and start again.
void resetBus() {
  Wire.end();
  pinMode(PIN_I2C_SDA, INPUT_PULLUP);
  pinMode(PIN_I2C_SCL, OUTPUT_OPEN_DRAIN);
  for (int i = 0; i < 9 && digitalRead(PIN_I2C_SDA) == LOW; i++) {
    digitalWrite(PIN_I2C_SCL, LOW);
    delayMicroseconds(10);
    digitalWrite(PIN_I2C_SCL, HIGH);
    delayMicroseconds(10);
  }
  pinMode(PIN_I2C_SDA, OUTPUT_OPEN_DRAIN);
  digitalWrite(PIN_I2C_SDA, LOW);
  delayMicroseconds(10);
  digitalWrite(PIN_I2C_SCL, HIGH);
  delayMicroseconds(10);
  digitalWrite(PIN_I2C_SDA, HIGH);  // STOP
  delayMicroseconds(10);
  startBus();
  recoveries++;
}

}  // namespace

void i2cBegin() {
  mutex = xSemaphoreCreateMutex();
  startBus();
}

void i2cResult(bool ok) {
  if (ok) {
    failsInARow = 0;
    return;
  }
  if (++failsInARow >= FAILS_BEFORE_RESET) {
    failsInARow = 0;
    resetBus();
  }
}

uint32_t i2cRecoveries() { return recoveries; }

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
