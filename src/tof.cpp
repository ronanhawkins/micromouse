#include "tof.h"

#include <Arduino.h>
#include <VL53L0X.h>

#include "config.h"
#include "i2c_bus.h"

namespace {

constexpr float NO_READING = 9999.0f;

VL53L0X sensors[3];
const int xshut[3] = {PIN_XSHUT_L, PIN_XSHUT_F, PIN_XSHUT_R};
const uint8_t addr[3] = {TOF_ADDR_L, TOF_ADDR_F, TOF_ADDR_R};
bool ok[3];
volatile float mm[3] = {NO_READING, NO_READING, NO_READING};
volatile uint32_t seq[3];

// Non-blocking version of VL53L0X::readRangeContinuousMillimeters().
void poll(int i) {
  VL53L0X& s = sensors[i];
  I2CGuard g;
  if ((s.readReg(VL53L0X::RESULT_INTERRUPT_STATUS) & 0x07) == 0) return;
  uint16_t range = s.readReg16Bit(VL53L0X::RESULT_RANGE_STATUS + 10);
  s.writeReg(VL53L0X::SYSTEM_INTERRUPT_CLEAR, 0x01);
  // ~8190 means "nothing in range".
  mm[i] = range >= 8000 ? NO_READING : float(range);
  seq[i] = seq[i] + 1;
}

void tofTask(void*) {
  TickType_t last = xTaskGetTickCount();
  for (;;) {
    for (int i = 0; i < 3; i++)
      if (ok[i]) poll(i);
    vTaskDelayUntil(&last, pdMS_TO_TICKS(5));
  }
}

}  // namespace

bool tofBegin() {
  // Hold all sensors in reset. Done on every boot so a warm reset doesn't
  // find sensors already moved away from 0x29.
  for (int i = 0; i < 3; i++) {
    pinMode(xshut[i], OUTPUT);
    digitalWrite(xshut[i], LOW);
  }
  delay(10);

  bool all = true;
  for (int i = 0; i < 3; i++) {
    digitalWrite(xshut[i], HIGH);
    delay(5);
    I2CGuard g;
    VL53L0X& s = sensors[i];
    s.setBus(&Wire);
    s.setTimeout(100);
    ok[i] = s.init();
    if (ok[i]) {
      s.setAddress(addr[i]);
      s.setMeasurementTimingBudget(TOF_BUDGET_US);
      s.startContinuous(0);
    }
    all = all && ok[i];
  }
  xTaskCreate(tofTask, "tof", 3072, nullptr, 5, nullptr);
  return all;
}

bool tofOk(TofId id) { return ok[id]; }
float tofMm(TofId id) { return mm[id]; }
uint32_t tofSeq(TofId id) { return seq[id]; }

void tofAverage(int n, float out[3]) {
  float sum[3] = {};
  int got[3] = {};
  uint32_t lastSeq[3] = {seq[0], seq[1], seq[2]};
  uint32_t start = millis();
  for (;;) {
    bool done = true;
    for (int i = 0; i < 3; i++) {
      if (!ok[i] || got[i] >= n) continue;
      done = false;
      if (seq[i] != lastSeq[i]) {
        lastSeq[i] = seq[i];
        sum[i] += mm[i];
        got[i]++;
      }
    }
    if (done || millis() - start > 500) break;
    delay(2);
  }
  for (int i = 0; i < 3; i++) out[i] = got[i] ? sum[i] / got[i] : NO_READING;
}
