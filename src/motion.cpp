#include "motion.h"

#include <Arduino.h>
#include <esp_timer.h>

#include "encoders.h"
#include "imu.h"
#include "motors.h"
#include "profile.h"
#include "tof.h"

namespace {

Profile fwd;
Profile rot;
portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

volatile bool enabled = false;
volatile bool steering = false;
volatile bool aborted = false;
volatile bool resetRequested = false;

volatile float fwdError = 0, rotError = 0;
float lastFwdError = 0, lastRotError = 0;
float distanceMm = 0, angleDeg = 0;
int32_t lastL = 0, lastR = 0;

float feedforward(float speed, float accel) {
  float v = FF_VOLTS_PER_MMPS * speed + FF_VOLTS_PER_MMPS2 * accel;
  if (speed > 1) v += FF_BIAS_VOLTS;
  else if (speed < -1) v -= FF_BIAS_VOLTS;
  return v;
}

// Degrees to add to the rotation error, from side-wall distances.
// Positive steers left.
float steeringAdjust() {
  float l = tofMm(TOF_LEFT), r = tofMm(TOF_RIGHT);
  bool hasL = l < SIDE_WALL_MM, hasR = r < SIDE_WALL_MM;
  float offCentre;  // + means the mouse is left of centre
  if (hasL && hasR) offCentre = (r - l) / 2;
  else if (hasL) offCentre = SIDE_NOMINAL_MM - l;
  else if (hasR) offCentre = r - SIDE_NOMINAL_MM;
  else return 0;
  float adj = -STEER_KP * offCentre;  // left of centre -> steer right
  if (adj > STEER_MAX_DEG) adj = STEER_MAX_DEG;
  if (adj < -STEER_MAX_DEG) adj = -STEER_MAX_DEG;
  return adj;
}

void controlTick() {
  int32_t l = encoderLeft(), r = encoderRight();
  float dl = (l - lastL) * MM_PER_COUNT;
  float dr = (r - lastR) * MM_PER_COUNT;
  lastL = l;
  lastR = r;
  float gyro = imuReadZ();

  // Integrate the gyro over the real time since the last tick, so a late or
  // skipped tick doesn't lose rotation.
  static int64_t lastUs = esp_timer_get_time();
  int64_t nowUs = esp_timer_get_time();
  float dt = (nowUs - lastUs) * 1e-6f;
  lastUs = nowUs;
  if (dt > 0.05f) dt = 0.05f;

  float dFwd = (dl + dr) / 2;
  float dRot = gyro * dt;
  distanceMm += dFwd;
  angleDeg += dRot;

  if (resetRequested) {
    fwdError = 0;
    rotError = 0;
    lastFwdError = lastRotError = 0;
    distanceMm = angleDeg = 0;
    resetRequested = false;
  }
  // Disabled: leave the motors alone (motionEnable(false) / abort already
  // switched them off), so open-loop tests like `motor` can drive them.
  if (!enabled || aborted) return;

  portENTER_CRITICAL(&mux);
  fwd.update(CONTROL_DT);
  rot.update(CONTROL_DT);
  float v = fwd.speed(), a = fwd.accel();
  float w = rot.speed(), wa = rot.accel();
  portEXIT_CRITICAL(&mux);

  fwdError += v * CONTROL_DT - dFwd;
  rotError += w * CONTROL_DT - dRot;
  if (steering) rotError += steeringAdjust();

  float fwdOut = FWD_KP * fwdError + FWD_KD * (fwdError - lastFwdError) / CONTROL_DT;
  float rotOut = ROT_KP * rotError + ROT_KD * (rotError - lastRotError) / CONTROL_DT;
  // Settling on a target with the profile stopped: kick past static friction.
  if (fwd.finished() && fabsf(v) < 1 && fabsf(fwdError) > SETTLE_FWD_MM) fwdOut += copysignf(STICTION_VOLTS, fwdError);
  if (rot.finished() && fabsf(w) < 1 && fabsf(rotError) > SETTLE_ROT_DEG) rotOut += copysignf(STICTION_VOLTS, rotError);
  lastFwdError = fwdError;
  lastRotError = rotError;

  // Wheel speeds for the profile, for feedforward.
  constexpr float MM_PER_DEG = WHEELBASE_MM / 2 * 3.14159265f / 180;
  float vl = v - w * MM_PER_DEG, vr = v + w * MM_PER_DEG;
  float al = a - wa * MM_PER_DEG, ar = a + wa * MM_PER_DEG;

  motorsSetVolts(fwdOut - rotOut + feedforward(vl, al), fwdOut + rotOut + feedforward(vr, ar));

  if (fabsf(fwdError) > FWD_ERROR_ABORT_MM) {
    aborted = true;
    motorsOff();
  }
}

void controlTask(void*) {
  lastL = encoderLeft();
  lastR = encoderRight();
  TickType_t last = xTaskGetTickCount();
  for (;;) {
    controlTick();
    // If a tick overran (e.g. I2C timeouts), don't try to catch up: that
    // would run this top-priority task flat out, starve everything else and
    // trip the watchdog (the mouse reboots). Always give up at least a tick.
    if (xTaskDelayUntil(&last, pdMS_TO_TICKS(CONTROL_PERIOD_MS)) == pdFALSE) {
      vTaskDelay(1);
      last = xTaskGetTickCount();
    }
  }
}

// Wait for the profile, then for the controller to pull the remaining
// error in (the profile finishes on time whether or not the mouse kept up).
bool waitFor(Profile& p, const volatile float& error, float tolerance) {
  while (!p.finished()) {
    if (aborted) return false;
    delay(1);
  }
  unsigned long start = millis();
  while (fabsf(error) > tolerance && millis() - start < SETTLE_TIMEOUT_MS) {
    if (aborted) return false;
    delay(2);
  }
  return !aborted;
}

}  // namespace

void motionBegin() {
  // Above the ToF task (5) and Arduino loop (1).
  xTaskCreate(controlTask, "control", 4096, nullptr, configMAX_PRIORITIES - 2, nullptr);
}

void motionEnable(bool on) {
  if (on) motionReset();
  enabled = on;
  if (!on) motorsOff();
}

bool motionEnabled() { return enabled; }

void motionReset() {
  portENTER_CRITICAL(&mux);
  fwd.stop();
  rot.stop();
  portEXIT_CRITICAL(&mux);
  aborted = false;
  resetRequested = true;
  while (resetRequested) delay(1);
}

bool motionMove(float mm, float topSpeed, float endSpeed, float accel) {
  portENTER_CRITICAL(&mux);
  fwd.start(mm, topSpeed, endSpeed, accel);
  portEXIT_CRITICAL(&mux);
  // Only settle when stopping; a move ending at speed hands straight on.
  return waitFor(fwd, fwdError, endSpeed > 0 ? 1e9f : SETTLE_FWD_MM);
}

bool motionTurn(float deg, float rate, float accel) {
  portENTER_CRITICAL(&mux);
  rot.start(deg, rate, 0, accel);
  portEXIT_CRITICAL(&mux);
  return waitFor(rot, rotError, SETTLE_ROT_DEG);
}

void motionSteering(bool on) { steering = on; }
bool motionAborted() { return aborted; }

void motionAbort() {
  aborted = true;
  motorsOff();
}

float motionDistanceMm() { return distanceMm; }
float motionAngleDeg() { return angleDeg; }
