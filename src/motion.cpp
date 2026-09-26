#include "motion.h"

#include <Arduino.h>

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

float fwdError = 0, rotError = 0;
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

  float dFwd = (dl + dr) / 2;
  float dRot = gyro * CONTROL_DT;
  distanceMm += dFwd;
  angleDeg += dRot;

  if (resetRequested) {
    fwdError = rotError = lastFwdError = lastRotError = 0;
    distanceMm = angleDeg = 0;
    resetRequested = false;
  }
  if (!enabled || aborted) {
    motorsOff();
    return;
  }

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
    vTaskDelayUntil(&last, pdMS_TO_TICKS(CONTROL_PERIOD_MS));
  }
}

bool waitFor(Profile& p) {
  while (!p.finished()) {
    if (aborted) return false;
    delay(1);
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
  return waitFor(fwd);
}

bool motionTurn(float deg, float rate, float accel) {
  portENTER_CRITICAL(&mux);
  rot.start(deg, rate, 0, accel);
  portEXIT_CRITICAL(&mux);
  return waitFor(rot);
}

void motionSteering(bool on) { steering = on; }
bool motionAborted() { return aborted; }

void motionAbort() {
  aborted = true;
  motorsOff();
}

float motionDistanceMm() { return distanceMm; }
float motionAngleDeg() { return angleDeg; }
