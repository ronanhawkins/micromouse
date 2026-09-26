#include "profile.h"

#include <math.h>

void Profile::start(float distance, float topSpeed, float finalSpeed, float accel) {
  sign_ = distance < 0 ? -1.0f : 1.0f;
  target_ = fabsf(distance);
  position_ = 0;
  // Carry on from the current speed (e.g. a move that ended at finalSpeed).
  speed_ = fabsf(speed_);
  topSpeed_ = topSpeed;
  finalSpeed_ = finalSpeed;
  accel_ = accel;
  running_ = target_ > 0;
}

void Profile::stop() {
  running_ = false;
  speed_ = 0;
  accelNow_ = 0;
}

void Profile::update(float dt) {
  if (!running_) {
    accelNow_ = 0;
    if (finalSpeed_ == 0) speed_ = 0;
    return;
  }
  float remaining = target_ - position_;
  float brakeDist = (speed_ * speed_ - finalSpeed_ * finalSpeed_) / (2 * accel_);
  float want = remaining <= brakeDist ? finalSpeed_ : topSpeed_;
  // Never stall short of the target when braking to zero.
  float crawl = 0.05f * topSpeed_;
  if (want < crawl && remaining > 0) want = crawl;

  float before = speed_;
  if (speed_ < want) speed_ = fminf(speed_ + accel_ * dt, want);
  else if (speed_ > want) speed_ = fmaxf(speed_ - accel_ * dt, want);
  accelNow_ = sign_ * (speed_ - before) / dt;

  position_ += speed_ * dt;
  if (position_ >= target_) {
    position_ = target_;
    speed_ = finalSpeed_;
    running_ = false;
  }
}
