#pragma once
// Trapezoidal speed profile, advanced once per control tick. Used for both
// forward motion (mm) and rotation (deg). Structure follows
// ukmars/mazerunner-core (MIT).

class Profile {
 public:
  // Move `distance` (signed) reaching at most `topSpeed`, ending at
  // `finalSpeed`, with acceleration `accel`. Speeds and accel are magnitudes.
  void start(float distance, float topSpeed, float finalSpeed, float accel);
  void stop();  // abandon the move, speed 0
  void update(float dt);

  bool finished() const { return !running_; }
  // Signed, in the direction of the current move.
  float position() const { return sign_ * position_; }
  float speed() const { return sign_ * speed_; }
  float accel() const { return accelNow_; }

 private:
  volatile bool running_ = false;
  float target_ = 0;
  float position_ = 0;
  float speed_ = 0;
  float topSpeed_ = 0;
  float finalSpeed_ = 0;
  float accel_ = 0;
  float accelNow_ = 0;
  float sign_ = 1;
};
