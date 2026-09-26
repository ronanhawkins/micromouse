#pragma once
// Closed-loop motion. A high-priority task runs the controllers every
// CONTROL_PERIOD_MS. The move functions below block the calling task until the
// move finishes or aborts.
//
// Forward: PD on accumulated position error from the encoders.
// Rotation: PD on accumulated angle error from the gyro, plus optional
// side-wall steering.
// Both add feedforward volts from the profile speed and acceleration.

#include "config.h"

void motionBegin();

// Motors on/off. Disabling also resets controller errors.
void motionEnable(bool on);
bool motionEnabled();

// Zero accumulated errors and odometry. Call with the mouse stationary.
void motionReset();

bool motionMove(float mm, float topSpeed, float endSpeed, float accel);  // + forwards
bool motionTurn(float deg, float rate, float accel);                    // + left (CCW)
void motionSteering(bool on);

// Set when the forward error exceeds FWD_ERROR_ABORT_MM (stuck/crash) or
// motionAbort() is called. Motors are switched off. Cleared by motionReset().
bool motionAborted();
void motionAbort();

// Odometry since the last motionReset().
float motionDistanceMm();
float motionAngleDeg();
