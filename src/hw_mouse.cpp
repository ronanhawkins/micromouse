#include "hw_mouse.h"

#include <Arduino.h>

#include "motion.h"
#include "storage.h"
#include "tof.h"
#include "ui.h"

namespace {
float centreToBackWallMm() { return CELL_MM / 2 - HALF_WALL_MM - BACK_TO_AXLE_MM; }
}  // namespace

bool HwMouse::leaveBackWall() {
  return motionMove(centreToBackWallMm(), speeds_.straight / 2, 0, speeds_.accel);
}

bool HwMouse::returnToBackWall() {
  // Stop a few mm short so the motors don't push against the wall.
  return motionMove(-(centreToBackWallMm() - 3), speeds_.straight / 4, 0, speeds_.accel);
}

mm::WallReading HwMouse::senseWalls() {
  delay(20);  // let the chassis settle so readings aren't taken mid-wobble
  float d[3];
  tofAverage(WALL_SAMPLES, d);
  return mm::WallReading{d[TOF_LEFT] < SIDE_WALL_MM, d[TOF_FRONT] < FRONT_WALL_MM, d[TOF_RIGHT] < SIDE_WALL_MM};
}

bool HwMouse::alignToFrontWall() {
  float d[3];
  tofAverage(WALL_SAMPLES, d);
  if (d[TOF_FRONT] >= FRONT_WALL_MM) return true;
  float err = d[TOF_FRONT] - FRONT_CENTRED_MM;
  if (fabsf(err) < 2 || fabsf(err) > FRONT_ALIGN_MAX_MM) return true;
  return motionMove(err, speeds_.straight / 4, 0, speeds_.accel);
}

bool HwMouse::forward(int cells) {
  motionSteering(true);
  bool ok = motionMove(cells * CELL_MM, speeds_.straight, 0, speeds_.accel);
  motionSteering(false);
  if (!ok) return false;
  return alignToFrontWall();
}

bool HwMouse::turn(mm::Move::Type type) {
  float deg = type == mm::Move::TURN_LEFT ? 90 : type == mm::Move::TURN_RIGHT ? -90 : 180;
  return motionTurn(deg, speeds_.turnRate, speeds_.turnAccel);
}

void HwMouse::mapUpdated(const mm::Maze& maze) {
  storageSaveMaze(maze);
  uiBlink();
}
