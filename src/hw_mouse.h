#pragma once
// The real mouse behind mm::MouseIO: motion control + ToF wall sensing.

#include <mouse_io.h>

#include "config.h"

class HwMouse : public mm::MouseIO {
 public:
  void setSpeeds(const SpeedSet& s) { speeds_ = s; }

  // Every search/run starts and ends with the mouse's back against the
  // start cell's south wall, so the start pose is the same whether the
  // mouse drove home itself or was put there by hand.
  // leaveBackWall: back against the wall -> start cell centre, facing north.
  // returnToBackWall: start cell centre facing north -> back to the wall.
  bool leaveBackWall();
  bool returnToBackWall();

  mm::WallReading senseWalls() override;
  bool forward(int cells) override;
  bool turn(mm::Move::Type type) override;
  void mapUpdated(const mm::Maze& maze) override;

 private:
  // Stopped with a wall ahead: nudge to the exact cell centre.
  bool alignToFrontWall();
  bool quarterTurn(float deg);

  SpeedSet speeds_ = SEARCH_SPEEDS;
};
