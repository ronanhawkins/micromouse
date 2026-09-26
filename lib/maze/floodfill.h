#pragma once
// Cost-to-target map over (cell, heading) states.
//
// A plain cell-distance flood fill ignores turns, so the "shortest" path it
// finds can zig-zag. The mouse turns in place and must slow down for every
// turn, so each state's cost is the time-like cost of driving from that cell,
// facing that heading, to the nearest target. Search and the speed run
// both use this, so what the mouse explores is the path it will later run.

#include <stdint.h>

#include "maze.h"

namespace mm {

struct Costs {
  uint16_t straight = 10;  // one cell, continuing straight
  uint16_t turn90 = 25;    // in-place quarter turn (includes decel/accel)
  uint16_t turn180 = 40;   // in-place about-turn
};

constexpr uint16_t COST_INF = 0xFFFF;

class FloodFill {
 public:
  // Compute costs to reach any of `targets`. Unknown walls are treated
  // according to `mode`.
  void run(const Maze& maze, const Cell* targets, int targetCount, Unknown mode,
           const Costs& costs = Costs());

  // Convenience: flood to the maze's goal cells.
  void runToGoal(const Maze& maze, Unknown mode, const Costs& costs = Costs());

  uint16_t cost(Cell c, Heading h) const { return cost_[cellIndex(c)][h]; }

  // Best direction to leave cell `c` when currently facing `h`, or -1 if the
  // targets are unreachable. Ties prefer straight ahead, then right, left,
  // back. `c` must not itself be a target.
  int bestExit(const Maze& maze, Cell c, Heading h, Unknown mode) const;

  // Cost of leaving via `exitDir` and then following the map (COST_INF if
  // blocked or unreachable).
  uint32_t exitCost(const Maze& maze, Cell c, Heading h, Heading exitDir, Unknown mode) const;

 private:
  uint16_t cost_[CELL_COUNT][4];
  Costs costs_;
};

}  // namespace mm
