#pragma once
// Turning a cost map into a list of moves for the speed run.

#include <stdint.h>

#include "floodfill.h"
#include "maze.h"

namespace mm {

struct Move {
  enum Type : uint8_t { FORWARD, TURN_LEFT, TURN_RIGHT, TURN_AROUND } type;
  uint8_t cells;  // FORWARD only: number of cells, straights already merged
};

struct Path {
  // Worst case is a turn between every cell of a 256-cell route.
  static constexpr int MAX_MOVES = 2 * CELL_COUNT;
  Move moves[MAX_MOVES];
  int count = 0;
  uint32_t cost = 0;  // planner cost (same units as Costs)
  Cell end{0, 0};
  Heading endHeading = NORTH;
  bool valid = false;
  bool overflow = false;

  void clear() { count = 0; cost = 0; valid = false; overflow = false; }
  void addForward();
  void addTurn(Move::Type t);
};

// Follow `ff` from (start, heading) to a target using only moves allowed
// under `mode`. For the speed run use Unknown::BLOCKED, so the mouse only
// drives through walls it has actually seen.
Path planPath(const Maze& maze, const FloodFill& ff, Cell start, Heading heading, Unknown mode);

// Human-readable, e.g. "F3 R F1 L F2".
void pathToString(const Path& p, char* buf, int bufLen);

}  // namespace mm
