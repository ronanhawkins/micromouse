#pragma once
// Hardware-independent maze map. No Arduino includes: this compiles on the
// desktop (simulator, unit tests) and on the ESP32.
//
// Coordinates: x runs east 0..15, y runs north 0..15. The start cell is (0,0)
// in the south-west corner, and the mouse starts facing north.

#include <stdint.h>

namespace mm {

constexpr int MAZE_SIZE = 16;
constexpr int CELL_COUNT = MAZE_SIZE * MAZE_SIZE;

enum Heading : uint8_t { NORTH = 0, EAST = 1, SOUTH = 2, WEST = 3 };

inline Heading rightOf(Heading h) { return Heading((h + 1) & 3); }
inline Heading leftOf(Heading h) { return Heading((h + 3) & 3); }
inline Heading behind(Heading h) { return Heading((h + 2) & 3); }

struct Cell {
  int8_t x;
  int8_t y;
  bool operator==(const Cell& o) const { return x == o.x && y == o.y; }
  bool operator!=(const Cell& o) const { return !(*this == o); }
};

inline Cell neighbour(Cell c, Heading h) {
  static const int8_t dx[4] = {0, 1, 0, -1};
  static const int8_t dy[4] = {1, 0, -1, 0};
  return Cell{int8_t(c.x + dx[h]), int8_t(c.y + dy[h])};
}

inline bool inBounds(Cell c) {
  return c.x >= 0 && c.x < MAZE_SIZE && c.y >= 0 && c.y < MAZE_SIZE;
}

inline int cellIndex(Cell c) { return c.y * MAZE_SIZE + c.x; }
inline Cell cellAt(int index) { return Cell{int8_t(index % MAZE_SIZE), int8_t(index / MAZE_SIZE)}; }

// How unknown walls are treated when asking "can I move this way?".
enum class Unknown : uint8_t {
  OPEN,     // optimistic: used while searching
  BLOCKED,  // pessimistic: used for the speed run
};

class Maze {
 public:
  Maze() { reset(); }

  // Forget everything except the outer boundary and the start cell's east
  // wall (a classic start cell is walled on three sides).
  void reset();

  // Wipe to "every wall known and absent" apart from the boundary. Used to
  // build a ground-truth maze from a file or generator.
  void clearAllKnown();

  bool hasWall(Cell c, Heading h) const { return cells_[cellIndex(c)] & (1u << h); }
  bool isKnown(Cell c, Heading h) const { return cells_[cellIndex(c)] & (0x10u << h); }

  // Record a wall observation. Updates the neighbour's side too.
  void setWall(Cell c, Heading h, bool present);

  bool canMove(Cell c, Heading h, Unknown mode) const {
    if (!inBounds(neighbour(c, h))) return false;
    if (!isKnown(c, h)) return mode == Unknown::OPEN;
    return !hasWall(c, h);
  }

  // True once all four walls of the cell have been seen.
  bool isVisited(Cell c) const { return (cells_[cellIndex(c)] & 0xF0u) == 0xF0u; }

  // Goal cells. Defaults to the classic 2x2 centre block.
  void setGoalCentre();
  void clearGoals() { goalCount_ = 0; }
  void addGoal(Cell c);
  bool isGoal(Cell c) const;
  int goalCount() const { return goalCount_; }
  Cell goal(int i) const { return goals_[i]; }

  // Raw byte per cell: bits 0-3 wall present (N,E,S,W), bits 4-7 known.
  // Exposed so firmware can save/restore the map.
  uint8_t raw(int index) const { return cells_[index]; }
  void setRaw(int index, uint8_t v) { cells_[index] = v; }

 private:
  uint8_t cells_[CELL_COUNT];
  Cell goals_[8];
  int goalCount_ = 0;
};

}  // namespace mm
