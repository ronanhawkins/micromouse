#include "maze.h"

namespace mm {

void Maze::reset() {
  for (int i = 0; i < CELL_COUNT; i++) cells_[i] = 0;
  for (int i = 0; i < MAZE_SIZE; i++) {
    setWall(Cell{int8_t(i), 0}, SOUTH, true);
    setWall(Cell{int8_t(i), MAZE_SIZE - 1}, NORTH, true);
    setWall(Cell{0, int8_t(i)}, WEST, true);
    setWall(Cell{MAZE_SIZE - 1, int8_t(i)}, EAST, true);
  }
  setWall(Cell{0, 0}, EAST, true);
  setGoalCentre();
}

void Maze::clearAllKnown() {
  for (int i = 0; i < CELL_COUNT; i++) cells_[i] = 0xF0;
  for (int i = 0; i < MAZE_SIZE; i++) {
    setWall(Cell{int8_t(i), 0}, SOUTH, true);
    setWall(Cell{int8_t(i), MAZE_SIZE - 1}, NORTH, true);
    setWall(Cell{0, int8_t(i)}, WEST, true);
    setWall(Cell{MAZE_SIZE - 1, int8_t(i)}, EAST, true);
  }
}

void Maze::setWall(Cell c, Heading h, bool present) {
  const uint8_t wallBit = 1u << h;
  const uint8_t knownBit = 0x10u << h;
  uint8_t& a = cells_[cellIndex(c)];
  a = present ? (a | wallBit) : (a & ~wallBit);
  a |= knownBit;

  Cell n = neighbour(c, h);
  if (!inBounds(n)) return;
  Heading back = behind(h);
  uint8_t& b = cells_[cellIndex(n)];
  b = present ? (b | (1u << back)) : (b & ~(1u << back));
  b |= 0x10u << back;
}

void Maze::setGoalCentre() {
  goalCount_ = 0;
  const int8_t lo = MAZE_SIZE / 2 - 1, hi = MAZE_SIZE / 2;
  addGoal(Cell{lo, lo});
  addGoal(Cell{hi, lo});
  addGoal(Cell{lo, hi});
  addGoal(Cell{hi, hi});
}

void Maze::addGoal(Cell c) {
  if (goalCount_ < 8 && !isGoal(c)) goals_[goalCount_++] = c;
}

bool Maze::isGoal(Cell c) const {
  for (int i = 0; i < goalCount_; i++)
    if (goals_[i] == c) return true;
  return false;
}

}  // namespace mm
