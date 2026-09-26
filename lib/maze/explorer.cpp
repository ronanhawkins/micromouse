#include "explorer.h"

namespace mm {

namespace {
const Cell START{0, 0};
}

void Explorer::senseAndUpdate() {
  if (maze_.isVisited(pos_)) return;
  WallReading w = io_.senseWalls();
  maze_.setWall(pos_, leftOf(heading_), w.left);
  maze_.setWall(pos_, heading_, w.front);
  maze_.setWall(pos_, rightOf(heading_), w.right);
  io_.mapUpdated(maze_);
}

bool Explorer::doTurn(Heading to) {
  int d = (to - heading_) & 3;
  if (d == 0) return true;
  Move::Type t = d == 1 ? Move::TURN_RIGHT : d == 2 ? Move::TURN_AROUND : Move::TURN_LEFT;
  if (!io_.turn(t)) return false;
  heading_ = to;
  stats_.turns++;
  return true;
}

bool Explorer::face(Heading h) {
  if (!doTurn(h)) {
    stats_.aborted = true;
    return false;
  }
  return true;
}

bool Explorer::searchTo(const Cell* targets, int count) {
  auto isTarget = [&](Cell c) {
    for (int i = 0; i < count; i++)
      if (targets[i] == c) return true;
    return false;
  };

  // Every step either reveals a wall or moves strictly closer to the
  // target on the current map, so this bound is never reached in practice.
  for (int guard = 0; guard < CELL_COUNT * 8; guard++) {
    senseAndUpdate();
    if (isTarget(pos_)) return true;

    ff_.run(maze_, targets, count, Unknown::OPEN, costs_);
    int exit = ff_.bestExit(maze_, pos_, heading_, Unknown::OPEN);
    if (exit < 0) return false;  // target walled off

    if (!doTurn(Heading(exit))) {
      stats_.aborted = true;
      return false;
    }

    // Drive through already-visited cells without stopping, as long as the
    // plan keeps going straight. Stop in any unvisited cell to look at it.
    int n = 1;
    Cell next = neighbour(pos_, heading_);
    while (maze_.isVisited(next) && !isTarget(next) &&
           ff_.bestExit(maze_, next, heading_, Unknown::OPEN) == heading_) {
      next = neighbour(next, heading_);
      n++;
    }

    if (!io_.forward(n)) {
      stats_.aborted = true;
      return false;
    }
    for (int i = 0; i < n; i++) pos_ = neighbour(pos_, heading_);
    stats_.cellsMoved += n;
  }
  return false;
}

bool Explorer::searchToGoal() {
  Cell goals[8];
  for (int i = 0; i < maze_.goalCount(); i++) goals[i] = maze_.goal(i);
  bool ok = searchTo(goals, maze_.goalCount());
  if (ok) stats_.reachedGoal = true;
  stats_.legs++;
  return ok;
}

bool Explorer::searchToStart() {
  bool ok = searchTo(&START, 1);
  stats_.legs++;
  return ok;
}

bool Explorer::isProvenOptimal() {
  ff_.runToGoal(maze_, Unknown::OPEN, costs_);
  uint16_t optimistic = ff_.cost(START, NORTH);
  ff_.runToGoal(maze_, Unknown::BLOCKED, costs_);
  uint16_t pessimistic = ff_.cost(START, NORTH);
  return pessimistic != COST_INF && optimistic == pessimistic;
}

bool Explorer::fullSearch(int maxRoundTrips) {
  for (int trip = 0; trip < maxRoundTrips; trip++) {
    if (!searchToGoal()) return false;
    if (!searchToStart()) return false;
    stats_.proven = isProvenOptimal();
    if (stats_.proven) break;
  }
  stats_.backAtStart = pos_ == START;
  return face(NORTH);
}

Path Explorer::speedRunPath() {
  ff_.runToGoal(maze_, Unknown::BLOCKED, costs_);
  return planPath(maze_, ff_, START, NORTH, Unknown::BLOCKED);
}

}  // namespace mm
