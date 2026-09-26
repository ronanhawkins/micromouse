#pragma once
// Search strategy: flood-fill exploration using MouseIO.
//
//  1. Search from start to the goal (unknown walls assumed open).
//  2. Search back to the start (this explores alternative routes).
//  3. Repeat goal <-> start until the best path is proven optimal:
//     the cost with unknown walls OPEN equals the cost with them BLOCKED,
//     so no unexplored cell could give a shorter route.
//  4. The speed-run path uses only seen walls.

#include "floodfill.h"
#include "maze.h"
#include "mouse_io.h"
#include "path.h"

namespace mm {

struct ExploreStats {
  int cellsMoved = 0;
  int turns = 0;
  int legs = 0;
  bool reachedGoal = false;
  bool backAtStart = false;
  bool proven = false;
  bool aborted = false;
};

class Explorer {
 public:
  Explorer(Maze& maze, MouseIO& io, const Costs& costs = Costs())
      : maze_(maze), io_(io), costs_(costs) {}

  // Place the mouse (normally start cell, facing north).
  void setPose(Cell c, Heading h) { pos_ = c; heading_ = h; }
  Cell position() const { return pos_; }
  Heading heading() const { return heading_; }

  // Explore until we reach any of `targets`. Returns false if the targets
  // are unreachable or the IO aborted.
  bool searchTo(const Cell* targets, int count);
  bool searchToGoal();
  bool searchToStart();

  // Full search: start -> goal -> start, then extra round trips (up to
  // maxRoundTrips in total) until the path is proven optimal. Ends at the
  // start cell, facing north, ready for a speed run.
  bool fullSearch(int maxRoundTrips = 4);

  // True when no unexplored cells could shorten the start->goal route.
  bool isProvenOptimal();

  // Speed-run path from the start cell facing north, using seen walls only.
  Path speedRunPath();

  // Turn in place to face `h`.
  bool face(Heading h);

  const ExploreStats& stats() const { return stats_; }

 private:
  void senseAndUpdate();
  bool doTurn(Heading to);

  Maze& maze_;
  MouseIO& io_;
  Costs costs_;
  FloodFill ff_;
  Cell pos_{0, 0};
  Heading heading_ = NORTH;
  ExploreStats stats_;
};

}  // namespace mm
