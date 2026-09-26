#pragma once
// The boundary between the maze logic and the world. The explorer only
// talks to the mouse through this interface. The desktop simulator
// implements it against a maze file; the firmware implements it with
// motion control and the ToF sensors.

#include "path.h"

namespace mm {

// Walls seen from the current cell, relative to the mouse's heading.
struct WallReading {
  bool left;
  bool front;
  bool right;
};

class MouseIO {
 public:
  virtual ~MouseIO() {}

  // Read walls while stopped at a cell centre. The side sensors would also
  // see the posts at cell boundaries, so walls are only sampled here.
  virtual WallReading senseWalls() = 0;

  // Drive `cells` cells straight, ending stopped at a cell centre.
  // Return false to abort the run (crash, user abort, ...).
  virtual bool forward(int cells) = 0;

  // Turn in place. `type` is never FORWARD.
  virtual bool turn(Move::Type type) = 0;

  // Execute a whole speed-run path. Hardware can override this to blend
  // moves at higher speed; the default just replays them.
  virtual bool runPath(const Path& path) {
    for (int i = 0; i < path.count; i++) {
      const Move& m = path.moves[i];
      bool ok = (m.type == Move::FORWARD) ? forward(m.cells) : turn(m.type);
      if (!ok) return false;
    }
    return true;
  }

  // Called after every map update, e.g. to save the map or draw it.
  virtual void mapUpdated(const Maze& maze) { (void)maze; }
};

}  // namespace mm
