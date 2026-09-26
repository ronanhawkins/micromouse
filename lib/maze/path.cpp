#include "path.h"

#include <stdio.h>

namespace mm {

void Path::addForward() {
  if (count > 0 && moves[count - 1].type == Move::FORWARD && moves[count - 1].cells < 255) {
    moves[count - 1].cells++;
    return;
  }
  if (count < MAX_MOVES) moves[count++] = Move{Move::FORWARD, 1};
  else overflow = true;
}

void Path::addTurn(Move::Type t) {
  if (count < MAX_MOVES) moves[count++] = Move{t, 0};
  else overflow = true;
}

Path planPath(const Maze& maze, const FloodFill& ff, Cell start, Heading heading, Unknown mode) {
  Path p;
  p.clear();
  Cell c = start;
  Heading h = heading;
  if (ff.cost(c, h) == COST_INF) return p;
  p.cost = ff.cost(c, h);

  for (int steps = 0; steps < CELL_COUNT * 2; steps++) {
    if (ff.cost(c, h) == 0) {
      p.valid = !p.overflow;
      break;
    }
    int exit = ff.bestExit(maze, c, h, mode);
    if (exit < 0) break;
    Heading d = Heading(exit);
    int turn = (d - h) & 3;
    if (turn == 1) p.addTurn(Move::TURN_RIGHT);
    else if (turn == 2) p.addTurn(Move::TURN_AROUND);
    else if (turn == 3) p.addTurn(Move::TURN_LEFT);
    h = d;
    p.addForward();
    c = neighbour(c, h);
  }
  p.end = c;
  p.endHeading = h;
  return p;
}

void pathToString(const Path& p, char* buf, int bufLen) {
  int n = 0;
  if (bufLen > 0) buf[0] = 0;
  for (int i = 0; i < p.count && n < bufLen - 1; i++) {
    const Move& m = p.moves[i];
    int w;
    switch (m.type) {
      case Move::FORWARD: w = snprintf(buf + n, bufLen - n, "F%d ", m.cells); break;
      case Move::TURN_LEFT: w = snprintf(buf + n, bufLen - n, "L "); break;
      case Move::TURN_RIGHT: w = snprintf(buf + n, bufLen - n, "R "); break;
      default: w = snprintf(buf + n, bufLen - n, "A "); break;
    }
    if (w < 0) break;
    n += w;
  }
  if (n > 0 && n < bufLen && buf[n - 1] == ' ') buf[n - 1] = 0;
}

}  // namespace mm
