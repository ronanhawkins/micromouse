#include "maze_file.h"

#include <string.h>

namespace mm {

namespace {

constexpr int LINES = 2 * MAZE_SIZE + 1;
constexpr int WIDTH = 4 * MAZE_SIZE + 1;

// Character at column `col` of a line, or ' ' past its end.
char at(const char* line, int len, int col) { return col < len ? line[col] : ' '; }

struct Rng {
  uint32_t s;
  uint32_t next() {
    s ^= s << 13;
    s ^= s >> 17;
    s ^= s << 5;
    return s;
  }
  int below(int n) { return int(next() % uint32_t(n)); }
};

}  // namespace

bool parseMazeText(const char* text, Maze& out) {
  const char* lines[LINES];
  int lens[LINES];
  int found = 0;

  const char* p = text;
  while (*p && found < LINES) {
    const char* end = p;
    while (*end && *end != '\n') end++;
    int len = int(end - p);
    if (len > 0 && p[len - 1] == '\r') len--;
    // Skip anything before the first post row.
    if (found > 0 || (len > 0 && p[0] == 'o')) {
      lines[found] = p;
      lens[found] = len;
      found++;
    }
    p = *end ? end + 1 : end;
  }
  if (found != LINES) return false;
  for (int i = 0; i < LINES; i += 2)
    if (at(lines[i], lens[i], 0) != 'o' || at(lines[i], lens[i], WIDTH - 1) != 'o') return false;

  out.clearAllKnown();
  out.clearGoals();
  bool anyGoal = false;

  for (int k = 0; k < MAZE_SIZE; k++) {
    const int8_t y = int8_t(MAZE_SIZE - 1 - k);
    const char* hl = lines[2 * k];
    const char* vl = lines[2 * k + 1];
    for (int8_t x = 0; x < MAZE_SIZE; x++) {
      Cell c{x, y};
      out.setWall(c, NORTH, at(hl, lens[2 * k], 4 * x + 2) == '-');
      out.setWall(c, WEST, at(vl, lens[2 * k + 1], 4 * x) == '|');
      char mark = at(vl, lens[2 * k + 1], 4 * x + 2);
      if (mark == 'G' || mark == 'g') {
        out.addGoal(c);
        anyGoal = true;
      }
    }
    out.setWall(Cell{MAZE_SIZE - 1, y}, EAST, true);
  }
  // The outer boundary is always walled, whatever the file says.
  for (int8_t i = 0; i < MAZE_SIZE; i++) {
    out.setWall(Cell{i, MAZE_SIZE - 1}, NORTH, true);
    out.setWall(Cell{i, 0}, SOUTH, true);
    out.setWall(Cell{0, i}, WEST, true);
  }
  if (!anyGoal) out.setGoalCentre();
  return true;
}

void mazeToText(const Maze& maze, char* buf) {
  char* w = buf;
  for (int k = 0; k <= MAZE_SIZE; k++) {
    // Post row: north walls of row y (or south walls of row 0 at the end).
    for (int8_t x = 0; x < MAZE_SIZE; x++) {
      bool wall = k < MAZE_SIZE ? maze.hasWall(Cell{x, int8_t(MAZE_SIZE - 1 - k)}, NORTH)
                                : maze.hasWall(Cell{x, 0}, SOUTH);
      memcpy(w, wall ? "o---" : "o   ", 4);
      w += 4;
    }
    *w++ = 'o';
    *w++ = '\n';
    if (k == MAZE_SIZE) break;

    const int8_t y = int8_t(MAZE_SIZE - 1 - k);
    for (int8_t x = 0; x < MAZE_SIZE; x++) {
      Cell c{x, y};
      *w++ = maze.hasWall(c, WEST) ? '|' : ' ';
      *w++ = ' ';
      *w++ = maze.isGoal(c) ? 'G' : (x == 0 && y == 0) ? 'S' : ' ';
      *w++ = ' ';
    }
    *w++ = maze.hasWall(Cell{MAZE_SIZE - 1, y}, EAST) ? '|' : ' ';
    *w++ = '\n';
  }
  *w = 0;
}

void generateMaze(Maze& out, uint32_t seed, int extraOpenings) {
  Rng rng{seed ? seed : 0x9E3779B9u};

  out.clearAllKnown();
  out.setGoalCentre();
  for (int i = 0; i < CELL_COUNT; i++) out.setRaw(i, 0xFF);  // every wall present and known

  const Cell start{0, 0};
  auto forbidden = [&](Cell c, Heading h) {
    Cell n = neighbour(c, h);
    // Start cell stays closed to the east.
    if ((c == start && h == EAST) || (n == start && h == WEST)) return true;
    // Goal perimeter is opened separately (exactly one entrance).
    return out.isGoal(c) != out.isGoal(n);
  };

  // Depth-first carve from the start. Goal cells are pre-marked visited so
  // the carve never enters them.
  bool visited[CELL_COUNT] = {};
  for (int i = 0; i < out.goalCount(); i++) visited[cellIndex(out.goal(i))] = true;
  Cell stack[CELL_COUNT];
  int sp = 0;
  stack[sp++] = start;
  visited[cellIndex(start)] = true;
  while (sp > 0) {
    Cell c = stack[sp - 1];
    Heading options[4];
    int n = 0;
    for (int h = 0; h < 4; h++) {
      Cell nb = neighbour(c, Heading(h));
      if (inBounds(nb) && !visited[cellIndex(nb)] && !forbidden(c, Heading(h)))
        options[n++] = Heading(h);
    }
    if (n == 0) {
      sp--;
      continue;
    }
    Heading h = options[rng.below(n)];
    Cell nb = neighbour(c, h);
    out.setWall(c, h, false);
    visited[cellIndex(nb)] = true;
    stack[sp++] = nb;
  }

  // Open the inside of the goal, then one entrance on its perimeter.
  Cell entranceCells[8];
  Heading entranceDirs[8];
  int ne = 0;
  for (int i = 0; i < out.goalCount(); i++) {
    Cell g = out.goal(i);
    for (int h = 0; h < 4; h++) {
      Cell nb = neighbour(g, Heading(h));
      if (!inBounds(nb)) continue;
      if (out.isGoal(nb)) {
        out.setWall(g, Heading(h), false);
      } else if (ne < 8) {
        entranceCells[ne] = g;
        entranceDirs[ne] = Heading(h);
        ne++;
      }
    }
  }
  if (ne > 0) {
    int e = rng.below(ne);
    out.setWall(entranceCells[e], entranceDirs[e], false);
  }

  // Knock out some extra walls so there are alternative routes.
  for (int tries = 0, done = 0; done < extraOpenings && tries < extraOpenings * 50; tries++) {
    Cell c{int8_t(rng.below(MAZE_SIZE)), int8_t(rng.below(MAZE_SIZE))};
    Heading h = Heading(rng.below(4));
    Cell nb = neighbour(c, h);
    if (!inBounds(nb) || forbidden(c, h) || out.isGoal(c) || !out.hasWall(c, h)) continue;
    out.setWall(c, h, false);
    done++;
  }
}

}  // namespace mm
