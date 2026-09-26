// Desktop simulator: runs the real Explorer (lib/maze) against maze files or
// randomly generated mazes, using an idealised mouse that senses walls
// perfectly at each cell centre.
//
//   ./sim [-v] maze1.txt maze2.txt ...   run each maze file
//   ./sim [-v] --random N [seed]         run N generated mazes
//
// Exit status is non-zero if any maze fails.

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <string>
#include <vector>

#include "explorer.h"
#include "maze_file.h"

using namespace mm;

namespace {

class SimMouse : public MouseIO {
 public:
  explicit SimMouse(const Maze& truth) : truth_(truth) {}

  WallReading senseWalls() override {
    return WallReading{truth_.hasWall(pos, leftOf(heading)), truth_.hasWall(pos, heading),
                       truth_.hasWall(pos, rightOf(heading))};
  }

  bool forward(int cells) override {
    for (int i = 0; i < cells; i++) {
      if (truth_.hasWall(pos, heading)) {
        crashed = true;
        return false;
      }
      pos = neighbour(pos, heading);
    }
    return true;
  }

  bool turn(Move::Type type) override {
    if (type == Move::TURN_LEFT) heading = leftOf(heading);
    else if (type == Move::TURN_RIGHT) heading = rightOf(heading);
    else heading = behind(heading);
    return true;
  }

  Cell pos{0, 0};
  Heading heading = NORTH;
  bool crashed = false;

 private:
  const Maze& truth_;
};

// Draw the mouse's map: known walls as in the file format, unknown walls as
// '.', unvisited cells marked with '?', and the speed-run route with '*'.
void render(const Maze& known, const Path& run) {
  bool onRoute[CELL_COUNT] = {};
  Cell c{0, 0};
  Heading h = NORTH;
  onRoute[0] = true;
  for (int i = 0; i < run.count; i++) {
    const Move& m = run.moves[i];
    if (m.type == Move::TURN_LEFT) h = leftOf(h);
    else if (m.type == Move::TURN_RIGHT) h = rightOf(h);
    else if (m.type == Move::TURN_AROUND) h = behind(h);
    else
      for (int k = 0; k < m.cells; k++) {
        c = neighbour(c, h);
        onRoute[cellIndex(c)] = true;
      }
  }

  auto wallStr = [&](Cell cell, Heading hd, const char* wall, const char* open, const char* unk) {
    if (!known.isKnown(cell, hd)) return unk;
    return known.hasWall(cell, hd) ? wall : open;
  };
  for (int k = 0; k <= MAZE_SIZE; k++) {
    std::string line;
    for (int8_t x = 0; x < MAZE_SIZE; x++) {
      Cell cell = k < MAZE_SIZE ? Cell{x, int8_t(MAZE_SIZE - 1 - k)} : Cell{x, 0};
      line += "o";
      line += wallStr(cell, k < MAZE_SIZE ? NORTH : SOUTH, "---", "   ", " . ");
    }
    printf("%so\n", line.c_str());
    if (k == MAZE_SIZE) break;
    line.clear();
    int8_t y = int8_t(MAZE_SIZE - 1 - k);
    for (int8_t x = 0; x < MAZE_SIZE; x++) {
      Cell cell{x, y};
      line += wallStr(cell, WEST, "|", " ", ".");
      char mid = onRoute[cellIndex(cell)] ? '*' : known.isGoal(cell) ? 'G' : known.isVisited(cell) ? ' ' : '?';
      line += ' ';
      line += mid;
      line += ' ';
    }
    line += wallStr(Cell{MAZE_SIZE - 1, y}, EAST, "|", " ", ".");
    printf("%s\n", line.c_str());
  }
}

struct Result {
  bool ok;
  bool skipped;
};

Result runOne(const char* name, const Maze& truth, bool verbose) {
  // The best possible run on the fully known maze.
  FloodFill ff;
  ff.runToGoal(truth, Unknown::BLOCKED);
  uint16_t trueCost = ff.cost(Cell{0, 0}, NORTH);
  if (trueCost == COST_INF) {
    printf("%-28s SKIP  goal unreachable in file\n", name);
    return {true, true};
  }

  Maze known;  // what the mouse knows: just the boundary to begin with
  known.clearGoals();
  for (int i = 0; i < truth.goalCount(); i++) known.addGoal(truth.goal(i));

  SimMouse mouse(truth);
  Explorer ex(known, mouse);
  bool searched = ex.fullSearch();
  const ExploreStats& st = ex.stats();
  Path run = ex.speedRunPath();

  bool poseAgrees = mouse.pos == ex.position() && mouse.heading == ex.heading();
  bool ok = searched && !mouse.crashed && poseAgrees && st.reachedGoal && st.backAtStart && run.valid &&
            run.cost >= trueCost && (!st.proven || run.cost == trueCost);

  // Replay the run on the true maze to be sure it drives through no walls.
  if (ok) {
    SimMouse replay(truth);
    ok = replay.runPath(run) && truth.isGoal(replay.pos);
  }

  int visited = 0;
  for (int i = 0; i < CELL_COUNT; i++) visited += known.isVisited(cellAt(i));

  char route[1024];
  pathToString(run, route, sizeof route);
  printf("%-28s %s  legs %d  cells %4d  turns %4d  visited %3d/256  %s  run %u (best %u)\n", name,
         ok ? "OK  " : "FAIL", st.legs, st.cellsMoved, st.turns, visited,
         st.proven ? "proven  " : "unproven", run.cost, trueCost);
  if (verbose) {
    printf("  route: %s\n", route);
    render(known, run);
  }
  if (!ok)
    printf("  searched=%d crashed=%d poseAgrees=%d goal=%d home=%d valid=%d\n", searched, mouse.crashed,
           poseAgrees, st.reachedGoal, st.backAtStart, run.valid);
  return {ok, false};
}

bool readFile(const char* path, std::string& out) {
  FILE* f = fopen(path, "rb");
  if (!f) return false;
  char buf[4096];
  size_t n;
  out.clear();
  while ((n = fread(buf, 1, sizeof buf, f)) > 0) out.append(buf, n);
  fclose(f);
  return true;
}

}  // namespace

int main(int argc, char** argv) {
  bool verbose = false;
  int randomCount = 0;
  uint32_t seed = 1;
  std::vector<const char*> files;

  for (int i = 1; i < argc; i++) {
    if (!strcmp(argv[i], "-v")) {
      verbose = true;
    } else if (!strcmp(argv[i], "--random") && i + 1 < argc) {
      randomCount = atoi(argv[++i]);
      if (i + 1 < argc && argv[i + 1][0] != '-') seed = uint32_t(strtoul(argv[++i], nullptr, 10));
    } else {
      files.push_back(argv[i]);
    }
  }
  if (files.empty() && randomCount == 0) {
    fprintf(stderr, "usage: %s [-v] maze.txt ... | [-v] --random N [seed]\n", argv[0]);
    return 2;
  }

  int total = 0, failed = 0, skipped = 0, unproven = 0;
  auto tally = [&](Result r) {
    total++;
    if (!r.ok) failed++;
    if (r.skipped) skipped++;
  };

  for (const char* path : files) {
    std::string text;
    Maze truth;
    const char* base = strrchr(path, '/');
    base = base ? base + 1 : path;
    if (!readFile(path, text) || !parseMazeText(text.c_str(), truth)) {
      printf("%-28s SKIP  not a 16x16 maze file\n", base);
      total++;
      skipped++;
      continue;
    }
    tally(runOne(base, truth, verbose));
  }

  for (int i = 0; i < randomCount; i++) {
    Maze truth;
    generateMaze(truth, seed + uint32_t(i));
    char name[32];
    snprintf(name, sizeof name, "random-%u", unsigned(seed + uint32_t(i)));
    tally(runOne(name, truth, verbose));
  }
  (void)unproven;

  printf("\n%d mazes, %d failed, %d skipped\n", total, failed, skipped);
  return failed ? 1 : 0;
}
