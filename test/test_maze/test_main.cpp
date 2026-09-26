// Unit tests for lib/maze. Run with: pio test -e native

#include <stdio.h>
#include <string.h>
#include <unity.h>

#include "explorer.h"
#include "maze_file.h"

using namespace mm;

void setUp() {}
void tearDown() {}

namespace {

// Perfect-sensing mouse on a ground-truth maze (same idea as tools/sim).
class TruthMouse : public MouseIO {
 public:
  explicit TruthMouse(const Maze& t) : truth(t) {}
  WallReading senseWalls() override {
    return {truth.hasWall(pos, leftOf(h)), truth.hasWall(pos, h), truth.hasWall(pos, rightOf(h))};
  }
  bool forward(int n) override {
    for (int i = 0; i < n; i++) {
      if (truth.hasWall(pos, h)) return crashed = true, false;
      pos = neighbour(pos, h);
    }
    return true;
  }
  bool turn(Move::Type t) override {
    h = t == Move::TURN_LEFT ? leftOf(h) : t == Move::TURN_RIGHT ? rightOf(h) : behind(h);
    return true;
  }
  const Maze& truth;
  Cell pos{0, 0};
  Heading h = NORTH;
  bool crashed = false;
};

}  // namespace

void test_new_maze_has_boundary_and_start_walls() {
  Maze m;
  TEST_ASSERT_TRUE(m.hasWall(Cell{0, 0}, WEST));
  TEST_ASSERT_TRUE(m.hasWall(Cell{0, 0}, SOUTH));
  TEST_ASSERT_TRUE(m.hasWall(Cell{0, 0}, EAST));
  TEST_ASSERT_TRUE(m.hasWall(Cell{1, 0}, WEST));  // same wall seen from the neighbour
  TEST_ASSERT_FALSE(m.isKnown(Cell{0, 0}, NORTH));
  TEST_ASSERT_TRUE(m.hasWall(Cell{15, 15}, NORTH));
  TEST_ASSERT_TRUE(m.hasWall(Cell{15, 15}, EAST));
  TEST_ASSERT_EQUAL(4, m.goalCount());
  TEST_ASSERT_TRUE(m.isGoal(Cell{7, 7}));
  TEST_ASSERT_TRUE(m.isGoal(Cell{8, 8}));
}

void test_set_wall_updates_both_sides() {
  Maze m;
  m.setWall(Cell{3, 4}, NORTH, true);
  TEST_ASSERT_TRUE(m.hasWall(Cell{3, 5}, SOUTH));
  TEST_ASSERT_TRUE(m.isKnown(Cell{3, 5}, SOUTH));
  m.setWall(Cell{3, 5}, SOUTH, false);
  TEST_ASSERT_FALSE(m.hasWall(Cell{3, 4}, NORTH));
  TEST_ASSERT_TRUE(m.canMove(Cell{3, 4}, NORTH, Unknown::BLOCKED));
  TEST_ASSERT_TRUE(m.canMove(Cell{3, 4}, EAST, Unknown::OPEN));
  TEST_ASSERT_FALSE(m.canMove(Cell{3, 4}, EAST, Unknown::BLOCKED));
}

void test_floodfill_open_maze_prefers_fewer_turns() {
  Maze m;
  FloodFill ff;
  ff.runToGoal(m, Unknown::OPEN);
  Costs k;
  // From (0,0) facing north the start's east wall forces: north 7, turn, east 7.
  TEST_ASSERT_EQUAL_UINT16(14 * k.straight + k.turn90, ff.cost(Cell{0, 0}, NORTH));
  TEST_ASSERT_EQUAL_UINT16(0, ff.cost(Cell{7, 7}, SOUTH));
  // Straight ahead is preferred over an equally short staircase.
  TEST_ASSERT_EQUAL(NORTH, ff.bestExit(m, Cell{0, 0}, NORTH, Unknown::OPEN));

  Path p = planPath(m, ff, Cell{0, 0}, NORTH, Unknown::OPEN);
  TEST_ASSERT_TRUE(p.valid);
  char s[64];
  pathToString(p, s, sizeof s);
  TEST_ASSERT_EQUAL_STRING("F7 R F7", s);
}

void test_unknown_blocked_means_unreachable_on_fresh_map() {
  Maze m;
  FloodFill ff;
  ff.runToGoal(m, Unknown::BLOCKED);
  TEST_ASSERT_EQUAL_UINT16(COST_INF, ff.cost(Cell{0, 0}, NORTH));
  Path p = planPath(m, ff, Cell{0, 0}, NORTH, Unknown::BLOCKED);
  TEST_ASSERT_FALSE(p.valid);
}

void test_parse_and_write_round_trip() {
  Maze a;
  generateMaze(a, 42);
  static char text[MAZE_TEXT_BYTES];
  mazeToText(a, text);

  Maze b;
  TEST_ASSERT_TRUE(parseMazeText(text, b));
  for (int i = 0; i < CELL_COUNT; i++) TEST_ASSERT_EQUAL_HEX8(a.raw(i), b.raw(i));
  TEST_ASSERT_EQUAL(4, b.goalCount());

  // CRLF line endings and a header line are tolerated.
  static char crlf[MAZE_TEXT_BYTES * 2 + 32];
  char* w = crlf;
  w += sprintf(w, "some header\r\n");
  for (const char* r = text; *r; r++) {
    if (*r == '\n') *w++ = '\r';
    *w++ = *r;
  }
  *w = 0;
  Maze c;
  TEST_ASSERT_TRUE(parseMazeText(crlf, c));
  for (int i = 0; i < CELL_COUNT; i++) TEST_ASSERT_EQUAL_HEX8(a.raw(i), c.raw(i));

  TEST_ASSERT_FALSE(parseMazeText("o---o\n|   |\no---o\n", c));
}

void test_generated_mazes_are_legal() {
  for (uint32_t seed = 1; seed <= 50; seed++) {
    Maze m;
    generateMaze(m, seed);
    TEST_ASSERT_TRUE(m.hasWall(Cell{0, 0}, EAST));
    TEST_ASSERT_FALSE(m.hasWall(Cell{0, 0}, NORTH));
    int entrances = 0;
    for (int i = 0; i < m.goalCount(); i++)
      for (int h = 0; h < 4; h++) {
        Cell n = neighbour(m.goal(i), Heading(h));
        if (inBounds(n) && !m.isGoal(n) && !m.hasWall(m.goal(i), Heading(h))) entrances++;
      }
    TEST_ASSERT_EQUAL(1, entrances);
  }
}

void test_explorer_solves_generated_mazes_optimally() {
  for (uint32_t seed = 1; seed <= 100; seed++) {
    Maze truth;
    generateMaze(truth, seed);
    FloodFill best;
    best.runToGoal(truth, Unknown::BLOCKED);

    Maze known;
    TruthMouse mouse(truth);
    Explorer ex(known, mouse);
    TEST_ASSERT_TRUE(ex.fullSearch(8));
    TEST_ASSERT_FALSE(mouse.crashed);
    TEST_ASSERT_TRUE(ex.stats().reachedGoal);
    TEST_ASSERT_TRUE(ex.stats().backAtStart);
    TEST_ASSERT_EQUAL(NORTH, mouse.h);
    TEST_ASSERT_TRUE(mouse.pos == ex.position());

    Path run = ex.speedRunPath();
    TEST_ASSERT_TRUE(run.valid);
    if (ex.stats().proven) TEST_ASSERT_EQUAL_UINT32(best.cost(Cell{0, 0}, NORTH), run.cost);

    TruthMouse replay(truth);
    TEST_ASSERT_TRUE(replay.runPath(run));
    TEST_ASSERT_TRUE(truth.isGoal(replay.pos));
  }
}

void test_explorer_reports_walled_off_goal() {
  Maze truth;
  truth.clearAllKnown();
  truth.setGoalCentre();
  truth.setWall(Cell{0, 0}, EAST, true);
  truth.setWall(Cell{0, 0}, NORTH, true);  // start fully enclosed

  Maze known;
  TruthMouse mouse(truth);
  Explorer ex(known, mouse);
  TEST_ASSERT_FALSE(ex.searchToGoal());
  TEST_ASSERT_FALSE(ex.stats().reachedGoal);
}

int main(int, char**) {
  UNITY_BEGIN();
  RUN_TEST(test_new_maze_has_boundary_and_start_walls);
  RUN_TEST(test_set_wall_updates_both_sides);
  RUN_TEST(test_floodfill_open_maze_prefers_fewer_turns);
  RUN_TEST(test_unknown_blocked_means_unreachable_on_fresh_map);
  RUN_TEST(test_parse_and_write_round_trip);
  RUN_TEST(test_generated_mazes_are_legal);
  RUN_TEST(test_explorer_solves_generated_mazes_optimally);
  RUN_TEST(test_explorer_reports_walled_off_goal);
  return UNITY_END();
}
