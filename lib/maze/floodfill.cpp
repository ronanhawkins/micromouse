#include "floodfill.h"

namespace mm {

namespace {

// Small binary min-heap of (cost, state) with lazy deletion. A state is
// pushed at most once per incoming edge (4 per state) plus once as a target.
struct Heap {
  static constexpr int CAP = CELL_COUNT * 4 * 4 + 32;
  uint32_t items[CAP];  // (cost << 16) | state
  int size = 0;

  void push(uint16_t cost, uint16_t state) {
    int i = size++;
    uint32_t v = (uint32_t(cost) << 16) | state;
    while (i > 0) {
      int p = (i - 1) / 2;
      if (items[p] <= v) break;
      items[i] = items[p];
      i = p;
    }
    items[i] = v;
  }

  uint32_t pop() {
    uint32_t top = items[0];
    uint32_t v = items[--size];
    int i = 0;
    for (;;) {
      int l = 2 * i + 1;
      if (l >= size) break;
      int m = (l + 1 < size && items[l + 1] < items[l]) ? l + 1 : l;
      if (items[m] >= v) break;
      items[i] = items[m];
      i = m;
    }
    items[i] = v;
    return top;
  }
};

// Static so it doesn't live on a small embedded task stack.
Heap heap;

uint16_t turnCost(const Costs& k, Heading from, Heading to) {
  int d = (to - from) & 3;
  if (d == 0) return 0;
  if (d == 2) return k.turn180;
  return k.turn90;
}

}  // namespace

void FloodFill::run(const Maze& maze, const Cell* targets, int targetCount, Unknown mode,
                    const Costs& costs) {
  costs_ = costs;
  for (int i = 0; i < CELL_COUNT; i++)
    for (int h = 0; h < 4; h++) cost_[i][h] = COST_INF;

  heap.size = 0;
  for (int t = 0; t < targetCount; t++) {
    int idx = cellIndex(targets[t]);
    for (int h = 0; h < 4; h++) {
      cost_[idx][h] = 0;
      heap.push(0, uint16_t(idx * 4 + h));
    }
  }

  auto relax = [&](int idx, int h, uint32_t c) {
    if (c < cost_[idx][h]) {
      cost_[idx][h] = uint16_t(c);
      if (heap.size < Heap::CAP) heap.push(uint16_t(c), uint16_t(idx * 4 + h));
    }
  };

  // Dijkstra on the reversed graph: find every state that can reach the
  // popped one in a single action.
  while (heap.size > 0) {
    uint32_t item = heap.pop();
    uint16_t d = item >> 16;
    int state = item & 0xFFFF;
    int idx = state / 4;
    Heading h = Heading(state % 4);
    if (d != cost_[idx][h]) continue;  // stale entry
    Cell c = cellAt(idx);

    // Turning in place into heading h.
    relax(idx, leftOf(h), uint32_t(d) + costs.turn90);
    relax(idx, rightOf(h), uint32_t(d) + costs.turn90);
    relax(idx, behind(h), uint32_t(d) + costs.turn180);

    // Driving forward along h into c from the cell behind.
    Cell p = neighbour(c, behind(h));
    if (inBounds(p) && maze.canMove(p, h, mode)) relax(cellIndex(p), h, uint32_t(d) + costs.straight);
  }
}

void FloodFill::runToGoal(const Maze& maze, Unknown mode, const Costs& costs) {
  Cell goals[8];
  for (int i = 0; i < maze.goalCount(); i++) goals[i] = maze.goal(i);
  run(maze, goals, maze.goalCount(), mode, costs);
}

uint32_t FloodFill::exitCost(const Maze& maze, Cell c, Heading h, Heading exitDir,
                             Unknown mode) const {
  if (!maze.canMove(c, exitDir, mode)) return COST_INF;
  uint16_t rest = cost(neighbour(c, exitDir), exitDir);
  if (rest == COST_INF) return COST_INF;
  return uint32_t(turnCost(costs_, h, exitDir)) + costs_.straight + rest;
}

int FloodFill::bestExit(const Maze& maze, Cell c, Heading h, Unknown mode) const {
  const Heading order[4] = {h, rightOf(h), leftOf(h), behind(h)};
  int best = -1;
  uint32_t bestCost = COST_INF;
  for (Heading d : order) {
    uint32_t cc = exitCost(maze, c, h, d, mode);
    if (cc < bestCost) {
      bestCost = cc;
      best = d;
    }
  }
  return best;
}

}  // namespace mm
