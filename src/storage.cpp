#include "storage.h"

#include <Preferences.h>

namespace {
constexpr const char* NS = "mouse";
constexpr const char* KEY = "maze";
}

void storageSaveMaze(const mm::Maze& maze) {
  uint8_t buf[mm::CELL_COUNT];
  for (int i = 0; i < mm::CELL_COUNT; i++) buf[i] = maze.raw(i);
  Preferences p;
  p.begin(NS, false);
  p.putBytes(KEY, buf, sizeof buf);
  p.end();
}

bool storageLoadMaze(mm::Maze& maze) {
  uint8_t buf[mm::CELL_COUNT];
  Preferences p;
  p.begin(NS, true);
  size_t n = p.getBytes(KEY, buf, sizeof buf);
  p.end();
  if (n != sizeof buf) return false;
  for (int i = 0; i < mm::CELL_COUNT; i++) maze.setRaw(i, buf[i]);
  return true;
}

void storageClearMaze() {
  Preferences p;
  p.begin(NS, false);
  p.remove(KEY);
  p.end();
}
