#pragma once
// Keep the maze map in flash (NVS) so a reset or battery swap between the
// search and the speed run doesn't lose it.

#include <maze.h>

void storageSaveMaze(const mm::Maze& maze);
bool storageLoadMaze(mm::Maze& maze);  // false if nothing saved
void storageClearMaze();
