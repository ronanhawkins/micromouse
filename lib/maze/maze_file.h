#pragma once
// Ground-truth mazes for the simulator and tests.
//
// Text format (micromouseonline/mazefiles): posts are 'o', horizontal walls
// '---', vertical walls '|', goal cells marked 'G', start cell 'S'. The first
// line is the north edge of the maze.

#include <stdint.h>

#include "maze.h"

namespace mm {

// Parse a 16x16 maze from text. Every wall ends up known. Goal cells come
// from 'G' markers, or the classic centre if there are none. Returns false
// if the text isn't a 16x16 maze.
bool parseMazeText(const char* text, Maze& out);

// Write a maze in the same format. `buf` needs at least 33 * 67 + 1 bytes.
constexpr int MAZE_TEXT_BYTES = (2 * MAZE_SIZE + 1) * (4 * MAZE_SIZE + 2) + 1;
void mazeToText(const Maze& maze, char* buf);

// Random legal classic maze:
//  - start cell walled on three sides, open to the north
//  - 2x2 goal in the centre with exactly one entrance
//  - every cell reachable, with `extraOpenings` walls removed to add loops
void generateMaze(Maze& out, uint32_t seed, int extraOpenings = 20);

}  // namespace mm
