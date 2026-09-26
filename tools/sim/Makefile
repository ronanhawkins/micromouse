# Desktop simulator for the maze logic in lib/maze.
#   make -C tools/sim
#   ./tools/sim/sim --random 200
#   ./tools/sim/sim third_party/mazefiles/classic/*.txt   (after tools/fetch_mazes.sh)

CXX ?= c++
CXXFLAGS ?= -std=c++17 -O2 -Wall -Wextra -Werror
MAZE_DIR := ../../lib/maze
SRCS := sim_main.cpp $(wildcard $(MAZE_DIR)/*.cpp)

sim: $(SRCS) $(wildcard $(MAZE_DIR)/*.h)
	$(CXX) $(CXXFLAGS) -I$(MAZE_DIR) -o $@ $(SRCS)

clean:
	rm -f sim

.PHONY: clean
