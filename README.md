# A* Pathfinding on a Weighted Grid (C++)

A small, dependency-free C++17 implementation of A* pathfinding, the algorithm game AI uses to move characters around a map. It supports weighted terrain, 4-way or 8-way movement, and a side-by-side comparison with Dijkstra's algorithm to show how much work a good heuristic saves.

## Demo

```
$ ./astar maps/demo.txt --compare
== A* ==
##############################
#S******.#..........~~~~.....#
#......*.#.*********~~~~.....#
#..####*.#.*######.*~~~~..#..#
#..#.***.#.*.....#.*~~~~..#..#
#..#.*####.*###..#.*****..#..#
#..#.*******#....#####.*..#..#
#..######...#........#.*..#..#
#.......#...#####....#.*###..#
#.......#.......#....#.******#
#..^^^..#####...#....######.*#
#..^^^......#...#.........#.*#
#..^^^......#...######....#.G#
#...........#.............#..#
##############################
Path cost: 50.00 | Steps: 50 | Cells explored: 216

== Dijkstra (no heuristic) ==
Path cost: 50.00 | Steps: 50 | Cells explored: 279

Same path cost: yes. A* explored 23% fewer cells.
```

Both algorithms find a path with the same optimal cost, but A* gets there by exploring fewer cells because the heuristic steers it toward the goal.

## Features

- **Weighted terrain:** open floor costs 1, water (`~`) costs 3 and hills (`^`) cost 5, so the agent will take a longer route to avoid slow ground.
- **4-way or 8-way movement:** diagonal steps cost √2, and characters can't cut diagonally between two walls.
- **Admissible heuristics:** Manhattan distance for 4-way movement and octile distance for 8-way movement. Neither ever overestimates the real cost, so A* is guaranteed to find the shortest path.
- **Dijkstra mode:** turns the heuristic off so you can compare the two algorithms.
- **Unit tests:** cover corridors, blocked maps, terrain avoidance, diagonal cost, corner cutting, A* vs. Dijkstra agreement and invalid input.

## Build and run

You need a C++17 compiler (g++ or clang++) and `make`.

```
make            # builds ./astar and ./tests
make test       # runs the unit tests
make demo       # runs the A* vs. Dijkstra comparison on the demo map

./astar maps/demo.txt              # A*, 4-way movement
./astar maps/demo.txt --diagonal   # A*, 8-way movement
./astar maps/demo.txt --dijkstra   # Dijkstra, no heuristic
./astar maps/demo.txt --compare    # both, side by side
```

On Windows you can use MinGW or WSL, or compile directly with
`g++ -std=c++17 -O2 main.cpp -o astar`.

## Map format

Maps are plain text files. Each character is one cell:

| Char | Meaning | Cost |
| --- | --- | --- |
| `S` | Start | 1 |
| `G` | Goal | 1 |
| `.` | Open floor | 1 |
| `~` | Water | 3 |
| `^` | Hills | 5 |
| `#` | Wall | impassable |

## How it works

1. Start with the start cell in an **open list** (a priority queue sorted by `f = g + h`).
   - `g` is the real cost of the path so far.
   - `h` is the heuristic's estimate of the cost left to reach the goal.
2. Repeatedly take the cell with the lowest `f`, mark it **closed** and look at its neighbors.
3. If going through the current cell gives a neighbor a cheaper `g` than before, record the new cost and the parent, then push the neighbor onto the open list.
4. When the goal is taken off the open list, follow the parents back to the start to rebuild the path.

Instead of updating priorities in place, the code pushes duplicate entries and skips any that are already closed ("lazy deletion"). This keeps the code simple and is a common approach in practice.

## Project structure

```
pathfinder.hpp   Grid, A* search, heuristics, path rendering
main.cpp         Command-line demo
tests.cpp        Unit tests
maps/demo.txt    Example map
Makefile
```

## Next steps

- [ ] Port the search into an Unreal Engine project and drive an NPC with a Behavior Tree
- [ ] Add path smoothing so movement looks natural instead of grid-locked
- [ ] Add a jump point search (JPS) mode for faster searches on uniform-cost grids
- [ ] Visualize the explored cells, not just the final path
