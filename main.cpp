// main.cpp
// Command-line demo for the A* pathfinder.
//
// Usage:
//   ./astar maps/demo.txt              A* with 4-way movement
//   ./astar maps/demo.txt --diagonal   allow 8-way movement
//   ./astar maps/demo.txt --dijkstra   run without a heuristic
//   ./astar maps/demo.txt --compare    run A* and Dijkstra side by side

#include <cstdio>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

#include "pathfinder.hpp"

static std::vector<std::string> loadMap(const std::string& path) {
    std::ifstream in(path);
    if (!in) throw std::runtime_error("Could not open map file: " + path);
    std::vector<std::string> rows;
    std::string line;
    while (std::getline(in, line)) {
        if (!line.empty() && line.back() == '\r') line.pop_back();  // Windows line endings
        if (!line.empty()) rows.push_back(line);
    }
    return rows;
}

static void printResult(const char* label, const Grid& grid, const SearchResult& r, bool drawMap) {
    std::cout << "== " << label << " ==\n";
    if (!r.found) {
        std::cout << "No path found. Cells explored: " << r.nodesExpanded << "\n\n";
        return;
    }
    if (drawMap) {
        for (const auto& row : renderPath(grid, r)) std::cout << row << '\n';
    }
    std::printf("Path cost: %.2f | Steps: %zu | Cells explored: %d\n\n",
                r.cost, r.path.size() - 1, r.nodesExpanded);
}

int main(int argc, char** argv) {
    if (argc < 2) {
        std::cerr << "Usage: " << argv[0] << " <map.txt> [--diagonal] [--dijkstra] [--compare]\n";
        return 1;
    }

    bool diagonal = false, dijkstra = false, compare = false;
    for (int i = 2; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "--diagonal") diagonal = true;
        else if (arg == "--dijkstra") dijkstra = true;
        else if (arg == "--compare") compare = true;
        else { std::cerr << "Unknown option: " << arg << '\n'; return 1; }
    }

    try {
        Grid grid(loadMap(argv[1]));

        if (compare) {
            SearchResult a = findPath(grid, diagonal, true);
            SearchResult d = findPath(grid, diagonal, false);
            printResult("A*", grid, a, true);
            printResult("Dijkstra (no heuristic)", grid, d, false);
            if (a.found && d.found) {
                std::printf("Same path cost: %s. A* explored %.0f%% fewer cells.\n",
                            std::abs(a.cost - d.cost) < 1e-9 ? "yes" : "NO",
                            100.0 * (1.0 - double(a.nodesExpanded) / d.nodesExpanded));
            }
        } else {
            SearchResult r = findPath(grid, diagonal, !dijkstra);
            printResult(dijkstra ? "Dijkstra" : "A*", grid, r, true);
        }
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << '\n';
        return 1;
    }
    return 0;
}
