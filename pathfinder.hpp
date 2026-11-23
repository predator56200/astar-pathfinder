// pathfinder.hpp
// A* pathfinding on a 2D grid with weighted terrain.
//
// Map legend:
//   S = start      G = goal
//   . = open floor (cost 1)
//   ~ = water      (cost 3)
//   ^ = hills      (cost 5)
//   # = wall       (impassable)
//
// Setting useHeuristic = false turns A* into Dijkstra's algorithm,
// which is handy for showing why a good heuristic matters.

#pragma once

#include <algorithm>
#include <cmath>
#include <limits>
#include <queue>
#include <stdexcept>
#include <string>
#include <tuple>
#include <vector>

struct Point {
    int x = 0;
    int y = 0;
    bool operator==(const Point& other) const { return x == other.x && y == other.y; }
};

struct SearchResult {
    bool found = false;
    std::vector<Point> path;  // start..goal, inclusive
    double cost = 0.0;        // total movement cost of the path
    int nodesExpanded = 0;    // how many cells the search had to fully explore
};

class Grid {
public:
    explicit Grid(const std::vector<std::string>& rows) : rows_(rows) {
        height_ = static_cast<int>(rows_.size());
        width_ = 0;
        for (const auto& r : rows_) width_ = std::max(width_, static_cast<int>(r.size()));

        bool hasStart = false, hasGoal = false;
        for (int y = 0; y < height_; ++y) {
            for (int x = 0; x < static_cast<int>(rows_[y].size()); ++x) {
                if (rows_[y][x] == 'S') { start = {x, y}; hasStart = true; }
                if (rows_[y][x] == 'G') { goal = {x, y}; hasGoal = true; }
            }
        }
        if (!hasStart || !hasGoal) throw std::runtime_error("Map needs one 'S' and one 'G'.");
    }

    int width() const { return width_; }
    int height() const { return height_; }

    bool inBounds(Point p) const { return p.x >= 0 && p.y >= 0 && p.x < width_ && p.y < height_; }

    // Ragged rows are allowed: anything past the end of a row counts as wall.
    char at(Point p) const {
        if (!inBounds(p) || p.x >= static_cast<int>(rows_[p.y].size())) return '#';
        return rows_[p.y][p.x];
    }

    bool walkable(Point p) const { return at(p) != '#'; }

    double terrainCost(Point p) const {
        switch (at(p)) {
            case '~': return 3.0;
            case '^': return 5.0;
            default:  return 1.0;  // '.', 'S', 'G'
        }
    }

    const std::vector<std::string>& rows() const { return rows_; }

    Point start, goal;

private:
    std::vector<std::string> rows_;
    int width_ = 0;
    int height_ = 0;
};

// Estimated cost from a to b. The cheapest terrain costs 1, so these
// never overestimate the real cost (they are "admissible"), which is
// what guarantees A* still finds the shortest path.
inline double heuristic(Point a, Point b, bool diagonal) {
    double dx = std::abs(a.x - b.x);
    double dy = std::abs(a.y - b.y);
    if (!diagonal) return dx + dy;                       // Manhattan distance
    const double D2 = std::sqrt(2.0);
    return (dx + dy) + (D2 - 2.0) * std::min(dx, dy);    // Octile distance
}

inline SearchResult findPath(const Grid& grid, bool diagonal = false, bool useHeuristic = true) {
    const int W = grid.width();
    const int H = grid.height();
    const double INF = std::numeric_limits<double>::infinity();
    auto index = [W](Point p) { return p.y * W + p.x; };

    std::vector<double> gCost(W * H, INF);   // best known cost from start
    std::vector<int> parent(W * H, -1);      // for rebuilding the path
    std::vector<bool> closed(W * H, false);  // already fully explored

    // Open list: (f = g + h, h, cell index). Smallest f first; ties go to
    // the cell that looks closer to the goal, which keeps the search focused.
    using Entry = std::tuple<double, double, int>;
    std::priority_queue<Entry, std::vector<Entry>, std::greater<Entry>> open;

    auto h = [&](Point p) { return useHeuristic ? heuristic(p, grid.goal, diagonal) : 0.0; };

    gCost[index(grid.start)] = 0.0;
    open.emplace(h(grid.start), h(grid.start), index(grid.start));

    const int dx4[] = {1, -1, 0, 0};
    const int dy4[] = {0, 0, 1, -1};
    const int dxD[] = {1, 1, -1, -1};
    const int dyD[] = {1, -1, 1, -1};

    SearchResult result;

    while (!open.empty()) {
        auto [f, hc, current] = open.top();
        open.pop();
        if (closed[current]) continue;  // stale entry, a cheaper route was already found
        closed[current] = true;
        ++result.nodesExpanded;

        Point cp{current % W, current / W};
        if (cp == grid.goal) {
            result.found = true;
            result.cost = gCost[current];
            for (int i = current; i != -1; i = parent[i]) result.path.push_back({i % W, i / W});
            std::reverse(result.path.begin(), result.path.end());
            return result;
        }

        auto relax = [&](Point next, double stepMultiplier) {
            if (!grid.inBounds(next) || !grid.walkable(next)) return;
            int ni = index(next);
            if (closed[ni]) return;
            double newG = gCost[current] + grid.terrainCost(next) * stepMultiplier;
            if (newG < gCost[ni]) {
                gCost[ni] = newG;
                parent[ni] = current;
                open.emplace(newG + h(next), h(next), ni);
            }
        };

        for (int i = 0; i < 4; ++i) relax({cp.x + dx4[i], cp.y + dy4[i]}, 1.0);

        if (diagonal) {
            for (int i = 0; i < 4; ++i) {
                // No corner cutting: both side cells must be open, like a real
                // character that can't squeeze diagonally between two walls.
                Point sideA{cp.x + dxD[i], cp.y};
                Point sideB{cp.x, cp.y + dyD[i]};
                if (!grid.walkable(sideA) || !grid.walkable(sideB)) continue;
                relax({cp.x + dxD[i], cp.y + dyD[i]}, std::sqrt(2.0));
            }
        }
    }

    return result;  // found == false: no path exists
}

// Returns the map with the path drawn as '*' (start and goal stay as S and G).
inline std::vector<std::string> renderPath(const Grid& grid, const SearchResult& result) {
    std::vector<std::string> out = grid.rows();
    for (auto& row : out) row.resize(grid.width(), ' ');
    for (const Point& p : result.path) {
        char& c = out[p.y][p.x];
        if (c != 'S' && c != 'G') c = '*';
    }
    return out;
}
