// tests.cpp
// Small unit tests for the pathfinder. Run with: make test

#include <cmath>
#include <cstdio>

#include "pathfinder.hpp"

static int failures = 0;
static int passed = 0;

#define CHECK(cond, name)                                   \
    do {                                                    \
        if (cond) { ++passed; std::printf("  PASS  %s\n", name); } \
        else { ++failures; std::printf("  FAIL  %s\n", name); }    \
    } while (0)

static bool near(double a, double b) { return std::abs(a - b) < 1e-9; }

int main() {
    std::printf("Running pathfinder tests...\n");

    {   // Straight corridor: cost should equal the number of steps.
        Grid g({"S...G"});
        auto r = findPath(g);
        CHECK(r.found && near(r.cost, 4.0) && r.path.size() == 5, "straight corridor");
    }

    {   // A wall that fully blocks the goal means no path.
        Grid g({"S.#.G"});
        auto r = findPath(g);
        CHECK(!r.found && r.path.empty(), "blocked path returns not found");
    }

    {   // Expensive hills: the cheaper detour should win (4 vs 6).
        Grid g({"S^G",
                "..."});
        auto r = findPath(g);
        CHECK(r.found && near(r.cost, 4.0), "avoids expensive terrain when a detour is cheaper");
    }

    {   // Diagonal moves can't cut between two walls.
        Grid g({"S#",
                "#G"});
        CHECK(!findPath(g, true).found, "no diagonal corner cutting");
    }

    {   // Open 3x3 with diagonals: two diagonal steps.
        Grid g({"S..",
                "...",
                "..G"});
        auto r = findPath(g, true);
        CHECK(r.found && near(r.cost, 2.0 * std::sqrt(2.0)), "diagonal movement cost");
    }

    {   // A* and Dijkstra must agree on cost, and A* should explore no more cells.
        Grid g({"S.........",
                "......#...",
                ".#......#.",
                "......#...",
                "......#...",
                "......#...",
                ".........G"});
        for (bool diag : {false, true}) {
            auto a = findPath(g, diag, true);
            auto d = findPath(g, diag, false);
            CHECK(a.found && d.found && near(a.cost, d.cost),
                  diag ? "A* matches Dijkstra cost (8-way)" : "A* matches Dijkstra cost (4-way)");
            CHECK(a.nodesExpanded <= d.nodesExpanded,
                  diag ? "A* explores no more cells than Dijkstra (8-way)"
                       : "A* explores no more cells than Dijkstra (4-way)");
        }
    }

    {   // Maps without a start or goal are rejected.
        bool threw = false;
        try { Grid g({"...G"}); } catch (const std::runtime_error&) { threw = true; }
        CHECK(threw, "missing start throws an error");
    }

    std::printf("\n%d passed, %d failed\n", passed, failures);
    return failures == 0 ? 0 : 1;
}
