#ifndef ALGORITHM_H
#define ALGORITHM_H

#include <limits>

#include "puzzle.h"

namespace stp::algorithm {

constexpr int kFound = -1;

template <uint8_t width, uint8_t height>
int IDAStarSearch(const Puzzle<width, height>& puzzle, int g, int bound,
                  std::vector<State<width, height>>& path) {
    const auto& state = path.back();
    if (const int f = g + puzzle.HCost(state); f > bound) {
        return f;
    }
    if (puzzle.GoalTest(state)) {
        return kFound;
    }

    int min_cost = std::numeric_limits<int>::max();
    for (const auto& successor : puzzle.GetSuccessors(state)) {
        if (std::ranges::find(path, successor) == path.end()) {
            path.push_back(successor);
            const auto t = IDAStarSearch(puzzle, g + 1, bound, path);
            if (t == kFound) {
                return kFound;
            }
            if (t < min_cost) {
                min_cost = t;
            }
            path.pop_back();
        }
    }
    return min_cost;
}

template <uint8_t width, uint8_t height>
bool IDAStar(const Puzzle<width, height>& puzzle, const State<width, height>& state,
             std::vector<State<width, height>>& path) {
    path.clear();
    path.push_back(state);
    int bound = puzzle.HCost(state);
    while (true) {
        const auto t = IDAStarSearch(puzzle, 0, bound, path);
        if (t == kFound) {
            return true;
        }
        if (t == std::numeric_limits<int>::max()) {
            return false;
        }
        bound = t;
    }
}
}  // namespace stp::algorithm

#endif  // ALGORITHM_H
