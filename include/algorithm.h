#ifndef ALGORITHM_H
#define ALGORITHM_H

#include <limits>

#include "puzzle.h"

namespace stp::algorithm {

constexpr int kFound = -1;

constexpr int kInf = std::numeric_limits<int>::max();

template <uint8_t width, uint8_t height>
class IDAStar {
public:
    explicit IDAStar(const Puzzle<width, height>& puzzle) : puzzle_(puzzle) {}

    int Search(int g, const int bound) {
        const auto& state = path_.back();
        if (const int f = g + puzzle_.HCost(state); f > bound) {
            return f;
        }
        if (puzzle_.GoalTest(state)) {
            return kFound;
        }
        ++node_expanded_;
        int min_cost = kInf;
        for (const auto& successor : puzzle_.GetSuccessors(state)) {
            if (std::ranges::find(path_, successor) == path_.end()) {
                path_.push_back(successor);
                const auto t = Search(g + 1, bound);
                if (t == kFound) {
                    return kFound;
                }
                if (t < min_cost) {
                    min_cost = t;
                }
                path_.pop_back();
            }
        }
        return min_cost;
    }

    bool operator()(const State<width, height>& state) {
        path_.clear();
        path_.push_back(state);
        int bound = puzzle_.HCost(state);
        while (true) {
            const auto t = Search(0, bound);
            if (t == kFound) {
                return true;
            }
            if (t == kInf) {
                return false;
            }
            bound = t;
        }
    }

    auto Path() const { return path_; }

    auto NodeExpanded() const { return node_expanded_; }

private:
    const Puzzle<width, height>& puzzle_;
    std::vector<State<width, height>> path_{};
    uint64_t node_expanded_{};
};
}  // namespace stp::algorithm

#endif  // ALGORITHM_H
