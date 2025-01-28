#ifndef ALGORITHM_H
#define ALGORITHM_H

#include <limits>
#include <memory_resource>

#include "puzzle.h"

namespace stp::algorithm {

constexpr int kFound = -1;

constexpr int kInf = std::numeric_limits<int>::max();

template <uint8_t width, uint8_t height>
class IDAStar {
public:
    explicit IDAStar(const Puzzle<width, height>& puzzle) : puzzle_(puzzle) {}

    auto& Solution() { return solution_path_; }

    auto NodeExpanded() { return node_expanded_; }

    bool operator()(State<width, height>& state) {
        solution_path_.clear();
        node_expanded_ = 0;
        solution_path_.emplace_back(state);
        int bound = puzzle_.HCost(state);
        while (true) {
            const auto t = Search(state, 0, bound);
            if (t == kFound) {
                return true;
            }
            if (t == kInf) {
                return false;
            }
            bound = t;
        }
    }

private:
    using State = State<width, height>;
    using Puzzle = Puzzle<width, height>;
    const Puzzle& puzzle_;

    std::pmr::unsynchronized_pool_resource pool_{};
    std::deque<State> solution_path_{};
    uint64_t node_expanded_{};

    int Search(State& state, int g, const int bound, const Action& last_action = {}) {
        if (const int f = g + puzzle_.HCost(state); f > bound) {
            return f;
        }
        if (puzzle_.GoalTest(state)) {
            return kFound;
        }
        ++node_expanded_;
        int min_cost = kInf;
        std::pmr::vector<Action> actions(&pool_);
        puzzle_.GetActions(state, actions);
        for (const auto& action : actions) {
            if (action == last_action.Reverse()) {
                continue;
            }
            Puzzle::ApplyAction(state, action);
            if (std::ranges::find(solution_path_, state) == solution_path_.end()) {
                solution_path_.emplace_back(state);
                const auto t = Search(state, g + 1, bound, action);
                if (t == kFound) {
                    return kFound;
                }
                if (t < min_cost) {
                    min_cost = t;
                }
                solution_path_.pop_back();
            }
            Puzzle::UndoAction(state, action);
        }
        return min_cost;
    }
};
}  // namespace stp::algorithm

#endif  // ALGORITHM_H
