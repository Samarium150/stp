#ifndef IDA_STAR_H_
#define IDA_STAR_H_

#include <deque>
#include <functional>
#include <limits>
#include <memory_resource>

#include "puzzle.h"

namespace stp::algorithm {

constexpr auto kFound = -1;

constexpr auto kInf = std::numeric_limits<int>::max();

template <uint8_t width, uint8_t height>
class IDAStar {
public:
    explicit IDAStar(const Puzzle<width, height>& puzzle)
        : puzzle_(puzzle),
          heuristic_([this](const State& state) { return puzzle_.HCost(state); }) {}

    auto& Solution() { return solution_path_; }

    auto NodeExpanded() { return node_expanded_; }

    auto& Heuristic() { return heuristic_; }

    bool operator()(State<width, height>& state) {
        solution_path_.clear();
        node_expanded_ = 0;
        solution_path_.emplace_back(state);
        auto bound = heuristic_(state);
        while (true) {
            const auto t = Search(state, 0u, bound);
            if (t == kFound) {
                pool_.release();
                return true;
            }
            if (t == kInf) {
                pool_.release();
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
    std::function<unsigned(const State&)> heuristic_ = [](const State&) { return 0; };

    int Search(State& state, const unsigned g, const unsigned bound,
               const Action& last_action = {}) {
        if (const int f = g + heuristic_(state); f > static_cast<int>(bound)) {
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
            const auto edge_cost = puzzle_.ApplyAction(state, action);
            if (std::ranges::find(solution_path_, state) == solution_path_.end()) {
                solution_path_.emplace_back(state);
                const auto t = Search(state, g + edge_cost, bound, action);
                if (t == kFound) {
                    return kFound;
                }
                if (t < min_cost) {
                    min_cost = t;
                }
                solution_path_.pop_back();
            }
            puzzle_.UndoAction(state, action);
        }
        return min_cost;
    }
};
}  // namespace stp::algorithm

#endif  // IDA_STAR_H_
