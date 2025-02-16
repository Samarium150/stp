#ifndef IDA_STAR_H_
#define IDA_STAR_H_

#include <deque>
#include <functional>
#include <memory_resource>

#include "puzzle.h"

namespace stp::algorithm {

template <uint8_t width, uint8_t height>
class IDAStar {
public:
    explicit IDAStar(const Puzzle<width, height>& puzzle)
        : puzzle_(puzzle),
          heuristic_([this](const State& state) { return puzzle_.get().HCost(state); }) {}

    auto& Solution() { return solution_path_; }

    auto NodeExpanded() { return node_expanded_; }

    void SetHeuristic(std::function<unsigned(const State<width, height>&)> heuristic) {
        heuristic_ = std::move(heuristic);
    }

    bool operator()(State<width, height>& state) noexcept {
        solution_path_.clear();
        node_expanded_ = 0;
        auto bound = heuristic_(state);
        while (true) {
            const auto t = Search(state, 0u, bound, solution_path_);
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
    std::reference_wrapper<const Puzzle> puzzle_;

    std::pmr::unsynchronized_pool_resource pool_{};
    std::deque<Action> solution_path_{};
    uint64_t node_expanded_{};
    std::function<unsigned(const State&)> heuristic_ = [](const State&) { return 0; };

    int Search(State& state, const unsigned g, const unsigned bound,
               std::deque<Action>& path) noexcept {
        if (const int f = g + heuristic_(state); f > static_cast<int>(bound)) {
            return f;
        }
        const auto& puzzle = puzzle_.get();
        if (puzzle.GoalTest(state)) {
            return kFound;
        }
        ++node_expanded_;
        int min_cost = kInf;
        std::pmr::vector<Action> actions(&pool_);
        puzzle.GetActions(state, actions);
        for (const auto& action : actions) {
            if (!path.empty() && action == path.back().Reverse()) {
                continue;
            }
            path.emplace_back(action);
            const auto edge_cost = puzzle.ApplyAction(state, action);
            const auto t = Search(state, g + edge_cost, bound, path);
            if (t == kFound) {
                return kFound;
            }
            if (t < min_cost) {
                min_cost = t;
            }
            path.pop_back();
            puzzle.UndoAction(state, action);
        }
        return min_cost;
    }
};
}  // namespace stp::algorithm

#endif  // IDA_STAR_H_
