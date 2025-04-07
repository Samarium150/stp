#ifndef A_STAR_H_
#define A_STAR_H_

#include "puzzle.h"
#include "ranking.h"

namespace stp::algorithm {

template <uint8_t width, uint8_t height>
class AStar {
public:
    explicit AStar(const Puzzle<width, height>& puzzle) noexcept
        : puzzle_(puzzle),
          heuristic_([this](const State& state) { return puzzle_.get().HCost(state); }) {}

    auto& Solution() const noexcept { return solution_path_; }

    auto NodeExpanded() const noexcept { return node_expanded_; }

    void SetHeuristic(std::function<unsigned(const State<width, height>&)> heuristic) noexcept {
        heuristic_ = std::move(heuristic);
    }

    void SetPhi(std::function<unsigned(unsigned, unsigned)> phi) noexcept { phi_ = std::move(phi); }

    bool operator()(State<width, height>& state) noexcept {
        const auto puzzle = puzzle_.get();
        if (puzzle.GoalTest(state)) {
            return true;
        }
        std::priority_queue<Node, std::vector<Node>, std::greater<>> open;
        std::unordered_map<uint64_t, unsigned> closed;
        std::unordered_map<uint64_t, std::pair<uint64_t, Action>> came_from;
        std::vector<Action> actions;
        open.emplace(util::Ranking(state), 0, phi_(heuristic_(state), 0));
        const auto rank = util::Ranking(state);
        came_from.emplace(std::make_pair(rank, std::pair{rank, Action{}}));
        while (!open.empty()) {
            auto [current_rank, current_g, current_f] = open.top();
            open.pop();
            auto current_state = State();
            util::UnRanking(current_rank, current_state);
            if (puzzle.GoalTest(current_state)) {
                while (came_from.contains(current_rank)) {
                    auto& [prev_rank, prev_action] = came_from[current_rank];
                    if (prev_rank == current_rank) {
                        break;
                    }
                    solution_path_.push_back(prev_action);
                    current_rank = prev_rank;
                }
                std::ranges::reverse(solution_path_);
                return true;
            }
            auto closed_it = closed.find(current_rank);
            if (closed_it != closed.end() && closed_it->second <= current_g) {
                continue;
            }
            closed[current_rank] = current_g;
            ++node_expanded_;
            puzzle.GetActions(current_state, actions);
            for (const auto& action : actions) {
                const auto edge_cost = puzzle.ApplyAction(current_state, action);
                const auto successor_g = current_g + edge_cost;
                const auto successor_h = heuristic_(current_state);
                const auto successor_f = phi_(successor_h, successor_g);
                const auto successor_rank = util::Ranking(current_state);
                open.emplace(successor_rank, successor_g, successor_f);
                if (!came_from.contains(successor_rank) || successor_g < closed[successor_rank]) {
                    came_from[successor_rank] = {current_rank, action};
                }
                puzzle.UndoAction(current_state, action);
            }
            actions.clear();
        }
        return false;
    }

private:
    using State = State<width, height>;
    using Puzzle = Puzzle<width, height>;

    std::reference_wrapper<const Puzzle> puzzle_;

    std::function<unsigned(const State&)> heuristic_ = [](const State&) { return 0; };
    std::function<unsigned(unsigned, unsigned)> phi_ = [](const unsigned h, const unsigned g) {
        return g + h;
    };

    std::vector<Action> solution_path_{};
    uint64_t node_expanded_{};

    struct Node {
        uint64_t rank{};
        unsigned g{};
        unsigned f{};

        bool operator>(const Node& other) const { return f > other.f; }
    };
};
}  // namespace stp::algorithm
#endif  // A_STAR_H_
