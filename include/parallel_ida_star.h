#ifndef PARALLEL_IDA_STAR_H_
#define PARALLEL_IDA_STAR_H_

#include <barrier>
#include <deque>
#include <functional>
#include <numeric>

#include "concurrentqueue.h"
#include "puzzle.h"

namespace stp::algorithm {

template <uint8_t width, uint8_t height>
class ParallelIDAStar {
public:
    explicit ParallelIDAStar(const Puzzle<width, height>& puzzle)
        : puzzle_(puzzle),
          heuristic_([this](const State& state) { return puzzle_.get().HCost(state); }),
          num_threads_(std::thread::hardware_concurrency()) {}

    auto& Solution() { return solution_path_; }

    auto NodeExpanded() const { return node_expanded_.load(); }

    void SetHeuristic(std::function<unsigned(const State<width, height>&)> heuristic) {
        heuristic_ = std::move(heuristic);
    }

    bool operator()(State<width, height>& state) noexcept {
        solution_path_.clear();
        solution_found_.store(false, std::memory_order_relaxed);
        node_expanded_.store(0, std::memory_order_relaxed);
        if (puzzle_.get().GoalTest(state)) {
            return true;
        }

        tasks_units_.clear();
        InitTasks(state, solution_path_);
        solution_path_.clear();

        auto bound = heuristic_(state);
        if (tasks_units_.empty()) {
            while (true) {
                const auto t = Search(state, 0u, bound, solution_path_);
                if (t == kFound) {
                    return true;
                }
                if (t == kInf) {
                    return false;
                }
                bound = t;
            }
        }

        std::vector<std::jthread> threads;
        std::barrier barrier(num_threads_ + 1);
        Queue tasks;
        while (true) {
            for (std::size_t i = 0; i < tasks_units_.size(); ++i) {
                tasks.enqueue(i);
            }

            for (auto i = 0u; i < std::thread::hardware_concurrency(); ++i) {
                threads.emplace_back([this, state, bound, &tasks, &barrier] {
                    std::size_t index;
                    while (!solution_found_.load(std::memory_order_relaxed) &&
                           tasks.try_dequeue(index)) {
                        auto& task = tasks_units_[index];
                        State start_state = std::move(state);
                        std::deque<Action> path;
                        auto g = std::accumulate(
                            task.init_actions.begin(), task.init_actions.end(), 0u,
                            [this, &path, &start_state](const unsigned cost, const auto& action) {
                                path.emplace_back(action);
                                return cost + puzzle_.get().ApplyAction(start_state, action);
                            });
                        int t = Search(start_state, g, bound, path);
                        if (t == kFound) {
                            task.solution = std::move(path);
                            solution_found_.store(true, std::memory_order_relaxed);
                        } else {
                            task.next_bound = t;
                        }
                    }
                    barrier.arrive_and_wait();
                });
            }
            barrier.arrive_and_wait();
            threads.clear();

            int best_bound = kInf;
            for (auto& task : tasks_units_) {
                if (!task.solution.empty()) {
                    solution_path_ = std::move(task.solution);
                    return true;
                }
                best_bound = std::min(best_bound, task.next_bound);
            }
            if (best_bound == kInf) {
                return false;
            }
            bound = best_bound;
        }
        return false;
    }

private:
    using State = State<width, height>;
    using Puzzle = Puzzle<width, height>;
    using Queue = moodycamel::ConcurrentQueue<std::size_t>;

    std::reference_wrapper<const Puzzle> puzzle_;

    std::function<unsigned(const State&)> heuristic_ = [](const State&) { return 0; };
    std::size_t num_threads_;
    std::deque<Action> solution_path_;
    std::atomic_bool solution_found_{false};
    std::atomic_uint64_t node_expanded_{0};

    static constexpr int init_task_depth_ = 5;

    struct Task {
        std::array<Action, init_task_depth_> init_actions{};
        std::deque<Action> solution{};
        int next_bound = kInf;
    };
    std::vector<Task> tasks_units_;

    void InitTasks(State& state, std::deque<Action>& path, const Action& last_action = {},
                   const unsigned depth = 0) noexcept {
        if (depth == init_task_depth_) {
            Task task;
            std::copy(path.begin(), path.end(), task.init_actions.begin());
            tasks_units_.push_back(task);
            return;
        }
        auto& puzzle = puzzle_.get();
        std::vector<Action> actions;
        puzzle.GetActions(state, actions);
        for (const auto& action : actions) {
            if (action == last_action.Reverse()) {
                continue;
            }
            path.emplace_back(action);
            puzzle.ApplyAction(state, action);
            InitTasks(state, path, action, depth + 1);
            puzzle.UndoAction(state, action);
            path.pop_back();
        }
    }

    int Search(State& state, const unsigned g, const unsigned bound,
               std::deque<Action>& path) noexcept {
        if (solution_found_.load(std::memory_order_relaxed)) {
            return kInf;
        }
        if (const int f = g + heuristic_(state); f > static_cast<int>(bound)) {
            return f;
        }
        auto& puzzle = puzzle_.get();
        if (puzzle.GoalTest(state)) {
            return kFound;
        }
        node_expanded_.fetch_add(1, std::memory_order_relaxed);
        int min_cost = kInf;
        std::vector<Action> actions;
        puzzle.GetActions(state, actions);
        for (const auto& action : actions) {
            if (!path.empty() && action == path.back().Reverse()) {
                continue;
            }
            path.emplace_back(action);
            const auto edge_cost = puzzle.ApplyAction(state, action);
            const int t = Search(state, g + edge_cost, bound, path);
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

#endif  // PARALLEL_IDA_STAR_H_
