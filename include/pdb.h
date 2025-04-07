/**
 * Copyright (c) 2025. Samarium
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#ifndef PDB_H_
#define PDB_H_

#include <barrier>
#include <numeric>

#include "blockingconcurrentqueue.h"
#include "packed_storage.h"
#include "puzzle.h"
#include "ranking.h"

namespace stp::algorithm {

template <uint8_t width, uint8_t height>
class PatternDatabase {
public:
    template <typename Container,
              std::enable_if_t<std::is_convertible_v<typename Container::value_type, uint8_t>,
                               bool> = true>
    PatternDatabase(Puzzle<width, height>& puzzle, const Container& pattern) : puzzle_(puzzle) {
        const auto pattern_size = static_cast<uint16_t>(pattern.size());
        if (pattern_size == 0 || pattern_size > width * height) {
            throw std::invalid_argument("the pattern size must be between 1 and the puzzle size");
        }
        if (std::ranges::find(pattern, 0) == pattern.end()) {
            throw std::invalid_argument("the pattern must contain a blank tile indicated by 0");
        }
        pattern_.assign(pattern.begin(), pattern.end());
        const uint16_t factorial_base = width * height;
        db_.Resize(FactorialUpperK(factorial_base, factorial_base - pattern_size),
                   std::numeric_limits<uint64_t>::max());
    }

    PatternDatabase(const PatternDatabase&) noexcept = default;

    PatternDatabase& operator=(const PatternDatabase&) noexcept = default;

    PatternDatabase(PatternDatabase&& pdb) noexcept = default;

    PatternDatabase& operator=(PatternDatabase&& pdb) noexcept = default;

    void BuildSeq(const bool is_additive = true) noexcept {
        auto& puzzle = puzzle_.get();
        puzzle.abstraction_has_edge_cost_ = !is_additive;

        constexpr auto MAX_COST = std::numeric_limits<uint8_t>::max();
        const auto size = db_.Size();

        const auto goal = GetPDBHash(puzzle.Goal(), 0);
        db_.Set(goal, 0u);

        std::array<std::deque<uint64_t>, MAX_COST + 1> buckets;
        buckets[0].push_back(goal);

        size_t visited = 1;
        for (uint8_t current_cost = 0; current_cost <= MAX_COST && visited < size; ++current_cost) {
            auto& bucket = buckets[current_cost];
            while (!bucket.empty()) {
                const uint64_t s = bucket.front();
                bucket.pop_front();

                auto stored = db_.Get(s);
                if (stored != current_cost) {
                    continue;
                }

                State state(true);
                GetStateFromHash(s, state);
                std::vector<Action> actions;
                puzzle.GetActions(state, actions);
                for (auto& action : actions) {
                    const auto raw_cost = current_cost + puzzle.ApplyAction(state, action);
                    if (raw_cost > MAX_COST) {
                        continue;
                    }
                    const auto cost = static_cast<uint8_t>(raw_cost);
                    const auto hash = GetPDBHash(state);
                    if (stored = db_.Get(hash); cost < stored) {
                        db_.Set(hash, cost);
                        if (stored == std::numeric_limits<uint8_t>::max()) {
                            ++visited;
                        }
                        buckets[cost].push_back(hash);
                    }
                    puzzle.UndoAction(state, action);
                }
            }
            bucket.shrink_to_fit();
        }
    }

    void Build(const bool is_additive = true) noexcept {
        auto& puzzle = puzzle_.get();
        puzzle.abstraction_has_edge_cost_ = !is_additive;

        constexpr auto MAX_COST = std::numeric_limits<uint8_t>::max();
        const auto size = db_.Size();
        constexpr size_t CHUNK_SIZE = 4096;
        const auto num_thread = std::thread::hardware_concurrency();

        std::array<std::vector<std::atomic_bool>, MAX_COST + 1> buckets;
        std::ranges::for_each(buckets, [size](auto& bucket) {
            bucket = std::vector<std::atomic_bool>((size + CHUNK_SIZE - 1) / CHUNK_SIZE);
        });

        Queue tasks((size + CHUNK_SIZE - 1) / CHUNK_SIZE);

        const auto goal = util::Ranking(puzzle.Goal(), pattern_);
        db_.Set(goal, 0u);
        buckets[0][goal / CHUNK_SIZE] = true;

        std::vector<std::jthread> threads(num_thread);
        std::atomic_size_t visited = 1;
        std::barrier barrier(num_thread + 1);
        for (uint8_t current_cost = 0; current_cost <= MAX_COST && visited < size; ++current_cost) {
            auto& bucket = buckets[current_cost];
            for (size_t i = 0; i < size; i += CHUNK_SIZE) {
                if (bucket[i / CHUNK_SIZE].load(std::memory_order_relaxed)) {
                    tasks.enqueue({i, std::min(size, i + CHUNK_SIZE)});
                }
            }
            bucket.clear();
            for (size_t i = 0; i < num_thread; ++i) {
                threads.emplace_back([this, &buckets, &tasks, &visited, current_cost, &barrier] {
                    uint64_t count = 0;
                    std::vector<std::pair<uint64_t, uint8_t>> successors;
                    std::vector<std::pair<uint64_t, uint8_t>> zero_cost_successors;
                    std::pair<uint64_t, uint64_t> range{0, 0};
                    while (tasks.try_dequeue(range)) {
                        const auto [start, end] = range;
                        for (uint64_t hash = start; hash < end; ++hash) {
                            if (const auto cost = db_.GetAtomic(hash); cost == current_cost) {
                                GetSuccessors(hash, cost, successors);
                            }
                        }
                        while (!successors.empty()) {
                            for (const auto& [hash, cost] : successors) {
                                uint8_t record = db_.GetAtomic(hash);
                                while (cost < record) {
                                    if (db_.CompareAndSet(hash, record, cost)) {
                                        if (record == std::numeric_limits<uint8_t>::max()) {
                                            ++count;
                                        }
                                        if (cost == current_cost) {
                                            zero_cost_successors.emplace_back(hash, cost);
                                        } else {
                                            buckets[cost][hash / CHUNK_SIZE].store(
                                                true, std::memory_order_relaxed);
                                        }
                                        break;
                                    }
                                    record = db_.GetAtomic(hash);
                                    if (cost >= record) {
                                        break;
                                    }
                                }
                            }
                            successors.clear();
                            for (const auto& [hash, cost] : zero_cost_successors) {
                                GetSuccessors(hash, cost, successors);
                            }
                            zero_cost_successors.clear();
                        }
                    }
                    visited.fetch_add(count, std::memory_order_relaxed);
                    barrier.arrive_and_wait();
                });
            }
            barrier.arrive_and_wait();
            threads.clear();
        }
    }

    void GetSuccessors(const uint64_t hash, const uint8_t cost,
                       std::vector<std::pair<uint64_t, uint8_t>>& successors) const noexcept {
        auto state = State(true);
        util::UnRanking(hash, state, pattern_);
        std::vector<Action> actions;
        puzzle_.get().GetActions(state, actions);
        for (auto& action : actions) {
            const auto action_cost = puzzle_.get().ApplyAction(state, action);
            const auto successor_cost = static_cast<uint8_t>(cost + action_cost);
            const auto successor_hash = util::Ranking(state, pattern_);
            successors.emplace_back(successor_hash, successor_cost);
            puzzle_.get().UndoAction(state, action);
        }
    }

    static uint64_t FactorialUpperK(const unsigned n, const unsigned k) noexcept {
        uint64_t value = 1;
        for (auto i = n; i > k; --i) {
            value *= i;
        }
        return value;
    }

    [[nodiscard]] unsigned HCost(const State<width, height>& state) const noexcept {
        return db_.Get(util::Ranking(state, pattern_));
    }

private:
    using State = State<width, height>;
    using Puzzle = Puzzle<width, height>;
    using Queue = moodycamel::ConcurrentQueue<std::pair<uint64_t, uint64_t>>;

    std::reference_wrapper<Puzzle> puzzle_;
    std::vector<uint8_t> pattern_{};
    util::PackedStorage<> db_{};
};

template <uint8_t width, uint8_t height>
class AdditivePDBHeuristic {
public:
    auto& PDBs() noexcept { return pattern_dbs_; }

    void Add(PatternDatabase<width, height>& pdb) noexcept { pattern_dbs_.push_back(pdb); }

    [[nodiscard]] unsigned HCost(const State<width, height>& state) const noexcept {
        return std::accumulate(
            pattern_dbs_.begin(), pattern_dbs_.end(), 0u,
            [&state](const auto& h, const auto& pdb) { return h + pdb.HCost(state); });
    }

private:
    std::vector<PatternDatabase<width, height>> pattern_dbs_{};
};

}  // namespace stp::algorithm

#endif  // PDB_H_
