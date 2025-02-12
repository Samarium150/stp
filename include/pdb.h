#ifndef PDB_H_
#define PDB_H_

#include <barrier>
#include <numeric>

#include "blockingconcurrentqueue.h"
#include "packed_storage.h"
#include "puzzle.h"

namespace stp::algorithm {

template <uint8_t width, uint8_t height>
class PatternDatabase {
public:
    template <typename Container,
              std::enable_if_t<std::is_convertible_v<typename Container::value_type, uint8_t>,
                               bool> = true>
    PatternDatabase(Puzzle<width, height>& puzzle, const Container& pattern)
        : puzzle_(puzzle),
          num_threads_(std::thread::hardware_concurrency()),
          thread_data_(num_threads_) {
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

    PatternDatabase(const PatternDatabase& pdb)
        : puzzle_(pdb.GetPuzzle()),
          pattern_(pdb.Pattern()),
          db_(pdb.DB()),
          num_threads_(std::thread::hardware_concurrency()),
          thread_data_(num_threads_) {}

    PatternDatabase& operator=(const PatternDatabase& pdb) {
        puzzle_ = pdb.GetPuzzle();
        pattern_ = pdb.Pattern();
        db_ = pdb.DB();
        num_threads_ = std::thread::hardware_concurrency();
        thread_data_ = std::vector<ThreadData>(num_threads_);
        return *this;
    }

    PatternDatabase(PatternDatabase&& pdb) noexcept
        : puzzle_(pdb.GetPuzzle()),
          pattern_(std::move(pdb.Pattern())),
          db_(std::move(pdb.DB())),
          num_threads_(std::thread::hardware_concurrency()),
          thread_data_(num_threads_) {}

    PatternDatabase& operator=(PatternDatabase&& pdb) noexcept {
        puzzle_ = pdb.GetPuzzle();
        pattern_ = std::move(pdb.Pattern());
        db_ = std::move(pdb.DB());
        num_threads_ = std::thread::hardware_concurrency();
        thread_data_ = std::vector<ThreadData>(num_threads_);
        return *this;
    }

    auto& GetPuzzle() { return puzzle_.get(); }

    auto& GetPuzzle() const { return puzzle_.get(); }

    auto& Pattern() const { return pattern_; }

    auto& DB() const { return db_; }

    auto& DB() { return db_; }

    auto Size() const { return db_.Size(); }

    void BuildSeq(const bool is_additive = true) {
        auto& puzzle = puzzle_.get();
        puzzle.abstraction_has_edge_cost_ = !is_additive;

        constexpr auto MAX_COST = std::numeric_limits<uint8_t>::max();
        const auto size = db_.Size();

        const auto goal = GetPDBHash(puzzle.Goal(), 0);
        db_.Set(goal, 0u);

        std::array<std::deque<uint64_t>, MAX_COST + 1> buckets;
        buckets[0].push_back(goal);

        std::size_t visited = 1;
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

    void Build(const bool is_additive = true) {
        auto& puzzle = puzzle_.get();
        puzzle.abstraction_has_edge_cost_ = !is_additive;

        constexpr auto MAX_COST = std::numeric_limits<uint8_t>::max();
        const auto size = db_.Size();
        constexpr std::size_t CHUNK_SIZE = 4096;

        std::array<std::vector<std::atomic_bool>, MAX_COST + 1> buckets;
        std::ranges::for_each(buckets, [size](auto& bucket) {
            bucket = std::vector<std::atomic_bool>((size + CHUNK_SIZE - 1) / CHUNK_SIZE);
        });

        Queue tasks((size + CHUNK_SIZE - 1) / CHUNK_SIZE);

        const auto goal = GetPDBHash(puzzle.Goal());
        db_.Set(goal, 0u);
        buckets[0][goal / CHUNK_SIZE] = true;
        std::vector<std::jthread> threads(num_threads_);
        std::atomic_size_t visited = 1;
        std::barrier barrier(num_threads_ + 1);
        for (uint8_t current_cost = 0; current_cost <= MAX_COST && visited < size; ++current_cost) {
            auto& bucket = buckets[current_cost];
            for (std::size_t i = 0; i < size; i += CHUNK_SIZE) {
                if (bucket[i / CHUNK_SIZE].load(std::memory_order_relaxed)) {
                    tasks.enqueue({i, std::min(size, i + CHUNK_SIZE)});
                }
            }
            bucket.clear();
            for (std::size_t thread_id = 0; thread_id < num_threads_; ++thread_id) {
                threads.emplace_back(
                    [this, &buckets, &tasks, &visited, current_cost, thread_id, &barrier] {
                        uint64_t count = 0;
                        std::vector<std::pair<uint64_t, uint8_t>> successors;
                        std::vector<std::pair<uint64_t, uint8_t>> zero_cost_successors;
                        std::pair<uint64_t, uint64_t> range{0, 0};
                        while (tasks.try_dequeue(range)) {
                            const auto [start, end] = range;
                            for (uint64_t hash = start; hash < end; ++hash) {
                                if (const auto cost = db_.GetAtomic(hash); cost == current_cost) {
                                    GetSuccessors(hash, cost, successors, thread_id);
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
                                    GetSuccessors(hash, cost, successors, thread_id);
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

    uint64_t GetPDBHash(const State<width, height>& state,
                        const std::optional<std::size_t> thread_id = std::nullopt) const noexcept {
        if (thread_id.has_value()) {
            auto& thread_data = thread_data_[thread_id.value()];
            auto& dual = thread_data.dual;
            auto& locations = thread_data.locations;
            auto& values = thread_data.values;
            std::ranges::fill(dual, 0);
            std::ranges::fill(locations, 0);
            return Ranking(state, dual, locations, values);
        }
        thread_local std::array<uint8_t, width * height> dual{};
        thread_local std::array<uint8_t, width * height> locations{};
        thread_local std::deque<uint8_t> values{};
        return Ranking(state, dual, locations, values);
    }

    void GetStateFromHash(
        const uint64_t hash, State<width, height>& state,
        const std::optional<std::size_t> thread_id = std::nullopt) const noexcept {
        if (thread_id.has_value()) {
            auto& thread_data = thread_data_[thread_id.value()];
            auto& dual = thread_data.dual;
            UnRanking(hash, state, dual);
            return;
        }
        thread_local std::array<uint8_t, width * height> dual{};
        UnRanking(hash, state, dual);
    }

    void GetSuccessors(const uint64_t hash, const uint8_t cost,
                       std::vector<std::pair<uint64_t, uint8_t>>& successors,
                       const std::size_t thread_id = 0) const noexcept {
        auto state = State(true);
        GetStateFromHash(hash, state, thread_id);
        auto& actions = thread_data_[thread_id].actions;
        puzzle_.get().GetActions(state, actions);
        for (auto& action : actions) {
            const auto action_cost = puzzle_.get().ApplyAction(state, action);
            const auto successor_cost = static_cast<uint8_t>(cost + action_cost);
            const auto successor_hash = GetPDBHash(state, thread_id);
            successors.emplace_back(successor_hash, successor_cost);
            puzzle_.get().UndoAction(state, action);
        }
        actions.clear();
    }

    static uint64_t FactorialUpperK(const unsigned n, const unsigned k) {
        uint64_t value = 1;
        for (auto i = n; i > k; --i) {
            value *= i;
        }
        return value;
    }

    unsigned HCost(const State<width, height>& state) const { return db_.Get(GetPDBHash(state)); }

private:
    /**
     * One of the linear time ranking functions
     * designed by Wendy Myrvold and Frank Ruskey in their paper
     * [Ranking and unranking permutations in linear time]
     * @tparam C1 Container type for the dual
     * @tparam C2 Container type for the locations
     * @tparam C3 Container type for the values
     * @param state the state to rank
     * @param dual the cache for the `dual` representation of the abstract state
     * @param locations the cache for store indices of pattern tiles
     * @param values the cache for the `swapped` index
     * @return the rank of the state
     * @see https://doi.org/10.1016/S0020-0190(01)00141-7
     */
    template <typename C1, typename C2, typename C3>
    uint64_t Ranking(const State<width, height>& state, C1& dual, C2& locations,
                     C3& values) const noexcept {
        const auto puzzle_size = static_cast<uint16_t>(state.Size());
        for (auto i = 0; i < puzzle_size; ++i) {
            if (state[i] != -1) {
                dual[static_cast<std::size_t>(state[i])] = static_cast<uint8_t>(i);
            }
        }
        const auto pattern_size = static_cast<uint8_t>(pattern_.size());
        for (uint8_t i = 0; i < pattern_size; ++i) {
            locations[puzzle_size - i - 1] = dual[pattern_[pattern_size - i - 1]];
        }
        std::ranges::fill(dual, 0);
        const uint16_t last = puzzle_size - pattern_size;
        for (auto i = last; i < puzzle_size; ++i) {
            dual[locations[i]] = static_cast<uint8_t>(i);
        }
        for (uint8_t i = puzzle_size - 1; i >= last; --i) {
            values.emplace_back(locations[i]);
            std::swap(locations[i], locations[dual[i]]);
            std::swap(dual[values.back()], dual[i]);
        }
        uint64_t hash = 0;
        uint8_t count = last + 1;
        while (!values.empty()) {
            hash *= count++;
            hash += values.back();
            values.pop_back();
        }
        return hash;
    }

    /**
     * One of the linear time unranking functions
     * designed by Wendy Myrvold and Frank Ruskey in their paper
     * [Ranking and unranking permutations in linear time]
     * @tparam Container type for the dual
     * @param hash the hash for unranking
     * @param state the output state
     * @param dual the cache for the `dual` representation of the abstract state
     * @see https://doi.org/10.1016/S0020-0190(01)00141-7
     */
    template <typename Container>
    void UnRanking(uint64_t hash, State<width, height>& state, Container& dual) const noexcept {
        const auto puzzle_size = static_cast<uint16_t>(state.Size());
        for (auto i = 0; i < puzzle_size; ++i) {
            dual[i] = static_cast<uint8_t>(i);
        }
        const auto last = static_cast<uint8_t>(puzzle_size - pattern_.size());
        for (auto i = static_cast<uint8_t>(puzzle_size - 1); i >= last; --i) {
            std::swap(dual[hash % (i + 1)], dual[i]);
            hash /= (i + 1);
            state[dual[i]] = pattern_[i - last];
        }
        state.UpdateBlankPosition();
    }

    using State = State<width, height>;
    using Puzzle = Puzzle<width, height>;
    std::reference_wrapper<Puzzle> puzzle_;

    std::vector<uint8_t> pattern_{};
    util::PackedStorage<> db_{};

    using Queue = moodycamel::ConcurrentQueue<std::pair<uint64_t, uint64_t>>;

    std::size_t num_threads_;
    struct alignas(64) ThreadData {
        std::array<uint8_t, width * height> dual{};
        std::array<uint8_t, width * height> locations{};
        std::deque<uint8_t> values{};
        std::vector<Action> actions{};
    };
    mutable std::vector<ThreadData> thread_data_;
};

template <uint8_t width, uint8_t height>
class AdditivePDBHeuristic {
public:
    auto& PDBs() { return pattern_dbs_; }

    void Add(PatternDatabase<width, height>& pdb) { pattern_dbs_.push_back(pdb); }

    unsigned HCost(const State<width, height>& state) {
        return std::accumulate(
            pattern_dbs_.begin(), pattern_dbs_.end(), 0u,
            [&state](const auto& h, const auto& pdb) { return h + pdb.HCost(state); });
    }

private:
    std::vector<PatternDatabase<width, height>> pattern_dbs_{};
};
}  // namespace stp::algorithm

#endif  // PDB_H_
