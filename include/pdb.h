#ifndef PDB_H_
#define PDB_H_

#include <numeric>

#include "puzzle.h"

namespace stp::algorithm {

template <uint8_t width, uint8_t height>
class PatternDatabase {
public:
    template <typename Container,
              std::enable_if_t<std::is_convertible_v<typename Container::value_type, uint8_t>,
                               bool> = true>
    PatternDatabase(Puzzle<width, height>& puzzle, const Container& pattern) : puzzle_(puzzle) {
        const auto pattern_size = pattern.size();
        if (pattern_size == 0 || pattern_size > width * height) {
            throw std::invalid_argument("the pattern size must be between 1 and the puzzle size");
        }
        if (std::ranges::find(pattern, 0) == pattern.end()) {
            throw std::invalid_argument("the pattern must contain a blank tile indicated by 0");
        }
        pattern_.assign(pattern.begin(), pattern.end());
        const uint16_t factorial_base = width * height;
        const std::size_t expected_size =
            FactorialUpperK(factorial_base, factorial_base - pattern_size);
        db_.resize(expected_size, std::numeric_limits<uint8_t>::max());
    }

    PatternDatabase(const PatternDatabase& pdb)
        : puzzle_(pdb.GetPuzzle()), pattern_(pdb.Pattern()), db_(pdb.DB()) {}

    PatternDatabase& operator=(const PatternDatabase& pdb) {
        puzzle_ = pdb.GetPuzzle();
        pattern_ = pdb.Pattern();
        db_ = pdb.DB();
        return *this;
    }

    PatternDatabase(PatternDatabase&& pdb) noexcept
        : puzzle_(pdb.GetPuzzle()), pattern_(std::move(pdb.Pattern())), db_(std::move(pdb.DB())) {}

    PatternDatabase& operator=(PatternDatabase&& pdb) noexcept {
        puzzle_ = pdb.GetPuzzle();
        pattern_ = std::move(pdb.Pattern());
        db_ = std::move(pdb.DB());
        return *this;
    }

    auto& GetPuzzle() { return puzzle_.get(); }

    auto& GetPuzzle() const { return puzzle_.get(); }

    auto& Pattern() const { return pattern_; }

    auto& DB() const { return db_; }

    auto& DB() { return db_; }

    auto Size() const { return db_.size(); }

    void Build(const bool additive = true) {
        auto& puzzle = puzzle_.get();
        if (!additive) {
            puzzle.abstraction_has_edge_cost_ = true;
        }
        std::queue<std::pair<uint64_t, unsigned>> queue;
        queue.emplace(GetPDBHash(puzzle.Goal()), 0u);
        std::vector<Action> actions{};
        while (!queue.empty()) {
            auto [hash, cost] = queue.front();
            queue.pop();
            if (cost >= db_[hash]) {
                continue;
            }
            db_[hash] = cost;
            auto state = State(true);
            GetStateFromHash(hash, state);
            actions.clear();
            puzzle.GetActions(state, actions);
            for (const auto& action : actions) {
                auto new_cost = cost + puzzle.ApplyAction(state, action);
                queue.emplace(GetPDBHash(state), new_cost);
                puzzle.UndoAction(state, action);
            }
        }
    }

    uint64_t GetPDBHash(const State<width, height>& state) const {
        std::ranges::fill(dual_, 0);
        std::ranges::fill(locations_, 0);
        const uint8_t puzzle_size = state.Size();
        for (uint8_t i = 0; i < puzzle_size; ++i) {
            if (state[i] != -1) {
                dual_[state[i]] = i;
            }
        }
        const uint8_t pattern_size = pattern_.size();
        for (uint8_t i = 0; i < pattern_size; ++i) {
            locations_[puzzle_size - i - 1] = dual_[pattern_[pattern_size - i - 1]];
        }
        std::ranges::fill(dual_, 0);
        const uint8_t last = puzzle_size - pattern_size;
        for (uint8_t i = last; i < puzzle_size; ++i) {
            dual_[locations_[i]] = i;
        }
        for (uint8_t i = puzzle_size - 1; i >= last; --i) {
            values_.emplace_back(locations_[i]);
            std::swap(locations_[i], locations_[dual_[i]]);
            std::swap(dual_[values_.back()], dual_[i]);
        }
        uint64_t hash = 0;
        uint8_t count = last + 1;
        while (!values_.empty()) {
            hash *= count++;
            hash += values_.back();
            values_.pop_back();
        }
        return hash;
    }

    void GetStateFromHash(uint64_t hash, State<width, height>& state) const {
        const uint8_t puzzle_size = state.Size();
        for (uint8_t i = 0; i < puzzle_size; ++i) {
            dual_[i] = i;
        }
        const uint8_t last = puzzle_size - pattern_.size();
        for (uint8_t i = puzzle_size - 1; i >= last; --i) {
            std::swap(dual_[hash % (i + 1)], dual_[i]);
            hash /= (i + 1);
            state[dual_[i]] = pattern_[i - last];
        }
        state.UpdateBlankPosition();
    }

    static uint64_t FactorialUpperK(const unsigned n, const unsigned k) {
        uint64_t value = 1;
        for (auto i = n; i > k; --i) {
            value *= i;
        }
        return value;
    }

    unsigned HCost(const State<width, height>& state) const {
        auto hash = GetPDBHash(state);
        return hash < db_.size() ? db_[hash] : 0u;
    }

private:
    using State = State<width, height>;
    using Puzzle = Puzzle<width, height>;
    std::reference_wrapper<Puzzle> puzzle_;

    std::vector<uint8_t> pattern_{};
    std::vector<uint8_t> db_{};

    mutable std::array<uint8_t, width * height> dual_{};
    mutable std::array<uint8_t, width * height> locations_{};
    mutable std::deque<uint8_t> values_{};
};

template <uint8_t width, uint8_t height>
class AdditivePDBHeuristic {
public:
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
