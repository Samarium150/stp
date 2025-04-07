#ifndef RANKING_H_
#define RANKING_H_

#include "puzzle.h"

namespace stp::util {

constexpr auto default_pattern = []<uint8_t width, uint8_t height> {
    const auto& data = State<width, height>().Data();
    return std::vector<uint8_t>{data.begin(), data.end()};
};

/**
 * One of the linear time ranking functions
 * designed by Wendy Myrvold and Frank Ruskey in their paper
 * [Ranking and unranking permutations in linear time]
 * @param state the state to rank
 * @param pattern_opt the optional pattern of tiles
 * @return the rank of the state
 * @see https://doi.org/10.1016/S0020-0190(01)00141-7
 */
template <uint8_t width, uint8_t height>
    requires(width > 0 && width <= 16 && height > 0 && height <= 16)
uint64_t Ranking(const State<width, height>& state,
                 const std::optional<std::vector<uint8_t>>& pattern_opt = std::nullopt) noexcept {
    thread_local std::array<uint8_t, width * height> dual;
    thread_local std::array<uint8_t, width * height> locations;
    thread_local std::deque<uint8_t> values;
    const auto pattern = pattern_opt.value_or(default_pattern.operator()<width, height>());
    const auto puzzle_size = static_cast<uint16_t>(state.Size());
    for (auto i = 0; i < puzzle_size; ++i) {
        if (state[i] != -1) {
            dual[static_cast<size_t>(state[i])] = static_cast<uint8_t>(i);
        }
    }
    const auto pattern_size = static_cast<uint16_t>(pattern.size());
    for (auto i = 0; i < pattern_size; ++i) {
        locations[puzzle_size - i - 1] = dual[pattern[pattern_size - i - 1]];
    }
    std::ranges::fill(dual, 0);
    const uint16_t last = puzzle_size - pattern_size;
    for (auto i = last; i < puzzle_size; ++i) {
        dual[locations[i]] = static_cast<uint8_t>(i);
    }
    for (auto i = puzzle_size - 1; i >= last; --i) {
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
 * @param hash the hash for unranking
 * @param state the output state
 * @param pattern_opt the optional pattern of tiles
 * @see https://doi.org/10.1016/S0020-0190(01)00141-7
 */
template <uint8_t width, uint8_t height>
    requires(width > 0 && width <= 16 && height > 0 && height <= 16)
void UnRanking(uint64_t hash, State<width, height>& state,
               const std::optional<std::vector<uint8_t>>& pattern_opt = std::nullopt) noexcept {
    thread_local std::array<uint8_t, width * height> dual;
    const auto pattern = pattern_opt.value_or(default_pattern.operator()<width, height>());
    const auto puzzle_size = static_cast<uint16_t>(state.Size());
    for (auto i = 0; i < puzzle_size; ++i) {
        dual[i] = static_cast<uint8_t>(i);
    }
    const auto last = static_cast<uint16_t>(puzzle_size - pattern.size());
    for (auto i = puzzle_size - 1; i >= last; --i) {
        std::swap(dual[hash % (i + 1)], dual[i]);
        hash /= (i + 1);
        state[dual[i]] = pattern[i - last];
    }
    state.UpdateBlankPosition();
}

}  // namespace stp::util

#endif  // RANKING_H_
