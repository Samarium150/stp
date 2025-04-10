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

#ifndef PUZZLE_H_
#define PUZZLE_H_

#include <algorithm>
#include <iomanip>
#include <iostream>
#include <ranges>
#include <sstream>

namespace stp {

template <uint8_t width, uint8_t height>
    requires(width > 0 && width <= 16 && height > 0 && height <= 16)
class State {
public:
    explicit State(const bool abstract = false) noexcept {
        if (abstract) {
            std::ranges::fill(data_, -1);
        } else {
            Reset();
        }
    }

    template <typename Container,
              std::enable_if_t<std::is_convertible_v<typename Container::value_type, uint8_t>,
                               bool> = true>
    explicit State(const Container& container) {
        if (container.size() != width * height) {
            throw std::invalid_argument("expected a container of size " +
                                        std::to_string(container.size()) + " instead of " +
                                        std::to_string(width * height));
        }
        if (std::ranges::any_of(container,
                                [](const auto& x) { return x < 0 || x >= width * height; })) {
            throw std::invalid_argument("expected a container of values in [0, " +
                                        std::to_string(width * height) + "]");
        }
        if (auto blank = std::ranges::find(container, 0); blank == container.end()) {
            throw std::invalid_argument("blank tile not found in the container");
        } else {
            blank_ = static_cast<uint8_t>(std::distance(container.begin(), blank));
        }
        std::copy(container.begin(), container.end(), data_.begin());
    }

    State(const State&) noexcept = default;

    State& operator=(const State&) noexcept = default;

    State(State&&) noexcept = default;

    State& operator=(State&&) noexcept = default;

    void Reset() noexcept {
        for (int16_t i = 0; i < static_cast<int16_t>(data_.size()); ++i) {
            data_[static_cast<size_t>(i)] = i;
        }
        blank_ = 0;
    }

    bool UpdateBlankPosition() noexcept {
        if (auto blank = std::ranges::find(data_, 0); blank == data_.end()) {
            return false;
        } else {
            blank_ = static_cast<uint8_t>(std::distance(data_.begin(), blank));
            return true;
        }
    }

    [[nodiscard]] auto& Data() const noexcept { return data_; }

    [[nodiscard]] auto Size() const noexcept { return data_.size(); }

    [[nodiscard]] auto& Blank() const noexcept { return blank_; }

    bool operator==(const State& state) const noexcept { return data_ == state.data_; }

    bool operator!=(const State& state) const noexcept { return !(*this == state); }

    auto& operator[](const uint8_t i) const noexcept { return data_[i]; }

    auto& operator[](const uint8_t i) noexcept { return data_[i]; }

    explicit operator std::string() const noexcept {
        std::stringstream ss;
        for (uint8_t row = 0; row < height; ++row) {
            for (uint8_t col = 0; col < width; ++col) {
                ss << std::setw(width);
                if (auto index = PosToIdx(row, col); index == blank_ && data_[index] == 0) {
                    ss << "X";
                } else if (data_[index] == -1) {
                    ss << "*";
                } else {
                    ss << std::to_string(data_[index]);
                }
            }
            ss << std::endl;
        }
        return ss.str();
    }

    friend std::ostream& operator<<(std::ostream& os, const State& state) noexcept {
        return os << std::string(state);
    }

    static uint8_t PosToIdx(const uint8_t row, const uint8_t col) noexcept {
        return row * width + col;
    }

    static std::pair<uint8_t, uint8_t> IdxToPos(const uint8_t idx) noexcept {
        return {idx / width, idx % width};
    }

    void SwapBlank(uint8_t index) noexcept {
        std::swap(data_[blank_], data_[index]);
        blank_ = index;
    }

private:
    std::array<int16_t, width * height> data_{};
    uint8_t blank_{};
};

class Action {
public:
    enum Direction : uint8_t { kLeft, kUp, kRight, kDown, kNoSlide };

    Action() noexcept = default;

    Action(const Direction direction, const uint8_t times) : direction_(direction), times_(times) {}

    Action(const Action&) noexcept = default;

    Action& operator=(const Action&) noexcept = default;

    Action(Action&&) noexcept = default;

    Action& operator=(Action&&) noexcept = default;

    [[nodiscard]] auto& GetDirection() const noexcept { return direction_; }

    [[nodiscard]] auto& Times() const noexcept { return times_; }

    friend std::ostream& operator<<(std::ostream& os, const Direction& direction) noexcept {
        switch (direction) {
            case kLeft:
                os << "Left";
                break;
            case kRight:
                os << "Right";
                break;
            case kUp:
                os << "Up";
                break;
            case kDown:
                os << "Down";
                break;
            default:
                os << "NoSlide";
                break;
        }
        return os;
    }

    explicit operator std::string() const noexcept {
        std::stringstream ss;
        ss << "(" << direction_ << ", " << std::to_string(times_) << ")";
        return ss.str();
    }

    friend std::ostream& operator<<(std::ostream& os, const Action& action) noexcept {
        return os << std::string(action);
    }

    bool operator==(const Action& action) const noexcept { return direction_ == action.direction_; }

    bool operator!=(const Action& action) const noexcept { return !(*this == action); }

    [[nodiscard]] std::pair<int8_t, int8_t> GetOffset() const noexcept {
        switch (direction_) {
            case kLeft:
                return {0, -1};
            case kUp:
                return {-1, 0};
            case kRight:
                return {0, 1};
            case kDown:
                return {1, 0};
            default:
                return {0, 0};
        }
    }

    [[nodiscard]] Action Reverse() const noexcept {
        switch (direction_) {
            case kLeft:
                return {kRight, times_};
            case kRight:
                return {kLeft, times_};
            case kUp:
                return {kDown, times_};
            case kDown:
                return {kUp, times_};
            default:
                return {direction_, times_};
        }
    }

private:
    Direction direction_ = kNoSlide;
    uint8_t times_ = 1;
};

template <uint8_t width, uint8_t height>
    requires(width > 0 && width <= 16 && height > 0 && height <= 16)
class Puzzle {
public:
    explicit Puzzle(const bool enable_bulk_move = false) noexcept
        : enable_bulk_move_(enable_bulk_move) {
        GenerateValidActions();
        UpdateDistance();
    }

    explicit Puzzle(const State<width, height>& goal, const bool enable_bulk_move = false) noexcept
        : enable_bulk_move_(enable_bulk_move) {
        GenerateValidActions();
        SetGoal(goal);
    }

    void GenerateValidActions() noexcept {
        for (uint8_t blank = 0; blank < width * height; ++blank) {
            const auto [row, col] = State::IdxToPos(blank);
            auto& actions = valid_actions_[blank];
            if (enable_bulk_move_) {
                actions.reserve(width + height - 2);
                if (row > 0 && height > 1) actions.emplace_back(Action::kUp, row);
                if (col > 0 && width > 1) actions.emplace_back(Action::kLeft, col);
                if (row < height - 1 && height > 1)
                    actions.emplace_back(Action::kDown, static_cast<uint8_t>(height - row - 1));
                if (col < width - 1 && width > 1)
                    actions.emplace_back(Action::kRight, static_cast<uint8_t>(width - col - 1));
            } else {
                actions.reserve(4);
                if (row > 0 && height > 1) actions.emplace_back(Action::kUp, 1);
                if (col > 0 && width > 1) actions.emplace_back(Action::kLeft, 1);
                if (row < height - 1 && height > 1) actions.emplace_back(Action::kDown, 1);
                if (col < width - 1 && width > 1) actions.emplace_back(Action::kRight, 1);
                actions.shrink_to_fit();
            }
        }
    }

    void SetGoal(const State<width, height>& goal) noexcept {
        goal_ = goal;
        UpdateDistance();
    }

    void UpdateDistance() noexcept {
        for (uint8_t i = 0; i < width * height; ++i) {
            if (const auto tile = goal_[i]; tile != 0) {
                for (uint8_t j = 0; j < width * height; ++j) {
                    distance_[static_cast<size_t>(tile)][j] =
                        Pack(static_cast<uint8_t>(std::abs(i / width - j / width)),
                             static_cast<uint8_t>(std::abs(i % width - j % width)));
                }
            }
        }
    }

    [[nodiscard]] auto& Goal() const noexcept { return goal_; }

    [[nodiscard]] bool GoalTest(const State<width, height>& state) const noexcept {
        return state == goal_;
    }

    template <typename Allocator>
    void GetActions(const State<width, height>& state,
                    std::vector<Action, Allocator>& actions) const noexcept {
        actions.reserve(width + height - 2);
        for (const auto& action : valid_actions_[state.Blank()]) {
            for (uint8_t i = 1; i <= action.Times(); ++i) {
                actions.emplace_back(action.GetDirection(), i);
            }
        }
    }

    unsigned ApplyAction(State<width, height>& state, const Action& action,
                         const bool dry_run = false) const noexcept {
        const auto [row_offset, col_offset] = action.GetOffset();
        uint8_t index = state.Blank();
        auto [row, col] = State::IdxToPos(index);
        int16_t new_row = row + row_offset * action.Times();
        int16_t new_col = col + col_offset * action.Times();
        if (new_row < 0 || new_row >= height || new_col < 0 || new_col >= width) {
            std::cerr << "Invalid action: " << std::string(action) << std::endl;
            std::cerr << "Current state: " << std::endl << state;
            return std::numeric_limits<unsigned>::max();
        }
        unsigned tile_with_cost = 0;
        for (uint8_t i = 0; i < action.Times(); ++i) {
            new_row = row + row_offset;
            new_col = col + col_offset;
            index = State::PosToIdx(static_cast<uint8_t>(new_row), static_cast<uint8_t>(new_col));
            tile_with_cost += state[index] != -1 ? 1 : 0;
            if (!dry_run) {
                state.SwapBlank(index);
            }
            std::tie(row, col) = State::IdxToPos(index);
        }
        return abstraction_has_edge_cost_ || !enable_bulk_move_
                   ? edge_cost_ * tile_with_cost
                   : edge_cost_ / action.Times() * tile_with_cost;
    }

    void UndoAction(State<width, height>& state, const Action& action) const noexcept {
        ApplyAction(state, action.Reverse());
    }

    [[nodiscard]] unsigned HCost(const State<width, height>& state) const noexcept {
        auto r = 0u;
        auto c = 0u;
        for (uint8_t i = 0; i < width * height; ++i) {
            if (const auto tile = state[i]; tile != 0) {
                const auto [row_dist, col_dist] = Unpack(distance_[static_cast<size_t>(tile)][i]);
                r += row_dist;
                c += col_dist;
            }
        }
        return enable_bulk_move_ ? (r / (height - 1) + c / (width - 1)) * edge_cost_
                                 : (r + c) * edge_cost_;
    }

    bool abstraction_has_edge_cost_ = false;

private:
    using State = State<width, height>;
    using Distance = uint8_t;

    static Distance Pack(const uint8_t x, const uint8_t y) noexcept {
        // NOLINTNEXTLINE
        return (x & 0x0F) | (y << 4);
    }

    static std::pair<uint8_t, uint8_t> Unpack(const Distance entry) noexcept {
        return {entry & 0x0F, entry >> 4};
    }

    State goal_{};
    std::array<std::vector<Action>, width * height> valid_actions_{};
    std::array<std::array<Distance, width * height>, width * height> distance_{};
    /**
     * If it is enabled, up to (width - 1) tiles can be moved in a row,
     * up to (height - 1) tiles can be moved in a column
     *
     * In a 15-puzzle the asymptotic branching factor will become 6
     * instead of 2.1304 when enabled
     * @see https://doi.org/10.1016/S0004-3702(01)00094-7
     */
    bool enable_bulk_move_;
    unsigned edge_cost_ = enable_bulk_move_ ? 6u : 1u;
};

namespace algorithm {

constexpr auto kFound = -1;

constexpr auto kInf = std::numeric_limits<int>::max();

}  // namespace algorithm
}  // namespace stp

#endif  // PUZZLE_H_
