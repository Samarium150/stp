#ifndef PUZZLE_H
#define PUZZLE_H

#include <array>
#include <iomanip>
#include <iostream>
#include <sstream>

namespace stp {

template <uint8_t width, uint8_t height>
    requires(width > 0 && width <= 16 && height > 0 && height <= 16)
class State {
public:
    State() { Reset(); }

    State(const State&) = default;

    State& operator=(const State&) = default;

    State(State&&) = default;

    State& operator=(State&&) = default;

    template <typename Container,
              typename = std::enable_if_t<
                  std::is_convertible_v<typename Container::value_type, uint8_t>, uint8_t>>
    explicit State(const Container& container) {
        if (container.size() != width * height) {
            throw std::invalid_argument("expected a container of size " +
                                        std::to_string(width * height) + " instead of " +
                                        std::to_string(container.size()));
        }
        int16_t blank_ = -1;
        for (std::size_t i = 0; i < container.size(); ++i) {
            if (container[i] == 0) {
                blank_ = static_cast<int16_t>(i);
                break;
            }
        }
        if (blank_ == -1) {
            throw std::invalid_argument("no blank tile found");
        }
        std::copy(container.begin(), container.end(), data_.begin());
        this->blank_ = static_cast<uint8_t>(blank_);
    }

    void Reset() {
        for (uint8_t i = 0; i < data_.size(); ++i) {
            data_[i] = i;
        }
        blank_ = 0;
    }

    auto& Data() const { return data_; }

    auto Size() const { return data_.size(); }

    auto& Blank() const { return blank_; }

    bool operator==(const State& state) const { return data_ == state.Data(); }

    bool operator!=(const State& state) const { return !(*this == state); }

    auto& operator[](const uint8_t i) const { return data_[i]; }

    explicit operator std::string() const {
        std::stringstream ss;
        for (uint8_t row = 0; row < height; ++row) {
            for (uint8_t col = 0; col < width; ++col) {
                ss << std::setw(width);
                if (auto index = PosToIdx(row, col); data_[index] == 0) {
                    ss << "X";
                } else {
                    ss << std::to_string(data_[index]);
                }
            }
            ss << std::endl;
        }
        return ss.str();
    }

    friend std::ostream& operator<<(std::ostream& os, const State& state) {
        return os << std::string(state);
    }

    static uint8_t PosToIdx(const uint8_t row, const uint8_t col) { return row * width + col; }

    static std::pair<uint8_t, uint8_t> IdxToPos(const uint8_t idx) {
        return std::make_pair(idx / width, idx % width);
    }

    void SwapBlank(uint8_t index) {
        std::swap(data_[blank_], data_[index]);
        blank_ = index;
    }

private:
    std::array<uint8_t, width * height> data_{};
    uint8_t blank_{};
};

class Action {
public:
    enum Direction : uint8_t { kLeft, kUp, kRight, kDown, kNoSlide };

    Action() = default;

    Action(const Action&) = default;

    Action& operator=(const Action&) = default;

    Action(Action&&) = default;

    Action& operator=(Action&&) = default;

    Action(const Direction direction, const uint8_t times) : direction_(direction), times_(times) {}

    [[nodiscard]] auto& GetDirection() const { return direction_; }

    [[nodiscard]] auto& Times() const { return times_; }

    friend std::ostream& operator<<(std::ostream& os, const Direction& direction) {
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

    explicit operator std::string() const {
        std::stringstream ss;
        ss << "(" << direction_ << ", " << std::to_string(times_) << ")";
        return ss.str();
    }

    friend std::ostream& operator<<(std::ostream& os, const Action& action) {
        return os << std::string(action);
    }

    bool operator==(const Action& action) const {
        return direction_ == action.direction_ && times_ == action.times_;
    }

    bool operator!=(const Action& action) const { return !(*this == action); }

    [[nodiscard]] std::pair<int8_t, int8_t> GetOffset() const {
        switch (direction_) {
            case kLeft:
                return std::make_pair(0, -1);
            case kUp:
                return std::make_pair(-1, 0);
            case kRight:
                return std::make_pair(0, 1);
            case kDown:
                return std::make_pair(1, 0);
            default:
                return std::make_pair(0, 0);
        }
    }

    [[nodiscard]] Action Reverse() const {
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
    explicit Puzzle(const bool enable_bulk_move = false) : enable_bulk_move_(enable_bulk_move) {
        GenerateValidActions();
        UpdateDistance();
    }

    explicit Puzzle(const State<width, height>& goal, const bool enable_bulk_move = false)
        : enable_bulk_move_(enable_bulk_move) {
        GenerateValidActions();
        SetGoal(goal);
    }

    void GenerateValidActions() {
        for (uint8_t blank = 0; blank < width * height; ++blank) {
            const auto [row, col] = State::IdxToPos(blank);
            std::vector<Action> actions;
            if (enable_bulk_move_) {
                actions.reserve(width + height - 2);
                if (row > 0) actions.emplace_back(Action::kUp, row);
                if (col > 0) actions.emplace_back(Action::kLeft, col);
                if (row < height - 1)
                    actions.emplace_back(Action::kDown, static_cast<uint8_t>(height - row - 1));
                if (col < width - 1)
                    actions.emplace_back(Action::kRight, static_cast<uint8_t>(width - col - 1));
            } else {
                actions.reserve(4);
                if (row > 0) actions.emplace_back(Action::kUp, 1);
                if (col > 0) actions.emplace_back(Action::kLeft, 1);
                if (row < height - 1) actions.emplace_back(Action::kDown, 1);
                if (col < width - 1) actions.emplace_back(Action::kRight, 1);
                actions.shrink_to_fit();
            }
            valid_actions_[blank] = actions;
        }
    }

    void SetGoal(const State<width, height>& goal) {
        goal_ = goal;
        UpdateDistance();
    }

    void UpdateDistance() {
        for (uint8_t i = 0; i < width * height; ++i) {
            auto tile = goal_[i];
            if (tile == 0) {
                continue;
            }
            for (uint8_t j = 0; j < width * height; ++j) {
                distance_[tile][j] = static_cast<uint8_t>(std::abs(i / width - j / width) +
                                                          std::abs(i % width - j % width));
            }
        }
    }

    const auto& Goal() const { return goal_; }

    bool GoalTest(const State<width, height>& state) const { return state == goal_; }

    template <typename Container,
              typename = std::enable_if_t<std::is_same_v<typename Container::value_type, Action>>>
    void GetActions(const State<width, height>& state, Container& actions) const {
        actions.reserve(width + height - 2);
        for (const auto& action : valid_actions_[state.Blank()]) {
            for (uint8_t i = 1; i <= action.Times(); ++i) {
                actions.emplace_back(action.GetDirection(), i);
            }
        }
    }

    static void ApplyAction(State<width, height>& state, const Action& action) {
        const auto [row_offset, col_offset] = action.GetOffset();
        uint8_t index = state.Blank();
        for (uint8_t i = 0; i < action.Times(); ++i) {
            const auto [row, col] = State::IdxToPos(index);
            const int16_t new_row = row + row_offset;
            const int16_t new_col = col + col_offset;
            if (new_row < 0 || new_row >= height || new_col < 0 || new_col >= width) {
                std::cerr << "Invalid action: " << std::string(action) << std::endl;
                std::cerr << "Current state: " << std::endl << state;
                return;
            }
            index = State::PosToIdx(static_cast<uint8_t>(new_row), static_cast<uint8_t>(new_col));
            state.SwapBlank(index);
        }
    }

    static void UndoAction(State<width, height>& state, const Action& action) {
        ApplyAction(state, action.Reverse());
    }

    unsigned HCost(const State<width, height>& state) const {
        auto h = 0u;
        for (uint8_t i = 0; i < width * height; ++i) {
            const auto& tile = state[i];
            if (tile != 0) {
                h += distance_[tile][i];
            }
        }
        return h;
    }

private:
    using State = State<width, height>;
    State goal_{};
    std::array<std::vector<Action>, width * height> valid_actions_{};
    std::array<std::array<uint8_t, width * height>, width * height> distance_{};
    /**
     * If it is enabled, up to (width - 1) tiles can be moved in a row,
     * up to (height - 1) tiles can be moved in a column
     *
     * In a 15-puzzle, the branching factor will become 6 when enabled
     */
    bool enable_bulk_move_;
};
}  // namespace stp

#endif  // PUZZLE_H
