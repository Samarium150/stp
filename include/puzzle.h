#ifndef PUZZLE_H
#define PUZZLE_H

#include <array>
#include <iomanip>
#include <iostream>
#include <ostream>

namespace stp {

template <uint8_t width, uint8_t height>
    requires(width > 0 && width <= 16 && height > 0 && height <= 16)
class State {
public:
    std::array<uint8_t, width * height> board_{};
    uint8_t blank_{};

    State() { Reset(); }

    State(const State& state) {
        std::copy(state.board_.begin(), state.board_.end(), board_.begin());
        blank_ = state.blank_;
    }

    State& operator=(const State& state) {
        std::copy(state.board_.begin(), state.board_.end(), board_.begin());
        blank_ = state.blank_;
        return *this;
    }

    template <typename Container, typename T = typename Container::value_type,
              std::enable_if_t<std::is_convertible_v<T, uint8_t>, uint8_t> = 0>
    explicit State(const Container& container) {
        if (container.size() != width * height) {
            throw std::invalid_argument("expected a container of size " +
                                        std::to_string(width * height) + " instead of " +
                                        std::to_string(container.size()));
        }
        int16_t blank_ = -1;
        for (int16_t i = 0; i < container.size(); ++i) {
            if (container[i] == 0) {
                blank_ = i;
                break;
            }
        }
        if (blank_ == -1) {
            throw std::invalid_argument("no blank tile found");
        }
        std::copy(container.begin(), container.end(), board_.begin());
        this->blank_ = static_cast<uint8_t>(blank_);
    }

    void Reset() {
        for (uint8_t i = 0; i < board_.size(); ++i) {
            board_[i] = i;
        }
        blank_ = 0;
    }

    bool operator==(const State& other) const { return board_ == other.board_; }

    bool operator!=(const State& other) const { return !(*this == other); }

    uint8_t operator[](const uint8_t i) const { return board_[i]; }

    explicit operator std::string() const {
        std::stringstream ss;
        for (uint8_t row = 0; row < height; ++row) {
            for (uint8_t col = 0; col < width; ++col) {
                ss << std::setw(width);
                if (auto index = PosToIdx(row, col); board_[index] == 0) {
                    ss << "X";
                } else {
                    ss << std::to_string(board_[index]);
                }
            }
            ss << std::endl;
        }
        return ss.str();
    }

    static uint8_t PosToIdx(const uint8_t row, const uint8_t col) { return row * width + col; }

    static std::pair<uint8_t, uint8_t> IdxToPos(const uint8_t idx) {
        return std::make_pair(idx / width, idx % width);
    }

    void SwapBlank(uint8_t index) {
        std::swap(board_[blank_], board_[index]);
        blank_ = index;
    }
};

template <uint8_t width, uint8_t height>
static std::ostream& operator<<(std::ostream& os, const State<width, height>& state) {
    return os << std::string(state);
}

enum Action : uint8_t { kLeft, kUp, kRight, kDown, kNoSlide };

inline std::pair<int8_t, int8_t> GetOffset(const Action action) {
    switch (action) {
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

inline std::ostream& operator<<(std::ostream& os, const Action& action) {
    switch (action) {
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

template <uint8_t width, uint8_t height>
    requires(width > 0 && width <= 16 && height > 0 && height <= 16)
class Puzzle {
public:
    State<width, height> goal_{};
    std::array<std::vector<Action>, width * height> valid_actions_{};
    std::array<std::array<int, width * height>, width * height> distance_{};

    Puzzle() {
        GenerateValidActions();
        UpdateDistance();
    }

    explicit Puzzle(const State<width, height>& goal) {
        GenerateValidActions();
        SetGoal(goal);
    }

    void GenerateValidActions() {
        for (auto blank = 0; blank < width * height; ++blank) {
            std::vector<Action> actions;
            actions.reserve(4);
            const auto [row, col] = State::IdxToPos(blank);
            if (row > 0) actions.push_back(kUp);
            if (col > 0) actions.push_back(kLeft);
            if (row < height - 1) actions.push_back(kDown);
            if (col < width - 1) actions.push_back(kRight);
            actions.shrink_to_fit();
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
                distance_[tile][j] =
                    std::abs(i / width - j / width) + std::abs(i % width - j % width);
            }
        }
    }

    bool GoalTest(const State<width, height>& state) const { return state == goal_; }

    std::vector<State<width, height>> GetSuccessors(const State<width, height>& state) const {
        std::vector<State> successors;
        successors.reserve(4);
        for (const auto& action : valid_actions_[state.blank_]) {
            successors.emplace_back(state);
            ApplyAction(successors.back(), action);
        }
        return successors;
    }

    static void ApplyAction(State<width, height>& state, const Action& action) {
        const auto [row, col] = State::IdxToPos(state.blank_);
        const auto [row_offset, col_offset] = GetOffset(action);
        const int16_t new_row = row + row_offset;
        const int16_t new_col = col + col_offset;
        if (new_row < 0 || new_row >= height || new_col < 0 || new_col >= width) {
            std::cerr << "Invalid action: " << action << std::endl;
            std::cerr << "Current state: " << std::endl << state;
            return;
        }
        const auto new_index = State::PosToIdx(new_row, new_col);
        state.SwapBlank(new_index);
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
};
}  // namespace stp

#endif  // PUZZLE_H
