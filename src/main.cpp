#include <array>
#include <chrono>
#include <iostream>
#include <random>

#include "algorithm.h"

constexpr uint8_t WIDTH = 4;
constexpr uint8_t HEIGHT = 4;

int main() {
    const auto puzzle = stp::Puzzle<WIDTH, HEIGHT>();
    constexpr auto board = std::array{6, 0, 14, 12, 1, 15, 9, 10, 11, 4, 7, 2, 8, 3, 5, 13};
    const auto state = stp::State<WIDTH, HEIGHT>(board);
    std::cout << state;
    const auto start_time = std::chrono::high_resolution_clock::now();
    auto algo = stp::algorithm::IDAStar(puzzle);
    const auto ret = algo(state);
    const auto end_time = std::chrono::high_resolution_clock::now();
    const auto elapsed_time =
        std::chrono::duration_cast<std::chrono::seconds>(end_time - start_time).count();
    if (ret) {
        std::cout << "Solution found!" << std::endl;
    } else {
        std::cout << "Solution not found!" << std::endl;
    }
    std::cout << "Time elapsed: " << elapsed_time << " seconds" << std::endl;
    std::cout << "Node expanded: " << algo.NodeExpanded() << std::endl;
    std::cout << "Solution path:" << std::endl;
    for (const auto& s : algo.Path()) {
        std::cout << s << std::endl;
    }
    return 0;
}
