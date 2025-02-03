#include <array>
#include <iostream>

#include "ida_star.h"

constexpr uint8_t WIDTH = 4;
constexpr uint8_t HEIGHT = 4;

using State = stp::State<WIDTH, HEIGHT>;
using Puzzle = stp::Puzzle<WIDTH, HEIGHT>;

int main() {
    const auto puzzle = Puzzle();
    constexpr auto board = std::array{14, 13, 15, 7, 11, 12, 9, 5, 6, 0, 2, 1, 4, 8, 10, 3};
    auto start = State(board);
    std::cout << start;
    const auto start_time = std::chrono::high_resolution_clock::now();
    auto algo = stp::algorithm::IDAStar(puzzle);
    const auto ret = algo(start);
    const auto end_time = std::chrono::high_resolution_clock::now();
    const auto elapsed_time =
        std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count();
    if (ret) {
        std::cout << "Solution found!" << std::endl;
    } else {
        std::cout << "Solution not found!" << std::endl;
    }
    std::cout << "Time elapsed: " << elapsed_time << " ms" << std::endl;
    std::cout << "Node expanded: " << algo.NodeExpanded() << std::endl;
    std::cout << "Solution path:" << std::endl;
    std::ranges::for_each(algo.Solution(),
                          [](const auto& state) { std::cout << state << std::endl; });
    return 0;
}
