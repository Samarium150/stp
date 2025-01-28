#include <chrono>
#include <iostream>
#include <random>
#include <vector>

#include "algorithm.h"

constexpr uint8_t WIDTH = 3;
constexpr uint8_t HEIGHT = 2;

int main() {
    thread_local std::mt19937 rng(std::random_device{}());
    const auto puzzle = stp::Puzzle<WIDTH, HEIGHT>();
    auto board = puzzle.Goal().Data();
    std::ranges::shuffle(board, rng);
    const auto state = stp::State<WIDTH, HEIGHT>(board);
    std::cout << state;
    std::vector<stp::State<WIDTH, HEIGHT>> path;
    const auto start_time = std::chrono::high_resolution_clock::now();
    const auto ret = stp::algorithm::IDAStar(puzzle, state, path);
    const auto end_time = std::chrono::high_resolution_clock::now();
    const auto elapsed_time =
        std::chrono::duration_cast<std::chrono::seconds>(end_time - start_time).count();
    if (ret) {
        std::cout << "Solution found!" << std::endl;
    } else {
        std::cout << "Solution not found!" << std::endl;
    }
    std::cout << "Time elapsed: " << elapsed_time << " seconds" << std::endl;
    std::cout << "Solution path:" << std::endl;
    for (const auto& s : path) {
        std::cout << s << std::endl;
    }
    return 0;
}
