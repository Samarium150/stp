#include <array>
#include <iostream>

#include "parallel_ida_star.h"
#include "pdb.h"
#include "process_info.h"

constexpr uint8_t WIDTH = 4;
constexpr uint8_t HEIGHT = 4;

using State = stp::State<WIDTH, HEIGHT>;
using Puzzle = stp::Puzzle<WIDTH, HEIGHT>;

int main() {
    auto puzzle = Puzzle();
    constexpr auto board = std::array{14, 13, 15, 7, 11, 12, 9, 5, 6, 0, 2, 1, 4, 8, 10, 3};
    auto start = State(board);
    std::cout << start;
    auto algo = stp::algorithm::ParallelIDAStar(puzzle);
    auto pdb_h = stp::algorithm::AdditivePDBHeuristic<WIDTH, HEIGHT>();
    std::cout << "Running IDA* algorithm with Manhattan distance heuristic..." << std::endl;
    auto start_time = std::chrono::high_resolution_clock::now();
    auto ret = algo(start);
    auto end_time = std::chrono::high_resolution_clock::now();
    auto elapsed_time =
        std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count();
    if (ret) {
        std::cout << "Solution found!" << std::endl;
    } else {
        std::cout << "Solution not found!" << std::endl;
    }
    std::cout << "Time elapsed: " << elapsed_time << " ms" << std::endl;
    std::cout << "Node expanded: " << algo.NodeExpanded() << std::endl;
    auto usage = GetProcessUsage();
    std::cout << "Max memory usage: "
              << static_cast<double>(usage.resident_memory_peak_bytes) / 1024.00 / 1024.00 << " MB"
              << std::endl
              << std::endl;
    for (uint8_t i = 0; i < WIDTH; ++i) {
        std::vector pattern{0};
        for (uint8_t j = 0; j < HEIGHT; ++j) {
            if (i == 0 && j == 0) {
                continue;
            }
            pattern.emplace_back(i * WIDTH + j);
        }
        std::cout << "Build PDB with pattern: ";
        std::ranges::copy(pattern, std::ostream_iterator<int>(std::cout, " "));
        std::cout << std::endl;
        stp::algorithm::PatternDatabase pdb(puzzle, pattern);
        start_time = std::chrono::high_resolution_clock::now();
        pdb.Build();
        end_time = std::chrono::high_resolution_clock::now();
        elapsed_time =
            std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count();
        std::cout << "Finished, time elapsed: " << elapsed_time << " ms" << std::endl << std::endl;
        pdb_h.Add(pdb);
    }
    algo.SetHeuristic([&](const State& state) { return pdb_h.HCost(state); });
    std::cout << "Running IDA* algorithm with Pattern database heuristic..." << std::endl;
    start = State(board);
    start_time = std::chrono::high_resolution_clock::now();
    ret = algo(start);
    end_time = std::chrono::high_resolution_clock::now();
    elapsed_time =
        std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count();
    if (ret) {
        std::cout << "Solution found!" << std::endl;
    } else {
        std::cout << "Solution not found!" << std::endl;
    }
    std::cout << "Time elapsed: " << elapsed_time << " ms" << std::endl;
    std::cout << "Node expanded: " << algo.NodeExpanded() << std::endl;
    usage = GetProcessUsage();
    std::cout << "Max memory usage: "
              << static_cast<double>(usage.resident_memory_peak_bytes) / 1024.00 / 1024.00 << " MB"
              << std::endl
              << std::endl;
    std::cout << "Solution path:" << std::endl;
    std::ranges::for_each(algo.Solution(),
                          [](const auto& state) { std::cout << state << std::endl; });
    return 0;
}
