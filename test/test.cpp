#include <gtest/gtest.h>

#include <fstream>
#include <iostream>
#include <string>
#include <vector>

#include "algorithm.h"

constexpr uint8_t SIZE = 4;

std::vector<std::string> LoadLinesFromFile(const std::string& filename) {
    std::ifstream file(filename);
    if (!file.is_open()) {
        throw std::runtime_error("Could not open file: " + filename);
    }

    std::vector<std::string> lines;
    std::string line;

    while (std::getline(file, line)) {
        lines.push_back(line);
    }

    file.close();
    return lines;
}

stp::State<SIZE, SIZE> LoadInstanceFromLine(const std::string& line) {
    std::istringstream iss(line);
    std::vector<unsigned> tiles;

    unsigned tile;
    while (iss >> tile) {
        tiles.push_back(tile);
    }
    tiles.erase(tiles.begin());

    if (tiles.size() != SIZE * SIZE) {
        throw std::runtime_error("Error: Unexpected number of values in the input.");
    }

    return stp::State<SIZE, SIZE>(tiles);
}

class STPTest : public testing::TestWithParam<std::string> {
protected:
    stp::Puzzle<SIZE, SIZE> puzzle_{};
    stp::algorithm::IDAStar<SIZE, SIZE> ida_{puzzle_};
};

TEST_P(STPTest, TestInstance) {
    const std::string line = GetParam();
    EXPECT_FALSE(line.empty());
    const auto state = LoadInstanceFromLine(line);
    std::cout << state << std::endl;
    const auto start = std::chrono::high_resolution_clock::now();
    ASSERT_TRUE(ida_(state));
    const auto end = std::chrono::high_resolution_clock::now();
    const auto duration =
        std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
    std::cout << "Test runtime: " << duration << " ms" << std::endl;
    std::cout << "Node expanded: " << ida_.NodeExpanded() << std::endl;
}

INSTANTIATE_TEST_SUITE_P(KORF100, STPTest, testing::ValuesIn(LoadLinesFromFile("data/korf100.txt")),
                         [](const testing::TestParamInfo<std::string>& info) {
                             std::istringstream iss(info.param);
                             std::size_t index;
                             iss >> index;
                             return std::to_string(index);
                         });
