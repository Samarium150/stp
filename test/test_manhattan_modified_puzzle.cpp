#include "parallel_ida_star.h"
#include "test_utils.h"

class TestModifiedPuzzle : public testing::TestWithParam<std::size_t> {
protected:
    stp::Puzzle<kSize, kSize> puzzle_{true};
    stp::algorithm::ParallelIDAStar<kSize, kSize> ida_star_{puzzle_};
};

TEST_P(TestModifiedPuzzle, TestInstance) {
    const auto [instance, _] = kInstances[GetParam()];
    auto state = stp::State<kSize, kSize>(instance);
    std::cout << state << std::endl;
    const auto start = std::chrono::high_resolution_clock::now();
    ASSERT_TRUE(ida_star_(state));
    const auto end = std::chrono::high_resolution_clock::now();
    const auto duration =
        std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
    const auto solution_length = ida_star_.Solution().size();
    std::cout << "Time elapsed: " << duration << " ms" << std::endl;
    std::cout << "Solution length: " << solution_length << std::endl;
    std::cout << "Node expanded: " << ida_star_.NodeExpanded() << std::endl;
}

INSTANTIATE_TEST_SUITE_P(KORF100, TestModifiedPuzzle,
                         ::testing::Range<std::size_t>(0, kNumInstances - 1), GenerateTestName);
