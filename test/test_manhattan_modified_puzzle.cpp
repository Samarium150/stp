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
                         ::testing::Range<std::size_t>(0, kNumInstances), GenerateTestName);
