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

#include <chrono>

#include "parallel_ida_star.h"
#include "pdb.h"
#include "test_utils.h"

class TestPDBStandardPuzzle : public testing::TestWithParam<std::size_t> {
protected:
    static void SetUpTestSuite() {
        puzzle_ = new stp::Puzzle<kSize, kSize>();
        heuristic_ = new stp::algorithm::AdditivePDBHeuristic<kSize, kSize>();
        auto pdb1 =
            stp::algorithm::PatternDatabase(*puzzle_, std::vector<uint8_t>{0, 1, 2, 3, 4, 5, 6, 7});
        auto pdb2 = stp::algorithm::PatternDatabase(
            *puzzle_, std::vector<uint8_t>{0, 8, 9, 10, 11, 12, 13, 14, 15});
        heuristic_->Add(pdb1);
        heuristic_->Add(pdb2);
        auto& pdb = heuristic_->PDBs();
        std::cout << "Building PDBs before testing..." << std::endl;
        auto start_time = std::chrono::high_resolution_clock::now();
        pdb[0].Build();
        auto end_time = std::chrono::high_resolution_clock::now();
        auto elapsed_time =
            std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count();
        std::cout << "PDB1 finished, time elapsed: " << elapsed_time << " ms" << std::endl;
        start_time = std::chrono::high_resolution_clock::now();
        pdb[1].Build();
        end_time = std::chrono::high_resolution_clock::now();
        elapsed_time =
            std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count();
        std::cout << "PDB2 finished, time elapsed: " << elapsed_time << " ms" << std::endl;
        ida_star_ = new stp::algorithm::ParallelIDAStar(*puzzle_);
        ida_star_->SetHeuristic(
            [](const stp::State<kSize, kSize>& state) { return heuristic_->HCost(state); });
    }

    static void TearDownTestSuite() {
        delete puzzle_;
        delete heuristic_;
        delete ida_star_;
        puzzle_ = nullptr;
        heuristic_ = nullptr;
        ida_star_ = nullptr;
    }

    static stp::Puzzle<kSize, kSize>* puzzle_;
    static stp::algorithm::AdditivePDBHeuristic<kSize, kSize>* heuristic_;
    static stp::algorithm::ParallelIDAStar<kSize, kSize>* ida_star_;
};

stp::Puzzle<kSize, kSize>* TestPDBStandardPuzzle::puzzle_ = nullptr;
stp::algorithm::AdditivePDBHeuristic<kSize, kSize>* TestPDBStandardPuzzle::heuristic_ = nullptr;
stp::algorithm::ParallelIDAStar<kSize, kSize>* TestPDBStandardPuzzle::ida_star_ = nullptr;

TEST_P(TestPDBStandardPuzzle, TestInstance) {
    const auto [instance, length] = kInstances[GetParam()];
    auto state = stp::State<kSize, kSize>(instance);
    std::cout << state << std::endl;
    const auto start = std::chrono::high_resolution_clock::now();
    ASSERT_TRUE((*ida_star_)(state));
    const auto end = std::chrono::high_resolution_clock::now();
    const auto duration =
        std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
    const auto solution_length = ida_star_->Solution().size();
    EXPECT_EQ(solution_length, length);
    std::cout << "Time elapsed: " << duration << " ms" << std::endl;
    std::cout << "Solution length: " << solution_length << std::endl;
    std::cout << "Node expanded: " << ida_star_->NodeExpanded() << std::endl;
}

INSTANTIATE_TEST_SUITE_P(KORF100, TestPDBStandardPuzzle,
                         ::testing::Range<std::size_t>(0, kNumInstances), GenerateTestName);
