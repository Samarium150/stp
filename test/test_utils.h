#ifndef TEST_UTILS_H_
#define TEST_UTILS_H_

#include <gtest/gtest.h>

#include <array>

constexpr uint8_t kSize = 4;

using Instance = std::array<uint8_t, kSize * kSize>;
using Length = std::size_t;
using Param = std::pair<Instance, Length>;

constexpr unsigned kNumInstances = 100;

extern const std::array<Param, kNumInstances> kInstances;

std::string GenerateTestName(const testing::TestParamInfo<std::size_t>& info);

#endif  // TEST_UTILS_H_
