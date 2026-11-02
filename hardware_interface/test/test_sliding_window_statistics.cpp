// Copyright 2026 ros2_control Development Team
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <gmock/gmock.h>

#include <cmath>
#include <limits>
#include <random>
#include <vector>

#include "hardware_interface/types/statistics_types.hpp"

using ros2_control::MovingAverageStatistics;
using ros2_control::SlidingWindowStatistics;

namespace
{
struct Reference
{
  double average;
  double min;
  double max;
  double standard_deviation;
};

Reference reference(const std::vector<double> & values)
{
  double sum = 0.0;
  double min = std::numeric_limits<double>::max();
  double max = std::numeric_limits<double>::lowest();
  for (const double v : values)
  {
    sum += v;
    min = std::min(min, v);
    max = std::max(max, v);
  }
  const double average = sum / static_cast<double>(values.size());
  double square_diff = 0.0;
  for (const double v : values)
  {
    square_diff += (v - average) * (v - average);
  }
  return {average, min, max, std::sqrt(square_diff / static_cast<double>(values.size()))};
}
}  // namespace

TEST(TestSlidingWindowStatistics, empty_window_is_nan)
{
  SlidingWindowStatistics stats(5);
  const auto data = stats.get_statistics();
  EXPECT_EQ(data.sample_count, 0u);
  EXPECT_TRUE(std::isnan(data.average));
  EXPECT_TRUE(std::isnan(data.min));
  EXPECT_TRUE(std::isnan(data.max));
  EXPECT_TRUE(std::isnan(data.standard_deviation));
}

TEST(TestSlidingWindowStatistics, zero_window_size_is_disabled)
{
  SlidingWindowStatistics stats;
  EXPECT_EQ(stats.get_window_size(), 0u);
  stats.add_measurement(1.0);
  EXPECT_EQ(stats.get_count(), 0u);
  EXPECT_TRUE(std::isnan(stats.get_statistics().average));
}

TEST(TestSlidingWindowStatistics, partially_filled_window_uses_only_observed_samples)
{
  SlidingWindowStatistics stats(10);
  for (const double v : {2.0, 4.0, 4.0, 4.0, 5.0, 5.0, 7.0, 9.0})
  {
    stats.add_measurement(v);
  }
  const auto data = stats.get_statistics();
  EXPECT_EQ(data.sample_count, 8u);
  EXPECT_DOUBLE_EQ(data.average, 5.0);
  EXPECT_DOUBLE_EQ(data.min, 2.0);
  EXPECT_DOUBLE_EQ(data.max, 9.0);
  EXPECT_DOUBLE_EQ(data.standard_deviation, 2.0);
}

TEST(TestSlidingWindowStatistics, outlier_leaves_window_after_window_size_samples)
{
  constexpr std::size_t window = 100;
  SlidingWindowStatistics recent(window);
  MovingAverageStatistics since_startup;
  since_startup.reset();

  recent.add_measurement(8.9);
  since_startup.add_measurement(8.9);
  for (std::size_t i = 0; i < window; ++i)
  {
    recent.add_measurement(500.0);
    since_startup.add_measurement(500.0);
  }

  const auto data = recent.get_statistics();
  EXPECT_EQ(data.sample_count, window);
  EXPECT_DOUBLE_EQ(data.average, 500.0);
  EXPECT_DOUBLE_EQ(data.min, 500.0);
  EXPECT_DOUBLE_EQ(data.max, 500.0);
  EXPECT_NEAR(data.standard_deviation, 0.0, 1e-6);

  EXPECT_DOUBLE_EQ(since_startup.get_min(), 8.9);
  EXPECT_GT(since_startup.get_standard_deviation(), 40.0);
}

TEST(TestSlidingWindowStatistics, extremes_are_recomputed_when_evicted)
{
  SlidingWindowStatistics stats(3);
  for (const double v : {10.0, 1.0, 5.0})
  {
    stats.add_measurement(v);
  }
  EXPECT_DOUBLE_EQ(stats.get_statistics().max, 10.0);

  stats.add_measurement(4.0);
  EXPECT_DOUBLE_EQ(stats.get_statistics().max, 5.0);
  EXPECT_DOUBLE_EQ(stats.get_statistics().min, 1.0);

  stats.add_measurement(6.0);
  EXPECT_DOUBLE_EQ(stats.get_statistics().min, 4.0);
  EXPECT_DOUBLE_EQ(stats.get_statistics().max, 6.0);
}

TEST(TestSlidingWindowStatistics, matches_reference_over_a_long_random_run)
{
  constexpr std::size_t window = 37;
  SlidingWindowStatistics stats(window);
  std::mt19937 rng(42);
  std::normal_distribution<double> noise(500.0, 3.0);
  std::vector<double> all;
  for (int i = 0; i < 100000; ++i)
  {
    const double v = noise(rng);
    all.push_back(v);
    stats.add_measurement(v);
    if (i % 997 == 0 || i == 99999)
    {
      const std::size_t n = std::min(all.size(), window);
      const auto expected = reference(std::vector<double>(all.end() - n, all.end()));
      const auto data = stats.get_statistics();
      ASSERT_EQ(data.sample_count, n);
      EXPECT_NEAR(data.average, expected.average, 1e-9);
      EXPECT_DOUBLE_EQ(data.min, expected.min);
      EXPECT_DOUBLE_EQ(data.max, expected.max);
      EXPECT_NEAR(data.standard_deviation, expected.standard_deviation, 1e-6);
    }
  }
}

TEST(TestSlidingWindowStatistics, non_finite_samples_are_ignored)
{
  SlidingWindowStatistics stats(4);
  stats.add_measurement(1.0);
  stats.add_measurement(std::numeric_limits<double>::quiet_NaN());
  stats.add_measurement(std::numeric_limits<double>::infinity());
  stats.add_measurement(3.0);
  const auto data = stats.get_statistics();
  EXPECT_EQ(data.sample_count, 2u);
  EXPECT_DOUBLE_EQ(data.average, 2.0);
}

TEST(TestSlidingWindowStatistics, reset_and_resize_clear_the_window)
{
  SlidingWindowStatistics stats(4);
  stats.add_measurement(1.0);
  stats.add_measurement(2.0);
  stats.reset();
  EXPECT_EQ(stats.get_count(), 0u);
  EXPECT_EQ(stats.get_window_size(), 4u);

  stats.add_measurement(7.0);
  stats.set_window_size(2);
  EXPECT_EQ(stats.get_count(), 0u);
  EXPECT_EQ(stats.get_window_size(), 2u);
  for (const double v : {1.0, 2.0, 3.0})
  {
    stats.add_measurement(v);
  }
  EXPECT_DOUBLE_EQ(stats.get_statistics().average, 2.5);
}
