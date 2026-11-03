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

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <random>
#include <vector>

#include "hardware_interface/types/statistics_types.hpp"

using ros2_control::TimeWindowStatistics;

namespace
{
void expect_nan_statistics(const TimeWindowStatistics::StatisticData & data)
{
  EXPECT_EQ(data.sample_count, 0u);
  EXPECT_TRUE(std::isnan(data.average));
  EXPECT_TRUE(std::isnan(data.min));
  EXPECT_TRUE(std::isnan(data.max));
  EXPECT_TRUE(std::isnan(data.standard_deviation));
}
}  // namespace

TEST(TestTimeWindowStatistics, disabled_for_non_positive_duration)
{
  TimeWindowStatistics stats;
  EXPECT_FALSE(stats.is_enabled());
  stats.add_measurement(1.0, 0.0);
  expect_nan_statistics(stats.get_statistics());

  for (const double duration : {0.0, -1.0, -0.5})
  {
    stats.set_window_duration(duration);
    EXPECT_FALSE(stats.is_enabled());
    stats.add_measurement(1.0, 0.0);
    expect_nan_statistics(stats.get_statistics());
  }
}

TEST(TestTimeWindowStatistics, empty_window_is_nan)
{
  TimeWindowStatistics stats(1.0);
  EXPECT_TRUE(stats.is_enabled());
  EXPECT_DOUBLE_EQ(stats.get_window_duration(), 1.0);
  expect_nan_statistics(stats.get_statistics());
}

TEST(TestTimeWindowStatistics, statistics_across_buckets_match_population_statistics)
{
  // Samples spread over several buckets of a 1 s window, all within the window
  TimeWindowStatistics stats(1.0);
  const std::vector<double> values = {2.0, 4.0, 4.0, 4.0, 5.0, 5.0, 7.0, 9.0};
  for (std::size_t i = 0; i < values.size(); ++i)
  {
    stats.add_measurement(values[i], 0.1 * static_cast<double>(i));
  }
  const auto data = stats.get_statistics();
  EXPECT_EQ(data.sample_count, values.size());
  EXPECT_DOUBLE_EQ(data.average, 5.0);
  EXPECT_DOUBLE_EQ(data.min, 2.0);
  EXPECT_DOUBLE_EQ(data.max, 9.0);
  EXPECT_NEAR(data.standard_deviation, 2.0, 1e-12);
}

TEST(TestTimeWindowStatistics, matches_reference_over_a_long_random_run)
{
  constexpr double duration = 0.5;
  constexpr double rate = 1000.0;
  TimeWindowStatistics stats(duration);
  std::mt19937 rng(42);
  std::normal_distribution<double> noise(rate, 3.0);
  std::vector<double> all;
  for (int i = 0; i < 20000; ++i)
  {
    all.push_back(noise(rng));
    stats.add_measurement(all.back(), static_cast<double>(i) / rate);
  }
  // The window covers at least (1 - 1/BUCKETS) of the duration and at most the whole duration
  const auto data = stats.get_statistics();
  const auto min_samples = static_cast<uint64_t>(
    duration * rate * (1.0 - 1.0 / static_cast<double>(TimeWindowStatistics::NUM_BUCKETS)));
  const auto max_samples = static_cast<uint64_t>(duration * rate) + 1;
  ASSERT_GE(data.sample_count, min_samples);
  ASSERT_LE(data.sample_count, max_samples);

  const std::vector<double> window(
    all.end() - static_cast<std::ptrdiff_t>(data.sample_count), all.end());
  double sum = 0.0;
  for (const double v : window)
  {
    sum += v;
  }
  const double average = sum / static_cast<double>(window.size());
  double square_diff = 0.0;
  for (const double v : window)
  {
    square_diff += (v - average) * (v - average);
  }
  EXPECT_NEAR(data.average, average, 1e-9);
  EXPECT_DOUBLE_EQ(data.min, *std::min_element(window.begin(), window.end()));
  EXPECT_DOUBLE_EQ(data.max, *std::max_element(window.begin(), window.end()));
  EXPECT_NEAR(
    data.standard_deviation, std::sqrt(square_diff / static_cast<double>(window.size())), 1e-9);
}

TEST(TestTimeWindowStatistics, outlier_leaves_window_after_duration)
{
  constexpr double duration = 1.0;
  constexpr double rate = 100.0;
  TimeWindowStatistics stats(duration);
  stats.add_measurement(8.0, 0.0);
  int i = 1;
  // Still within the window: the outlier dominates the minimum
  for (; i < 50; ++i)
  {
    stats.add_measurement(rate, static_cast<double>(i) / rate);
  }
  EXPECT_DOUBLE_EQ(stats.get_statistics().min, 8.0);

  // More than one duration later, only the steady samples remain
  for (; i < 250; ++i)
  {
    stats.add_measurement(rate, static_cast<double>(i) / rate);
  }
  const auto data = stats.get_statistics();
  EXPECT_DOUBLE_EQ(data.min, rate);
  EXPECT_DOUBLE_EQ(data.max, rate);
  EXPECT_DOUBLE_EQ(data.average, rate);
  EXPECT_NEAR(data.standard_deviation, 0.0, 1e-12);
  EXPECT_GE(data.sample_count, 90u);
  EXPECT_LE(data.sample_count, 101u);
}

TEST(TestTimeWindowStatistics, gap_longer_than_window_drops_all_old_samples)
{
  TimeWindowStatistics stats(1.0);
  stats.add_measurement(1.0, 0.0);
  stats.add_measurement(2.0, 0.5);
  stats.add_measurement(3.0, 10.0);
  const auto data = stats.get_statistics();
  EXPECT_EQ(data.sample_count, 1u);
  EXPECT_DOUBLE_EQ(data.average, 3.0);
}

TEST(TestTimeWindowStatistics, time_going_backwards_restarts_the_window)
{
  // e.g. a simulation that is reset
  TimeWindowStatistics stats(1.0);
  stats.add_measurement(1.0, 100.0);
  stats.add_measurement(5.0, 1.0);
  const auto data = stats.get_statistics();
  EXPECT_EQ(data.sample_count, 1u);
  EXPECT_DOUBLE_EQ(data.average, 5.0);
}

TEST(TestTimeWindowStatistics, non_finite_samples_are_ignored)
{
  TimeWindowStatistics stats(1.0);
  stats.add_measurement(1.0, 0.0);
  stats.add_measurement(std::numeric_limits<double>::quiet_NaN(), 0.1);
  stats.add_measurement(std::numeric_limits<double>::infinity(), 0.2);
  stats.add_measurement(5.0, std::numeric_limits<double>::quiet_NaN());
  stats.add_measurement(3.0, 0.3);
  const auto data = stats.get_statistics();
  EXPECT_EQ(data.sample_count, 2u);
  EXPECT_DOUBLE_EQ(data.average, 2.0);
}

TEST(TestTimeWindowStatistics, reset_and_set_duration_clear_the_window)
{
  TimeWindowStatistics stats(1.0);
  stats.add_measurement(1.0, 0.0);
  stats.reset();
  expect_nan_statistics(stats.get_statistics());
  EXPECT_TRUE(stats.is_enabled());

  stats.add_measurement(1.0, 0.0);
  stats.set_window_duration(2.0);
  expect_nan_statistics(stats.get_statistics());
  EXPECT_DOUBLE_EQ(stats.get_window_duration(), 2.0);
}
