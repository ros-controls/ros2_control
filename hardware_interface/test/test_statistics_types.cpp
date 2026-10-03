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

#include <atomic>
#include <cmath>
#include <memory>
#include <thread>

#include "gmock/gmock.h"
#include "hardware_interface/types/statistics_types.hpp"

using ros2_control::MovingAverageStatistics;
using ros2_control::MovingAverageStatisticsData;

TEST(TestMovingAverageStatisticsData, get_statistics_returns_snapshot)
{
  auto collector = std::make_shared<MovingAverageStatistics>();
  collector->reset();
  MovingAverageStatisticsData data;

  collector->add_measurement(1.0);
  data.update_statistics(collector);
  const auto & snapshot = data.get_statistics();
  const auto & current = data.get_current_data();
  EXPECT_EQ(snapshot.sample_count, 1u);
  EXPECT_DOUBLE_EQ(snapshot.average, 1.0);
  EXPECT_DOUBLE_EQ(current, 1.0);

  collector->add_measurement(3.0);
  data.update_statistics(collector);

  // The previously returned values must not change when the statistics are updated.
  EXPECT_EQ(snapshot.sample_count, 1u);
  EXPECT_DOUBLE_EQ(snapshot.average, 1.0);
  EXPECT_DOUBLE_EQ(current, 1.0);

  EXPECT_EQ(data.get_statistics().sample_count, 2u);
  EXPECT_DOUBLE_EQ(data.get_statistics().average, 2.0);
  EXPECT_DOUBLE_EQ(data.get_current_data(), 3.0);
}

TEST(TestMovingAverageStatisticsData, concurrent_update_and_read)
{
  auto collector = std::make_shared<MovingAverageStatistics>();
  collector->reset();
  MovingAverageStatisticsData data;
  std::atomic<bool> done{false};
  constexpr uint64_t kSamples = 20000;

  std::thread writer(
    [&]()
    {
      for (uint64_t i = 1; i <= kSamples; ++i)
      {
        collector->add_measurement(static_cast<double>(i));
        data.update_statistics(collector);
      }
      done = true;
    });

  uint64_t reads = 0;
  while (!done)
  {
    const auto stats = data.get_statistics();
    const double current = data.get_current_data();
    if (stats.sample_count > 0)
    {
      // Every snapshot must be internally consistent: measurements are 1..n
      EXPECT_DOUBLE_EQ(stats.min, 1.0);
      EXPECT_DOUBLE_EQ(stats.max, static_cast<double>(stats.sample_count));
      EXPECT_FALSE(std::isnan(current));
    }
    ++reads;
  }
  writer.join();

  EXPECT_GT(reads, 0u);
  EXPECT_EQ(data.get_statistics().sample_count, kSamples);
  EXPECT_DOUBLE_EQ(data.get_current_data(), static_cast<double>(kSamples));
}
