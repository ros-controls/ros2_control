// Copyright 2025 PAL Robotics S.L.
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

/// \author Sai Kishor Kothakota

#ifndef HARDWARE_INTERFACE__TYPES__STATISTICS_TYPES_HPP_
#define HARDWARE_INTERFACE__TYPES__STATISTICS_TYPES_HPP_

#include <limits>
#include <memory>

#include "libstatistics_collector/moving_average_statistics/moving_average.hpp"
#include "libstatistics_collector/moving_average_statistics/types.hpp"
#include "realtime_tools/mutex.hpp"

namespace ros2_control
{
struct MovingAverageStatisticsData
{
  using MovingAverageStatistics =
    libstatistics_collector::moving_average_statistics::MovingAverageStatistics;
  using StatisticData = libstatistics_collector::moving_average_statistics::StatisticData;

public:
  MovingAverageStatisticsData() { reset(); }

  void update_statistics(std::shared_ptr<MovingAverageStatistics> & statistics)
  {
    std::unique_lock<realtime_tools::prio_inherit_mutex> lock(mutex_);
    statistics_data.average = statistics->Average();
    statistics_data.min = statistics->Min();
    statistics_data.max = statistics->Max();
    statistics_data.standard_deviation = statistics->StandardDeviation();
    statistics_data.sample_count = statistics->GetCount();
    statistics_data = statistics->GetStatistics();
  }

  void reset()
  {
    statistics_data.average = std::numeric_limits<double>::quiet_NaN();
    statistics_data.min = std::numeric_limits<double>::quiet_NaN();
    statistics_data.max = std::numeric_limits<double>::quiet_NaN();
    statistics_data.standard_deviation = std::numeric_limits<double>::quiet_NaN();
    statistics_data.sample_count = 0;
  }

  const StatisticData & get_statistics() const
  {
    std::unique_lock<realtime_tools::prio_inherit_mutex> lock(mutex_);
    return statistics_data;
  }

private:
  StatisticData statistics_data;
  mutable realtime_tools::prio_inherit_mutex mutex_;
};
}  // namespace ros2_control

namespace hardware_interface
{
struct HardwareComponentStatisticsCollector
{
  HardwareComponentStatisticsCollector()
  {
    execution_time = std::make_shared<MovingAverageStatistics>();
    periodicity = std::make_shared<MovingAverageStatistics>();
  }

  using MovingAverageStatistics =
    libstatistics_collector::moving_average_statistics::MovingAverageStatistics;

  void reset_statistics()
  {
    execution_time->Reset();
    periodicity->Reset();
  }

  std::shared_ptr<MovingAverageStatistics> execution_time = nullptr;
  std::shared_ptr<MovingAverageStatistics> periodicity = nullptr;
};
}  // namespace hardware_interface

#endif  // HARDWARE_INTERFACE__TYPES__STATISTICS_TYPES_HPP_
