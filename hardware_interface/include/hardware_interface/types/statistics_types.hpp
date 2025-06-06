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
#if !defined(_WIN32) && !defined(__APPLE__)
#include "realtime_tools/mutex.hpp"
#define DEFAULT_MUTEX realtime_tools::prio_inherit_mutex
#else
#define DEFAULT_MUTEX std::mutex
#endif

namespace ros2_control
{
/**
 * @brief Data structure to store the statistics of a moving average. The data is protected by a
 * mutex and the data can be updated and retrieved.
 */
class MovingAverageStatisticsData
{
  using MovingAverageStatisticsCollector =
    libstatistics_collector::moving_average_statistics::MovingAverageStatistics;
  using StatisticData = libstatistics_collector::moving_average_statistics::StatisticData;

public:
  MovingAverageStatisticsData()
  {
    reset();
    reset_statistics_sample_count_ = std::numeric_limits<unsigned int>::max();
  }

  /**
   * @brief Update the statistics data with the new statistics data.
   * @param statistics statistics collector to update the current statistics data.
   */
  void update_statistics(const std::shared_ptr<MovingAverageStatisticsCollector> & statistics)
  {
    std::unique_lock<DEFAULT_MUTEX> lock(mutex_);
    if (statistics->GetCount() > 0)
    {
      statistics_data.average = statistics->Average();
      statistics_data.min = statistics->Min();
      statistics_data.max = statistics->Max();
      statistics_data.standard_deviation = statistics->StandardDeviation();
      statistics_data.sample_count = statistics->GetCount();
      statistics_data = statistics->GetStatistics();
    }
    if (statistics->GetCount() >= reset_statistics_sample_count_)
    {
      statistics->Reset();
    }
  }

  /**
   * @brief Set the number of samples to reset the statistics.
   * @param reset_sample_count number of samples to reset the statistics.
   */
  void set_reset_statistics_sample_count(unsigned int reset_sample_count)
  {
    std::unique_lock<DEFAULT_MUTEX> lock(mutex_);
    reset_statistics_sample_count_ = reset_sample_count;
  }

  void reset()
  {
    statistics_data.average = std::numeric_limits<double>::quiet_NaN();
    statistics_data.min = std::numeric_limits<double>::quiet_NaN();
    statistics_data.max = std::numeric_limits<double>::quiet_NaN();
    statistics_data.standard_deviation = std::numeric_limits<double>::quiet_NaN();
    statistics_data.sample_count = 0;
  }

  /**
   * @brief Get the statistics data.
   * @return statistics data.
   */
  const StatisticData & get_statistics() const
  {
    std::unique_lock<DEFAULT_MUTEX> lock(mutex_);
    return statistics_data;
  }

private:
  /// Statistics data
  StatisticData statistics_data;
  /// Number of samples to reset the statistics
  unsigned int reset_statistics_sample_count_ = std::numeric_limits<unsigned int>::max();
  /// Mutex to protect the statistics data
  mutable DEFAULT_MUTEX mutex_;
};
}  // namespace ros2_control

namespace hardware_interface
{
/**
 * @brief Data structure with two moving average statistics collectors. One for the execution time
 * and the other for the periodicity.
 */
struct HardwareComponentStatisticsCollector
{
  HardwareComponentStatisticsCollector()
  {
    execution_time = std::make_shared<MovingAverageStatisticsCollector>();
    periodicity = std::make_shared<MovingAverageStatisticsCollector>();
  }

  using MovingAverageStatisticsCollector =
    libstatistics_collector::moving_average_statistics::MovingAverageStatistics;

  /**
   * @brief Resets the internal statistics of the execution time and periodicity statistics
   * collectors.
   */
  void reset_statistics()
  {
    execution_time->Reset();
    periodicity->Reset();
  }

  /// Execution time statistics collector
  std::shared_ptr<MovingAverageStatisticsCollector> execution_time = nullptr;
  /// Periodicity statistics collector
  std::shared_ptr<MovingAverageStatisticsCollector> periodicity = nullptr;
};
}  // namespace hardware_interface

#endif  // HARDWARE_INTERFACE__TYPES__STATISTICS_TYPES_HPP_
