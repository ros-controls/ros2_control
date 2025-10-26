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

#include <algorithm>
#include <limits>
#include <memory>

#include "hardware_interface/introspection.hpp"
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
 *  A class for calculating moving average statistics. This operates in constant memory and constant
 * time. Note: reset() must be called manually in order to start a new measurement window.
 *
 *  The statistics calculated are average, maximum, minimum, and standard deviation (population).
 *  All are calculated online without storing the observation data. Specifically, the average is a
 * running sum and the variance is obtained by Welford's online algorithm (reference:
 * https://en.wikipedia.org/wiki/Algorithms_for_calculating_variance#Welford%27s_online_algorithm)
 *  for standard deviation.
 *
 *  When statistics are not available, e.g. no observations have been made, NaNs are returned.
 */
class MovingAverageStatistics
{
public:
  using StatisticData = libstatistics_collector::moving_average_statistics::StatisticData;
  MovingAverageStatistics() = default;

  ~MovingAverageStatistics() = default;

  /**
   *  Returns the arithmetic mean of all data recorded. If no observations have been made, returns
   * NaN.
   *
   *  @return The arithmetic mean of all data recorded, or NaN if the sample count is 0.
   */
  double get_average() const
  {
    std::lock_guard<DEFAULT_MUTEX> lock(mutex_);
    return statistics_data_.average;
  }

  /**
   *  Returns the maximum value recorded. If size of list is zero, returns NaN.
   *
   *  @return The maximum value recorded, or NaN if size of data is zero.
   */
  double get_max() const
  {
    std::lock_guard<DEFAULT_MUTEX> lock(mutex_);
    return statistics_data_.max;
  }

  /**
   *  Returns the minimum value recorded. If size of list is zero, returns NaN.
   *
   *  @return The minimum value recorded, or NaN if size of data is zero.
   */
  double get_min() const
  {
    std::lock_guard<DEFAULT_MUTEX> lock(mutex_);
    return statistics_data_.min;
  }

  /**
   *  Returns the standard deviation (population) of all data recorded. If size of list is zero,
   * returns NaN.
   *
   *  Variance is obtained by Welford's online algorithm,
   *  see
   * https://en.wikipedia.org/wiki/Algorithms_for_calculating_variance#Welford%27s_online_algorithm
   *
   *  @return The standard deviation (population) of all data recorded, or NaN if size of data is
   * zero.
   */
  double get_standard_deviation() const
  {
    std::lock_guard<DEFAULT_MUTEX> lock(mutex_);
    return statistics_data_.standard_deviation;
  }

  /**
   *  Return a StatisticData object, containing average, minimum, maximum, standard deviation
   * (population), and sample count. For the case of no observations, the average, min, max, and
   * standard deviation are NaN.
   *
   *  @return StatisticData object, containing average, minimum, maximum, standard deviation
   * (population), and sample count.
   */
  const StatisticData & get_statistics_const_ptr() const
  {
    std::lock_guard<DEFAULT_MUTEX> lock(mutex_);
    return statistics_data_;
  }

  StatisticData get_statistics() const
  {
    std::lock_guard<DEFAULT_MUTEX> lock(mutex_);
    return statistics_data_;
  }

  /**
   *  Get the current measurement value.
   *  This is the last value added to the statistics collector.
   *
   *  @return The current measurement value, or NaN if no measurements have been made.
   */
  const double & get_current_measurement_const_ptr() const
  {
    std::lock_guard<DEFAULT_MUTEX> lock(mutex_);
    return current_measurement_;
  }

  double get_current_measurement() const
  {
    std::lock_guard<DEFAULT_MUTEX> lock(mutex_);
    return current_measurement_;
  }

  /**
   *  Reset all calculated values. Equivalent to a new window for a moving average.
   */
  void reset()
  {
    std::lock_guard<DEFAULT_MUTEX> lock(mutex_);
    statistics_data_.average = 0.0;
    statistics_data_.min = std::numeric_limits<double>::max();
    statistics_data_.max = std::numeric_limits<double>::lowest();
    statistics_data_.standard_deviation = 0.0;
    statistics_data_.sample_count = 0;
    current_measurement_ = std::numeric_limits<double>::quiet_NaN();
    sum_of_square_diff_from_mean_ = 0;
  }

  void reset_current_measurement()
  {
    std::lock_guard<DEFAULT_MUTEX> lock(mutex_);
    current_measurement_ = 0.0;
  }

  /**
   *  Observe a sample for the given window. The input item is used to calculate statistics.
   *  Note: any input values of NaN will be discarded and not added as a measurement.
   *
   *  @param item The item that was observed
   */
  void add_measurement(const double item)
  {
    std::lock_guard<DEFAULT_MUTEX> guard{mutex_};

    current_measurement_ = item;
    if (std::isfinite(item))
    {
      statistics_data_.sample_count = statistics_data_.sample_count + 1;
      const double previous_average = statistics_data_.average;
      statistics_data_.average =
        previous_average + (current_measurement_ - previous_average) /
                             static_cast<double>(statistics_data_.sample_count);
      statistics_data_.min = std::min(statistics_data_.min, current_measurement_);
      statistics_data_.max = std::max(statistics_data_.max, current_measurement_);
      sum_of_square_diff_from_mean_ =
        sum_of_square_diff_from_mean_ + (current_measurement_ - previous_average) *
                                          (current_measurement_ - statistics_data_.average);
      statistics_data_.standard_deviation = std::sqrt(
        sum_of_square_diff_from_mean_ / static_cast<double>(statistics_data_.sample_count));
    }
  }

  /**
   * Return the number of samples observed
   *
   * @return the number of samples observed
   */
  uint64_t get_count() const
  {
    std::lock_guard<DEFAULT_MUTEX> lock(mutex_);
    return statistics_data_.sample_count;
  }

private:
  mutable DEFAULT_MUTEX mutex_;
  StatisticData statistics_data_;
  double current_measurement_ = std::numeric_limits<double>::quiet_NaN();
  double sum_of_square_diff_from_mean_ = 0.0;
};

/**
 * @brief Data structure to store the statistics of a moving average. The data is protected by a
 * mutex and the data can be updated and retrieved.
 */
class MovingAverageStatisticsData
{
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
  void update_statistics(const std::shared_ptr<MovingAverageStatistics> & statistics)
  {
    std::unique_lock<DEFAULT_MUTEX> lock(mutex_);
    if (statistics->get_count() > 0)
    {
      statistics_data_.average = statistics->get_average();
      statistics_data_.min = statistics->get_min();
      statistics_data_.max = statistics->get_max();
      statistics_data_.standard_deviation = statistics->get_standard_deviation();
      statistics_data_.sample_count = statistics->get_count();
      statistics_data_ = statistics->get_statistics();
      current_data_ = statistics->get_current_measurement();
    }
    if (statistics->get_count() >= reset_statistics_sample_count_)
    {
      statistics->reset();
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
    statistics_data_.average = std::numeric_limits<double>::quiet_NaN();
    statistics_data_.min = std::numeric_limits<double>::quiet_NaN();
    statistics_data_.max = std::numeric_limits<double>::quiet_NaN();
    statistics_data_.standard_deviation = std::numeric_limits<double>::quiet_NaN();
    statistics_data_.sample_count = 0;
  }

  /**
   * @brief Get the statistics data.
   * @return statistics data.
   */
  const StatisticData & get_statistics() const
  {
    std::unique_lock<DEFAULT_MUTEX> lock(mutex_);
    return statistics_data_;
  }

  const double & get_current_data() const
  {
    std::unique_lock<DEFAULT_MUTEX> lock(mutex_);
    return current_data_;
  }

private:
  /// Mutex to protect the statistics data
  mutable DEFAULT_MUTEX mutex_;
  /// Statistics data
  StatisticData statistics_data_;
  /// Current data value, used to calculate the statistics
  double current_data_ = std::numeric_limits<double>::quiet_NaN();
  /// Number of samples to reset the statistics
  unsigned int reset_statistics_sample_count_ = std::numeric_limits<unsigned int>::max();
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
    execution_time = std::make_shared<ros2_control::MovingAverageStatistics>();
    periodicity = std::make_shared<ros2_control::MovingAverageStatistics>();
  }

  /**
   * @brief Resets the internal statistics of the execution time and periodicity statistics
   * collectors.
   */
  void reset_statistics()
  {
    execution_time->reset();
    periodicity->reset();
  }

  /// Execution time statistics collector
  std::shared_ptr<ros2_control::MovingAverageStatistics> execution_time = nullptr;
  /// Periodicity statistics collector
  std::shared_ptr<ros2_control::MovingAverageStatistics> periodicity = nullptr;
};
}  // namespace hardware_interface

#endif  // HARDWARE_INTERFACE__TYPES__STATISTICS_TYPES_HPP_
