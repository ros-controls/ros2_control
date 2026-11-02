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
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <memory>
#include <mutex>
#include <vector>

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
 *  Statistics over the most recent `window_size` finite samples: average, minimum, maximum and
 * standard deviation (population). Unlike MovingAverageStatistics, an old outlier leaves the
 * statistics once `window_size` newer samples have arrived, so they describe the recent behaviour
 * rather than the whole history.
 *
 *  The samples live in a ring buffer allocated by set_window_size(), so add_measurement() never
 * allocates. It keeps a running sum and sum of squares for the average and standard deviation, and
 * rescans the window (O(window_size)) only when the evicted sample was the current minimum or
 * maximum, or when the ring wraps. The wrap rescan stops rounding error from accumulating in the
 * running sums over a long run, and keeps add_measurement() amortized O(1).
 *
 *  A window size of 0 disables the collector: add_measurement() is a no-op and the statistics stay
 * NaN. When statistics are not available, e.g. no observations have been made, NaNs are returned.
 */
class SlidingWindowStatistics
{
public:
  using StatisticData = libstatistics_collector::moving_average_statistics::StatisticData;

  SlidingWindowStatistics() { reset_locked(); }

  explicit SlidingWindowStatistics(std::size_t window_size) { set_window_size(window_size); }

  /**
   *  Set the number of samples the statistics are computed over, and clear the window. Allocates;
   * call it outside the real-time loop.
   *
   *  @param window_size Number of most recent samples to keep. 0 disables the collector.
   */
  void set_window_size(std::size_t window_size)
  {
    std::lock_guard<DEFAULT_MUTEX> lock(mutex_);
    samples_.assign(window_size, 0.0);
    reset_locked();
  }

  std::size_t get_window_size() const
  {
    std::lock_guard<DEFAULT_MUTEX> lock(mutex_);
    return samples_.size();
  }

  /**
   *  Clear the window, keeping its size.
   */
  void reset()
  {
    std::lock_guard<DEFAULT_MUTEX> lock(mutex_);
    reset_locked();
  }

  /**
   *  Observe a sample. Once the window is full, the oldest sample is evicted.
   *  Note: any input values of NaN or infinity will be discarded and not added as a measurement.
   *
   *  @param item The item that was observed
   */
  void add_measurement(const double item)
  {
    std::lock_guard<DEFAULT_MUTEX> guard{mutex_};
    const std::size_t capacity = samples_.size();
    if (capacity == 0 || !std::isfinite(item))
    {
      return;
    }

    bool rescan = false;
    if (count_ == capacity)
    {
      const double evicted = samples_[next_];
      sum_ -= evicted;
      sum_of_squares_ -= evicted * evicted;
      rescan = evicted <= min_ || evicted >= max_;
    }
    else
    {
      ++count_;
    }

    samples_[next_] = item;
    next_ = (next_ + 1) % capacity;
    rescan = rescan || next_ == 0;

    if (rescan)
    {
      recompute();
    }
    else
    {
      sum_ += item;
      sum_of_squares_ += item * item;
      min_ = std::min(min_, item);
      max_ = std::max(max_, item);
    }
  }

  /**
   *  Return a StatisticData object, containing average, minimum, maximum, standard deviation
   * (population) and sample count over the current window. For an empty window, the average, min,
   * max and standard deviation are NaN.
   */
  StatisticData get_statistics() const
  {
    std::lock_guard<DEFAULT_MUTEX> lock(mutex_);
    StatisticData data;
    data.sample_count = count_;
    if (count_ == 0)
    {
      data.average = std::numeric_limits<double>::quiet_NaN();
      data.min = std::numeric_limits<double>::quiet_NaN();
      data.max = std::numeric_limits<double>::quiet_NaN();
      data.standard_deviation = std::numeric_limits<double>::quiet_NaN();
      return data;
    }
    const double n = static_cast<double>(count_);
    data.average = sum_ / n;
    data.min = min_;
    data.max = max_;
    // Floating-point cancellation can leave a tiny negative variance for a near-constant signal.
    data.standard_deviation =
      std::sqrt(std::max(0.0, sum_of_squares_ / n - data.average * data.average));
    return data;
  }

  uint64_t get_count() const
  {
    std::lock_guard<DEFAULT_MUTEX> lock(mutex_);
    return count_;
  }

private:
  void reset_locked()
  {
    count_ = 0;
    next_ = 0;
    sum_ = 0.0;
    sum_of_squares_ = 0.0;
    min_ = std::numeric_limits<double>::max();
    max_ = std::numeric_limits<double>::lowest();
  }

  // The filled slots are always [0, count_): the ring only starts overwriting once it is full.
  void recompute()
  {
    sum_ = 0.0;
    sum_of_squares_ = 0.0;
    min_ = std::numeric_limits<double>::max();
    max_ = std::numeric_limits<double>::lowest();
    for (std::size_t i = 0; i < count_; ++i)
    {
      const double sample = samples_[i];
      sum_ += sample;
      sum_of_squares_ += sample * sample;
      min_ = std::min(min_, sample);
      max_ = std::max(max_, sample);
    }
  }

  mutable DEFAULT_MUTEX mutex_;
  std::vector<double> samples_;
  std::size_t count_ = 0;
  std::size_t next_ = 0;
  double sum_ = 0.0;
  double sum_of_squares_ = 0.0;
  double min_ = std::numeric_limits<double>::max();
  double max_ = std::numeric_limits<double>::lowest();
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
