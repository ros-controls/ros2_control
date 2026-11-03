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
#include <array>
#include <atomic>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <memory>
#include <mutex>

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
 *  Statistics (average, minimum, maximum, standard deviation (population) and sample count) over
 * the samples observed within the most recent `window_duration` seconds. Unlike
 * MovingAverageStatistics, an old outlier leaves the statistics once the window has moved past it,
 * so they describe the recent behaviour rather than the whole history.
 *
 *  The window is split into NUM_BUCKETS time buckets, each holding the running statistics of its
 * samples, so the memory is constant and independent of the sample rate, and add_measurement() is
 * O(1) and never allocates. The price is the resolution of the window's trailing edge: it advances
 * one bucket at a time, so the statistics cover between (NUM_BUCKETS - 1) / NUM_BUCKETS of the
 * duration and the full duration.
 *
 *  The time of a sample is given by the caller, so the window follows whichever clock drives the
 * measurements (e.g. simulation time). If the time goes back past the current bucket (e.g. a reset
 * simulation), the window is restarted.
 *
 *  A non-positive (or non-finite) duration disables the collector: add_measurement() is a no-op.
 * When statistics are not available, e.g. no observations have been made, NaNs are returned.
 */
class TimeWindowStatistics
{
public:
  using StatisticData = libstatistics_collector::moving_average_statistics::StatisticData;

  /// Number of time buckets the window is split into.
  static constexpr std::size_t NUM_BUCKETS = 10;

  /// Construct a disabled collector.
  TimeWindowStatistics() = default;

  /**
   *  @param window_duration Duration of the window in seconds. Non-positive disables the collector.
   */
  explicit TimeWindowStatistics(double window_duration) { set_window_duration(window_duration); }

  /**
   *  Set the duration of the window and clear it.
   *
   *  @param window_duration Duration of the window in seconds. Non-positive disables the collector.
   */
  void set_window_duration(double window_duration)
  {
    std::lock_guard<DEFAULT_MUTEX> lock(mutex_);
    window_duration_ = window_duration;
    const bool enabled = std::isfinite(window_duration) && window_duration > 0.0;
    bucket_width_ = enabled ? window_duration / static_cast<double>(NUM_BUCKETS) : 0.0;
    reset_locked();
    enabled_.store(enabled, std::memory_order_release);
  }

  /// @return The duration of the window in seconds, as set by set_window_duration().
  double get_window_duration() const
  {
    std::lock_guard<DEFAULT_MUTEX> lock(mutex_);
    return window_duration_;
  }

  /// @return true if the window duration is positive, i.e. measurements are collected.
  bool is_enabled() const { return enabled_.load(std::memory_order_acquire); }

  /// Clear the window, keeping its duration.
  void reset()
  {
    std::lock_guard<DEFAULT_MUTEX> lock(mutex_);
    reset_locked();
  }

  /**
   *  Observe a sample. Samples older than the window are dropped.
   *  Note: samples whose value or time is NaN or infinity are discarded.
   *
   *  @param item The item that was observed
   *  @param time_seconds The time at which the item was observed, in seconds
   */
  void add_measurement(const double item, const double time_seconds)
  {
    // Disabled by default: don't take the lock in the real-time loop for nothing
    if (!is_enabled() || !std::isfinite(item) || !std::isfinite(time_seconds))
    {
      return;
    }
    std::lock_guard<DEFAULT_MUTEX> guard{mutex_};
    if (bucket_width_ <= 0.0)
    {
      return;  // disabled concurrently
    }
    const auto id = static_cast<int64_t>(std::floor(time_seconds / bucket_width_));
    if (id < latest_id_)
    {
      reset_locked();
    }
    latest_id_ = std::max(latest_id_, id);

    // A slot holding another id belongs to a bucket that has already left the window
    Bucket & bucket = buckets_[slot(id)];
    if (bucket.id != id)
    {
      bucket = Bucket{};
      bucket.id = id;
    }
    // Welford's online algorithm, as in MovingAverageStatistics
    ++bucket.count;
    const double delta = item - bucket.mean;
    bucket.mean += delta / static_cast<double>(bucket.count);
    bucket.m2 += delta * (item - bucket.mean);
    bucket.min = std::min(bucket.min, item);
    bucket.max = std::max(bucket.max, item);
  }

  /**
   *  Return a StatisticData object, containing average, minimum, maximum, standard deviation
   * (population) and sample count over the current window. For an empty window, the average, min,
   * max and standard deviation are NaN.
   */
  StatisticData get_statistics() const
  {
    // Copy under the lock and merge outside of it, to keep the real-time writer's wait short
    std::array<Bucket, NUM_BUCKETS> buckets;
    int64_t latest_id;
    {
      std::lock_guard<DEFAULT_MUTEX> lock(mutex_);
      buckets = buckets_;
      latest_id = latest_id_;
    }

    uint64_t count = 0;
    double mean = 0.0;
    double m2 = 0.0;
    double min = std::numeric_limits<double>::max();
    double max = std::numeric_limits<double>::lowest();
    for (const auto & bucket : buckets)
    {
      const bool in_window = bucket.count > 0 && bucket.id <= latest_id &&
                             bucket.id > latest_id - static_cast<int64_t>(NUM_BUCKETS);
      if (!in_window)
      {
        continue;
      }
      // Merge of two populations (Chan et al.), numerically stable unlike a sum of squares
      const uint64_t merged_count = count + bucket.count;
      const double delta = bucket.mean - mean;
      const double weight = static_cast<double>(bucket.count) / static_cast<double>(merged_count);
      mean += delta * weight;
      m2 += bucket.m2 + delta * delta * static_cast<double>(count) * weight;
      count = merged_count;
      min = std::min(min, bucket.min);
      max = std::max(max, bucket.max);
    }
    StatisticData data;
    if (count == 0)
    {
      data.average = std::numeric_limits<double>::quiet_NaN();
      data.min = std::numeric_limits<double>::quiet_NaN();
      data.max = std::numeric_limits<double>::quiet_NaN();
      data.standard_deviation = std::numeric_limits<double>::quiet_NaN();
      data.sample_count = 0;
      return data;
    }
    data.average = mean;
    data.min = min;
    data.max = max;
    data.standard_deviation = std::sqrt(m2 / static_cast<double>(count));
    data.sample_count = count;
    return data;
  }

private:
  struct Bucket
  {
    int64_t id = std::numeric_limits<int64_t>::min();
    uint64_t count = 0;
    double mean = 0.0;
    double m2 = 0.0;
    double min = std::numeric_limits<double>::max();
    double max = std::numeric_limits<double>::lowest();
  };

  static std::size_t slot(int64_t id)
  {
    const auto n = static_cast<int64_t>(NUM_BUCKETS);
    return static_cast<std::size_t>(((id % n) + n) % n);
  }

  void reset_locked()
  {
    buckets_.fill(Bucket{});
    latest_id_ = std::numeric_limits<int64_t>::min();
  }

  mutable DEFAULT_MUTEX mutex_;
  std::array<Bucket, NUM_BUCKETS> buckets_{};
  double window_duration_ = -1.0;
  double bucket_width_ = 0.0;
  // Atomic so that the disabled (default) case is checked without locking
  std::atomic<bool> enabled_{false};
  int64_t latest_id_ = std::numeric_limits<int64_t>::min();
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
    recent_periodicity = std::make_shared<ros2_control::TimeWindowStatistics>();
  }

  /**
   * @brief Resets the internal statistics of the execution time and periodicity statistics
   * collectors.
   */
  void reset_statistics()
  {
    execution_time->reset();
    periodicity->reset();
    recent_periodicity->reset();
  }

  /// Execution time statistics collector
  std::shared_ptr<ros2_control::MovingAverageStatistics> execution_time = nullptr;
  /// Periodicity statistics collector
  std::shared_ptr<ros2_control::MovingAverageStatistics> periodicity = nullptr;
  /// Periodicity statistics collector over a recent time window, disabled until its duration is
  /// set (see ResourceManagerParams::hardware_components_periodicity_window_duration)
  std::shared_ptr<ros2_control::TimeWindowStatistics> recent_periodicity = nullptr;
};
}  // namespace hardware_interface

#endif  // HARDWARE_INTERFACE__TYPES__STATISTICS_TYPES_HPP_
