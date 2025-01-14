// Copyright (c) 2021, Stogl Robotics Consulting UG (haftungsbeschränkt)
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

//
// Author: Denis Stogl
//

#ifndef HARDWARE_INTERFACE__HARDWARE_COMPONENT_INFO_HPP_
#define HARDWARE_INTERFACE__HARDWARE_COMPONENT_INFO_HPP_

#include <memory>
#include <string>
#include <vector>

#include <rclcpp/time.hpp>
#include "libstatistics_collector/moving_average_statistics/moving_average.hpp"
#include "libstatistics_collector/moving_average_statistics/types.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "realtime_tools/mutex.hpp"

namespace hardware_interface
{
using MovingAverageStatistics =
  libstatistics_collector::moving_average_statistics::MovingAverageStatistics;
using StatisticData = libstatistics_collector::moving_average_statistics::StatisticData;

struct MovingAverageStatisticsData
{
public:
  MovingAverageStatisticsData() { statistics = std::make_unique<MovingAverageStatistics>(); }

  void add_measurement(double measurement)
  {
    std::unique_lock<realtime_tools::prio_inherit_recursive_mutex> lock(mutex_);
    statistics->AddMeasurement(measurement);
    statistics_data = statistics->GetStatistics();
  }

  void reset()
  {
    statistics->Reset();
    statistics_data = StatisticData();
  }

  const StatisticData & get_statistics() const
  {
    std::unique_lock<realtime_tools::prio_inherit_recursive_mutex> lock(mutex_);
    return statistics_data;
  }

private:
  std::unique_ptr<MovingAverageStatistics> statistics = nullptr;
  StatisticData statistics_data;
  mutable realtime_tools::prio_inherit_recursive_mutex mutex_;
};

struct HardwareComponentStatistics
{
  MovingAverageStatisticsData execution_time;
  MovingAverageStatisticsData periodicity;
};
/// Hardware Component Information
/**
 * This struct contains information about a given hardware component.
 */
struct HardwareComponentInfo
{
  /// Component name.
  std::string name;

  /// Component "classification": "actuator", "sensor" or "system"
  std::string type;

  /// Component group
  std::string group;

  /// Component pluginlib plugin name.
  std::string plugin_name;

  /// Component is async
  bool is_async;

  //// read/write rate
  unsigned int rw_rate;

  /// Component current state.
  rclcpp_lifecycle::State state;

  /// List of provided state interfaces by the component.
  std::vector<std::string> state_interfaces;

  /// List of provided command interfaces by the component.
  std::vector<std::string> command_interfaces;

  /// Read cycle statistics of the component.
  std::shared_ptr<HardwareComponentStatistics> read_statistics = nullptr;

  /// Write cycle statistics of the component.
  std::shared_ptr<HardwareComponentStatistics> write_statistics = nullptr;
};

}  // namespace hardware_interface
#endif  // HARDWARE_INTERFACE__HARDWARE_COMPONENT_INFO_HPP_
