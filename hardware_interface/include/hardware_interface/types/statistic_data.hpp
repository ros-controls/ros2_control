// Copyright 2019 Amazon.com, Inc. or its affiliates. All Rights Reserved.
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

// This file is a copy of the StatisticData struct from libstatistics_collector
// (libstatistics_collector/moving_average_statistics/types.hpp), which is being removed from
// ROS 2 Rolling onwards.

#ifndef HARDWARE_INTERFACE__TYPES__STATISTIC_DATA_HPP_
#define HARDWARE_INTERFACE__TYPES__STATISTIC_DATA_HPP_

#include <cmath>
#include <cstdint>
#include <sstream>
#include <string>

namespace ros2_control
{

/**
 *  A container for statistics data results for a set of recorded observations.
 */
struct StatisticData
{
  /// average of the observations
  double average = std::nan("");
  /// min value of the observations
  double min = std::nan("");
  /// max value of the observations
  double max = std::nan("");
  /// standard_deviation of the observations
  double standard_deviation = std::nan("");
  /// number of samples of the observation
  uint64_t sample_count = 0;
};

/**
 * Function which pretty prints the contents of a StatisticData struct.
 *
 * @param results the StatisticData to pretty print
 * @return std::string formatted struct contents in an easily readable format, e.g.,
 * "avg=1, min=2, max=3, std_dev=4, count=5"
 */
inline std::string statistics_data_to_string(const StatisticData & results)
{
  std::stringstream ss;
  ss << "avg=" << std::to_string(results.average) << ", min=" << std::to_string(results.min)
     << ", max=" << std::to_string(results.max)
     << ", std_dev=" << std::to_string(results.standard_deviation)
     << ", count=" << std::to_string(results.sample_count);
  return ss.str();
}

}  // namespace ros2_control

#endif  // HARDWARE_INTERFACE__TYPES__STATISTIC_DATA_HPP_
