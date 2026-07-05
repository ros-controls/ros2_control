// Copyright 2024 PAL Robotics S.L.
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

#ifndef HARDWARE_INTERFACE__INTROSPECTION_HPP_
#define HARDWARE_INTERFACE__INTROSPECTION_HPP_

#include <string>

#include "hardware_interface/types/statistics_types.hpp"
#include "pal_statistics/pal_statistics_macros.hpp"
#include "pal_statistics/pal_statistics_utils.hpp"

namespace pal_statistics
{
template <>
inline IdType customRegister(
  StatisticsRegistry & registry, const std::string & name,
  const libstatistics_collector::moving_average_statistics::StatisticData * variable,
  RegistrationsRAII * bookkeeping, bool enabled)
{
  registry.registerVariable(name + "/max", &variable->max, bookkeeping, enabled);
  registry.registerVariable(name + "/min", &variable->min, bookkeeping, enabled);
  registry.registerVariable(name + "/average", &variable->average, bookkeeping, enabled);
  registry.registerVariable(
    name + "/standard_deviation", &variable->standard_deviation, bookkeeping, enabled);
  std::function<double()> sample_func = [variable]
  { return static_cast<double>(variable->sample_count); };
  return registry.registerFunction(name + "/sample_count", sample_func, bookkeeping, enabled);
}
}  // namespace pal_statistics

namespace hardware_interface
{
constexpr char DEFAULT_REGISTRY_KEY[] = "ros2_control";
constexpr char DEFAULT_INTROSPECTION_TOPIC[] = "~/introspection_data";
constexpr char CM_STATISTICS_KEY[] = "cm_execution_statistics";
constexpr char CM_STATISTICS_TOPIC[] = "~/statistics";

#define REGISTER_ROS2_CONTROL_INTROSPECTION(ID, ENTITY)                      \
  REGISTER_ENTITY(                                                           \
    hardware_interface::DEFAULT_REGISTRY_KEY, get_name() + "." + ID, ENTITY, \
    &stats_registrations_, false);

#define UNREGISTER_ROS2_CONTROL_INTROSPECTION(ID) \
  UNREGISTER_ENTITY(DEFAULT_REGISTRY_KEY, get_name() + "." + ID);

#define CLEAR_ALL_ROS2_CONTROL_INTROSPECTION_REGISTRIES() CLEAR_ALL_REGISTRIES();

#define INITIALIZE_ROS2_CONTROL_INTROSPECTION_REGISTRY(node, topic, registry_key) \
  INITIALIZE_REGISTRY(node, topic, registry_key);

#define START_ROS2_CONTROL_INTROSPECTION_PUBLISHER_THREAD(registry_key) \
  START_PUBLISH_THREAD(registry_key);

#define PUBLISH_ROS2_CONTROL_INTROSPECTION_DATA_ASYNC(registry_key) \
  PUBLISH_ASYNC_STATISTICS(registry_key);

#define DEFAULT_REGISTER_ROS2_CONTROL_INTROSPECTION(ID, ENTITY) \
  REGISTER_ENTITY(DEFAULT_REGISTRY_KEY, ID, ENTITY);

#define DEFAULT_UNREGISTER_ROS2_CONTROL_INTROSPECTION(ID) \
  UNREGISTER_ENTITY(DEFAULT_REGISTRY_KEY, ID);
}  // namespace hardware_interface

#endif  // HARDWARE_INTERFACE__INTROSPECTION_HPP_
