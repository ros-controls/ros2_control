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

#include "pal_statistics/pal_statistics_macros.hpp"
#include "pal_statistics/pal_statistics_utils.hpp"

namespace hardware_interface
{
constexpr char DEFAULT_REGISTRY_KEY[] = "ros2_control";
constexpr char DEFAULT_INTROSPECTION_TOPIC[] = "~/introspection_data";

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
