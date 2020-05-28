// Copyright 2020 ROS2-Control Development Team
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


#ifndef ROS2_CONTROL_CORE__LOADERS_PLUGINLIB_HARDWARE_HPP_
#define ROS2_CONTROL_CORE__LOADERS_PLUGINLIB_HARDWARE_HPP_


#include "ros2_control_core/hardware/actuator_hardware.hpp"
#include "ros2_control_core/hardware/component_hardware.hpp"
#include "ros2_control_core/hardware/robot_hardware.hpp"
#include "ros2_control_core/hardware/sensor_hardware.hpp"

#include "ros2_control_core/loaders_pluginlib.hpp"
#include "ros2_control_core/visibility_control.h"

namespace ros2_control_core
{

class ComponentHardwareLoaderPluginlib : private ROS2ControlLoaderPluginlib< ros2_control_core_hardware::ComponentHardware >
{
public:
  ROS2_CONTROL_CORE_PUBLIC ComponentHardwareLoaderPluginlib() : ROS2ControlLoaderPluginlib< ros2_control_core_hardware::ComponentHardware >("ros2_control_core", "ros2_control_core_hardware::ComponentHardware") {};

  ROS2_CONTROL_CORE_PUBLIC ~ComponentHardwareLoaderPluginlib() = default;
};

class ActuatorHardwareLoaderPluginlib : public ROS2ControlLoaderPluginlib< ros2_control_core_hardware::ActuatorHardware >
{
public:
  ROS2_CONTROL_CORE_PUBLIC ActuatorHardwareLoaderPluginlib() : ROS2ControlLoaderPluginlib< ros2_control_core_hardware::ActuatorHardware >("ros2_control_hardware", "ros2_control_core_hardware::ActuatorHardware") {};

  ROS2_CONTROL_CORE_PUBLIC ~ActuatorHardwareLoaderPluginlib() = default;
};

class SensorHardwareLoaderPluginlib : public ROS2ControlLoaderPluginlib< ros2_control_core_hardware::SensorHardware >
{
public:
  ROS2_CONTROL_CORE_PUBLIC SensorHardwareLoaderPluginlib() : ROS2ControlLoaderPluginlib< ros2_control_core_hardware::SensorHardware >("ros2_control_hardware", "ros2_control_core_hardware::SensorHardware") {};

  ROS2_CONTROL_CORE_PUBLIC ~SensorHardwareLoaderPluginlib() = default;
};

class RobotHardwareLoaderPluginlib : public ROS2ControlLoaderPluginlib< ros2_control_core_hardware::RobotHardware >
{
public:
  ROS2_CONTROL_CORE_PUBLIC RobotHardwareLoaderPluginlib() : ROS2ControlLoaderPluginlib< ros2_control_core_hardware::RobotHardware >("ros2_control_hardware", "ros2_control_core_hardware::RobotHardware") {};

  ROS2_CONTROL_CORE_PUBLIC ~RobotHardwareLoaderPluginlib() = default;
};

}  // namespace ros2_control_core

#endif  // ROS2_CONTROL_CORE__LOADERS_PLUGINLIB_HARDWARE_HPP_
