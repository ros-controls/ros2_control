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


#ifndef ROS2_CONTROL_CORE__LOADERS_PLUGINLIB_COMPONENTS_HPP_
#define ROS2_CONTROL_CORE__LOADERS_PLUGINLIB_COMPONENTS_HPP_


#include "ros2_control_core/components/actuator.hpp"
#include "ros2_control_core/components/component.hpp"
#include "ros2_control_core/components/sensor.hpp"
#include "ros2_control_core/hardware/robot_hardware.hpp"

#include "ros2_control_core/loaders_pluginlib.hpp"
#include "ros2_control_core/visibility_control.h"

namespace ros2_control_core
{

// Create Loader for Components

template < typename ComponentHardwareType >
class ComponentBaseLoaderPluginlib : public ROS2ControlLoaderPluginlib< ros2_control_core_components::Component<ComponentHardwareType> >
{
public:
  ROS2_CONTROL_CORE_PUBLIC ComponentBaseLoaderPluginlib() : ROS2ControlLoaderPluginlib< ros2_control_core_components::Component<ComponentHardwareType> >("ros2_control_core", "ros2_control_core_components::Component") {};

  ROS2_CONTROL_CORE_PUBLIC ~ComponentBaseLoaderPluginlib() = default;
};

// TODO: Redefine all components like this?
class RobotLoaderPluginlib : public ROS2ControlLoaderPluginlib< ros2_control_core_components::Component<ros2_control_core_hardware::RobotHardware> >
{
public:
  ROS2_CONTROL_CORE_PUBLIC RobotLoaderPluginlib() : ROS2ControlLoaderPluginlib< ros2_control_core_components::Component<ros2_control_core_hardware::RobotHardware> >("ros2_control_components", "ros2_control_core_components::Robot") {};

  ROS2_CONTROL_CORE_PUBLIC ~RobotLoaderPluginlib() = default;
};

class ActuatorLoaderPluginlib : public ROS2ControlLoaderPluginlib< ros2_control_core_components::Actuator >
{
public:
  ROS2_CONTROL_CORE_PUBLIC ActuatorLoaderPluginlib() : ROS2ControlLoaderPluginlib< ros2_control_core_components::Actuator >("ros2_control_components", "ros2_control_core_components::Actuator") {};

  ROS2_CONTROL_CORE_PUBLIC ~ActuatorLoaderPluginlib() = default;
};

class SensorLoaderPluginlib : public ROS2ControlLoaderPluginlib< ros2_control_core_components::Sensor >
{
public:
  ROS2_CONTROL_CORE_PUBLIC SensorLoaderPluginlib() : ROS2ControlLoaderPluginlib< ros2_control_core_components::Sensor >("ros2_control_components", "ros2_control_core_components::Sensor") {};

  ROS2_CONTROL_CORE_PUBLIC ~SensorLoaderPluginlib() = default;
};

// class RobotLoaderPluginlib : public ROS2ControlLoaderPluginlib< ros2_control_core_components::Robot >
// {
// public:
//   ROS2_CONTROL_CORE_PUBLIC RobotLoaderPluginlib() : ROS2ControlLoaderPluginlib< ros2_control_core_components::Robot >("ros2_control_components", "ros2_control_core_components::Robot") {};
//
//   ROS2_CONTROL_CORE_PUBLIC ~RobotLoaderPluginlib() = default;
// };

}  // namespace ros2_control_core

#endif  // ROS2_CONTROL_CORE__LOADERS_PLUGINLIB_COMPONENTS_HPP_
