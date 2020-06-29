// Copyright 2017 Open Source Robotics Foundation, Inc.
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

#ifndef HARDWARE_INTERFACE__RESOURCE_MANAGER_HPP_
#define HARDWARE_INTERFACE__RESOURCE_MANAGER_HPP_

#include <map>
#include <memory>

#include "hardware_interface/actuator.hpp"
#include "hardware_interface/sensor.hpp"
#include "hardware_interface/system.hpp"
#include "hardware_interface/robot_hardware.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/utils/ros2_control_utils.hpp"
#include "hardware_interface/visibility_control.h"
#include "rclcpp/rclcpp.hpp"

namespace hardware_interface
{

typedef ros2_control_utils::ROS2ControlLoaderPluginlib<ActuatorInterface> ActuatorInterfaceLoaderType;
typedef ros2_control_utils::ROS2ControlLoaderPluginlib<SensorInterface> SensorInterfaceLoaderType;
typedef ros2_control_utils::ROS2ControlLoaderPluginlib<SystemInterface> SystemInterfaceLoaderType;
typedef ros2_control_utils::ROS2ControlLoaderPluginlib<RobotHardware> RobotHardwareLoaderType;

class ResourceManager
{
public:
  HARDWARE_INTERFACE_PUBLIC
  ResourceManager() : logger_(rclcpp::get_logger("ros2_control::ResourceManager")) {};

  HARDWARE_INTERFACE_PUBLIC
  ~ResourceManager() = default;

  HARDWARE_INTERFACE_PUBLIC
  hardware_interface_ret_t parse_system_from_urdf(const std::string & urdf_string);

  HARDWARE_INTERFACE_PUBLIC
  hardware_interface_ret_t init_system();

private:
  ComponentInfo system_info_;
  std::unique_ptr<SystemInterface> system_itf_;
  std::unique_ptr<RobotHardware> robot_hardware_;

  std::map<std::string, Actuator> actuators_;
  std::map<std::string, Sensor> sensors_;
  std::map<std::string, System> systems_;

  rclcpp::Logger logger_;
};

}  // namespace hardware_interface

#endif  // HARDWARE_INTERFACE__RESOURCE_MANAGER_HPP_
