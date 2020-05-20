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

/* This header must be included by all rclcpp headers which declare symbols
 * which are defined in the rclcpp library. When not building the rclcpp
 * library, i.e. when using the headers in other package's code, the contents
 * of this header change the visibility of certain symbols which the rclcpp
 * library cannot have, but the consuming code must have inorder to link.
 */

#ifndef ROS2_CONTROL_CORE__ROS2_CONTROL_TYPES_H_
#define ROS2_CONTROL_CORE__ROS2_CONTROL_TYPES_H_

#include <map>
#include <string>

#include "ros2_control_core/visibility_control.h"

#include "control_msgs/msg/interface_value.hpp"

// TODO: Do we need "ROS2_CONTROL_CORE_PUBLIC" before the variables?

namespace ros2_control_types
{

using return_type = int;
static constexpr return_type ROS2C_RETURN_OK = 0;
static constexpr return_type ROS2C_RETURN_ERROR = 1;

static constexpr return_type ROS2C_RETURN_ACTUATOR_CAN_NOT_READ = 20;

class CommunicationInterfaceDescription
{
public:
  std::string type;
  std::map<std::string, std::string> params;
};


class HardwareDescription
{
public:
  std::string name;
  CommunicationInterfaceDescription comm_interface_descr;
  std::map<std::string, std::string> params;
};

class ActuatorHardwareDescription : HardwareDescription
{
};

class SensorHardwareDescription : HardwareDescription
{
};

class RobotHardwareDescription : HardwareDescription
{
};


class ComponentDescription
{
public:
  std::string name;
  const std::string type;
  HardwareDescription hardware_descr;
  std::map<std::string, std::string> params;
};

class SimpleComponentDescription : ComponentDescription
{
public:
  uint8_t n_dof;
  control_msgs::msg::InterfaceValue min_values;
  control_msgs::msg::InterfaceValue max_values;
  std::string frame_id;
};


// typedef SimpleComponentDescription ActuatorDescription;
class ActuatorDescription : SimpleComponentDescription
{
};

// typedef SimpleComponentDescription SensorDescription;
class SensorDescription : SimpleComponentDescription
{
};

class RobotDescription : ComponentDescription
{
  bool is_modular;
};

}  // namespace ros2_control_core

#endif  // ROS2_CONTROL_CORE__ROS2_CONTROL_TYPES_H_
