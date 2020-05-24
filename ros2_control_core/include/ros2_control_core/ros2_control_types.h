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

#include "rclcpp/rclcpp.hpp"

#include "ros2_control_core/visibility_control.h"

#include "control_msgs/msg/interface_value.hpp"


// TODO: Do we need "ROS2_CONTROL_CORE_PUBLIC" before the variables?

namespace ros2_control_types
{

using return_type = int;
static constexpr return_type ROS2C_RETURN_OK = 0;
static constexpr return_type ROS2C_RETURN_ERROR = 1;

static constexpr return_type ROS2C_RETURN_ACTUATOR_CLAIMED_ERROR = 10;
static constexpr return_type ROS2C_RETURN_ACTUATOR_ALREADY_CLAIMED = 11;
static constexpr return_type ROS2C_RETURN_ACTUATOR_NOT_CLAIMED = 11;
static constexpr return_type ROS2C_RETURN_ACTUATOR_UNATHORIZED_UNCLAIM = 13;
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
  std::string type;
  CommunicationInterfaceDescription comm_interface_descr;
//   std::map<std::string, std::string> params;
  HardwareDescription() {
    parameter_names_.push_back("type");
  };

  std::vector<std::string> get_parameter_names() {
    add_child_parameters();

    return parameter_names_;
  };

protected:
  std::vector<std::string> parameter_names_;

  virtual void add_child_parameters() = 0;
};

class ActuatorHardwareDescription : public HardwareDescription
{
  void add_child_parameters()
  {
  };
};

class SensorHardwareDescription : public HardwareDescription
{
  void add_child_parameters()
  {
  };
};

class RobotHardwareDescription : public HardwareDescription
{
  void add_child_parameters()
  {
      parameter_names_.push_back("robot_hardware_example_param");
  };
};

template < typename HardwareDescriptionType >
class ComponentDescription
{
public:
  std::string name;
  const std::string type;
  HardwareDescriptionType hardware_descr;
  std::map<std::string, std::string> params;

  ComponentDescription(std::string name, const std::string type) : name(name), type(type)
  {
    parameter_names_.push_back(name + ".name");
  };

  std::vector<std::string> get_parameter_names() {
    add_child_parameters();

    for (const std::string param: hardware_descr.get_parameter_names())
    {
        parameter_names_.push_back(name + "." + type + "Hardware." + param);
    }

    return parameter_names_;
  };

protected:
  std::vector<std::string> parameter_names_;

  virtual void add_child_parameters() = 0;
};

template < typename HardwareDescriptionType >
class SimpleComponentDescription : public ComponentDescription<HardwareDescriptionType>
{
public:
  uint8_t n_dof;
  control_msgs::msg::InterfaceValue min_values;
  control_msgs::msg::InterfaceValue max_values;
  std::string frame_id;
};


// typedef SimpleComponentDescription ActuatorDescription;
class ActuatorDescription : public SimpleComponentDescription<ActuatorHardwareDescription>
{
  void add_child_parameters()
  {
  }
};

// typedef SimpleComponentDescription SensorDescription;
class SensorDescription : public SimpleComponentDescription<SensorHardwareDescription>
{
  void add_child_parameters()
  {
  }
};

class RobotDescription : public ComponentDescription<RobotHardwareDescription>
{
public:
  bool is_modular;

  RobotDescription(std::string name, const std::string type) : ComponentDescription<RobotHardwareDescription>(name, type){};

  void add_child_parameters()
  {
    parameter_names_.push_back(name + ".is_modular");
    parameter_names_.push_back(name + ".joints");
  }
};

}  // namespace ros2_control_core

#endif  // ROS2_CONTROL_CORE__ROS2_CONTROL_TYPES_H_
