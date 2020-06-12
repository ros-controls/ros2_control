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


#ifndef ROBOT_CONTROL_COMPONENTS__COMPONENTS_COMPONENT_HPP_
#define ROBOT_CONTROL_COMPONENTS__COMPONENTS_COMPONENT_HPP_

#include <set>
#include <string>
#include <vector>

#include "control_msgs/msg/interface_value.hpp"

#include "robot_control_components/components/component.hpp"

#include "robot_control_components/ros2_control_types.h"
#include "robot_control_components/visibility_control.h"


namespace robot_control_components
{

template < typename ComponentHardwareType >
class SimpleComponent : public Component< ComponentHardwareType >
{
public:
  ROS2_CONTROL_CORE_PUBLIC SimpleComponent() = default;

  ROS2_CONTROL_CORE_PUBLIC virtual ~SimpleComponent() = default;

  ROS2_CONTROL_CORE_PUBLIC ros2_control_types::return_type configure(const std::string parameters_path, const std::string type, const rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr logging_interface, const rclcpp::node_interfaces::NodeParametersInterface::SharedPtr parameters_interface, const rclcpp::node_interfaces::NodeServicesInterface::SharedPtr services_interface)
  {
    ros2_control_types::return_type ret = ros2_control_types::ROS2C_RETURN_OK;
    ret = Component< ComponentHardwareType >::configure(parameters_path, type, logging_interface, parameters_interface, services_interface);

    parameters_interface->declare_parameter(parameters_path + ".interface_names");
    parameters_interface->declare_parameter(parameters_path + ".n_dof", rclcpp::ParameterValue(1));
    parameters_interface->declare_parameter(parameters_path + ".min_values");
    parameters_interface->declare_parameter(parameters_path + ".max_values");

    return ret;
  };

  //TODO: this is simplest implementation. We need to do this more complex to support complex configurations of robots
  std::vector<std::string> get_interface_names()
  {
    return interface_names_;
  }

protected:
  std::vector<std::string> valid_interface_names_;
  //TODO: should we use set for easier search?
  std::vector<std::string> interface_names_;
  control_msgs::msg::InterfaceValue read_values_;
};

}  // namespace robot_control_components

#endif  // ROBOT_CONTROL_COMPONENTS__COMPONENTS_COMPONENT_HPP_
