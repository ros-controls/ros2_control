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

#include "ros2_control_core/components/robot.hpp"

using namespace ros2_control_core_components;


Robot::Robot(std::string name, const rclcpp::Node::SharedPtr node, std::string param_base_path) : Component(name), logger_(node->get_logger())
{
  std::map<std::string, rclcpp::Parameter> params;
  node->get_parameters(param_base_path, params);

  RCLCPP_INFO(node->get_logger(), "Robot Component created...");
}

Robot::Robot(const std::string name, const rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr logging_interface, const rclcpp::node_interfaces::NodeParametersInterface::SharedPtr param_interface, const rclcpp::node_interfaces::NodeServicesInterface::SharedPtr services_interface, const std::string param_base_path) : Component(name), logger_(logging_interface->get_logger())
{
  std::map<std::string, rclcpp::Parameter> params;


  param_interface->get_parameters_by_prefix(param_base_path, params);

  for (auto const& param : params)
  {
    RCLCPP_WARN(logger_, "String: %s, value: %s", param.first, param.second.as_string());
  }

  RCLCPP_INFO(logging_interface->get_logger(), "Robot Component created...");
}

ros2_control_types::return_type Robot::recover()
{
  RCLCPP_INFO(logger_, "Called recover in Robot class");
  return ros2_control_types::ROS2C_RETURN_OK;
}


ros2_control_types::return_type Robot::stop()
{
  return ros2_control_types::ROS2C_RETURN_OK;
}


