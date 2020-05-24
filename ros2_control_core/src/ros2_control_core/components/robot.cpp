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


Robot::Robot(const std::string parameters_path, const rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr logging_interface, const rclcpp::node_interfaces::NodeParametersInterface::SharedPtr parameters_interface, const rclcpp::node_interfaces::NodeServicesInterface::SharedPtr services_interface) : Component(parameters_path, "Robot", logging_interface, parameters_interface, services_interface)
{
  parameters_interface_->declare_parameter(parameters_path_ + ".joints");
  parameters_interface_->declare_parameter(parameters_path_ + ".is_modular");

  // Create all Actuator and Sensors defined for each joints
  joints_ = parameters_interface_->get_parameter(parameters_path + ".joints").as_string_array();
  n_dof_ = joints_.size();

  //TODO: is very suboptimal... maybe the base class shold not be templated... Because the robot should not access directly to Actuator/
  ros2_control_core::ActuatorLoaderPluginlib actuator_loader;
  ros2_control_core::SensorLoaderPluginlib sensor_loader;

  RCLCPP_WARN(logging_interface_->get_logger(), parameters_interface_->get_parameter(parameters_path + ".joints").value_to_string());

  for (auto joint: joints_)
  {
    parameters_interface_->declare_parameter(parameters_path + ".actuators." + joint + ".type");
    RCLCPP_WARN(logging_interface_->get_logger(), parameters_interface_->get_parameter(parameters_path + ".actuators." + joint + ".type").as_string());
    RCLCPP_WARN(logging_interface_->get_logger(), "Is available: %s", (actuator_loader.is_available(parameters_interface_->get_parameter(parameters_path + ".actuators." + joint + ".type").as_string()) ? "true" : "false"));

    actuators_[joint] = actuator_loader.create(parameters_interface_->get_parameter(parameters_path + ".actuators." + joint + ".type").as_string());

    actuators_[joint]->configure(parameters_path_ + ".actuators." + joint, logging_interface, parameters_interface, services_interface);

    parameters_interface_->declare_parameter(parameters_path + ".sensors." + joint + ".type");
    RCLCPP_WARN(logging_interface_->get_logger(), parameters_interface_->get_parameter(parameters_path + ".sensors." + joint + ".type").as_string());
    RCLCPP_WARN(logging_interface_->get_logger(), "Is available: %s", (sensor_loader.is_available(parameters_interface_->get_parameter(parameters_path + ".sensors." + joint + ".type").as_string()) ? "true" : "false"));

    sensors_[joint] = sensor_loader.create(parameters_interface_->get_parameter(parameters_path + ".sensors." + joint + ".type").as_string());

    sensors_[joint]->configure(parameters_path_ + ".sensors." + joint, logging_interface, parameters_interface, services_interface);
  }

//   std::map<std::string, rclcpp::Parameter> params;
//
//   param_interface->describe_parameters();
//
//   param_interface->get_parameters_by_prefix(param_base_path, params);
//
//   for (auto const& param : params)
//   {
//     RCLCPP_WARN(logging_interface_->get_logger(), "String: %s, value: %s", param.first, param.second.as_string());
//   }

RCLCPP_INFO(logging_interface_->get_logger(), "Robot Component created...");
}

ros2_control_types::return_type Robot::recover()
{
  RCLCPP_INFO(logging_interface_->get_logger(), "Called recover in Robot class");
  return ros2_control_types::ROS2C_RETURN_OK;
}


ros2_control_types::return_type Robot::stop()
{
  return ros2_control_types::ROS2C_RETURN_OK;
}


