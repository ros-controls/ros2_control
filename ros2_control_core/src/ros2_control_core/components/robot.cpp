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

ros2_control_types::return_type Robot::configure(const std::string parameters_path, const rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr logging_interface, const rclcpp::node_interfaces::NodeParametersInterface::SharedPtr parameters_interface, const rclcpp::node_interfaces::NodeServicesInterface::SharedPtr services_interface)
{
  ros2_control_types::return_type ret = ros2_control_types::ROS2C_RETURN_OK;
  ret = Component::configure(parameters_path, "Robot", logging_interface, parameters_interface, services_interface);

  parameters_interface_->declare_parameter(parameters_path_ + ".joints");
  parameters_interface_->declare_parameter(parameters_path_ + ".has_robots", rclcpp::ParameterValue(false));
  parameters_interface_->declare_parameter(parameters_path_ + ".has_tools", rclcpp::ParameterValue(false));

  rclcpp::Parameter param_joints = parameters_interface_->get_parameter(parameters_path + ".joints");
  joints_ = param_joints.as_string_array();
  n_dof_ = joints_.size();
  RCLCPP_DEBUG(logging_interface_->get_logger(), param_joints.as_string().c_str());

  ros2_control_utils::ROS2ControlLoaderPluginlib<Actuator> actuator_loader("ros2_control_components", "ros2_control_core_components::Actuator");
  ros2_control_utils::ROS2ControlLoaderPluginlib<Sensor> sensor_loader("ros2_control_components", "ros2_control_core_components::Sensor");
  ros2_control_utils::ROS2ControlLoaderPluginlib<Robot> robot_loader("ros2_control_components", "ros2_control_core_components::Robot");

  // Create all Actuator and Sensors defined for each joints
  actuators_ = ros2_control_utils::load_components_from_parameters<Actuator>(parameters_path_ + ".actuators", parameters_interface_, joints_, actuator_loader, logging_interface_->get_logger());
  sensors_ = ros2_control_utils::load_components_from_parameters<Sensor>(parameters_path_ + ".sensors", parameters_interface_, joints_, sensor_loader, logging_interface_->get_logger());

  // Initialize sub robots if available
  has_robots_ = parameters_interface_->get_parameter(parameters_path_ + ".has_robots").as_bool();
  if (has_robots_)
  {
    parameters_interface_->declare_parameter(parameters_path_ + ".robots" + ".robot_names");
    std::vector<std::string> robot_names = parameters_interface_->get_parameter(parameters_path_ + ".robots" + ".robot_names").as_string_array();

    robots_= ros2_control_utils::load_components_from_parameters<Robot>(parameters_path_ + ".robots", parameters_interface_, robot_names, robot_loader, logging_interface_->get_logger());
  }

  // Initalize tool sensor and actuators
  has_tools_ = parameters_interface_->get_parameter(parameters_path_ + ".has_tools").as_bool();
  if (has_tools_)
  {
    parameters_interface_->declare_parameter(parameters_path_ + ".tools" + ".actuator_names", rclcpp::ParameterValue(std::vector<std::string>()));
    parameters_interface_->declare_parameter(parameters_path_ + ".tools" + ".sensor_names", rclcpp::ParameterValue(std::vector<std::string>()));
    std::vector<std::string> tool_actuator_names = parameters_interface_->get_parameter(parameters_path_ + ".tools" + ".actuator_names").as_string_array();
    std::vector<std::string> tool_sensor_names = parameters_interface_->get_parameter(parameters_path_ + ".tools" + ".sensor_names").as_string_array();

    tool_actuators_ = ros2_control_utils::load_components_from_parameters<Actuator>(parameters_path_ + ".tools.actuators", parameters_interface_, tool_actuator_names, actuator_loader, logging_interface_->get_logger());
    tool_sensors_ = ros2_control_utils::load_components_from_parameters<Sensor>(parameters_path_ + ".tools.sensors", parameters_interface_, tool_sensor_names, sensor_loader, logging_interface_->get_logger());
  }

  if (has_hardware_)
  {
    ros2_control_utils::ROS2ControlLoaderPluginlib<ros2_control_core_hardware::RobotHardware> hw_loader("ros2_control_hardware", "ros2_control_core_hardware::RobotHardware");
    load_and_configure_hardware(hw_loader);
  }

  RCLCPP_INFO(logging_interface_->get_logger(), "Robot Component '" + name_ + "' created...");
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


