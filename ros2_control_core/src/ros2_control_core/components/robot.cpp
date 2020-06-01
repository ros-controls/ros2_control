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
  joint_names_ = param_joints.as_string_array();
  n_dof_ = joint_names_.size();
  RCLCPP_DEBUG(logging_interface_->get_logger(), param_joints.as_string().c_str());

  ros2_control_utils::ROS2ControlLoaderPluginlib<Actuator> actuator_loader("ros2_control_core", "ros2_control_core_components::Actuator");
  ros2_control_utils::ROS2ControlLoaderPluginlib<Sensor> sensor_loader("ros2_control_core", "ros2_control_core_components::Sensor");
  ros2_control_utils::ROS2ControlLoaderPluginlib<Robot> robot_loader("ros2_control_core", "ros2_control_core_components::Robot");

  // Create and configure Actuators and Sensors defined for each joints
  actuators_ = ros2_control_utils::load_components_from_parameters<Actuator>(parameters_path_ + ".actuators", parameters_interface_, joint_names_, actuator_loader, logging_interface_->get_logger());
  ret = configure_components(parameters_path_ + ".actuators", joint_names_, actuators_);
  sensors_ = ros2_control_utils::load_components_from_parameters<Sensor>(parameters_path_ + ".sensors", parameters_interface_, joint_names_, sensor_loader, logging_interface_->get_logger());
  ret = configure_components(parameters_path_ + ".sensors", joint_names_, sensors_);

  // Initialize sub robots if available
  has_robots_ = parameters_interface_->get_parameter(parameters_path_ + ".has_robots").as_bool();
  if (has_robots_)
  {
    parameters_interface_->declare_parameter(parameters_path_ + ".robots" + ".robot_names_");
    robot_names_ = parameters_interface_->get_parameter(parameters_path_ + ".robots" + ".robot_names_").as_string_array();

    robots_= ros2_control_utils::load_components_from_parameters<Robot>(parameters_path_ + ".robots", parameters_interface_, robot_names_, robot_loader, logging_interface_->get_logger());
    ret = configure_components(parameters_path_ + ".robots", robot_names_, robots_);
  }

  // Initalize tool sensor and actuators
  has_tools_ = parameters_interface_->get_parameter(parameters_path_ + ".has_tools").as_bool();
  if (has_tools_)
  {
    parameters_interface_->declare_parameter(parameters_path_ + ".tools" + ".actuator_names", rclcpp::ParameterValue(std::vector<std::string>()));
    parameters_interface_->declare_parameter(parameters_path_ + ".tools" + ".sensor_names", rclcpp::ParameterValue(std::vector<std::string>()));
    tool_actuator_names_ = parameters_interface_->get_parameter(parameters_path_ + ".tools" + ".actuator_names").as_string_array();
    tool_sensor_names_ = parameters_interface_->get_parameter(parameters_path_ + ".tools" + ".sensor_names").as_string_array();

    tool_actuators_ = ros2_control_utils::load_components_from_parameters<Actuator>(parameters_path_ + ".tools.actuators", parameters_interface_, tool_actuator_names_, actuator_loader, logging_interface_->get_logger());
    ret = configure_components(parameters_path_ + ".tools.actuators", tool_actuator_names_, tool_actuators_);
    tool_sensors_ = ros2_control_utils::load_components_from_parameters<Sensor>(parameters_path_ + ".tools.sensors", parameters_interface_, tool_sensor_names_, sensor_loader, logging_interface_->get_logger());
    ret = configure_components(parameters_path_ + ".tools.sensors", tool_sensor_names_, tool_sensors_);
  }

  if (has_hardware_)
  {
    //FIXME: why this does not work with "ros2_control_core" which is correct...
    ros2_control_utils::ROS2ControlLoaderPluginlib<ros2_control_core_hardware::RobotHardware> hw_loader("ros2_control_hardware", "ros2_control_core_hardware::RobotHardware");
    load_and_configure_hardware(hw_loader);
  }

  //FIXME:DEBUG
  RCLCPP_INFO(logging_interface_->get_logger(), "Robot Component '" + name_ + "' created...");

  return ret;
}

ros2_control_types::return_type Robot::init()
{
  ros2_control_types::return_type ret = ros2_control_types::ROS2C_RETURN_OK;
  RCLCPP_INFO(logging_interface_->get_logger(), "Called init in Robot class");

  ret = init_components(actuators_);
  ret = init_components(sensors_);
  ret = init_components(robots_);
  ret = init_components(tool_actuators_);
  ret = init_components(tool_sensors_);

  Component::init();

  //FIXME:DEBUG
  RCLCPP_INFO(logging_interface_->get_logger(), "%s initialization finished!", name_.c_str());
  return ret;
}

//TODO: ...
ros2_control_types::return_type Robot::recover()
{
  //FIXME:DEBUG
  RCLCPP_INFO(logging_interface_->get_logger(), "Called recover in Robot class");
  return ros2_control_types::ROS2C_RETURN_OK;
}

//TODO: ...
ros2_control_types::return_type Robot::stop()
{
  return ros2_control_types::ROS2C_RETURN_OK;
}


