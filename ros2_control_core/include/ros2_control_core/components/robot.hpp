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


#ifndef ROS2_CONTROL_CORE__COMPONENTS_ROBOT_HPP_
#define ROS2_CONTROL_CORE__COMPONENTS_ROBOT_HPP_

#include <string>
#include <vector>

#include "rclcpp/macros.hpp"
#include "rclcpp/rclcpp.hpp"

#include "ros2_control_core/loaders_pluginlib.hpp"
#include "ros2_control_core/ros2_control_types.h"
#include "ros2_control_core/visibility_control.h"

#include "ros2_control_core/components/component.hpp"
#include "ros2_control_core/components/actuator.hpp"
#include "ros2_control_core/components/sensor.hpp"

#include "ros2_control_core/hardware/robot_hardware.hpp"

#include "control_msgs/msg/dynamic_joint_state.hpp"



namespace ros2_control_core_components
{

class Robot : protected Component< ros2_control_core_hardware::RobotHardware >
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(Robot)

  ROS2_CONTROL_CORE_PUBLIC Robot() = default;

  ROS2_CONTROL_CORE_PUBLIC Robot(std::string name, const rclcpp::Node::SharedPtr node, std::string param_base_path);

  ROS2_CONTROL_CORE_PUBLIC Robot(const std::string parameters_path, const rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr logging_interface,  const rclcpp::node_interfaces::NodeParametersInterface::SharedPtr parameters_interface, const rclcpp::node_interfaces::NodeServicesInterface::SharedPtr services_interface);

  ROS2_CONTROL_CORE_PUBLIC ~Robot() = default;

  ROS2_CONTROL_CORE_PUBLIC ros2_control_types::return_type init(ros2_control_types::RobotDescription description);

  ROS2_CONTROL_CORE_PUBLIC ros2_control_types::return_type recover();

  ROS2_CONTROL_CORE_PUBLIC ros2_control_types::return_type read();

  ROS2_CONTROL_CORE_PUBLIC ros2_control_types::return_type write();

  ROS2_CONTROL_CORE_PUBLIC ros2_control_types::return_type stop();



protected:
  control_msgs::msg::DynamicJointState joint_states_;

  std::vector<std::string> joints_;

  std::map<std::string, Actuator::SharedPtr> actuators_;
  std::map<std::string, Sensor::SharedPtr> sensors_;
  std::map<std::string, Robot::SharedPtr> robots_;
};

}  // namespace ros2_control_core_components

#endif  // ROS2_CONTROL_CORE__COMPONENTS_ROBOT_HPP_
