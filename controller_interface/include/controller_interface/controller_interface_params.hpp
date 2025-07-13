// Copyright 2025 ros2_control Development Team
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

#ifndef CONTROLLER_INTERFACE__CONTROLLER_INTERFACE_PARAMS_HPP_
#define CONTROLLER_INTERFACE__CONTROLLER_INTERFACE_PARAMS_HPP_

#include <string>
#include <unordered_map>

#include "joint_limits/joint_limits.hpp"
#include "rclcpp/node_options.hpp"

/**
 * @brief Parameters for the Controller Interface
 * This struct holds the parameters required to initialize a controller interface.
 *
 * @var controller_name Name of the controller.
 * @var robot_description The URDF or SDF description of the robot.
 * @var cm_update_rate The update rate of the controller manager in Hz.
 * @var node_namespace The namespace for the controller node.
 * @var node_options Options for the controller node.
 * @var joint_limits A map of joint names to their limits.
 * @var soft_joint_limits A map of joint names to their soft limits.
 *
 * This struct is used to pass parameters to the controller interface during initialization.
 * It allows for easy configuration of the controller's behavior and interaction with the robot's
 * joints.
 */
namespace controller_interface
{

struct ControllerInterfaceParams
{
  ControllerInterfaceParams() = default;

  std::string controller_name = "";
  std::string robot_description = "";
  unsigned int update_rate = 0;

  std::string node_namespace = "";
  rclcpp::NodeOptions node_options = rclcpp::NodeOptions();

  std::unordered_map<std::string, joint_limits::JointLimits> hard_joint_limits = {};
  std::unordered_map<std::string, joint_limits::SoftJointLimits> soft_joint_limits = {};
};

}  // namespace controller_interface

#endif  // CONTROLLER_INTERFACE__CONTROLLER_INTERFACE_PARAMS_HPP_
