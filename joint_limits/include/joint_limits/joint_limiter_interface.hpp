// Copyright (c) 2021, Stogl Robotics Consulting UG (haftungsbeschränkt)
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

/// \author Denis Stogl

#ifndef JOINT_LIMITS__JOINT_LIMITER_INTERFACE_HPP_
#define JOINT_LIMITS__JOINT_LIMITER_INTERFACE_HPP_

#include <string>
#include <vector>

#include "joint_limits/joint_limits.hpp"
#include "joint_limits/visibility_control.h"
#include "rclcpp/node.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"

namespace joint_limits
{
template <typename LimitsType>
class JointLimiterInterface
{
public:
  JOINT_LIMITS_PUBLIC JointLimiterInterface() = default;

  JOINT_LIMITS_PUBLIC virtual ~JointLimiterInterface() = default;

  /// Initialization of every JointLimiter.
  /**
   * Initialization of JointLimiter for defined joints with their names.
   * Robot description topic provides a topic name where URDF of the robot can be found.
   * This is needed to use joint limits from URDF (not implemented yet!).
   * Override this method only if Initialization and reading joint limits should be adapted.
   * Otherwise, initialize your custom limiter in `on_limit` method.
   *
   * \param[in] joint_names names of joints where limits should be applied.
   * \param[in] node shared pointer to the node where joint limit parameters defined.
   * \param[in] robot_description_topic string of a topic where robot description is accessible.
   *
   */
  JOINT_LIMITS_PUBLIC virtual bool init(
    const std::vector<std::string> joint_names, const rclcpp::Node::SharedPtr & node,
    const std::string & robot_description_topic = "/robot_description");

  JOINT_LIMITS_PUBLIC virtual bool configure(
    const trajectory_msgs::msg::JointTrajectoryPoint & current_joint_states)
  {
    // TODO(destogl): add checks for position
    return on_configure(current_joint_states);
  }

  JOINT_LIMITS_PUBLIC virtual bool enforce(
    trajectory_msgs::msg::JointTrajectoryPoint & current_joint_states,
    trajectory_msgs::msg::JointTrajectoryPoint & desired_joint_states, const rclcpp::Duration & dt)
  {
    // TODO(destogl): add checks if sizes of vectors and number of limits correspond.
    return on_enforce(current_joint_states, desired_joint_states, dt);
  }

  // TODO(destogl): Make those protected?
  // Methods that each limiter implementation has to implement
  JOINT_LIMITS_PUBLIC virtual bool on_init() { return true; }

  JOINT_LIMITS_PUBLIC virtual bool on_configure(
    const trajectory_msgs::msg::JointTrajectoryPoint & /*current_joint_states*/)
  {
    return true;
  }

  JOINT_LIMITS_PUBLIC virtual bool on_enforce(
    trajectory_msgs::msg::JointTrajectoryPoint & current_joint_states,
    trajectory_msgs::msg::JointTrajectoryPoint & desired_joint_states,
    const rclcpp::Duration & dt) = 0;

protected:
  size_t number_of_joints_;
  std::vector<LimitsType> joint_limits_;
  rclcpp::Node::SharedPtr node_;
};

}  // namespace joint_limits

#endif  // JOINT_LIMITS__JOINT_LIMITER_INTERFACE_HPP_
