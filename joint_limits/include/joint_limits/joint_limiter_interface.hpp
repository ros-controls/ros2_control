// Copyright (c) 2023, Stogl Robotics Consulting UG (haftungsbeschränkt)
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
#include "joint_limits/joint_limits_rosparam.hpp"
#include "joint_limits/visibility_control.h"
#include "rclcpp/node.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
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
   * Override this method only if initialization and reading joint limits should be adapted.
   * Otherwise, initialize your custom limiter in `on_limit` method.
   *
   * \param[in] joint_names names of joints where limits should be applied.
   * \param[in] param_itf node parameters interface object to access parameters.
   * \param[in] logging_itf node logging interface to log if error happens.
   * \param[in] robot_description_topic string of a topic where robot description is accessible.
   */
  JOINT_LIMITS_PUBLIC virtual bool init(
    const std::vector<std::string> & joint_names,
    const rclcpp::node_interfaces::NodeParametersInterface::SharedPtr & param_itf,
    const rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr & logging_itf,
    const std::string & robot_description_topic = "/robot_description")
  {
    number_of_joints_ = joint_names.size();
    joint_names_ = joint_names;
    joint_limits_.resize(number_of_joints_);
    node_param_itf_ = param_itf;
    node_logging_itf_ = logging_itf;

    bool result = true;

    // TODO(destogl): get limits from URDF

    // Initialize and get joint limits from parameter server
    for (size_t i = 0; i < number_of_joints_; ++i)
    {
      // FIXME?(destogl): this seems to be a bit unclear because we use the same namespace for
      // limiters interface and rosparam helper functions - should we use here another namespace?
      if (!declare_parameters(joint_names[i], node_param_itf_, node_logging_itf_))
      {
        RCLCPP_ERROR(
          node_logging_itf_->get_logger(),
          "JointLimiter: Joint '%s': parameter declaration has failed", joint_names[i].c_str());
        result = false;
        break;
      }
      if (!get_joint_limits(joint_names[i], node_param_itf_, node_logging_itf_, joint_limits_[i]))
      {
        RCLCPP_ERROR(
          node_logging_itf_->get_logger(),
          "JointLimiter: Joint '%s': getting parameters has failed", joint_names[i].c_str());
        result = false;
        break;
      }
      RCLCPP_INFO(
        node_logging_itf_->get_logger(), "Limits for joint %zu (%s) are:\n%s", i,
        joint_names[i].c_str(), joint_limits_[i].to_string().c_str());
    }

    if (result)
    {
      result = on_init();
    }

    // avoid linters output
    (void)robot_description_topic;

    return result;
  }

  /**
   * Wrapper init method that accepts pointer to the Node.
   * For details see other init method.
   */
  JOINT_LIMITS_PUBLIC virtual bool init(
    const std::vector<std::string> & joint_names, const rclcpp::Node::SharedPtr & node,
    const std::string & robot_description_topic = "/robot_description")
  {
    return init(
      joint_names, node->get_node_parameters_interface(), node->get_node_logging_interface(),
      robot_description_topic);
  }

  /**
   * Wrapper init method that accepts pointer to the LifecycleNode.
   * For details see other init method.
   */
  JOINT_LIMITS_PUBLIC virtual bool init(
    const std::vector<std::string> & joint_names,
    const rclcpp_lifecycle::LifecycleNode::SharedPtr & lifecycle_node,
    const std::string & robot_description_topic = "/robot_description")
  {
    return init(
      joint_names, lifecycle_node->get_node_parameters_interface(),
      lifecycle_node->get_node_logging_interface(), robot_description_topic);
  }

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

protected:
  // Methods that each limiter implementation has to implement
  JOINT_LIMITS_PUBLIC virtual bool on_init() { return true; }

  JOINT_LIMITS_PUBLIC virtual bool on_configure(
    const trajectory_msgs::msg::JointTrajectoryPoint & /*current_joint_states*/)
  {
    return true;
  }

  /** \brief Enforce joint limits to desired joint state.
   *
   * Filter-specific implementation of the joint limits enforce algorithm.
   *
   * \param[in] current_joint_states current joint states a robot is in.
   * \param[out] desired_joint_states joint state that should be adjusted to obey the limits.
   * \param[in] dt time delta to calculate missing integrals and derivation in joint limits.
   */
  JOINT_LIMITS_PUBLIC virtual bool on_enforce(
    trajectory_msgs::msg::JointTrajectoryPoint & current_joint_states,
    trajectory_msgs::msg::JointTrajectoryPoint & desired_joint_states,
    const rclcpp::Duration & dt) = 0;

  size_t number_of_joints_;
  std::vector<std::string> joint_names_;
  std::vector<LimitsType> joint_limits_;
  rclcpp::Node::SharedPtr node_;
  rclcpp::node_interfaces::NodeParametersInterface::SharedPtr node_param_itf_;
  rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr node_logging_itf_;
};

}  // namespace joint_limits

#endif  // JOINT_LIMITS__JOINT_LIMITER_INTERFACE_HPP_
