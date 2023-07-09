// Copyright (c) 2023, PickNik Inc.
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

/// \author Dr. Denis Stogl

#ifndef JOINT_LIMITS__SIMPLE_JOINT_LIMITER_HPP_
#define JOINT_LIMITS__SIMPLE_JOINT_LIMITER_HPP_

#include <memory>
#include <string>

#include "joint_limits/joint_limiter_interface.hpp"
#include "joint_limits/joint_limits.hpp"
#include "rclcpp/rclcpp.hpp"

namespace joint_limits
{
template <typename LimitsType>
class SimpleJointLimiter : public JointLimiterInterface<LimitsType>
{
public:
  /** \brief Constructor */
  JOINT_LIMITS_PUBLIC SimpleJointLimiter();

  /** \brief Destructor */
  JOINT_LIMITS_PUBLIC ~SimpleJointLimiter();

  JOINT_LIMITS_PUBLIC bool on_enforce(
    trajectory_msgs::msg::JointTrajectoryPoint & current_joint_states,
    trajectory_msgs::msg::JointTrajectoryPoint & desired_joint_states,
    const rclcpp::Duration & dt) override;

private:
  rclcpp::Clock::SharedPtr clock_;
};

template <typename LimitsType>
SimpleJointLimiter<LimitsType>::SimpleJointLimiter() : JointLimiterInterface<LimitsType>()
{
  clock_ = std::make_shared<rclcpp::Clock>(rclcpp::Clock(RCL_ROS_TIME));
}

template <typename LimitsType>
SimpleJointLimiter<LimitsType>::~SimpleJointLimiter()
{
}

}  // namespace joint_limits

#endif  // JOINT_LIMITS__SIMPLE_JOINT_LIMITER_HPP_
