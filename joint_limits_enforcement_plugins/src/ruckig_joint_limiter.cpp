// Copyright (c) 2021, PickNik Inc.
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

/// \authors Andy Zelenak, Denis Stogl

#include "ruckig_joint_limiter/ruckig_joint_limiter.hpp"

#include <memory>
#include <string>

#include "joint_limits/joint_limits_rosparam.hpp"
#include "rcutils/logging_macros.h"
#include "ruckig/input_parameter.hpp"
#include "ruckig/output_parameter.hpp"
#include "ruckig/ruckig.hpp"

namespace ruckig_joint_limiter
{
template <>
RuckigJointLimiter<joint_limits::JointLimits>::RuckigJointLimiter()
: joint_limits::JointLimiterInterface<joint_limits::JointLimits>()
{
}

template <>
bool RuckigJointLimiter<joint_limits::JointLimits>::on_init()
{
  // Initialize Ruckig
  ruckig_ = std::make_shared<ruckig::Ruckig<0>>(number_of_joints_, 0.01 /*timestep*/);
  ruckig_input_ = std::make_shared<ruckig::InputParameter<0>>(number_of_joints_);
  ruckig_output_ = std::make_shared<ruckig::OutputParameter<0>>(number_of_joints_);

  // Velocity mode works best for smoothing one waypoint at a time
  ruckig_input_->control_interface = ruckig::ControlInterface::Velocity;

  for (auto joint = 0ul; joint < number_of_joints_; ++joint)
  {
    if (joint_limits_[joint].has_jerk_limits)
    {
      ruckig_input_->max_jerk.at(joint) = joint_limits_[joint].max_acceleration;
    }
    if (joint_limits_[joint].has_acceleration_limits)
    {
      ruckig_input_->max_acceleration.at(joint) = joint_limits_[joint].max_acceleration;
    }
    if (joint_limits_[joint].has_velocity_limits)
    {
      ruckig_input_->max_velocity.at(joint) = joint_limits_[joint].max_velocity;
    }
  }

  RCUTILS_LOG_INFO_NAMED("ruckig_joint_limiter", "Successfully initialized.");

  return true;
}

template <>
bool RuckigJointLimiter<joint_limits::JointLimits>::on_configure(
  const trajectory_msgs::msg::JointTrajectoryPoint & current_joint_states)
{
  // Initialize Ruckig with current_joint_states
  std::copy_n(
    current_joint_states.positions.begin(), number_of_joints_,
    ruckig_input_->current_position.begin());
  std::copy_n(
    current_joint_states.velocities.begin(), number_of_joints_,
    ruckig_input_->current_velocity.begin());
  std::copy_n(
    current_joint_states.accelerations.begin(), number_of_joints_,
    ruckig_input_->current_acceleration.begin());

  // Initialize output data
  ruckig_output_->new_position = ruckig_input_->current_position;
  ruckig_output_->new_velocity = ruckig_input_->current_velocity;
  ruckig_output_->new_acceleration = ruckig_input_->current_acceleration;

  return true;
}

template <>
bool RuckigJointLimiter<joint_limits::JointLimits>::on_enforce(
  trajectory_msgs::msg::JointTrajectoryPoint & /*current_joint_states*/,
  trajectory_msgs::msg::JointTrajectoryPoint & desired_joint_states,
  const rclcpp::Duration & /*dt*/)
{
  // We don't use current_joint_states. For stability, it is recommended to feed previous Ruckig
  // output back in as input for the next iteration. This assumes the robot tracks the command well.

  // Feed output from the previous timestep back as input
  for (auto joint = 0ul; joint < number_of_joints_; ++joint)
  {
    ruckig_input_->current_position.at(joint) = ruckig_output_->new_position.at(joint);
    ruckig_input_->current_velocity.at(joint) = ruckig_output_->new_velocity.at(joint);
    ruckig_input_->current_acceleration.at(joint) = ruckig_output_->new_acceleration.at(joint);

    // Target state is the next waypoint
    ruckig_input_->target_velocity.at(joint) = desired_joint_states.velocities.at(joint);
    ruckig_input_->target_acceleration.at(joint) = desired_joint_states.accelerations.at(joint);
  }

  ruckig::Result result = ruckig_->update(*ruckig_input_, *ruckig_output_);

  return result == ruckig::Result::Finished;
}

}  // namespace ruckig_joint_limiter

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  ruckig_joint_limiter::RuckigJointLimiter<joint_limits::JointLimits>,
  joint_limits::JointLimiterInterface<joint_limits::JointLimits>)
