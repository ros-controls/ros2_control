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
#include <vector>

#include "joint_limits/joint_limits_rosparam.hpp"
#include "rcutils/logging_macros.h"
#include "ruckig/brake.hpp"
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
bool RuckigJointLimiter<joint_limits::JointLimits>::on_init(/*const rclcpp::Duration & dt*/)
{
  // TODO(destogl): This should be used from parameter
  const rclcpp::Duration dt = rclcpp::Duration::from_seconds(0.005);

  // Initialize Ruckig
  ruckig_ = std::make_shared<ruckig::Ruckig<0>>(number_of_joints_, dt.seconds());
  ruckig_input_ = std::make_shared<ruckig::InputParameter<0>>(number_of_joints_);
  ruckig_output_ = std::make_shared<ruckig::OutputParameter<0>>(number_of_joints_);

  // Velocity mode works best for smoothing one waypoint at a time
  ruckig_input_->control_interface = ruckig::ControlInterface::Velocity;
  ruckig_input_->synchronization = ruckig::Synchronization::Time;

  for (auto joint = 0ul; joint < number_of_joints_; ++joint)
  {
    if (joint_limits_[joint].has_jerk_limits)
    {
      ruckig_input_->max_jerk.at(joint) = joint_limits_[joint].max_jerk;
    }
    else
    {
      ruckig_input_->max_jerk.at(joint) = DEFAULT_MAX_JERK;
    }
    if (joint_limits_[joint].has_acceleration_limits)
    {
      ruckig_input_->max_acceleration.at(joint) = joint_limits_[joint].max_acceleration;
    }
    else
    {
      ruckig_input_->max_acceleration.at(joint) = DEFAULT_MAX_ACCELERATION;
    }
    if (joint_limits_[joint].has_deceleration_limits)
    {
      RCUTILS_LOG_WARN_NAMED(
        "ruckig_joint_limiter",
        "Deceleration limits not supported in community version of Ruckig.");
    }
    if (joint_limits_[joint].has_velocity_limits)
    {
      ruckig_input_->max_velocity.at(joint) = joint_limits_[joint].max_velocity;
    }
    else
    {
      ruckig_input_->max_velocity.at(joint) = DEFAULT_MAX_VELOCITY;
    }
  }

  return true;
}

template <>
bool RuckigJointLimiter<joint_limits::JointLimits>::on_configure(
  const trajectory_msgs::msg::JointTrajectoryPoint & current_joint_states)
{
  // Initialize Ruckig with current_joint_states
  ruckig_input_->current_position = current_joint_states.positions;

  if (current_joint_states.velocities.size() == number_of_joints_)
  {
    ruckig_input_->current_velocity = current_joint_states.velocities;
  }
  else
  {
    ruckig_input_->current_velocity = std::vector<double>(number_of_joints_, 0.0);
  }
  if (current_joint_states.accelerations.size() == number_of_joints_)
  {
    ruckig_input_->current_acceleration = current_joint_states.accelerations;
  }
  else
  {
    ruckig_input_->current_acceleration = std::vector<double>(number_of_joints_, 0.0);
  }

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
  ruckig_input_->current_position = ruckig_output_->new_position;
  ruckig_input_->current_velocity = ruckig_output_->new_velocity;
  ruckig_input_->current_acceleration = ruckig_output_->new_acceleration;

  // Target state is the next waypoint
  if (desired_joint_states.positions.size() == number_of_joints_)
  {
    ruckig_input_->target_position = desired_joint_states.positions;
  }
  else
  {
    RCUTILS_LOG_WARN_NAMED(
      "ruckig_joint_limiter",
      "Size of desired positions (%zu) does not match number of joint (%zu).",
      desired_joint_states.positions.size(), number_of_joints_);
    return false;
  }
  // TODO(destogl): in current use-case we use only velocity
  if (desired_joint_states.velocities.size() == number_of_joints_)
  {
    ruckig_input_->target_velocity = desired_joint_states.velocities;
  }
  else
  {
    ruckig_input_->target_velocity = std::vector<double>(number_of_joints_, 0.0);
  }
  if (desired_joint_states.accelerations.size() == number_of_joints_)
  {
    ruckig_input_->target_acceleration = desired_joint_states.accelerations;
  }
  else
  {
    ruckig_input_->target_acceleration = std::vector<double>(number_of_joints_, 0.0);
  }

  // Ruckig fails if provided values are out of limit,
  // therefore we first limit things by scaling to the maximum ruckig velocity
  double max_vel_ratio = 1.0;
  for (auto joint = 0ul; joint < number_of_joints_; ++joint)
  {
    if (std::fabs(ruckig_input_->target_velocity.at(joint)) > ruckig_input_->max_velocity.at(joint))
    {
      const double ratio =
        std::fabs(ruckig_input_->target_velocity.at(joint) / ruckig_input_->max_velocity.at(joint));
      if (ratio > max_vel_ratio)
      {
        max_vel_ratio = ratio;
      }
    }
  }
  for (auto joint = 0ul; joint < number_of_joints_; ++joint)
  {
    // Set the target velocities to follow the ruckig joint limits
    if (max_vel_ratio != 1.0)
    {
      ruckig_input_->target_velocity.at(joint) =
        ruckig_input_->target_velocity.at(joint) / max_vel_ratio;
      RCUTILS_LOG_WARN_ONCE_NAMED(
        "ruckig_joint_limiter", "Limiting target velocity by a factor : %lf.", max_vel_ratio);
    }

    // Set the target accelerations to follow the ruckig joint limits
    ruckig_input_->target_acceleration.at(joint) = std::clamp(
      ruckig_input_->target_acceleration.at(joint),
      -0.999 * ruckig_input_->max_acceleration.at(joint),
      0.999 * ruckig_input_->max_acceleration.at(joint));
  }
  // apply ruckig
  ruckig::Result result = ruckig_->update(*ruckig_input_, *ruckig_output_);
  if (result == ruckig::Result::Working || result == ruckig::Result::Finished)
    RCUTILS_LOG_DEBUG_NAMED("ruckig_joint_limiter", "Rucking result %d", result);
  else
  {
    RCUTILS_LOG_WARN_ONCE_NAMED("ruckig_joint_limiter", "Rucking result %d", result);
    return false;
  }

  desired_joint_states.positions = ruckig_output_->new_position;
  desired_joint_states.velocities = ruckig_output_->new_velocity;
  desired_joint_states.accelerations = ruckig_output_->new_acceleration;

  for (auto joint = 0ul; joint < number_of_joints_; ++joint)
  {
    RCUTILS_LOG_DEBUG_NAMED(
      "ruckig_joint_limiter",
      "Desired position: %e \nDesired velocity: %e\n Desired acceleration: %e.",
      ruckig_input_->target_position.at(joint), ruckig_input_->target_velocity.at(joint),
      ruckig_input_->target_acceleration.at(joint));
  }

  for (auto joint = 0ul; joint < number_of_joints_; ++joint)
  {
    RCUTILS_LOG_DEBUG_NAMED(
      "ruckig_joint_limiter", "New position: %e \nNew velocity: %e\nNew acceleration: %e.",
      ruckig_output_->new_position.at(joint), ruckig_output_->new_velocity.at(joint),
      ruckig_output_->new_acceleration.at(joint));
  }

  return true;
}

}  // namespace ruckig_joint_limiter

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  ruckig_joint_limiter::RuckigJointLimiter<joint_limits::JointLimits>,
  joint_limits::JointLimiterInterface<joint_limits::JointLimits>)
