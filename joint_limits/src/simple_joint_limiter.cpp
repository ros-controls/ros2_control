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

/// \authors Nathan Brooks, Denis Stogl

#include "joint_limits/simple_joint_limiter.hpp"

#include <algorithm>

#include "rclcpp/duration.hpp"
#include "rcutils/logging_macros.h"

constexpr size_t ROS_LOG_THROTTLE_PERIOD = 1 * 1000;  // Milliseconds to throttle logs inside loops

namespace joint_limits
{
template <>
SimpleJointLimiter<JointLimits>::SimpleJointLimiter()
: joint_limits::JointLimiterInterface<JointLimits>()
{
}

template <>
bool SimpleJointLimiter<JointLimits>::on_enforce(
  trajectory_msgs::msg::JointTrajectoryPoint & current_joint_states,
  trajectory_msgs::msg::JointTrajectoryPoint & desired_joint_states, const rclcpp::Duration & dt)
{
  auto num_joints = joint_limits_.size();
  auto dt_seconds = dt.seconds();

  if (current_joint_states.velocities.empty())
  {
    // First update() after activating does not have velocity available, assume 0
    current_joint_states.velocities.resize(num_joints, 0.0);
  }

  // Clamp velocities to limits
  for (auto index = 0u; index < num_joints; ++index)
  {
    if (joint_limits_[index].has_velocity_limits)
    {
      if (std::abs(desired_joint_states.velocities[index]) > joint_limits_[index].max_velocity)
      {
        RCLCPP_WARN_STREAM_THROTTLE(
          node_->get_logger(), *node_->get_clock(), ROS_LOG_THROTTLE_PERIOD,
          "Joint(s) would exceed velocity limits, limiting");
        desired_joint_states.velocities[index] =
          copysign(joint_limits_[index].max_velocity, desired_joint_states.velocities[index]);
        double accel =
          (desired_joint_states.velocities[index] - current_joint_states.velocities[index]) /
          dt_seconds;
        // Recompute position
        desired_joint_states.positions[index] =
          current_joint_states.positions[index] +
          current_joint_states.velocities[index] * dt_seconds +
          0.5 * accel * dt_seconds * dt_seconds;
      }
    }
  }

  // Clamp acclerations to limits
  for (auto index = 0u; index < num_joints; ++index)
  {
    if (joint_limits_[index].has_acceleration_limits)
    {
      double accel =
        (desired_joint_states.velocities[index] - current_joint_states.velocities[index]) /
        dt_seconds;
      if (std::abs(accel) > joint_limits_[index].max_acceleration)
      {
        RCLCPP_WARN_STREAM_THROTTLE(
          node_->get_logger(), *node_->get_clock(), ROS_LOG_THROTTLE_PERIOD,
          "Joint(s) would exceed acceleration limits, limiting");
        desired_joint_states.velocities[index] =
          current_joint_states.velocities[index] +
          copysign(joint_limits_[index].max_acceleration, accel) * dt_seconds;
        // Recompute position
        desired_joint_states.positions[index] =
          current_joint_states.positions[index] +
          current_joint_states.velocities[index] * dt_seconds +
          0.5 * copysign(joint_limits_[index].max_acceleration, accel) * dt_seconds * dt_seconds;
      }
    }
  }

  // Check that stopping distance is within joint limits
  // - In joint mode, slow down only joints whose stopping distance isn't inside joint limits,
  // at maximum decel
  // - In Cartesian mode, slow down all joints at maximum decel if any don't have stopping distance
  // within joint limits
  bool position_limit_triggered = false;
  for (auto index = 0u; index < num_joints; ++index)
  {
    if (joint_limits_[index].has_acceleration_limits)
    {
      // delta_x = (v2*v2 - v1*v1) / (2*a)
      // stopping_distance = (- v1*v1) / (2*max_acceleration)
      // Here we assume we will not trigger velocity limits while maximally decelerating.
      // This is a valid assumption if we are not currently at a velocity limit since we are just
      // coming to a rest.
      double stopping_distance = std::abs(
        (-desired_joint_states.velocities[index] * desired_joint_states.velocities[index]) /
        (2 * joint_limits_[index].max_acceleration));
      // Check that joint limits are beyond stopping_distance and desired_velocity is towards
      // that limit
      // TODO(anyone): Should we consider sign on acceleration here?
      if (
        (desired_joint_states.velocities[index] < 0 &&
         (current_joint_states.positions[index] - joint_limits_[index].min_position <
          stopping_distance)) ||
        (desired_joint_states.velocities[index] > 0 &&
         (joint_limits_[index].max_position - current_joint_states.positions[index] <
          stopping_distance)))
      {
        RCLCPP_WARN_STREAM_THROTTLE(
          node_->get_logger(), *node_->get_clock(), ROS_LOG_THROTTLE_PERIOD,
          "Joint(s) would exceed position limits, limiting");
        position_limit_triggered = true;

        // We will limit all joints
        break;
      }
    }
  }

  if (position_limit_triggered)
  {
    // In Cartesian admittance mode, stop all joints if one would exceed limit
    for (auto index = 0u; index < num_joints; ++index)
    {
      if (joint_limits_[index].has_acceleration_limits)
      {
        // Compute accel to stop
        // Here we aren't explicitly maximally decelerating, but for joints near their limits this
        // should still result in max decel being used
        double accel_to_stop = -current_joint_states.velocities[index] / dt_seconds;
        double limited_accel = copysign(
          std::min(std::abs(accel_to_stop), joint_limits_[index].max_acceleration), accel_to_stop);

        desired_joint_states.velocities[index] =
          current_joint_states.velocities[index] + limited_accel * dt_seconds;
        // Recompute position
        desired_joint_states.positions[index] =
          current_joint_states.positions[index] +
          current_joint_states.velocities[index] * dt_seconds +
          0.5 * limited_accel * dt_seconds * dt_seconds;
      }
    }
  }
  return true;
}

}  // namespace joint_limits

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  joint_limits::SimpleJointLimiter<joint_limits::JointLimits>,
  joint_limits::JointLimiterInterface<joint_limits::JointLimits>)
