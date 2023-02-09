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

  // check for required inputs
  if (
    (desired_joint_states.positions.size() < num_joints) ||
    (desired_joint_states.velocities.size() < num_joints) ||
    (current_joint_states.positions.size() < num_joints))
  {
    return false;
  }

  std::vector<double> desired_accel(num_joints);
  std::vector<double> desired_vel(num_joints);
  std::vector<double> desired_pos(num_joints);
  std::vector<bool> pos_limit_trig_jnts(num_joints, false);
  std::vector<std::string> limited_jnts_vel, limited_jnts_acc;

  bool position_limit_triggered = false;

  for (auto index = 0u; index < num_joints; ++index)
  {
    desired_pos[index] = desired_joint_states.positions[index];

    // limit position
    if (joint_limits_[index].has_position_limits)
    {
      auto pos = std::max(
        std::min(joint_limits_[index].max_position, desired_pos[index]),
        joint_limits_[index].min_position);
      if (pos != desired_pos[index])
      {
        pos_limit_trig_jnts[index] = true;
        desired_pos[index] = pos;
      }
    }

    desired_vel[index] = desired_joint_states.velocities[index];

    // limit velocity
    if (joint_limits_[index].has_velocity_limits)
    {
      if (std::abs(desired_vel[index]) > joint_limits_[index].max_velocity)
      {
        desired_vel[index] = std::copysign(joint_limits_[index].max_velocity, desired_vel[index]);
        limited_jnts_vel.emplace_back(joint_names_[index]);
      }
    }

    desired_accel[index] =
      (desired_vel[index] - current_joint_states.velocities[index]) / dt_seconds;

    // limit acceleration
    if (joint_limits_[index].has_acceleration_limits)
    {
      if (std::abs(desired_accel[index]) > joint_limits_[index].max_acceleration)
      {
        desired_accel[index] =
          std::copysign(joint_limits_[index].max_acceleration, desired_accel[index]);
        desired_vel[index] =
          current_joint_states.velocities[index] + desired_accel[index] * dt_seconds;
        // recalc desired position after acceleration limiting
        desired_pos[index] = current_joint_states.positions[index] +
                             current_joint_states.velocities[index] * dt_seconds +
                             0.5 * desired_accel[index] * dt_seconds * dt_seconds;
        limited_jnts_acc.emplace_back(joint_names_[index]);
      }
    }

    // Check that stopping distance is within joint limits
    // Slow down all joints at maximum decel if any don't have stopping distance within joint limits
    if (joint_limits_[index].has_position_limits)
    {
      // delta_x = (v2*v2 - v1*v1) / (2*a)
      // stopping_distance = (- v1*v1) / (2*max_acceleration)
      // Here we assume we will not trigger velocity limits while maximally decelerating.
      // This is a valid assumption if we are not currently at a velocity limit since we are just
      // coming to a rest.
      double stopping_accel = joint_limits_[index].has_acceleration_limits
                                ? joint_limits_[index].max_acceleration
                                : std::abs(desired_vel[index] / dt_seconds);
      double stopping_distance =
        std::abs((-desired_vel[index] * desired_vel[index]) / (2 * stopping_accel));
      // Check that joint limits are beyond stopping_distance and desired_velocity is towards
      // that limit
      if (
        (desired_vel[index] < 0 &&
         (current_joint_states.positions[index] - joint_limits_[index].min_position <
          stopping_distance)) ||
        (desired_vel[index] > 0 &&
         (joint_limits_[index].max_position - current_joint_states.positions[index] <
          stopping_distance)))
      {
        pos_limit_trig_jnts[index] = true;
        position_limit_triggered = true;
      }
    }
  }

  if (position_limit_triggered)
  {
    std::ostringstream ostr;
    for (auto index = 0u; index < num_joints; ++index)
    {
      // Compute accel to stop
      // Here we aren't explicitly maximally decelerating, but for joints near their limits this
      // should still result in max decel being used
      desired_accel[index] = -current_joint_states.velocities[index] / dt_seconds;
      if (joint_limits_[index].has_acceleration_limits)
      {
        desired_accel[index] = std::copysign(
          std::min(std::abs(desired_accel[index]), joint_limits_[index].max_acceleration),
          desired_accel[index]);
      }

      // Recompute velocity and position
      desired_vel[index] =
        current_joint_states.velocities[index] + desired_accel[index] * dt_seconds;
      desired_pos[index] = current_joint_states.positions[index] +
                           current_joint_states.velocities[index] * dt_seconds +
                           0.5 * desired_accel[index] * dt_seconds * dt_seconds;
    }
  }

  if (
    std::count_if(
      pos_limit_trig_jnts.begin(), pos_limit_trig_jnts.end(), [](bool trig) { return trig; }) > 0)
  {
    std::ostringstream ostr;
    for (auto index = 0u; index < num_joints; ++index)
    {
      if (pos_limit_trig_jnts[index]) ostr << joint_names_[index] << " ";
    }
    ostr << "\b \b";  // erase last character
    RCLCPP_WARN_STREAM_THROTTLE(
      node_->get_logger(), *node_->get_clock(), ROS_LOG_THROTTLE_PERIOD,
      "Joint(s) [" << ostr.str().c_str() << "] would exceed position limits, limiting");
  }

  if (limited_jnts_vel.size() > 0)
  {
    std::ostringstream ostr;
    for (auto jnt : limited_jnts_vel) ostr << jnt << " ";
    ostr << "\b \b";  // erase last character
    RCLCPP_WARN_STREAM_THROTTLE(
      node_->get_logger(), *node_->get_clock(), ROS_LOG_THROTTLE_PERIOD,
      "Joint(s) [" << ostr.str().c_str() << "] would exceed velocity limits, limiting");
  }

  if (limited_jnts_acc.size() > 0)
  {
    std::ostringstream ostr;
    for (auto jnt : limited_jnts_acc) ostr << jnt << " ";
    ostr << "\b \b";  // erase last character
    RCLCPP_WARN_STREAM_THROTTLE(
      node_->get_logger(), *node_->get_clock(), ROS_LOG_THROTTLE_PERIOD,
      "Joint(s) [" << ostr.str().c_str() << "] would exceed acceleration limits, limiting");
  }

  desired_joint_states.positions = desired_pos;
  desired_joint_states.velocities = desired_vel;
  return true;
}

}  // namespace joint_limits

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  joint_limits::SimpleJointLimiter<joint_limits::JointLimits>,
  joint_limits::JointLimiterInterface<joint_limits::JointLimits>)
