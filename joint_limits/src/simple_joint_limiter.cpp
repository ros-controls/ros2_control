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

/// \authors Nathan Brooks, Dr. Denis Stogl

#include "joint_limits/simple_joint_limiter.hpp"

#include <algorithm>

#include "rclcpp/duration.hpp"
#include "rcutils/logging_macros.h"

constexpr size_t ROS_LOG_THROTTLE_PERIOD = 1 * 1000;  // Milliseconds to throttle logs inside loops

namespace joint_limits
{
template <>
bool SimpleJointLimiter<JointLimits>::on_enforce(
  trajectory_msgs::msg::JointTrajectoryPoint & current_joint_states,
  trajectory_msgs::msg::JointTrajectoryPoint & desired_joint_states, const rclcpp::Duration & dt)
{
  const auto dt_seconds = dt.seconds();
  // negative or null is not allowed
  if (dt_seconds <= 0.0) return false;

  if (current_joint_states.velocities.empty())
  {
    // First update() after activating does not have velocity available, assume 0
    current_joint_states.velocities.resize(number_of_joints_, 0.0);
  }

  // check for required inputs
  if (
    (desired_joint_states.positions.size() < number_of_joints_) ||
    (desired_joint_states.velocities.size() < number_of_joints_) ||
    (current_joint_states.positions.size() < number_of_joints_))
  {
    return false;
  }

  // TODO(destogl): please check if we get too much malloc from this initialization,
  // if so then we should use members instead local variables and initialize them in other method
  std::vector<double> desired_accel(number_of_joints_);
  std::vector<double> desired_vel(number_of_joints_);
  std::vector<double> desired_pos(number_of_joints_);
  std::vector<bool> pos_limit_trig_jnts(number_of_joints_, false);
  std::vector<std::string> limited_jnts_vel, limited_jnts_acc, limited_jnts_dec;

  bool position_limit_triggered = false;

  for (size_t index = 0; index < number_of_joints_; ++index)
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

    auto apply_acc_or_dec_limit =
      [&](const double max_acc_or_dec, std::vector<std::string> & limited_jnts)
    {
      if (std::abs(desired_accel[index]) > max_acc_or_dec)
      {
        desired_accel[index] = std::copysign(max_acc_or_dec, desired_accel[index]);
        desired_vel[index] =
          current_joint_states.velocities[index] + desired_accel[index] * dt_seconds;
        // recalc desired position after acceleration limiting
        desired_pos[index] = current_joint_states.positions[index] +
                             current_joint_states.velocities[index] * dt_seconds +
                             0.5 * desired_accel[index] * dt_seconds * dt_seconds;
        limited_jnts.emplace_back(joint_names_[index]);
      }
    };

    // check if decelerating - if velocity is changing toward 0
    bool deceleration_limit_applied = false;
    if (
      (desired_accel[index] < 0 && current_joint_states.velocities[index] > 0) ||
      (desired_accel[index] > 0 && current_joint_states.velocities[index] < 0))
    {
      // limit deceleration
      if (joint_limits_[index].has_deceleration_limits)
      {
        apply_acc_or_dec_limit(joint_limits_[index].max_deceleration, limited_jnts_dec);
        deceleration_limit_applied = true;
      }
    }

    // limit acceleration (fallback to acceleration if no deceleration limits)
    if (joint_limits_[index].has_acceleration_limits && !deceleration_limit_applied)
    {
      apply_acc_or_dec_limit(joint_limits_[index].max_acceleration, limited_jnts_acc);
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
      double stopping_deccel = std::abs(desired_vel[index] / dt_seconds);
      if (joint_limits_[index].has_deceleration_limits)
      {
        stopping_deccel = joint_limits_[index].max_deceleration;
      }
      else if (joint_limits_[index].has_acceleration_limits)
      {
        stopping_deccel = joint_limits_[index].max_acceleration;
      }

      double stopping_distance =
        std::abs((-desired_vel[index] * desired_vel[index]) / (2 * stopping_deccel));
      // compute stopping duration at stopping_deccel
      double stopping_duration = std::abs((desired_vel[index]) / (stopping_deccel));

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
      else
      {
        // compute the travel_distance at new desired velocity, in best case duration
        // stopping_duration
        double motion_after_stopping_duration = desired_vel[index] * stopping_duration;
        // re-check what happens if we don't slow down
        if (
          (desired_vel[index] < 0 &&
           (current_joint_states.positions[index] - joint_limits_[index].min_position <
            motion_after_stopping_duration)) ||
          (desired_vel[index] > 0 &&
           (joint_limits_[index].max_position - current_joint_states.positions[index] <
            motion_after_stopping_duration)))
        {
          pos_limit_trig_jnts[index] = true;
          position_limit_triggered = true;
        }
        // else no need to slow down. in worse case we won't hit the limit at current velocity
      }
    }
  }

  if (position_limit_triggered)
  {
    for (size_t index = 0; index < number_of_joints_; ++index)
    {
      // Compute accel to stop
      // Here we aren't explicitly maximally decelerating, but for joints near their limits this
      // should still result in max decel being used
      desired_accel[index] = -current_joint_states.velocities[index] / dt_seconds;
      if (joint_limits_[index].has_deceleration_limits)
      {
        desired_accel[index] = std::copysign(
          std::min(std::abs(desired_accel[index]), joint_limits_[index].max_deceleration),
          desired_accel[index]);
      }
      else if (joint_limits_[index].has_acceleration_limits)
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
    for (size_t index = 0; index < number_of_joints_; ++index)
    {
      if (pos_limit_trig_jnts[index]) ostr << joint_names_[index] << " ";
    }
    ostr << "\b \b";  // erase last character
    RCLCPP_WARN_STREAM_THROTTLE(
      node_logging_itf_->get_logger(), *clock_, ROS_LOG_THROTTLE_PERIOD,
      "Joint(s) [" << ostr.str().c_str() << "] would exceed position limits, limiting");
  }

  if (limited_jnts_vel.size() > 0)
  {
    std::ostringstream ostr;
    for (auto jnt : limited_jnts_vel) ostr << jnt << " ";
    ostr << "\b \b";  // erase last character
    RCLCPP_WARN_STREAM_THROTTLE(
      node_logging_itf_->get_logger(), *clock_, ROS_LOG_THROTTLE_PERIOD,
      "Joint(s) [" << ostr.str().c_str() << "] would exceed velocity limits, limiting");
  }

  if (limited_jnts_acc.size() > 0)
  {
    std::ostringstream ostr;
    for (auto jnt : limited_jnts_acc) ostr << jnt << " ";
    ostr << "\b \b";  // erase last character
    RCLCPP_WARN_STREAM_THROTTLE(
      node_logging_itf_->get_logger(), *clock_, ROS_LOG_THROTTLE_PERIOD,
      "Joint(s) [" << ostr.str().c_str() << "] would exceed acceleration limits, limiting");
  }

  if (limited_jnts_dec.size() > 0)
  {
    std::ostringstream ostr;
    for (auto jnt : limited_jnts_dec) ostr << jnt << " ";
    ostr << "\b \b";  // erase last character
    RCLCPP_WARN_STREAM_THROTTLE(
      node_logging_itf_->get_logger(), *clock_, ROS_LOG_THROTTLE_PERIOD,
      "Joint(s) [" << ostr.str().c_str() << "] would exceed deceleration limits, limiting");
  }

  desired_joint_states.positions = desired_pos;
  desired_joint_states.velocities = desired_vel;
  desired_joint_states.accelerations = desired_accel;
  return true;
}

}  // namespace joint_limits

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  joint_limits::SimpleJointLimiter<joint_limits::JointLimits>,
  joint_limits::JointLimiterInterface<joint_limits::JointLimits>)
