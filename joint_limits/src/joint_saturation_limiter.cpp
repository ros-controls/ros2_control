// Copyright (c) 2024, PickNik Inc.
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

/// \authors Nathan Brooks, Dr. Denis Stogl, Guillaume Walck

#include "joint_limits/joint_saturation_limiter.hpp"

#include <algorithm>

#include "rclcpp/duration.hpp"

constexpr size_t ROS_LOG_THROTTLE_PERIOD = 1 * 1000;  // Milliseconds to throttle logs inside loops
constexpr double VALUE_CONSIDERED_ZERO = 1e-10;

namespace joint_limits
{
template <>
bool JointSaturationLimiter<JointLimits>::on_enforce(
  trajectory_msgs::msg::JointTrajectoryPoint & current_joint_states,
  trajectory_msgs::msg::JointTrajectoryPoint & desired_joint_states, const rclcpp::Duration & dt)
{
  bool limits_enforced = false;

  const auto dt_seconds = dt.seconds();
  // negative or null is not allowed
  if (dt_seconds <= 0.0)
  {
    return false;
  }

  // TODO(gwalck) compute if the max are not implicitly violated with the given dt
  // e.g. for max vel 2.0 and max acc 5.0, with dt >0.4
  // velocity max is implicitly already violated due to max_acc * dt > 2.0

  // check for required inputs combination
  bool has_desired_position = (desired_joint_states.positions.size() == number_of_joints_);
  bool has_desired_velocity = (desired_joint_states.velocities.size() == number_of_joints_);
  bool has_desired_acceleration = (desired_joint_states.accelerations.size() == number_of_joints_);
  bool has_current_position = (current_joint_states.positions.size() == number_of_joints_);
  bool has_current_velocity = (current_joint_states.velocities.size() == number_of_joints_);

  // pos state and vel or pos cmd is required, vel state is optional
  if (!(has_current_position && (has_desired_position || has_desired_velocity)))
  {
    return false;
  }

  if (!has_current_velocity)
  {
    // First update() after activating does not have velocity available, assume 0
    current_joint_states.velocities.resize(number_of_joints_, 0.0);
  }

  // TODO(destogl): please check if we get too much malloc from this initialization,
  // if so then we should use members instead local variables and initialize them in other method
  std::vector<double> desired_pos(number_of_joints_);
  std::vector<double> desired_vel(number_of_joints_);
  std::vector<double> desired_acc(number_of_joints_);
  std::vector<double> expected_vel(number_of_joints_);
  std::vector<double> expected_pos(number_of_joints_);

  // limits triggered
  std::vector<std::string> limited_jnts_pos, limited_jnts_vel, limited_jnts_acc, limited_jnts_dec;

  bool braking_near_position_limit_triggered = false;

  for (size_t index = 0; index < number_of_joints_; ++index)
  {
    if (has_desired_position)
    {
      desired_pos[index] = desired_joint_states.positions[index];
    }
    if (has_desired_velocity)
    {
      desired_vel[index] = desired_joint_states.velocities[index];
    }
    if (has_desired_acceleration)
    {
      desired_acc[index] = desired_joint_states.accelerations[index];
    }

    if (has_desired_position)
    {
      // limit position
      if (joint_limits_[index].has_position_limits)
      {
        // clamp input pos_cmd
        auto pos = std::clamp(
          desired_pos[index], joint_limits_[index].min_position, joint_limits_[index].max_position);
        if (pos != desired_pos[index])
        {
          desired_pos[index] = pos;
          limited_jnts_pos.emplace_back(joint_names_[index]);
          limits_enforced = true;
        }
      }
      // priority to pos_cmd derivative over cmd_vel when not defined. If done always then we might
      // get jumps in the velocity based on the system's dynamics. Position limit clamping is done
      // below once again.
      const double position_difference = desired_pos[index] - current_joint_states.positions[index];
      if (
        std::fabs(position_difference) > VALUE_CONSIDERED_ZERO &&
        std::fabs(desired_vel[index]) <= VALUE_CONSIDERED_ZERO)
      {
        desired_vel[index] = position_difference / dt_seconds;
      }
    }

    // limit velocity
    if (joint_limits_[index].has_velocity_limits)
    {
      // if desired velocity is not defined calculate it from positions
      if (std::fabs(desired_vel[index]) <= VALUE_CONSIDERED_ZERO || std::isnan(desired_vel[index]))
      {
        desired_vel[index] =
          (desired_pos[index] - current_joint_states.positions[index]) / dt_seconds;
      }
      // clamp input vel_cmd
      if (std::fabs(desired_vel[index]) > joint_limits_[index].max_velocity)
      {
        desired_vel[index] = std::copysign(joint_limits_[index].max_velocity, desired_vel[index]);
        limited_jnts_vel.emplace_back(joint_names_[index]);
        limits_enforced = true;

        // recompute pos_cmd if needed
        if (has_desired_position)
        {
          desired_pos[index] =
            current_joint_states.positions[index] + desired_vel[index] * dt_seconds;
        }

        desired_acc[index] =
          (desired_vel[index] - current_joint_states.velocities[index]) / dt_seconds;
      }
    }

    // limit acceleration
    if (
      joint_limits_[index].has_acceleration_limits || joint_limits_[index].has_deceleration_limits)
    {
      // if(has_current_velocity)
      if (1)  // for now use a zero velocity if not provided
      {
        // limiting acc or dec function
        auto apply_acc_or_dec_limit = [&](
                                        const double max_acc_or_dec, std::vector<double> & acc,
                                        std::vector<std::string> & limited_jnts) -> bool
        {
          if (std::fabs(acc[index]) > max_acc_or_dec)
          {
            acc[index] = std::copysign(max_acc_or_dec, acc[index]);
            limited_jnts.emplace_back(joint_names_[index]);
            limits_enforced = true;
            return true;
          }
          else
          {
            return false;
          }
        };

        // if desired acceleration if not provided compute it from desired_vel and vel_state
        if (
          std::fabs(desired_acc[index]) <= VALUE_CONSIDERED_ZERO || std::isnan(desired_acc[index]))
        {
          desired_acc[index] =
            (desired_vel[index] - current_joint_states.velocities[index]) / dt_seconds;
        }

        // check if decelerating - if velocity is changing toward 0
        bool deceleration_limit_applied = false;
        bool limit_applied = false;
        if (
          (desired_acc[index] < 0 && current_joint_states.velocities[index] > 0) ||
          (desired_acc[index] > 0 && current_joint_states.velocities[index] < 0))
        {
          // limit deceleration
          if (joint_limits_[index].has_deceleration_limits)
          {
            limit_applied = apply_acc_or_dec_limit(
              joint_limits_[index].max_deceleration, desired_acc, limited_jnts_dec);
            deceleration_limit_applied = true;
          }
        }

        // limit acceleration (fallback to acceleration if no deceleration limits)
        if (joint_limits_[index].has_acceleration_limits && !deceleration_limit_applied)
        {
          limit_applied = apply_acc_or_dec_limit(
            joint_limits_[index].max_acceleration, desired_acc, limited_jnts_acc);
        }

        if (limit_applied)
        {
          // vel_cmd from integration of desired_acc, needed even if no vel output
          desired_vel[index] =
            current_joint_states.velocities[index] + desired_acc[index] * dt_seconds;
          if (has_desired_position)
          {
            // pos_cmd from from double integration of desired_acc
            desired_pos[index] = current_joint_states.positions[index] +
                                 current_joint_states.velocities[index] * dt_seconds +
                                 0.5 * desired_acc[index] * dt_seconds * dt_seconds;
          }
        }
      }
      // else we cannot compute acc, so not limiting it
    }

    // plan ahead for position limits
    if (joint_limits_[index].has_position_limits)
    {
      if (has_desired_velocity && !has_desired_position)
      {
        // Check  immediate next step when using vel_cmd only, other cases already handled
        // integrate pos
        expected_pos[index] =
          current_joint_states.positions[index] + desired_vel[index] * dt_seconds;
        // if expected_pos over limit
        auto pos = std::clamp(
          expected_pos[index], joint_limits_[index].min_position,
          joint_limits_[index].max_position);
        if (pos != expected_pos[index])
        {
          // TODO(gwalck) compute vel_cmd that would permit to slow down in time at full
          // deceleration in any case limit pos to max
          expected_pos[index] = pos;
          // and recompute vel_cmd that would lead to pos_max (not ideal as velocity would not be
          // zero)
          desired_vel[index] =
            (expected_pos[index] - current_joint_states.positions[index]) / dt_seconds;
          limited_jnts_pos.emplace_back(joint_names_[index]);
          limits_enforced = true;
        }
      }

      // Check that stopping distance is within joint limits
      // Slow down all joints at maximum decel if any don't have stopping distance within joint
      // limits

      // delta_x = (v2*v2 - v1*v1) / (2*a)
      // stopping_distance = (- v1*v1) / (2*max_acceleration)
      // Here we assume we will not trigger velocity limits while maximally decelerating.
      // This is a valid assumption if we are not currently at a velocity limit since we are just
      // coming to a rest.
      double stopping_deccel = std::fabs(desired_vel[index] / dt_seconds);
      if (joint_limits_[index].has_deceleration_limits)
      {
        stopping_deccel = joint_limits_[index].max_deceleration;
      }
      else if (joint_limits_[index].has_acceleration_limits)
      {
        stopping_deccel = joint_limits_[index].max_acceleration;
      }

      double stopping_distance =
        std::fabs((-desired_vel[index] * desired_vel[index]) / (2 * stopping_deccel));
      // compute stopping duration at stopping_deccel
      double stopping_duration = std::fabs((desired_vel[index]) / (stopping_deccel));

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
        limited_jnts_pos.emplace_back(joint_names_[index]);
        braking_near_position_limit_triggered = true;
        limits_enforced = true;
      }
      else
      {
        // compute the travel_distance at new desired velocity, with best case duration
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
          limited_jnts_pos.emplace_back(joint_names_[index]);
          braking_near_position_limit_triggered = true;
          limits_enforced = true;
        }
        // else no need to slow down. in worse case we won't hit the limit at current velocity
      }
    }
  }

  // update variables according to triggers
  if (braking_near_position_limit_triggered)
  {
    // this limit applies to all joints even if a single one is triggered
    for (size_t index = 0; index < number_of_joints_; ++index)
    {
      // Compute accel to stop
      // Here we aren't explicitly maximally decelerating, but for joints near their limits this
      // should still result in max decel being used
      desired_acc[index] = -current_joint_states.velocities[index] / dt_seconds;
      if (joint_limits_[index].has_deceleration_limits)
      {
        desired_acc[index] = std::copysign(
          std::min(std::fabs(desired_acc[index]), joint_limits_[index].max_deceleration),
          desired_acc[index]);
      }
      else if (joint_limits_[index].has_acceleration_limits)
      {
        desired_acc[index] = std::copysign(
          std::min(std::fabs(desired_acc[index]), joint_limits_[index].max_acceleration),
          desired_acc[index]);
      }

      // Recompute velocity and position
      if (has_desired_velocity)
      {
        desired_vel[index] =
          current_joint_states.velocities[index] + desired_acc[index] * dt_seconds;
      }
      if (has_desired_position)
      {
        desired_pos[index] = current_joint_states.positions[index] +
                             current_joint_states.velocities[index] * dt_seconds +
                             0.5 * desired_acc[index] * dt_seconds * dt_seconds;
      }
    }
    std::ostringstream ostr;
    for (auto jnt : limited_jnts_pos)
    {
      ostr << jnt << " ";
    }
    ostr << "\b \b";  // erase last character
    RCLCPP_WARN_STREAM_THROTTLE(
      node_logging_itf_->get_logger(), *clock_, ROS_LOG_THROTTLE_PERIOD,
      "Joint(s) [" << ostr.str().c_str()
                   << "] would exceed position limits"
                      " if continuing at current state, limiting all joints");
  }

  // display limitations

  // if position limiting
  if (limited_jnts_pos.size() > 0)
  {
    std::ostringstream ostr;
    for (auto jnt : limited_jnts_pos)
    {
      ostr << jnt << " ";
    }
    ostr << "\b \b";  // erase last character
    RCLCPP_WARN_STREAM_THROTTLE(
      node_logging_itf_->get_logger(), *clock_, ROS_LOG_THROTTLE_PERIOD,
      "Joint(s) [" << ostr.str().c_str() << "] would exceed position limits, limiting");
  }

  if (limited_jnts_vel.size() > 0)
  {
    std::ostringstream ostr;
    for (auto jnt : limited_jnts_vel)
    {
      ostr << jnt << " ";
    }
    ostr << "\b \b";  // erase last character
    RCLCPP_WARN_STREAM_THROTTLE(
      node_logging_itf_->get_logger(), *clock_, ROS_LOG_THROTTLE_PERIOD,
      "Joint(s) [" << ostr.str().c_str() << "] would exceed velocity limits, limiting");
  }

  if (limited_jnts_acc.size() > 0)
  {
    std::ostringstream ostr;
    for (auto jnt : limited_jnts_acc)
    {
      ostr << jnt << " ";
    }
    ostr << "\b \b";  // erase last character
    RCLCPP_WARN_STREAM_THROTTLE(
      node_logging_itf_->get_logger(), *clock_, ROS_LOG_THROTTLE_PERIOD,
      "Joint(s) [" << ostr.str().c_str() << "] would exceed acceleration limits, limiting");
  }

  if (limited_jnts_dec.size() > 0)
  {
    std::ostringstream ostr;
    for (auto jnt : limited_jnts_dec)
    {
      ostr << jnt << " ";
    }
    ostr << "\b \b";  // erase last character
    RCLCPP_WARN_STREAM_THROTTLE(
      node_logging_itf_->get_logger(), *clock_, ROS_LOG_THROTTLE_PERIOD,
      "Joint(s) [" << ostr.str().c_str() << "] would exceed deceleration limits, limiting");
  }

  if (has_desired_position)
  {
    desired_joint_states.positions = desired_pos;
  }
  if (has_desired_velocity)
  {
    desired_joint_states.velocities = desired_vel;
  }
  if (has_desired_acceleration)
  {
    desired_joint_states.accelerations = desired_acc;
  }

  return limits_enforced;
}

template <>
bool JointSaturationLimiter<JointLimits>::on_enforce(std::vector<double> & desired_joint_states)
{
  std::vector<std::string> limited_joints_effort;
  bool limits_enforced = false;

  bool has_cmd = (desired_joint_states.size() == number_of_joints_);
  if (!has_cmd)
  {
    return false;
  }

  for (size_t index = 0; index < number_of_joints_; ++index)
  {
    if (joint_limits_[index].has_effort_limits)
    {
      double max_effort = joint_limits_[index].max_effort;
      if (std::fabs(desired_joint_states[index]) > max_effort)
      {
        desired_joint_states[index] = std::copysign(max_effort, desired_joint_states[index]);
        limited_joints_effort.emplace_back(joint_names_[index]);
        limits_enforced = true;
      }
    }
  }

  if (limited_joints_effort.size() > 0)
  {
    std::ostringstream ostr;
    for (auto jnt : limited_joints_effort)
    {
      ostr << jnt << " ";
    }
    ostr << "\b \b";  // erase last character
    RCLCPP_WARN_STREAM_THROTTLE(
      node_logging_itf_->get_logger(), *clock_, ROS_LOG_THROTTLE_PERIOD,
      "Joint(s) [" << ostr.str().c_str() << "] would exceed effort limits, limiting");
  }

  return limits_enforced;
}

}  // namespace joint_limits

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  joint_limits::JointSaturationLimiter<joint_limits::JointLimits>,
  joint_limits::JointLimiterInterface<joint_limits::JointLimits>)
