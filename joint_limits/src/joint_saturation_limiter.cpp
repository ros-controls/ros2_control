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
bool JointSaturationLimiter<trajectory_msgs::msg::JointTrajectoryPoint>::on_enforce(
  const trajectory_msgs::msg::JointTrajectoryPoint & current_joint_states,
  trajectory_msgs::msg::JointTrajectoryPoint & desired_joint_states, const rclcpp::Duration & dt)
{
  std::lock_guard<std::mutex> lock(mutex_);
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
  const bool has_desired_position = (desired_joint_states.positions.size() == number_of_joints_);
  const bool has_desired_velocity = (desired_joint_states.velocities.size() == number_of_joints_);
  const bool has_desired_acceleration =
    (desired_joint_states.accelerations.size() == number_of_joints_);
  const bool has_current_position = (current_joint_states.positions.size() == number_of_joints_);
  const bool has_current_velocity = (current_joint_states.velocities.size() == number_of_joints_);

  // pos state and vel or pos cmd is required, vel state is optional
  if (!(has_current_position && (has_desired_position || has_desired_velocity)))
  {
    return false;
  }
  const std::vector<double> & current_joint_velocities =
    has_current_velocity ? current_joint_states.velocities
                         : std::vector<double>(number_of_joints_, 0.0);

  // reset values of vectors
  std::fill(desired_pos_.begin(), desired_pos_.end(), 0.0);
  std::fill(desired_vel_.begin(), desired_vel_.end(), 0.0);
  std::fill(desired_acc_.begin(), desired_acc_.end(), 0.0);
  std::fill(expected_pos_.begin(), expected_pos_.end(), 0.0);
  std::fill(expected_vel_.begin(), expected_vel_.end(), 0.0);

  std::fill(pos_limit_hit_.begin(), pos_limit_hit_.end(), false);
  std::fill(vel_limit_hit_.begin(), vel_limit_hit_.end(), false);
  std::fill(acc_limit_hit_.begin(), acc_limit_hit_.end(), false);
  std::fill(dec_limit_hit_.begin(), dec_limit_hit_.end(), false);
  std::fill(jerk_limit_hit_.begin(), jerk_limit_hit_.end(), false);

  bool braking_near_position_limit_triggered = false;

  clamp_joint_limits(
    has_desired_position, has_desired_velocity, has_desired_acceleration, current_joint_states,
    desired_joint_states, limits_enforced, current_joint_velocities,
    braking_near_position_limit_triggered, dt_seconds);

  if (braking_near_position_limit_triggered)
  {
    handle_braking_near_position_limit(
      current_joint_velocities, dt_seconds, has_desired_position, has_desired_velocity,
      current_joint_states);
  }

  // display limitations
  auto log_limits = [&](const std::vector<bool> & hits, const std::string & msg)
  {
    std::string out_str = "";
    for (size_t i = 0; i < number_of_joints_; ++i)
    {
      if (hits[i])
      {
        out_str += joint_names_[i] + " ";
      }
    }
    if (!out_str.empty())
    {
      out_str.pop_back();  // remove trailing space
      RCLCPP_WARN_STREAM_THROTTLE(
        node_logging_itf_->get_logger(), *clock_, ROS_LOG_THROTTLE_PERIOD,
        "Joint(s) [" << out_str << "] " << msg);
    }
  };

  log_limits(pos_limit_hit_, "would exceed position limits, limiting");
  log_limits(vel_limit_hit_, "would exceed velocity limits, limiting");
  log_limits(acc_limit_hit_, "would exceed acceleration limits, limiting");
  log_limits(dec_limit_hit_, "would exceed deceleration limits, limiting");
  log_limits(jerk_limit_hit_, "would exceed jerk limits, limiting");

  if (has_desired_position)
  {
    desired_joint_states.positions = desired_pos_;
  }
  if (has_desired_velocity)
  {
    desired_joint_states.velocities = desired_vel_;
  }
  if (has_desired_acceleration)
  {
    desired_joint_states.accelerations = desired_acc_;
  }

  return limits_enforced;
}

template <>
void JointSaturationLimiter<trajectory_msgs::msg::JointTrajectoryPoint>::clamp_joint_limits(
  const bool has_desired_position, const bool has_desired_velocity,
  const bool has_desired_acceleration,
  const trajectory_msgs::msg::JointTrajectoryPoint & current_joint_states,
  trajectory_msgs::msg::JointTrajectoryPoint & desired_joint_states, bool & limits_enforced,
  const std::vector<double> & current_joint_velocities,
  bool & braking_near_position_limit_triggered, const double dt_seconds)
{
  if (has_desired_position)
  {
    for (size_t index = 0; index < number_of_joints_; ++index)
    {
      desired_pos_[index] = desired_joint_states.positions[index];
    }
  }
  if (has_desired_velocity)
  {
    for (size_t index = 0; index < number_of_joints_; ++index)
    {
      desired_vel_[index] = desired_joint_states.velocities[index];
    }
  }
  if (has_desired_acceleration)
  {
    for (size_t index = 0; index < number_of_joints_; ++index)
    {
      desired_acc_[index] = desired_joint_states.accelerations[index];
    }
  }

  for (size_t index = 0; index < number_of_joints_; ++index)
  {
    if (has_desired_position)
    {
      // limit position
      if (joint_limits_[index].has_position_limits)
      {
        // clamp input pos_cmd
        auto pos = std::clamp(
          desired_pos_[index], joint_limits_[index].min_position,
          joint_limits_[index].max_position);
        if (pos != desired_pos_[index])
        {
          desired_pos_[index] = pos;
          pos_limit_hit_[index] = true;
          limits_enforced = true;
        }
      }
      // priority to pos_cmd derivative over cmd_vel when not defined. If done always then we might
      // get jumps in the velocity based on the system's dynamics. Position limit clamping is done
      // below once again.
      const double position_difference =
        desired_pos_[index] - current_joint_states.positions[index];
      if (
        std::fabs(position_difference) > VALUE_CONSIDERED_ZERO &&
        std::fabs(desired_vel_[index]) <= VALUE_CONSIDERED_ZERO)
      {
        desired_vel_[index] = position_difference / dt_seconds;
      }
    }

    // limit velocity
    if (joint_limits_[index].has_velocity_limits)
    {
      // if desired velocity is not defined calculate it from positions
      if (
        std::fabs(desired_vel_[index]) <= VALUE_CONSIDERED_ZERO || std::isnan(desired_vel_[index]))
      {
        desired_vel_[index] =
          (desired_pos_[index] - current_joint_states.positions[index]) / dt_seconds;
      }
      // clamp input vel_cmd
      if (std::fabs(desired_vel_[index]) > joint_limits_[index].max_velocity)
      {
        desired_vel_[index] = std::copysign(joint_limits_[index].max_velocity, desired_vel_[index]);
        vel_limit_hit_[index] = true;
        limits_enforced = true;

        // recompute pos_cmd if needed
        if (has_desired_position)
        {
          desired_pos_[index] =
            current_joint_states.positions[index] + desired_vel_[index] * dt_seconds;
        }

        desired_acc_[index] = (desired_vel_[index] - current_joint_velocities[index]) / dt_seconds;
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
                                        std::vector<bool> & limit_hit) -> bool
        {
          if (std::fabs(acc[index]) > max_acc_or_dec)
          {
            acc[index] = std::copysign(max_acc_or_dec, acc[index]);
            limit_hit[index] = true;
            limits_enforced = true;
            return true;
          }
          else
          {
            return false;
          }
        };

        // if desired acceleration if not provided compute it from desired_vel_ and vel_state
        if (
          std::fabs(desired_acc_[index]) <= VALUE_CONSIDERED_ZERO ||
          std::isnan(desired_acc_[index]))
        {
          desired_acc_[index] =
            (desired_vel_[index] - current_joint_velocities[index]) / dt_seconds;
        }

        // check if decelerating - if velocity is changing toward 0
        bool deceleration_limit_applied = false;
        bool limit_applied = false;
        if (
          (desired_acc_[index] < 0 && current_joint_velocities[index] > 0) ||
          (desired_acc_[index] > 0 && current_joint_velocities[index] < 0))
        {
          // limit deceleration
          if (joint_limits_[index].has_deceleration_limits)
          {
            limit_applied = apply_acc_or_dec_limit(
              joint_limits_[index].max_deceleration, desired_acc_, dec_limit_hit_);
            deceleration_limit_applied = true;
          }
        }

        // limit acceleration (fallback to acceleration if no deceleration limits)
        if (joint_limits_[index].has_acceleration_limits && !deceleration_limit_applied)
        {
          limit_applied = apply_acc_or_dec_limit(
            joint_limits_[index].max_acceleration, desired_acc_, acc_limit_hit_);
        }

        if (limit_applied)
        {
          // vel_cmd from integration of desired_acc_, needed even if no vel output
          desired_vel_[index] = current_joint_velocities[index] + desired_acc_[index] * dt_seconds;
          if (has_desired_position)
          {
            // pos_cmd from from double integration of desired_acc_
            desired_pos_[index] = current_joint_states.positions[index] +
                                  current_joint_velocities[index] * dt_seconds +
                                  0.5 * desired_acc_[index] * dt_seconds * dt_seconds;
          }
        }
      }
      // else we cannot compute acc, so not limiting it
    }

    // Limit jerk
    if (joint_limits_[index].has_jerk_limits)
    {
      // check if desired acceleration is zero or corrupted
      if (
        std::fabs(desired_acc_[index]) <= VALUE_CONSIDERED_ZERO || std::isnan(desired_acc_[index]))
      {
        desired_acc_[index] = (desired_vel_[index] - current_joint_velocities[index]) / dt_seconds;
      }
      const double current_acceleration =
        (current_joint_states.accelerations.size() == number_of_joints_)
          ? current_joint_states.accelerations[index]
          : 0.0f;

      // Calc desired jerk over this time
      double desired_jerk = (desired_acc_[index] - current_acceleration) / dt_seconds;

      // Limit Jerk
      if (std::fabs(desired_jerk) > joint_limits_[index].max_jerk)
      {
        desired_jerk = std::copysign(joint_limits_[index].max_jerk, desired_jerk);
        jerk_limit_hit_[index] = true;
        limits_enforced = true;

        // Backward recalculation: Update acceleration based on limited jerk
        desired_acc_[index] = current_acceleration + (desired_jerk * dt_seconds);

        // Backward recalculation: Recompute velocity and position
        desired_vel_[index] = current_joint_velocities[index] + desired_acc_[index] * dt_seconds;
        if (has_desired_position)
        {
          desired_pos_[index] = current_joint_states.positions[index] +
                                (current_joint_velocities[index] * dt_seconds) +
                                (0.5 * desired_acc_[index] * dt_seconds * dt_seconds);
        }
      }
    }

    // plan ahead for position limits
    if (joint_limits_[index].has_position_limits)
    {
      if (has_desired_velocity && !has_desired_position)
      {
        // Check  immediate next step when using vel_cmd only, other cases already handled
        // integrate pos
        expected_pos_[index] =
          current_joint_states.positions[index] + desired_vel_[index] * dt_seconds;
        // if expected_pos_ over limit
        auto pos = std::clamp(
          expected_pos_[index], joint_limits_[index].min_position,
          joint_limits_[index].max_position);
        if (pos != expected_pos_[index])
        {
          // TODO(gwalck) compute vel_cmd that would permit to slow down in time at full
          // deceleration in any case limit pos to max
          expected_pos_[index] = pos;
          // and recompute vel_cmd that would lead to pos_max (not ideal as velocity would not be
          // zero)
          desired_vel_[index] =
            (expected_pos_[index] - current_joint_states.positions[index]) / dt_seconds;
          pos_limit_hit_[index] = true;
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
      double stopping_deccel = std::fabs(desired_vel_[index] / dt_seconds);
      if (joint_limits_[index].has_deceleration_limits)
      {
        stopping_deccel = joint_limits_[index].max_deceleration;
      }
      else if (joint_limits_[index].has_acceleration_limits)
      {
        stopping_deccel = joint_limits_[index].max_acceleration;
      }

      double stopping_distance =
        std::fabs((-desired_vel_[index] * desired_vel_[index]) / (2 * stopping_deccel));
      // compute stopping duration at stopping_deccel
      double stopping_duration = std::fabs((desired_vel_[index]) / (stopping_deccel));

      // Check that joint limits are beyond stopping_distance and desired_velocity is towards
      // that limit
      if (
        (desired_vel_[index] < 0 &&
         (current_joint_states.positions[index] - joint_limits_[index].min_position <
          stopping_distance)) ||
        (desired_vel_[index] > 0 &&
         (joint_limits_[index].max_position - current_joint_states.positions[index] <
          stopping_distance)))
      {
        pos_limit_hit_[index] = true;
        braking_near_position_limit_triggered = true;
        limits_enforced = true;
      }
      else
      {
        // compute the travel_distance at new desired velocity, with best case duration
        // stopping_duration
        double motion_after_stopping_duration = desired_vel_[index] * stopping_duration;
        // re-check what happens if we don't slow down
        if (
          (desired_vel_[index] < 0 &&
           (current_joint_states.positions[index] - joint_limits_[index].min_position <
            motion_after_stopping_duration)) ||
          (desired_vel_[index] > 0 &&
           (joint_limits_[index].max_position - current_joint_states.positions[index] <
            motion_after_stopping_duration)))
        {
          pos_limit_hit_[index] = true;
          braking_near_position_limit_triggered = true;
          limits_enforced = true;
        }
        // else no need to slow down. in worse case we won't hit the limit at current velocity
      }
    }
  }
}

template <>
void JointSaturationLimiter<trajectory_msgs::msg::JointTrajectoryPoint>::
  handle_braking_near_position_limit(
    const std::vector<double> & current_joint_velocities, double dt_seconds,
    bool has_desired_position, bool has_desired_velocity,
    const trajectory_msgs::msg::JointTrajectoryPoint & current_joint_states)
{
  for (size_t index = 0; index < number_of_joints_; ++index)
  {
    desired_acc_[index] = -current_joint_velocities[index] / dt_seconds;
    if (joint_limits_[index].has_deceleration_limits)
    {
      desired_acc_[index] = std::copysign(
        std::min(std::fabs(desired_acc_[index]), joint_limits_[index].max_deceleration),
        desired_acc_[index]);
    }
    else if (joint_limits_[index].has_acceleration_limits)
    {
      desired_acc_[index] = std::copysign(
        std::min(std::fabs(desired_acc_[index]), joint_limits_[index].max_acceleration),
        desired_acc_[index]);
    }
  }

  if (has_desired_velocity)
  {
    for (size_t index = 0; index < number_of_joints_; ++index)
    {
      desired_vel_[index] = current_joint_velocities[index] + desired_acc_[index] * dt_seconds;
    }
  }
  if (has_desired_position)
  {
    for (size_t index = 0; index < number_of_joints_; ++index)
    {
      desired_pos_[index] = current_joint_states.positions[index] +
                            current_joint_velocities[index] * dt_seconds +
                            0.5 * desired_acc_[index] * dt_seconds * dt_seconds;
    }
  }

  auto log_limits = [&](const std::vector<bool> & hits, const std::string & msg)
  {
    std::string out_str = "";
    for (size_t i = 0; i < number_of_joints_; ++i)
    {
      if (hits[i])
      {
        out_str += joint_names_[i] + " ";
      }
    }
    if (!out_str.empty())
    {
      out_str.pop_back();  // remove trailing space
      RCLCPP_WARN_STREAM_THROTTLE(
        node_logging_itf_->get_logger(), *clock_, ROS_LOG_THROTTLE_PERIOD,
        "Joint(s) [" << out_str << "] " << msg);
    }
  };

  log_limits(
    pos_limit_hit_,
    "would exceed position limits"
    " if continuing at current state, limiting all joints");
}

}  // namespace joint_limits

#include "pluginlib/class_list_macros.hpp"

// typedefs are needed here to avoid issues with macro expansion. ref:
// https://stackoverflow.com/a/8942986
using int_map = std::map<int, int>;
using JointTrajectoryPointSaturationLimiter =
  joint_limits::JointSaturationLimiter<trajectory_msgs::msg::JointTrajectoryPoint>;
using JointTrajectoryPointLimiterInterfaceBase =
  joint_limits::JointLimiterInterface<trajectory_msgs::msg::JointTrajectoryPoint>;
PLUGINLIB_EXPORT_CLASS(
  JointTrajectoryPointSaturationLimiter, JointTrajectoryPointLimiterInterfaceBase)
