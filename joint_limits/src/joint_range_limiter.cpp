// Copyright 2024 PAL Robotics S.L.
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

/// \author Sai Kishor Kothakota

#include "joint_limits/joint_range_limiter.hpp"

#include <algorithm>
#include "joint_limits/joint_limiter_struct.hpp"
#include "rclcpp/duration.hpp"
#include "rcutils/logging_macros.h"

std::pair<double, double> compute_position_limits(
  joint_limits::JointLimits limits, double prev_command_pos, double dt)
{
  std::pair<double, double> pos_limits({limits.min_position, limits.max_position});
  if (limits.has_velocity_limits)
  {
    const double delta_pos = limits.max_velocity * dt;
    pos_limits.first = std::max(prev_command_pos - delta_pos, pos_limits.first);
    pos_limits.second = std::min(prev_command_pos + delta_pos, pos_limits.second);
  }
  return pos_limits;
}

std::pair<double, double> compute_position_limits(
  joint_limits::JointLimits limits, joint_limits::SoftJointLimits soft_limits,
  double prev_command_pos, double dt)
{
  std::pair<double, double> pos_limits({limits.min_position, limits.max_position});

  // velocity bounds
  double soft_min_vel = -limits.max_velocity;
  double soft_max_vel = limits.max_velocity;

  if (limits.has_position_limits)
  {
    soft_min_vel = std::clamp(
      -soft_limits.k_position * (prev_command_pos - soft_limits.min_position), -limits.max_velocity,
      limits.max_velocity);
    soft_max_vel = std::clamp(
      -soft_limits.k_position * (prev_command_pos - soft_limits.max_position), -limits.max_velocity,
      limits.max_velocity);
  }
  // Position bounds
  pos_limits.first = prev_command_pos + soft_min_vel * dt;
  pos_limits.second = prev_command_pos + soft_max_vel * dt;
  return pos_limits;
}

std::pair<double, double> compute_velocity_limits(
  joint_limits::JointLimits limits, double act_pos, double prev_command_vel, double dt)
{
  const double max_vel =
    limits.has_velocity_limits ? limits.max_velocity : std::numeric_limits<double>::infinity();
  std::pair<double, double> vel_limits({-max_vel, max_vel});
  if (limits.has_position_limits)
  {
    const double max_vel_with_pos_limits = (limits.max_position - act_pos) / dt;
    const double min_vel_with_pos_limits = (limits.min_position - act_pos) / dt;
    vel_limits.first = std::max(min_vel_with_pos_limits, vel_limits.first);
    vel_limits.second = std::min(max_vel_with_pos_limits, vel_limits.second);
  }
  if (limits.has_acceleration_limits)
  {
    const double delta_vel = limits.max_acceleration * dt;
    vel_limits.first = std::max(prev_command_vel - delta_vel, vel_limits.first);
    vel_limits.second = std::min(prev_command_vel + delta_vel, vel_limits.second);
  }
  return vel_limits;
}

std::pair<double, double> compute_effort_limits(
  joint_limits::JointLimits limits, double act_pos, double act_vel, double /*dt*/)
{
  const double max_effort =
    limits.has_effort_limits ? limits.max_effort : std::numeric_limits<double>::infinity();
  std::pair<double, double> eff_limits({-max_effort, max_effort});
  if (limits.has_position_limits)
  {
    if ((act_pos <= limits.min_position) && (act_vel <= 0.0))
    {
      eff_limits.first = 0.0;
    }
    else if ((act_pos >= limits.max_position) && (act_vel >= 0.0))
    {
      eff_limits.second = 0.0;
    }
  }
  if (limits.has_velocity_limits)
  {
    if (act_vel < -limits.max_velocity)
    {
      eff_limits.first = 0.0;
    }
    else if (act_vel > limits.max_velocity)
    {
      eff_limits.second = 0.0;
    }
  }
  return eff_limits;
}

constexpr size_t ROS_LOG_THROTTLE_PERIOD = 1 * 1000;  // Milliseconds to throttle logs inside loops
constexpr double VALUE_CONSIDERED_ZERO = 1e-10;

namespace joint_limits
{

template <>
bool JointRangeLimiter<JointLimits, JointControlInterfacesData>::on_init()
{
  const bool result = (number_of_joints_ != 1);
  if (!result && has_logging_interface())
  {
    RCLCPP_ERROR(
      node_logging_itf_->get_logger(),
      "JointLimiter: The JointRangeLimiter expects the number of joints to be 1, but given : %zu",
      number_of_joints_);
  }
  return result;
}

template <>
bool JointRangeLimiter<JointLimits, JointControlInterfacesData>::on_enforce(std::vector<double> &)
{
  if (has_logging_interface())
  {
    RCLCPP_WARN(
      node_logging_itf_->get_logger(),
      "JointRangeLimiter::on_enforce"
      "(std::vector<double> & desired_joint_states) is not needed for this limiter.");
  }
  return false;
}

template <>
bool JointRangeLimiter<JointLimits, JointControlInterfacesData>::on_enforce(
  JointControlInterfacesData & actual, JointControlInterfacesData & desired,
  const rclcpp::Duration & dt)
{
  bool limits_enforced = false;

  const auto dt_seconds = dt.seconds();
  // negative or null is not allowed
  if (dt_seconds <= 0.0)
  {
    return false;
  }

  const auto joint_limits = joint_limits_[0];
  const std::string joint_name = joint_names_[0];
  if (!prev_command_.has_data())
  {
    prev_command_ = actual;
  }

  if (desired.has_position())
  {
    const auto limits =
      compute_position_limits(joint_limits, prev_command_.position.value(), dt_seconds);
    desired.position = std::clamp(desired.position.value(), limits.first, limits.second);
  }

  if (desired.has_velocity())
  {
    const auto limits = compute_velocity_limits(
      joint_limits, actual.position.value(), prev_command_.velocity.value(), dt_seconds);
    desired.velocity = std::clamp(desired.velocity.value(), limits.first, limits.second);
  }

  if (desired.has_effort())
  {
    const auto limits = compute_effort_limits(
      joint_limits, actual.position.value(), actual.velocity.value(), dt_seconds);
    desired.effort = std::clamp(desired.effort.value(), limits.first, limits.second);
  }

  if (desired.has_acceleration())
  {
    desired.acceleration = std::clamp(
      desired.acceleration.value(), -joint_limits.max_deceleration, joint_limits.max_acceleration);
  }

  if (desired.has_jerk())
  {
    desired.jerk = std::clamp(desired.jerk.value(), -joint_limits.max_jerk, joint_limits.max_jerk);
  }

  return limits_enforced;
}

}  // namespace joint_limits

#include "pluginlib/class_list_macros.hpp"

// typedefs are needed here to avoid issues with macro expansion. ref:
// https://stackoverflow.com/a/8942986
typedef joint_limits::JointRangeLimiter<
  joint_limits::JointLimits, joint_limits::JointControlInterfacesData>
  JointInterfacesRangeLimiter;
typedef joint_limits::JointLimiterInterface<
  joint_limits::JointLimits, joint_limits::JointControlInterfacesData>
  JointInterfacesRangeLimiterBase;
PLUGINLIB_EXPORT_CLASS(JointInterfacesRangeLimiter, JointInterfacesRangeLimiterBase)
