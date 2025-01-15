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

/// \author Adrià Roig Moreno
#define _USE_MATH_DEFINES
#include "joint_limits/joint_soft_limiter.hpp"

namespace joint_limits
{

bool JointSoftLimiter::on_enforce(
  const JointControlInterfacesData & actual, JointControlInterfacesData & desired,
  const rclcpp::Duration & dt)
{
  bool limits_enforced = false;

  const auto dt_seconds = dt.seconds();
  // negative or null is not allowed
  if (dt_seconds <= 0.0)
  {
    return false;
  }

  const auto hard_limits = joint_limits_[0];
  joint_limits::SoftJointLimits soft_joint_limits;
  if (!soft_joint_limits_.empty())
  {
    soft_joint_limits = soft_joint_limits_[0];
  }

  const std::string joint_name = joint_names_[0];

  if (!prev_command_.has_data())
  {
    if (actual.has_position())
    {
      prev_command_.position = actual.position;
    }
    else if (desired.has_position())
    {
      prev_command_.position = desired.position;
    }
    if (actual.has_velocity())
    {
      prev_command_.velocity = actual.velocity;
    }
    else if (desired.has_velocity())
    {
      prev_command_.velocity = desired.velocity;
    }
    if (actual.has_effort())
    {
      prev_command_.effort = actual.effort;
    }
    else if (desired.has_effort())
    {
      prev_command_.effort = desired.effort;
    }
    if (actual.has_acceleration())
    {
      prev_command_.acceleration = actual.acceleration;
    }
    else if (desired.has_acceleration())
    {
      prev_command_.acceleration = desired.acceleration;
    }
    if (actual.has_jerk())
    {
      prev_command_.jerk = actual.jerk;
    }
    else if (desired.has_jerk())
    {
      prev_command_.jerk = desired.jerk;
    }
    if (actual.has_data())
    {
      prev_command_.joint_name = actual.joint_name;
    }
    else if (desired.has_data())
    {
      prev_command_.joint_name = desired.joint_name;
    }
  }

  double soft_min_vel = -std::numeric_limits<double>::infinity();
  double soft_max_vel = std::numeric_limits<double>::infinity();
  double position = std::numeric_limits<double>::infinity();

  if (actual.has_position())
  {
    position = actual.position.value();
  }
  else if (prev_command_.has_position() && std::isfinite(prev_command_.position.value()))
  {
    position = prev_command_.position.value();
  }

  if (hard_limits.has_velocity_limits)
  {
    soft_min_vel = -hard_limits.max_velocity;
    soft_max_vel = hard_limits.max_velocity;

    if (
      hard_limits.has_position_limits && has_soft_limits(soft_joint_limits) &&
      std::isfinite(position))
    {
      soft_min_vel = std::clamp(
        -soft_joint_limits.k_position * (position - soft_joint_limits.min_position),
        -hard_limits.max_velocity, hard_limits.max_velocity);

      soft_max_vel = std::clamp(
        -soft_joint_limits.k_position * (position - soft_joint_limits.max_position),
        -hard_limits.max_velocity, hard_limits.max_velocity);

      if (
        (position < (hard_limits.min_position - internal::POSITION_BOUNDS_TOLERANCE)) ||
        (position > (hard_limits.max_position + internal::POSITION_BOUNDS_TOLERANCE)))
      {
        soft_min_vel = 0.0;
        soft_max_vel = 0.0;
      }
      else if (
        (position < soft_joint_limits.min_position) || (position > soft_joint_limits.max_position))
      {
        const double soft_limit_reach_velocity = 1.0 * (M_PI / 180.0);
        soft_min_vel = std::copysign(soft_limit_reach_velocity, soft_min_vel);
        soft_max_vel = std::copysign(soft_limit_reach_velocity, soft_max_vel);
      }
    }
  }

  if (desired.has_position())
  {
    const auto position_limits = compute_position_limits(
      hard_limits, actual.velocity, actual.position, prev_command_.position, dt_seconds);

    double pos_low = -std::numeric_limits<double>::infinity();
    double pos_high = std::numeric_limits<double>::infinity();

    if (has_soft_position_limits(soft_joint_limits))
    {
      pos_low = soft_joint_limits.min_position;
      pos_high = soft_joint_limits.max_position;
    }

    if (hard_limits.has_velocity_limits)
    {
      pos_low = std::clamp(position + soft_min_vel * dt_seconds, pos_low, pos_high);
      pos_high = std::clamp(position + soft_max_vel * dt_seconds, pos_low, pos_high);
    }
    pos_low = std::max(pos_low, position_limits.lower_limit);
    pos_high = std::min(pos_high, position_limits.upper_limit);

    limits_enforced = is_limited(desired.position.value(), pos_low, pos_high);
    desired.position = std::clamp(desired.position.value(), pos_low, pos_high);
  }

  if (desired.has_velocity())
  {
    const auto velocity_limits = compute_velocity_limits(
      joint_name, hard_limits, desired.velocity.value(), actual.position, prev_command_.velocity,
      dt_seconds);

    if (hard_limits.has_acceleration_limits && actual.has_velocity())
    {
      soft_min_vel =
        std::max(actual.velocity.value() - hard_limits.max_acceleration * dt_seconds, soft_min_vel);
      soft_max_vel =
        std::min(actual.velocity.value() + hard_limits.max_acceleration * dt_seconds, soft_max_vel);
    }

    soft_min_vel = std::max(soft_min_vel, velocity_limits.lower_limit);
    soft_max_vel = std::min(soft_max_vel, velocity_limits.upper_limit);

    limits_enforced =
      is_limited(desired.velocity.value(), soft_min_vel, soft_max_vel) || limits_enforced;
    desired.velocity = std::clamp(desired.velocity.value(), soft_min_vel, soft_max_vel);
  }

  if (desired.has_effort())
  {
    const auto effort_limits =
      compute_effort_limits(hard_limits, actual.position, actual.velocity, dt_seconds);

    double soft_min_eff = effort_limits.lower_limit;
    double soft_max_eff = effort_limits.upper_limit;

    if (
      hard_limits.has_effort_limits && std::isfinite(soft_joint_limits.k_velocity) &&
      actual.has_velocity())
    {
      soft_min_eff = std::clamp(
        -soft_joint_limits.k_velocity * (actual.velocity.value() - soft_min_vel),
        -hard_limits.max_effort, hard_limits.max_effort);

      soft_max_eff = std::clamp(
        -soft_joint_limits.k_velocity * (actual.velocity.value() - soft_max_vel),
        -hard_limits.max_effort, hard_limits.max_effort);

      soft_min_eff = std::max(soft_min_eff, effort_limits.lower_limit);
      soft_max_eff = std::min(soft_max_eff, effort_limits.upper_limit);
    }

    limits_enforced =
      is_limited(desired.effort.value(), soft_min_eff, soft_max_eff) || limits_enforced;
    desired.effort = std::clamp(desired.effort.value(), soft_min_eff, soft_max_eff);
  }

  if (desired.has_acceleration())
  {
    const auto limits =
      compute_acceleration_limits(hard_limits, desired.acceleration.value(), actual.velocity);
    limits_enforced =
      is_limited(desired.acceleration.value(), limits.lower_limit, limits.upper_limit) ||
      limits_enforced;
    desired.acceleration =
      std::clamp(desired.acceleration.value(), limits.lower_limit, limits.upper_limit);
  }

  if (desired.has_jerk())
  {
    limits_enforced =
      is_limited(desired.jerk.value(), -hard_limits.max_jerk, hard_limits.max_jerk) ||
      limits_enforced;
    desired.jerk = std::clamp(desired.jerk.value(), -hard_limits.max_jerk, hard_limits.max_jerk);
  }

  if (desired.has_position() && !std::isfinite(desired.position.value()) && actual.has_position())
  {
    desired.position = actual.position;
    limits_enforced = true;
  }
  if (desired.has_velocity() && !std::isfinite(desired.velocity.value()))
  {
    desired.velocity = 0.0;
    limits_enforced = true;
  }
  if (desired.has_acceleration() && !std::isfinite(desired.acceleration.value()))
  {
    desired.acceleration = 0.0;
    limits_enforced = true;
  }
  if (desired.has_jerk() && !std::isfinite(desired.jerk.value()))
  {
    desired.jerk = 0.0;
    limits_enforced = true;
  }

  prev_command_ = desired;

  return limits_enforced;
}

}  // namespace joint_limits

#include "pluginlib/class_list_macros.hpp"

typedef joint_limits::JointSoftLimiter JointInterfacesSoftLimiter;
typedef joint_limits::JointLimiterInterface<joint_limits::JointControlInterfacesData>
  JointInterfacesLimiterInterfaceBase;
PLUGINLIB_EXPORT_CLASS(JointInterfacesSoftLimiter, JointInterfacesLimiterInterfaceBase)
