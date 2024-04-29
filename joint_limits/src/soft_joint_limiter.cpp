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

/// \author Adrià Roig Moreno

#include "joint_limits/joint_saturation_limiter.hpp"
#include "joint_limits/joint_limiter_struct.hpp"
#include "joint_limits/joint_limits_helpers.hpp"
#include <cmath>

constexpr double VALUE_CONSIDERED_ZERO = 1e-10;

namespace joint_limits
{

class SoftJointLimiter : public JointSaturationLimiter<JointControlInterfacesData>
{
public:

bool on_init()
{
  const bool result = (number_of_joints_ == 1);
  if (!result && has_logging_interface())
  {
    RCLCPP_ERROR(
      node_logging_itf_->get_logger(),
      "JointInterfacesSaturationLimiter: Expects the number of joints to be 1, but given : "
      "%zu",
      number_of_joints_);
  }
  prev_command_ = JointControlInterfacesData();
  return result;
}

bool on_enforce(
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

  const auto hard_limits = joint_limits_[0];
  joint_limits::SoftJointLimits soft_joint_limits;
  if(!soft_joint_limits_.empty())
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

  if (hard_limits.has_velocity_limits)
  {
    soft_min_vel = -hard_limits.max_velocity;
    soft_max_vel = hard_limits.max_velocity;

    if(hard_limits.has_position_limits && has_soft_limits(soft_joint_limits))
    {
      soft_min_vel = std::clamp(-soft_joint_limits.k_position * (prev_command_.position.value() - soft_joint_limits.min_position),
                                -hard_limits.max_velocity, hard_limits.max_velocity);

      soft_max_vel = std::clamp(-soft_joint_limits.k_position * (prev_command_.position.value() - soft_joint_limits.max_position),
                                -hard_limits.max_velocity, hard_limits.max_velocity);
    }
  }

  if(desired.has_position())
  {
    const auto position_limits =
      compute_position_limits(hard_limits, actual.velocity, prev_command_.position, dt_seconds);

    double pos_low = -std::numeric_limits<double>::infinity();
    double pos_high = std::numeric_limits<double>::infinity();

    if(has_soft_position_limits(soft_joint_limits))
    {
      pos_low = soft_joint_limits.min_position;
      pos_high = soft_joint_limits.max_position;
    }

    if(hard_limits.has_velocity_limits)
    {
      pos_low = prev_command_.position.value() + soft_min_vel * dt_seconds;
      pos_high = prev_command_.position.value() + soft_max_vel * dt_seconds;
    }

    pos_low  = std::max(pos_low,  position_limits.first);
    pos_high = std::min(pos_high, position_limits.second);

    limits_enforced = is_limited(desired.position.value(), pos_low, pos_high);
    desired.position = std::clamp(desired.position.value(), pos_low, pos_high);
  }

  if(desired.has_velocity())
  {
    const auto velocity_limits = compute_velocity_limits(
      joint_name, hard_limits, actual.position, prev_command_.velocity, dt_seconds);

    if(hard_limits.has_acceleration_limits)
    {
      soft_min_vel = std::max(prev_command_.velocity.value() - hard_limits.max_acceleration * dt_seconds, soft_min_vel);
      soft_max_vel = std::min(prev_command_.velocity.value() + hard_limits.max_acceleration * dt_seconds, soft_max_vel);
    }

    soft_min_vel = std::max(soft_min_vel,  velocity_limits.first);
    soft_max_vel = std::min(soft_max_vel, velocity_limits.second);

    limits_enforced = is_limited(desired.velocity.value(), soft_min_vel, soft_max_vel) || limits_enforced;
    desired.velocity = std::clamp(desired.velocity.value(), soft_min_vel, soft_max_vel);
  }

  if(desired.has_effort())
  {
    const auto effort_limits =
          compute_effort_limits(hard_limits, actual.position, actual.velocity, dt_seconds);

    double soft_min_eff = effort_limits.first;
    double soft_max_eff = effort_limits.second;

    if(hard_limits.has_effort_limits && std::isfinite(soft_joint_limits.k_velocity))
    {
      soft_min_eff = std::clamp(-soft_joint_limits.k_velocity * (prev_command_.velocity.value() - soft_min_vel),
                                -hard_limits.max_effort, hard_limits.max_effort);

      soft_max_eff = std::clamp(-soft_joint_limits.k_velocity * (prev_command_.velocity.value() - soft_max_vel),
                                -hard_limits.max_effort, hard_limits.max_effort);

      soft_min_eff = std::max(soft_min_eff,  effort_limits.first);
      soft_max_eff = std::min(soft_max_eff, effort_limits.second);
    }
    limits_enforced = is_limited(desired.effort.value(), soft_min_eff, soft_max_eff) || limits_enforced;
    desired.effort = std::clamp(desired.effort.value(), soft_min_eff, soft_max_eff);
  }

  if (desired.has_acceleration())
  {
    const auto limits =
      compute_acceleration_limits(hard_limits, desired.acceleration.value(), actual.velocity);
    limits_enforced =
      is_limited(desired.acceleration.value(), limits.first, limits.second) || limits_enforced;
    desired.acceleration = std::clamp(desired.acceleration.value(), limits.first, limits.second);
  }

  if (desired.has_jerk())
  {
    limits_enforced =
      is_limited(desired.jerk.value(), -hard_limits.max_jerk, hard_limits.max_jerk) ||
      limits_enforced;
    desired.jerk = std::clamp(desired.jerk.value(), -hard_limits.max_jerk, hard_limits.max_jerk);
  }

  prev_command_ = desired;

  return limits_enforced;
}

bool has_soft_position_limits(const joint_limits::SoftJointLimits &soft_joint_limits)
{
  return std::isfinite(soft_joint_limits.min_position) && std::isfinite(soft_joint_limits.max_position)
      && (soft_joint_limits.max_position - soft_joint_limits.min_position) > VALUE_CONSIDERED_ZERO;
}

bool has_soft_limits(const joint_limits::SoftJointLimits &soft_joint_limits)
{
  return has_soft_position_limits(soft_joint_limits) && std::isfinite(soft_joint_limits.k_position)
      && std::abs(soft_joint_limits.k_position) > VALUE_CONSIDERED_ZERO;
}

};

} // namespace joint_limits

#include "pluginlib/class_list_macros.hpp"

typedef joint_limits::SoftJointLimiter JointInterfacesSoftLimiter;
typedef joint_limits::JointLimiterInterface<joint_limits::JointControlInterfacesData>
  JointInterfacesLimiterInterfaceBase;
PLUGINLIB_EXPORT_CLASS(
  JointInterfacesSoftLimiter, JointInterfacesLimiterInterfaceBase)


