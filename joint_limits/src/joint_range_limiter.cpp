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

#include "joint_limits/joint_saturation_limiter.hpp"

#include <algorithm>
#include "joint_limits/data_structures.hpp"
#include "joint_limits/joint_limits_helpers.hpp"
#include "rclcpp/duration.hpp"
#include "rcutils/logging_macros.h"

namespace joint_limits
{

template <>
bool JointSaturationLimiter<JointControlInterfacesData>::on_init()
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

template <>
bool JointSaturationLimiter<JointControlInterfacesData>::on_enforce(
  const JointControlInterfacesData & actual, JointControlInterfacesData & desired,
  const rclcpp::Duration & dt)
{
  std::lock_guard<std::mutex> lock(mutex_);
  bool limits_enforced = false;

  const auto dt_seconds = dt.seconds();
  // negative or null is not allowed
  if (dt_seconds <= 0.0)
  {
    return false;
  }

  const auto joint_limits = joint_limits_[0];
  const std::string joint_name = joint_names_[0];
  // The following conditional filling is needed for cases of having certain information missing
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

  if (desired.has_position())
  {
    const auto limits = compute_position_limits(
      joint_name, joint_limits, actual.velocity, actual.position, prev_command_.position,
      dt_seconds);
    limits_enforced = is_limited(desired.position.value(), limits.lower_limit, limits.upper_limit);
    desired.position = std::clamp(desired.position.value(), limits.lower_limit, limits.upper_limit);
  }

  if (desired.has_velocity())
  {
    const auto limits = compute_velocity_limits(
      joint_name, joint_limits, desired.velocity.value(), actual.position, prev_command_.velocity,
      dt_seconds);
    limits_enforced =
      is_limited(desired.velocity.value(), limits.lower_limit, limits.upper_limit) ||
      limits_enforced;
    desired.velocity = std::clamp(desired.velocity.value(), limits.lower_limit, limits.upper_limit);
  }

  if (desired.has_effort())
  {
    const auto limits =
      compute_effort_limits(joint_limits, actual.position, actual.velocity, dt_seconds);
    limits_enforced =
      is_limited(desired.effort.value(), limits.lower_limit, limits.upper_limit) || limits_enforced;
    desired.effort = std::clamp(desired.effort.value(), limits.lower_limit, limits.upper_limit);
  }

  if (desired.has_acceleration())
  {
    const auto limits =
      compute_acceleration_limits(joint_limits, desired.acceleration.value(), actual.velocity);
    limits_enforced =
      is_limited(desired.acceleration.value(), limits.lower_limit, limits.upper_limit) ||
      limits_enforced;
    desired.acceleration =
      std::clamp(desired.acceleration.value(), limits.lower_limit, limits.upper_limit);
  }

  if (desired.has_jerk())
  {
    limits_enforced =
      is_limited(desired.jerk.value(), -joint_limits.max_jerk, joint_limits.max_jerk) ||
      limits_enforced;
    desired.jerk = std::clamp(desired.jerk.value(), -joint_limits.max_jerk, joint_limits.max_jerk);
  }

  prev_command_ = desired;

  return limits_enforced;
}

}  // namespace joint_limits

#include "pluginlib/class_list_macros.hpp"

// typedefs are needed here to avoid issues with macro expansion. ref:
// https://stackoverflow.com/a/8942986
typedef joint_limits::JointSaturationLimiter<joint_limits::JointControlInterfacesData>
  JointInterfacesSaturationLimiter;
typedef joint_limits::JointLimiterInterface<joint_limits::JointControlInterfacesData>
  JointInterfacesLimiterInterfaceBase;
PLUGINLIB_EXPORT_CLASS(JointInterfacesSaturationLimiter, JointInterfacesLimiterInterfaceBase)
