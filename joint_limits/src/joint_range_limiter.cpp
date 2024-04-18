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
#include "joint_limits/joint_limiter_struct.hpp"
#include "rclcpp/duration.hpp"
#include "rcutils/logging_macros.h"

bool is_limited(double value, double min, double max) { return value < min || value > max; }

std::pair<double, double> compute_position_limits(
  const joint_limits::JointLimits & limits, double prev_command_pos, double dt)
{
  std::pair<double, double> pos_limits({limits.min_position, limits.max_position});
  if (limits.has_velocity_limits)
  {
    const double delta_vel =
      limits.has_acceleration_limits ? limits.max_acceleration * dt : limits.max_velocity;
    const double max_vel = std::min(limits.max_velocity, delta_vel);
    const double delta_pos = max_vel * dt;
    pos_limits.first = std::max(prev_command_pos - delta_pos, pos_limits.first);
    pos_limits.second = std::min(prev_command_pos + delta_pos, pos_limits.second);
  }
  return pos_limits;
}

std::pair<double, double> compute_position_limits(
  const joint_limits::JointLimits & limits, const joint_limits::SoftJointLimits & soft_limits,
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
  const std::string & joint_name, const joint_limits::JointLimits & limits,
  const std::optional<double> & act_pos, const std::optional<double> & prev_command_vel, double dt)
{
  const double max_vel =
    limits.has_velocity_limits ? limits.max_velocity : std::numeric_limits<double>::infinity();
  std::pair<double, double> vel_limits({-max_vel, max_vel});
  if (limits.has_position_limits && act_pos.has_value())
  {
    const double max_vel_with_pos_limits = (limits.max_position - act_pos.value()) / dt;
    const double min_vel_with_pos_limits = (limits.min_position - act_pos.value()) / dt;
    vel_limits.first = std::max(min_vel_with_pos_limits, vel_limits.first);
    vel_limits.second = std::min(max_vel_with_pos_limits, vel_limits.second);
    if (act_pos.value() > limits.max_position || act_pos.value() < limits.min_position)
    {
      RCLCPP_ERROR_ONCE(
        rclcpp::get_logger("joint_limiter_interface"),
        "Joint position is out of bounds for the joint : '%s'. Joint velocity limits will be "
        "restrictred to zero.",
        joint_name.c_str());
      vel_limits = {0.0, 0.0};
    }
  }
  if (limits.has_acceleration_limits && prev_command_vel.has_value())
  {
    const double delta_vel = limits.max_acceleration * dt;
    vel_limits.first = std::max(prev_command_vel.value() - delta_vel, vel_limits.first);
    vel_limits.second = std::min(prev_command_vel.value() + delta_vel, vel_limits.second);
  }
  return vel_limits;
}

std::pair<double, double> compute_effort_limits(
  const joint_limits::JointLimits & limits, const std::optional<double> & act_pos,
  const std::optional<double> & act_vel, double /*dt*/)
{
  const double max_effort =
    limits.has_effort_limits ? limits.max_effort : std::numeric_limits<double>::infinity();
  std::pair<double, double> eff_limits({-max_effort, max_effort});
  if (limits.has_position_limits && act_pos.has_value() && act_vel.has_value())
  {
    if ((act_pos.value() <= limits.min_position) && (act_vel.value() <= 0.0))
    {
      eff_limits.first = 0.0;
    }
    else if ((act_pos.value() >= limits.max_position) && (act_vel.value() >= 0.0))
    {
      eff_limits.second = 0.0;
    }
  }
  if (limits.has_velocity_limits && act_vel.has_value())
  {
    if (act_vel.value() < -limits.max_velocity)
    {
      eff_limits.first = 0.0;
    }
    else if (act_vel.value() > limits.max_velocity)
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
bool JointSaturationLimiter<JointLimits, JointControlInterfacesData>::on_init()
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
bool JointSaturationLimiter<JointLimits, JointControlInterfacesData>::on_enforce(
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
    const auto limits =
      compute_position_limits(joint_limits, prev_command_.position.value(), dt_seconds);
    limits_enforced = is_limited(desired.position.value(), limits.first, limits.second);
    desired.position = std::clamp(desired.position.value(), limits.first, limits.second);
  }

  if (desired.has_velocity())
  {
    const auto limits = compute_velocity_limits(
      joint_name, joint_limits, actual.position, prev_command_.velocity, dt_seconds);
    limits_enforced =
      limits_enforced || is_limited(desired.velocity.value(), limits.first, limits.second);
    desired.velocity = std::clamp(desired.velocity.value(), limits.first, limits.second);
  }

  if (desired.has_effort())
  {
    const auto limits =
      compute_effort_limits(joint_limits, actual.position, actual.velocity, dt_seconds);
    limits_enforced =
      limits_enforced || is_limited(desired.effort.value(), limits.first, limits.second);
    desired.effort = std::clamp(desired.effort.value(), limits.first, limits.second);
  }

  if (desired.has_acceleration())
  {
    // limiting acc or dec function
    auto apply_acc_or_dec_limit = [&](const double max_acc_or_dec, double & acc) -> bool
    {
      if (std::fabs(acc) > max_acc_or_dec)
      {
        acc = std::copysign(max_acc_or_dec, acc);
        return true;
      }
      else
      {
        return false;
      }
    };

    // check if decelerating - if velocity is changing toward 0
    double desired_acc = desired.acceleration.value();
    if (
      joint_limits.has_deceleration_limits &&
      ((desired.acceleration.value() < 0 && actual.velocity.has_value() &&
        actual.velocity.value() > 0) ||
       (desired.acceleration.value() > 0 && actual.velocity.has_value() &&
        actual.velocity.value() < 0)))
    {
      // limit deceleration
      limits_enforced =
        limits_enforced || apply_acc_or_dec_limit(joint_limits.max_deceleration, desired_acc);
    }
    else
    {
      // limit acceleration (fallback to acceleration if no deceleration limits)
      if (joint_limits.has_acceleration_limits)
      {
        limits_enforced =
          limits_enforced || apply_acc_or_dec_limit(joint_limits.max_acceleration, desired_acc);
      }
    }
    desired.acceleration = desired_acc;
  }

  if (desired.has_jerk())
  {
    limits_enforced =
      limits_enforced ||
      is_limited(desired.jerk.value(), -joint_limits.max_jerk, joint_limits.max_jerk);
    desired.jerk = std::clamp(desired.jerk.value(), -joint_limits.max_jerk, joint_limits.max_jerk);
  }

  prev_command_ = desired;

  return limits_enforced;
}

}  // namespace joint_limits

#include "pluginlib/class_list_macros.hpp"

// typedefs are needed here to avoid issues with macro expansion. ref:
// https://stackoverflow.com/a/8942986
typedef joint_limits::JointSaturationLimiter<
  joint_limits::JointLimits, joint_limits::JointControlInterfacesData>
  JointInterfacesSaturationLimiter;
typedef joint_limits::JointLimiterInterface<
  joint_limits::JointLimits, joint_limits::JointControlInterfacesData>
  JointInterfacesLimiterInterfaceBase;
PLUGINLIB_EXPORT_CLASS(JointInterfacesSaturationLimiter, JointInterfacesLimiterInterfaceBase)
