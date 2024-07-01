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

#include "joint_limits/joint_limits_helpers.hpp"
#include <algorithm>
#include <cmath>
#include "rclcpp/logging.hpp"

namespace joint_limits
{
namespace internal
{
/**
 * @brief Check if the limits are in the correct order and swap them if they are not.
 */
void check_and_swap_limits(std::pair<double, double> & limits)
{
  if (limits.first > limits.second)
  {
    std::swap(limits.first, limits.second);
  }
}
}  // namespace internal

bool is_limited(double value, double min, double max) { return value < min || value > max; }

std::pair<double, double> compute_position_limits(
  const joint_limits::JointLimits & limits, const std::optional<double> & act_vel,
  const std::optional<double> & prev_command_pos, double dt)
{
  std::pair<double, double> pos_limits({limits.min_position, limits.max_position});
  if (limits.has_velocity_limits)
  {
    const double act_vel_abs = act_vel.has_value() ? std::fabs(act_vel.value()) : 0.0;
    const double delta_vel = limits.has_acceleration_limits
                               ? act_vel_abs + (limits.max_acceleration * dt)
                               : limits.max_velocity;
    const double max_vel = std::min(limits.max_velocity, delta_vel);
    const double delta_pos = max_vel * dt;
    pos_limits.first = std::max(prev_command_pos.value() - delta_pos, pos_limits.first);
    pos_limits.second = std::min(prev_command_pos.value() + delta_pos, pos_limits.second);
  }
  internal::check_and_swap_limits(pos_limits);
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
  RCLCPP_ERROR(
    rclcpp::get_logger("joint_limiter_interface"),
    "Joint velocity limits for joint '%s' are [%f, %f]", joint_name.c_str(), vel_limits.first,
    vel_limits.second);
  internal::check_and_swap_limits(vel_limits);
  RCLCPP_ERROR(
    rclcpp::get_logger("joint_limiter_interface"),
    "After swapping Joint velocity limits for joint '%s' are [%f, %f]", joint_name.c_str(),
    vel_limits.first, vel_limits.second);
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
  internal::check_and_swap_limits(eff_limits);
  return eff_limits;
}

std::pair<double, double> compute_acceleration_limits(
  const joint_limits::JointLimits & limits, double desired_acceleration,
  std::optional<double> actual_velocity)
{
  std::pair<double, double> acc_or_dec_limits(
    -std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity());
  if (
    limits.has_deceleration_limits &&
    ((desired_acceleration < 0 && actual_velocity && actual_velocity.value() > 0) ||
     (desired_acceleration > 0 && actual_velocity && actual_velocity.value() < 0)))
  {
    acc_or_dec_limits.first = -limits.max_deceleration;
    acc_or_dec_limits.second = limits.max_deceleration;
  }
  else if (limits.has_acceleration_limits)
  {
    acc_or_dec_limits.first = -limits.max_acceleration;
    acc_or_dec_limits.second = limits.max_acceleration;
  }
  return acc_or_dec_limits;
}

}  // namespace joint_limits
