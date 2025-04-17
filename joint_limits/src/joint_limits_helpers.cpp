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
void check_and_swap_limits(double & lower_limit, double & upper_limit)
{
  if (lower_limit > upper_limit)
  {
    std::swap(lower_limit, upper_limit);
  }
}

/**
 * @brief Verify if the actual position is within the limits and if not, log an error and throw an
 * exception.
 * @param joint_name The name of the joint.
 * @param actual_position The actual position of the joint.
 * @param limits The joint limits.
 * @throws std::runtime_error if the actual position is out of bounds.
 */
void verify_actual_position_within_limits(
  const std::string & joint_name, const std::optional<double> & actual_position,
  const joint_limits::JointLimits & limits)
{
  if (actual_position.has_value() && limits.has_position_limits)
  {
    const double actual_pos = actual_position.value();
    if (
      actual_pos > (limits.max_position + internal::OUT_OF_BOUNDS_EXCEPTION_TOLERANCE) ||
      actual_pos < (limits.min_position - internal::OUT_OF_BOUNDS_EXCEPTION_TOLERANCE))
    {
      const std::string error_message =
        "Joint position is out of bounds for the joint : '" + joint_name +
        "' actual position: " + std::to_string(actual_pos) + " limits: [" +
        std::to_string(limits.min_position) + ", " + std::to_string(limits.max_position) +
        "]. This could be due to a hardware failure (or) the physical limits of the joint being "
        "larger than the ones defined in the URDF. Please recheck the URDF and the hardware to "
        "verify the joint limits.";
      RCLCPP_ERROR_ONCE(rclcpp::get_logger("joint_limiter_interface"), "%s", error_message.c_str());
      // Throw an exception to indicate that the joint position is out of bounds
      throw std::runtime_error(error_message);
    }
  }
}
}  // namespace internal

bool is_limited(double value, double min, double max) { return value < min || value > max; }

PositionLimits compute_position_limits(
  const std::string & joint_name, const joint_limits::JointLimits & limits,
  const std::optional<double> & act_vel, const std::optional<double> & act_pos,
  const std::optional<double> & prev_command_pos, double dt)
{
  PositionLimits pos_limits(limits.min_position, limits.max_position);
  internal::verify_actual_position_within_limits(joint_name, act_pos, limits);
  if (limits.has_velocity_limits)
  {
    const double act_vel_abs = act_vel.has_value() ? std::fabs(act_vel.value()) : 0.0;
    const double delta_vel = limits.has_acceleration_limits
                               ? act_vel_abs + (limits.max_acceleration * dt)
                               : limits.max_velocity;
    const double max_vel = std::min(limits.max_velocity, delta_vel);
    const double delta_pos = max_vel * dt;
    /// @note: We use the previous command position to compute the limits here because using the
    /// actual position would be too conservative, usually there is a couple of cycles of delay
    /// between the command sent to the robot and the robot actually showing that in the state. That
    /// effectively limits the velocity with which the joint can be moved which is much lower than
    /// the actual velocity limit.
    const double position_reference = prev_command_pos.value();
    pos_limits.lower_limit = std::max(
      std::min(position_reference - delta_pos, pos_limits.upper_limit), pos_limits.lower_limit);
    pos_limits.upper_limit = std::min(
      std::max(position_reference + delta_pos, pos_limits.lower_limit), pos_limits.upper_limit);
  }
  internal::check_and_swap_limits(pos_limits.lower_limit, pos_limits.upper_limit);
  return pos_limits;
}

VelocityLimits compute_velocity_limits(
  const std::string & joint_name, const joint_limits::JointLimits & limits,
  const double & desired_vel, const std::optional<double> & act_pos,
  const std::optional<double> & prev_command_vel, double dt)
{
  const double max_vel =
    limits.has_velocity_limits ? limits.max_velocity : std::numeric_limits<double>::infinity();
  VelocityLimits vel_limits(-max_vel, max_vel);
  internal::verify_actual_position_within_limits(joint_name, act_pos, limits);
  if (limits.has_position_limits && act_pos.has_value())
  {
    const double actual_pos = act_pos.value();
    const double max_vel_with_pos_limits = (limits.max_position - actual_pos) / dt;
    const double min_vel_with_pos_limits = (limits.min_position - actual_pos) / dt;
    vel_limits.lower_limit = std::max(min_vel_with_pos_limits, vel_limits.lower_limit);
    vel_limits.upper_limit = std::min(max_vel_with_pos_limits, vel_limits.upper_limit);

    if (actual_pos > limits.max_position || actual_pos < limits.min_position)
    {
      if (
        (actual_pos < (limits.max_position + internal::POSITION_BOUNDS_TOLERANCE) &&
         (actual_pos > limits.min_position) && desired_vel >= 0.0) ||
        (actual_pos > (limits.min_position - internal::POSITION_BOUNDS_TOLERANCE) &&
         (actual_pos < limits.max_position) && desired_vel <= 0.0))
      {
        RCLCPP_WARN_EXPRESSION(
          rclcpp::get_logger("joint_limiter_interface"),
          prev_command_vel.has_value() && prev_command_vel.value() != 0.0,
          "Joint position %.5f is out of bounds[%.5f, %.5f] for the joint and we want to move "
          "further into bounds with vel %.5f: '%s'. Joint velocity limits will be "
          "restrictred to zero.",
          actual_pos, limits.min_position, limits.max_position, desired_vel, joint_name.c_str());
        vel_limits = VelocityLimits(0.0, 0.0);
      }
      // If the joint reports a position way out of bounds, then it would mean something is
      // extremely wrong, so no velocity command should be allowed as it might damage the robot
      else if (
        (actual_pos > (limits.max_position + internal::POSITION_BOUNDS_TOLERANCE)) ||
        (actual_pos < (limits.min_position - internal::POSITION_BOUNDS_TOLERANCE)))
      {
        RCLCPP_ERROR_ONCE(
          rclcpp::get_logger("joint_limiter_interface"),
          "Joint position is out of bounds for the joint : '%s'. Joint velocity limits will be "
          "restricted to zero.",
          joint_name.c_str());
        vel_limits = VelocityLimits(0.0, 0.0);
      }
    }
  }
  if (limits.has_acceleration_limits && prev_command_vel.has_value())
  {
    const double delta_vel = limits.max_acceleration * dt;
    vel_limits.lower_limit = std::max(prev_command_vel.value() - delta_vel, vel_limits.lower_limit);
    vel_limits.upper_limit = std::min(prev_command_vel.value() + delta_vel, vel_limits.upper_limit);
  }
  internal::check_and_swap_limits(vel_limits.lower_limit, vel_limits.upper_limit);
  return vel_limits;
}

EffortLimits compute_effort_limits(
  const joint_limits::JointLimits & limits, const std::optional<double> & act_pos,
  const std::optional<double> & act_vel, double /*dt*/)
{
  const double max_effort =
    limits.has_effort_limits ? limits.max_effort : std::numeric_limits<double>::infinity();
  EffortLimits eff_limits(-max_effort, max_effort);
  if (limits.has_position_limits && act_pos.has_value() && act_vel.has_value())
  {
    if ((act_pos.value() <= limits.min_position) && (act_vel.value() <= 0.0))
    {
      eff_limits.lower_limit = 0.0;
    }
    else if ((act_pos.value() >= limits.max_position) && (act_vel.value() >= 0.0))
    {
      eff_limits.upper_limit = 0.0;
    }
  }
  if (limits.has_velocity_limits && act_vel.has_value())
  {
    if (act_vel.value() < -limits.max_velocity)
    {
      eff_limits.lower_limit = 0.0;
    }
    else if (act_vel.value() > limits.max_velocity)
    {
      eff_limits.upper_limit = 0.0;
    }
  }
  internal::check_and_swap_limits(eff_limits.lower_limit, eff_limits.upper_limit);
  return eff_limits;
}

AccelerationLimits compute_acceleration_limits(
  const joint_limits::JointLimits & limits, double desired_acceleration,
  std::optional<double> actual_velocity)
{
  AccelerationLimits acc_or_dec_limits(
    -std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity());
  if (
    limits.has_deceleration_limits &&
    ((desired_acceleration < 0 && actual_velocity && actual_velocity.value() > 0) ||
     (desired_acceleration > 0 && actual_velocity && actual_velocity.value() < 0)))
  {
    acc_or_dec_limits.lower_limit = -limits.max_deceleration;
    acc_or_dec_limits.upper_limit = limits.max_deceleration;
  }
  else if (limits.has_acceleration_limits)
  {
    acc_or_dec_limits.lower_limit = -limits.max_acceleration;
    acc_or_dec_limits.upper_limit = limits.max_acceleration;
  }
  internal::check_and_swap_limits(acc_or_dec_limits.lower_limit, acc_or_dec_limits.upper_limit);
  return acc_or_dec_limits;
}

}  // namespace joint_limits
