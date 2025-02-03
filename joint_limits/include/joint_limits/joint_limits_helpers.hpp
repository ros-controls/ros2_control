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

#ifndef JOINT_LIMITS__JOINT_LIMITS_HELPERS_HPP_
#define JOINT_LIMITS__JOINT_LIMITS_HELPERS_HPP_

#include <optional>
#include <string>
#include <utility>
#include "joint_limits/data_structures.hpp"
#include "joint_limits/joint_limits.hpp"

namespace joint_limits
{
namespace internal
{
constexpr double POSITION_BOUNDS_TOLERANCE = 0.002;
}

/**
 * @brief Checks if a value is limited by the given limits.
 * @param value The value to check.
 * @param min The minimum limit.
 * @param max The maximum limit.
 * @return True if the value is limited, false otherwise.
 */
bool is_limited(double value, double min, double max);

/**
 * @brief Computes the position limits based on the velocity and acceleration limits.
 * @param limits The joint limits.
 * @param act_vel The actual velocity of the joint.
 * @param prev_command_pos The previous commanded position of the joint.
 * @param dt The time step.
 * @return The position limits, first is the lower limit and second is the upper limit.
 */
PositionLimits compute_position_limits(
  const joint_limits::JointLimits & limits, const std::optional<double> & act_vel,
  const std::optional<double> & act_pos, const std::optional<double> & prev_command_pos, double dt);

/**
 * @brief Computes the velocity limits based on the position and acceleration limits.
 * @param joint_name The name of the joint.
 * @param limits The joint limits.
 * @param act_pos The actual position of the joint.
 * @param prev_command_vel The previous commanded velocity of the joint.
 * @param dt The time step.
 * @return The velocity limits, first is the lower limit and second is the upper limit.
 */
VelocityLimits compute_velocity_limits(
  const std::string & joint_name, const joint_limits::JointLimits & limits,
  const double & desired_vel, const std::optional<double> & act_pos,
  const std::optional<double> & prev_command_vel, double dt);

/**
 * @brief Computes the effort limits based on the position and velocity limits.
 * @param limits The joint limits.
 * @param act_pos The actual position of the joint.
 * @param act_vel The actual velocity of the joint.
 * @param dt The time step.
 * @return The effort limits, first is the lower limit and second is the upper limit.
 */
EffortLimits compute_effort_limits(
  const joint_limits::JointLimits & limits, const std::optional<double> & act_pos,
  const std::optional<double> & act_vel, double /*dt*/);

/**
 * @brief Computes the acceleration limits based on the change in velocity and acceleration and
 * deceleration limits.
 * @param limits The joint limits.
 * @param desired_acceleration The desired acceleration.
 * @param actual_velocity The actual velocity of the joint.
 * @return The acceleration limits, first is the lower limit and second is the upper limit.
 */
AccelerationLimits compute_acceleration_limits(
  const JointLimits & limits, double desired_acceleration, std::optional<double> actual_velocity);

}  // namespace joint_limits

#endif  // JOINT_LIMITS__JOINT_LIMITS_HELPERS_HPP_
