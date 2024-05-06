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
#include "joint_limits/joint_limits.hpp"

namespace joint_limits
{

bool is_limited(double value, double min, double max);

std::pair<double, double> compute_position_limits(
  const joint_limits::JointLimits & limits, const std::optional<double> & act_vel,
  const std::optional<double> & prev_command_pos, double dt);

std::pair<double, double> compute_velocity_limits(
  const std::string & joint_name, const joint_limits::JointLimits & limits,
  const std::optional<double> & act_pos, const std::optional<double> & prev_command_vel, double dt);

std::pair<double, double> compute_effort_limits(
  const joint_limits::JointLimits & limits, const std::optional<double> & act_pos,
  const std::optional<double> & act_vel, double /*dt*/);

std::pair<double, double> compute_acceleration_limits(
  const JointLimits & limits, double desired_acceleration, std::optional<double> actual_velocity);

}  // namespace joint_limits

#endif  // JOINT_LIMITS__JOINT_LIMITS_HELPERS_HPP_
