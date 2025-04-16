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

#ifndef JOINT_LIMITS__DATA_STRUCTURES_HPP_
#define JOINT_LIMITS__DATA_STRUCTURES_HPP_

#include <limits>
#include <memory>
#include <optional>
#include <string>

#define DEFINE_LIMIT_STRUCT(LimitType)                             \
  struct LimitType                                                 \
  {                                                                \
    LimitType(double minimum_value, double maximum_value)          \
    : lower_limit(minimum_value), upper_limit(maximum_value)       \
    {                                                              \
    }                                                              \
    double lower_limit = -std::numeric_limits<double>::infinity(); \
    double upper_limit = std::numeric_limits<double>::infinity();  \
  };

namespace joint_limits
{

DEFINE_LIMIT_STRUCT(PositionLimits);
DEFINE_LIMIT_STRUCT(VelocityLimits);
DEFINE_LIMIT_STRUCT(EffortLimits);
DEFINE_LIMIT_STRUCT(AccelerationLimits);

struct JointControlInterfacesData
{
  std::string joint_name;
  std::optional<double> position = std::nullopt;
  std::optional<double> velocity = std::nullopt;
  std::optional<double> effort = std::nullopt;
  std::optional<double> acceleration = std::nullopt;
  std::optional<double> jerk = std::nullopt;

  bool has_data() const
  {
    return has_position() || has_velocity() || has_effort() || has_acceleration() || has_jerk();
  }

  bool has_position() const { return position.has_value(); }

  bool has_velocity() const { return velocity.has_value(); }

  bool has_effort() const { return effort.has_value(); }

  bool has_acceleration() const { return acceleration.has_value(); }

  bool has_jerk() const { return jerk.has_value(); }

  std::string to_string() const
  {
    std::string str;
    if (has_position())
    {
      str += "position: " + std::to_string(position.value()) + ", ";
    }
    if (has_velocity())
    {
      str += "velocity: " + std::to_string(velocity.value()) + ", ";
    }
    if (has_effort())
    {
      str += "effort: " + std::to_string(effort.value()) + ", ";
    }
    if (has_acceleration())
    {
      str += "acceleration: " + std::to_string(acceleration.value()) + ", ";
    }
    if (has_jerk())
    {
      str += "jerk: " + std::to_string(jerk.value());
    }
    // trim the last comma and space
    if (!str.empty() && str.back() == ' ')
    {
      str.pop_back();
    }
    if (!str.empty() && str.back() == ',')
    {
      str.pop_back();
    }
    return str;
  }
};

struct JointInterfacesCommandLimiterData
{
  std::string joint_name;
  JointControlInterfacesData actual;
  JointControlInterfacesData command;
  JointControlInterfacesData prev_command;
  JointControlInterfacesData limited;

  bool has_actual_data() const { return actual.has_data(); }

  bool has_command_data() const { return command.has_data(); }

  bool has_limited_data() const { return limited.has_data(); }

  std::string to_string() const
  {
    return "Joint : '" + joint_name + "', (actual: [" + actual.to_string() + "], command : [" +
           command.to_string() + "], limited: [" + limited.to_string() + "])";
  }
};

}  // namespace joint_limits
#endif  // JOINT_LIMITS__DATA_STRUCTURES_HPP_
