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
};

}  // namespace joint_limits
#endif  // JOINT_LIMITS__DATA_STRUCTURES_HPP_
