// Copyright 2020 PAL Robotics S.L.
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

/// \author Adolfo Rodriguez Tsouroukdissian

#ifndef JOINT_LIMITS__JOINT_LIMITS_HPP_
#define JOINT_LIMITS__JOINT_LIMITS_HPP_

#include <limits>

namespace joint_limits
{
struct JointLimits
{
  JointLimits()
  : min_position(std::numeric_limits<double>::quiet_NaN()),
    max_position(std::numeric_limits<double>::quiet_NaN()),
    max_velocity(std::numeric_limits<double>::quiet_NaN()),
    max_acceleration(std::numeric_limits<double>::quiet_NaN()),
    max_jerk(std::numeric_limits<double>::quiet_NaN()),
    max_effort(std::numeric_limits<double>::quiet_NaN()),
    has_position_limits(false),
    has_velocity_limits(false),
    has_acceleration_limits(false),
    has_jerk_limits(false),
    has_effort_limits(false),
    angle_wraparound(false)
  {
  }

  double min_position;
  double max_position;
  double max_velocity;
  double max_acceleration;
  double max_jerk;
  double max_effort;

  bool has_position_limits;
  bool has_velocity_limits;
  bool has_acceleration_limits;
  bool has_jerk_limits;
  bool has_effort_limits;
  bool angle_wraparound;
};

struct SoftJointLimits
{
  SoftJointLimits()
  : min_position(std::numeric_limits<double>::quiet_NaN()),
    max_position(std::numeric_limits<double>::quiet_NaN()),
    k_position(std::numeric_limits<double>::quiet_NaN()),
    k_velocity(std::numeric_limits<double>::quiet_NaN())
  {
  }

  double min_position;
  double max_position;
  double k_position;
  double k_velocity;
};

}  // namespace joint_limits

#endif  // JOINT_LIMITS__JOINT_LIMITS_HPP_
