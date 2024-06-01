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
#include <sstream>
#include <string>

namespace joint_limits
{
/**
 * JointLimits structure stores values from from yaml definition or `<limits>` tag in URDF.
 * The mapping from URDF attributes to members is the following:
 *   lower --> min_position
 *   upper --> max_position
 *   velocity --> max_velocity
 *   effort --> max_effort
 */
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

  std::string to_string()
  {
    std::stringstream ss_output;

    ss_output << "  has position limits: " << (has_position_limits ? "true" : "false") << " ["
              << min_position << ", " << max_position << "]\n";
    ss_output << "  has velocity limits: " << (has_velocity_limits ? "true" : "false") << " ["
              << max_velocity << "]\n";
    ss_output << "  has acceleration limits: " << (has_acceleration_limits ? "true" : "false")
              << " [" << max_acceleration << "]\n";
    ss_output << "  has jerk limits: " << (has_jerk_limits ? "true" : "false") << " [" << max_jerk
              << "]\n";
    ss_output << "  has effort limits: " << (has_effort_limits ? "true" : "false") << " ["
              << max_effort << "]\n";
    ss_output << "  angle wraparound: " << (angle_wraparound ? "true" : "false");

    return ss_output.str();
  }
};

/**
 * SoftJointLimits stores values from the `<safety_controller>` tag of URDF.
 * The meaning of the fields are:
 *
 * An element can contain the following attributes:
 *
 * **soft_lower_limit** (optional, defaults to 0) - An attribute specifying the lower joint boundary
 * where the safety controller starts limiting the position of the joint. This limit needs to be
 * larger than the lower joint limit (see above). See See safety limits for more details.
 *
 * **soft_upper_limit** (optional, defaults to 0) - An attribute specifying the upper joint boundary
 * where the safety controller starts limiting the position of the joint. This limit needs to be
 * smaller than the upper joint limit (see above). See See safety limits for more details.
 *
 * **k_position** (optional, defaults to 0) - An attribute specifying the relation between position
 * and velocity limits. See See safety limits for more details.
 *
 * k_velocity (required) - An attribute specifying the relation between effort and velocity limits.
 * See See safety limits for more details.
 */
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

  std::string to_string()
  {
    std::stringstream ss_output;

    ss_output << "  soft position limits: "
              << "[" << min_position << ", " << max_position << "]\n";

    ss_output << "  k-position: "
              << "[" << k_position << "]\n";

    ss_output << "  k-velocity: "
              << "[" << k_velocity << "]\n";

    return ss_output.str();
  }
};

}  // namespace joint_limits

#endif  // JOINT_LIMITS__JOINT_LIMITS_HPP_
