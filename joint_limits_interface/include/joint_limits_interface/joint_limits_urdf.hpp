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

#ifndef JOINT_LIMITS_INTERFACE__JOINT_LIMITS_URDF_HPP_
#define JOINT_LIMITS_INTERFACE__JOINT_LIMITS_URDF_HPP_

#include <urdf/urdfdom_compatibility.h>
#include <urdf_model/joint.h>
#include <joint_limits_interface/joint_limits.hpp>
#include <rclcpp/rclcpp.hpp>

namespace joint_limits_interface
{
/**
 * Populate a JointLimits instance from URDF joint data.
 * \param[in] urdf_joint URDF joint.
 * \param[out] limits Where URDF joint limit data gets written into. Limits in \e urdf_joint will
 * overwrite existing values. Values in \e limits not present in \e urdf_joint remain unchanged.
 * \return True if \e urdf_joint has a valid limits specification, false otherwise.
 */
inline bool getJointLimits(urdf::JointConstSharedPtr urdf_joint, JointLimits & limits)
{
  if (!urdf_joint || !urdf_joint->limits)
  {
    return false;
  }

  limits.has_position_limits =
    urdf_joint->type == urdf::Joint::REVOLUTE || urdf_joint->type == urdf::Joint::PRISMATIC;
  if (limits.has_position_limits)
  {
    limits.min_position = urdf_joint->limits->lower;
    limits.max_position = urdf_joint->limits->upper;
  }

  if (!limits.has_position_limits && urdf_joint->type == urdf::Joint::CONTINUOUS)
  {
    limits.angle_wraparound = true;
  }

  limits.has_velocity_limits = true;
  limits.max_velocity = urdf_joint->limits->velocity;

  limits.has_acceleration_limits = false;

  limits.has_effort_limits = true;
  limits.max_effort = urdf_joint->limits->effort;

  return true;
}

/**
 * Populate a SoftJointLimits instance from URDF joint data.
 * \param[in] urdf_joint URDF joint.
 * \param[out] soft_limits Where URDF soft joint limit data gets written into.
 * \return True if \e urdf_joint has a valid soft limits specification, false otherwise.
 */
inline bool getSoftJointLimits(urdf::JointConstSharedPtr urdf_joint, SoftJointLimits & soft_limits)
{
  if (!urdf_joint || !urdf_joint->safety)
  {
    return false;
  }

  soft_limits.min_position = urdf_joint->safety->soft_lower_limit;
  soft_limits.max_position = urdf_joint->safety->soft_upper_limit;
  soft_limits.k_position = urdf_joint->safety->k_position;
  soft_limits.k_velocity = urdf_joint->safety->k_velocity;

  return true;
}

}  // namespace joint_limits_interface

#endif  // JOINT_LIMITS_INTERFACE__JOINT_LIMITS_URDF_HPP_
