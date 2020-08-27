///////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2013, PAL Robotics S.L.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//   * Redistributions of source code must retain the above copyright notice,
//     this list of conditions and the following disclaimer.
//   * Redistributions in binary form must reproduce the above copyright
//     notice, this list of conditions and the following disclaimer in the
//     documentation and/or other materials provided with the distribution.
//   * Neither the name of PAL Robotics S.L. nor the names of its
//     contributors may be used to endorse or promote products derived from
//     this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//////////////////////////////////////////////////////////////////////////////

/// \author Adolfo Rodriguez Tsouroukdissian

#pragma once


#include <rclcpp/rclcpp.hpp>
#include <urdf_model/joint.h>
#include <urdf/urdfdom_compatibility.h>
#include <joint_limits_interface/joint_limits.hpp>

namespace joint_limits_interface
{

/**
 * \brief Populate a JointLimits instance from URDF joint data.
 * \param[in] urdf_joint URDF joint.
 * \param[out] limits Where URDF joint limit data gets written into. Limits in \e urdf_joint will overwrite existing
 * values. Values in \e limits not present in \e urdf_joint remain unchanged.
 * \return True if \e urdf_joint has a valid limits specification, false otherwise.
 */
inline bool getJointLimits(urdf::JointConstSharedPtr urdf_joint, JointLimits& limits)
{
  if (!urdf_joint || !urdf_joint->limits)
  {
    return false;
  }

  limits.has_position_limits = urdf_joint->type == urdf::Joint::REVOLUTE || urdf_joint->type == urdf::Joint::PRISMATIC;
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
 * \brief Populate a SoftJointLimits instance from URDF joint data.
 * \param[in] urdf_joint URDF joint.
 * \param[out] soft_limits Where URDF soft joint limit data gets written into.
 * \return True if \e urdf_joint has a valid soft limits specification, false otherwise.
 */
inline bool getSoftJointLimits(urdf::JointConstSharedPtr urdf_joint, SoftJointLimits& soft_limits)
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

}
