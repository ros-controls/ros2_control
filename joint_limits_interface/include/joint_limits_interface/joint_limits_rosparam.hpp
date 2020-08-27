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


#include <string>

#include <rclcpp/rclcpp.hpp>
#include <joint_limits_interface/joint_limits.hpp>

namespace joint_limits_interface
{

/**
 * \brief Populate a JointLimits instance from the ROS parameter server.
 *
 * It is assumed that the following parameter structure is followed on the provided NodeHandle. Unspecified parameters
 * are simply not added to the joint limits specification.
 * \code
 * joint_limits:
 *   foo_joint:
 *     has_position_limits: true
 *     min_position: 0.0
 *     max_position: 1.0
 *     has_velocity_limits: true
 *     max_velocity: 2.0
 *     has_acceleration_limits: true
 *     max_acceleration: 5.0
 *     has_jerk_limits: true
 *     max_jerk: 100.0
 *     has_effort_limits: true
 *     max_effort: 20.0
 *   bar_joint:
 *     has_position_limits: false # Continuous joint
 *     has_velocity_limits: true
 *     max_velocity: 4.0
 * \endcode
 *
 * This specification is similar to the one used by <a href="http://moveit.ros.org/wiki/MoveIt!">MoveIt!</a>,
 * but additionally supports jerk and effort limits.
 *
 * \param[in] joint_name Name of joint whose limits are to be fetched.
 * \param[in] nh NodeHandle where the joint limits are specified.
 * \param[out] limits Where joint limit data gets written into. Limits specified in the parameter server will overwrite
 * existing values. Values in \p limits not specified in the parameter server remain unchanged.
 * \return True if a limits specification is found (ie. the \p joint_limits/joint_name parameter exists in \p nh), false otherwise.
 */
inline bool getJointLimits(const std::string& joint_name, const rclcpp::Node::SharedPtr& nh, JointLimits& limits)
{
  // Node handle scoped where the joint limits are defined
  rclcpp::Node::SharedPtr limits_nh;
  try
  {
    const std::string limits_namespace = "joint_limits/" + joint_name;
    if (!nh->has_parameter(limits_namespace))
    {
      RCLCPP_DEBUG_STREAM(nh->get_logger(), "No joint limits specification found for joint '" << joint_name <<
                       "' in the parameter server (namespace " << std::string(nh->get_namespace()) + "/" + limits_namespace << ").");
      return false;
    }
    limits_nh = nh->create_sub_node(limits_namespace);
  }
  catch(const std::exception& ex)
  {
    RCLCPP_ERROR_STREAM(nh->get_logger(), ex.what());
    return false;
  }

  // Position limits
  bool has_position_limits = false;
  if(limits_nh->get_parameter("has_position_limits", has_position_limits))
  {
    if (!has_position_limits) {limits.has_position_limits = false;}
    double min_pos, max_pos;
    if (has_position_limits && limits_nh->get_parameter("min_position", min_pos) && limits_nh->get_parameter("max_position", max_pos))
    {
      limits.has_position_limits = true;
      limits.min_position = min_pos;
      limits.max_position = max_pos;
    }

    bool angle_wraparound;
    if (!has_position_limits && limits_nh->get_parameter("angle_wraparound", angle_wraparound))
    {
      limits.angle_wraparound = angle_wraparound;
    }
  }

  // Velocity limits
  bool has_velocity_limits = false;
  if(limits_nh->get_parameter("has_velocity_limits", has_velocity_limits))
  {
    if (!has_velocity_limits) {limits.has_velocity_limits = false;}
    double max_vel;
    if (has_velocity_limits && limits_nh->get_parameter("max_velocity", max_vel))
    {
      limits.has_velocity_limits = true;
      limits.max_velocity = max_vel;
    }
  }

  // Acceleration limits
  bool has_acceleration_limits = false;
  if(limits_nh->get_parameter("has_acceleration_limits", has_acceleration_limits))
  {
    if (!has_acceleration_limits) {limits.has_acceleration_limits = false;}
    double max_acc;
    if (has_acceleration_limits && limits_nh->get_parameter("max_acceleration", max_acc))
    {
      limits.has_acceleration_limits = true;
      limits.max_acceleration = max_acc;
    }
  }

  // Jerk limits
  bool has_jerk_limits = false;
  if(limits_nh->get_parameter("has_jerk_limits", has_jerk_limits))
  {
    if (!has_jerk_limits) {limits.has_jerk_limits = false;}
    double max_jerk;
    if (has_jerk_limits && limits_nh->get_parameter("max_jerk", max_jerk))
    {
      limits.has_jerk_limits = true;
      limits.max_jerk = max_jerk;
    }
  }

  // Effort limits
  bool has_effort_limits = false;
  if(limits_nh->get_parameter("has_effort_limits", has_effort_limits))
  {
    if (!has_effort_limits) {limits.has_effort_limits = false;}
    double max_effort;
    if (has_effort_limits && limits_nh->get_parameter("max_effort", max_effort))
    {
      limits.has_effort_limits = true;
      limits.max_effort = max_effort;
    }
  }

  return true;
}

/**
 * \brief Populate a SoftJointLimits instance from the ROS parameter server.
 *
 * It is assumed that the following parameter structure is followed on the provided NodeHandle. Only completely specified soft
 * joint limits specifications will be considered valid.
 * \code
 * joint_limits:
 *   foo_joint:
 *     soft_lower_limit: 0.0
 *     soft_upper_limit: 1.0
 *     k_position: 10.0
 *     k_velocity: 10.0
 * \endcode
 *
 * This specification is similar to the specification of the safety_controller tag in the URDF, adapted to the parameter server.
 *
 * \param[in] joint_name Name of joint whose limits are to be fetched.
 * \param[in] nh NodeHandle where the joint limits are specified.
 * \param[out] soft_limits Where soft joint limit data gets written into. Limits specified in the parameter server will overwrite
 * existing values.
 * \return True if a complete soft limits specification is found (ie. if all \p k_position, \p k_velocity, \p soft_lower_limit and
 * \p soft_upper_limit exist in \p joint_limits/joint_name namespace), false otherwise.
 */
inline bool getSoftJointLimits(const std::string& joint_name, const rclcpp::Node::SharedPtr& nh, SoftJointLimits& soft_limits)
{
  // Node handle scoped where the soft joint limits are defined
  rclcpp::Node::SharedPtr limits_nh;
  try
  {
    const std::string limits_namespace = "joint_limits/" + joint_name;
    if (!nh->has_parameter(limits_namespace))
    {
      RCLCPP_DEBUG_STREAM(nh->get_logger(), "No soft joint limits specification found for joint '" << joint_name <<
                       "' in the parameter server (namespace " << std::string(nh->get_namespace()) + "/" + limits_namespace << ").");
      return false;
    }
    limits_nh = nh->create_sub_node(limits_namespace);
  }
  catch(const std::exception& ex)
  {
    RCLCPP_ERROR_STREAM(nh->get_logger(), ex.what());
    return false;
  }

  // Override soft limits if complete specification is found
  bool has_soft_limits;
  if(limits_nh->get_parameter("has_soft_limits", has_soft_limits))
  {
    if(has_soft_limits && limits_nh->has_parameter("k_position") && limits_nh->has_parameter("k_velocity") && 
       limits_nh->has_parameter("soft_lower_limit") && limits_nh->has_parameter("soft_upper_limit"))
    {
      limits_nh->get_parameter("k_position", soft_limits.k_position);
      limits_nh->get_parameter("k_velocity", soft_limits.k_velocity);
      limits_nh->get_parameter("soft_lower_limit", soft_limits.min_position);
      limits_nh->get_parameter("soft_upper_limit", soft_limits.max_position);
      return true;
    }
  }

  return false;
}

}
