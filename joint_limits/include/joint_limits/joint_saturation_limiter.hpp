// Copyright (c) 2024, PickNik Inc.
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

/// \author Dr. Denis Stogl

#ifndef JOINT_LIMITS__JOINT_SATURATION_LIMITER_HPP_
#define JOINT_LIMITS__JOINT_SATURATION_LIMITER_HPP_

#include <memory>
#include <vector>

#include "joint_limits/joint_limiter_interface.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/duration.hpp"

namespace joint_limits
{
/**
 * Joint Saturation Limiter limits joints' position, velocity and acceleration by clamping values
 * to its minimal and maximal allowed values. Since the position, velocity and accelerations are
 * variables in physical relation, it might be that some values are limited lower then specified
 * limit. For example, if a joint is close to its position limit, velocity and acceleration will be
 * reduced accordingly.
 */
template <typename LimitsType>
class JointSaturationLimiter : public JointLimiterInterface<LimitsType>
{
public:
  /** \brief Constructor */
  JOINT_LIMITS_PUBLIC JointSaturationLimiter();

  /** \brief Destructor */
  JOINT_LIMITS_PUBLIC ~JointSaturationLimiter();

  JOINT_LIMITS_PUBLIC bool on_init() override { return true; }

  JOINT_LIMITS_PUBLIC bool on_configure(
    const trajectory_msgs::msg::JointTrajectoryPoint & /*current_joint_states*/) override
  {
    return true;
  }

  /** \brief Enforce joint limits to desired position, velocity and acceleration using clamping.
   * Class implements this method accepting following data types:
   * - trajectory_msgs::msg::JointTrajectoryPoint: limiting position, velocity and acceleration;
   *
   * Implementation of saturation approach for joints with position, velocity or acceleration limits
   * and values. First, position limits are checked to adjust desired velocity accordingly, then
   * velocity and finally acceleration.
   * The method support partial existence of limits, e.g., missing position limits for continuous
   * joins.
   *
   * \param[in] current_joint_states current joint states a robot is in.
   * \param[in,out] desired_joint_states joint state that should be adjusted to obey the limits.
   * \param[in] dt time delta to calculate missing integrals and derivation in joint limits.
   * \returns true if limits are enforced, otherwise false.
   */
  JOINT_LIMITS_PUBLIC bool on_enforce(
    trajectory_msgs::msg::JointTrajectoryPoint & current_joint_states,
    trajectory_msgs::msg::JointTrajectoryPoint & desired_joint_states,
    const rclcpp::Duration & dt) override;

  /** \brief Enforce joint limits to desired arbitrary physical quantity.
   * Additional, commonly used method for enforcing limits by clamping desired input value.
   * The limit is defined in effort limits of the `joint::limits/JointLimit` structure.
   *
   * If `has_effort_limits` is set to false, the limits will be not enforced to a joint.
   *
   * \param[in,out] desired_joint_states physical quantity that should be adjusted to obey the
   * limits. \returns true if limits are enforced, otherwise false.
   */
  JOINT_LIMITS_PUBLIC bool on_enforce(std::vector<double> & desired_joint_states) override;

private:
  rclcpp::Clock::SharedPtr clock_;
};

template <typename LimitsType>
JointSaturationLimiter<LimitsType>::JointSaturationLimiter() : JointLimiterInterface<LimitsType>()
{
  clock_ = std::make_shared<rclcpp::Clock>(rclcpp::Clock(RCL_ROS_TIME));
}

template <typename LimitsType>
JointSaturationLimiter<LimitsType>::~JointSaturationLimiter()
{
}

}  // namespace joint_limits

#endif  // JOINT_LIMITS__JOINT_SATURATION_LIMITER_HPP_
