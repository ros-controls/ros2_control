// Copyright (c) 2023, PickNik Inc.
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
#include <string>
#include <utility>
#include <vector>

#include "joint_limits/joint_limiter_interface.hpp"
#include "joint_limits/joint_limits.hpp"
#include "rclcpp/rclcpp.hpp"

namespace joint_limits
{
template <typename LimitsType>
class JointSaturationLimiter : public JointLimiterInterface<LimitsType>
{
public:
  /** \brief Constructor */
  JOINT_LIMITS_PUBLIC JointSaturationLimiter();

  /** \brief Destructor */
  JOINT_LIMITS_PUBLIC ~JointSaturationLimiter();

  JOINT_LIMITS_PUBLIC bool on_enforce(
    trajectory_msgs::msg::JointTrajectoryPoint & current_joint_states,
    trajectory_msgs::msg::JointTrajectoryPoint & desired_joint_states,
    const rclcpp::Duration & dt) override;

  /**
   * generic function for enforcing of effort.
   *
   * \return std::pair<bool, std::vector<double>>, where bool shows if the limits have been enforced
   * and the std::vector<double> contains the values
   *
   */
  JOINT_LIMITS_PUBLIC std::pair<bool, std::vector<double>> on_enforce(
    std::vector<double> desired, const rclcpp::Duration & dt);

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
