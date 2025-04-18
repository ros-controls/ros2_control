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

#ifndef JOINT_LIMITS__JOINT_SOFT_LIMITER_HPP_
#define JOINT_LIMITS__JOINT_SOFT_LIMITER_HPP_

#include <cmath>
#include "joint_limits/data_structures.hpp"
#include "joint_limits/joint_limits_helpers.hpp"
#include "joint_limits/joint_saturation_limiter.hpp"

constexpr double VALUE_CONSIDERED_ZERO = 1e-10;

namespace joint_limits
{

class JointSoftLimiter : public JointSaturationLimiter<JointControlInterfacesData>
{
public:
  bool on_init() override
  {
    const bool result = (number_of_joints_ == 1);
    if (!result && has_logging_interface())
    {
      RCLCPP_ERROR(
        node_logging_itf_->get_logger(),
        "JointInterfacesSaturationLimiter: Expects the number of joints to be 1, but given : "
        "%zu",
        number_of_joints_);
    }
    prev_command_ = JointControlInterfacesData();
    return result;
  }

  bool on_enforce(
    const JointControlInterfacesData & actual, JointControlInterfacesData & desired,
    const rclcpp::Duration & dt) override;

  bool has_soft_position_limits(const joint_limits::SoftJointLimits & soft_joint_limits)
  {
    return std::isfinite(soft_joint_limits.min_position) &&
           std::isfinite(soft_joint_limits.max_position) &&
           (soft_joint_limits.max_position - soft_joint_limits.min_position) >
             VALUE_CONSIDERED_ZERO;
  }

  bool has_soft_limits(const joint_limits::SoftJointLimits & soft_joint_limits)
  {
    return has_soft_position_limits(soft_joint_limits) &&
           std::isfinite(soft_joint_limits.k_position) &&
           std::abs(soft_joint_limits.k_position) > VALUE_CONSIDERED_ZERO;
  }
};

}  // namespace joint_limits

#endif  // JOINT_LIMITS__JOINT_SOFT_LIMITER_HPP_
