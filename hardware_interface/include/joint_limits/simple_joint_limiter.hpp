// Copyright (c) 2021, PickNik, Inc.
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

/// \author Denis Stogl

#ifndef JOINT_LIMITS__SIMPLE_JOINT_LIMITER_HPP_
#define JOINT_LIMITS__SIMPLE_JOINT_LIMITER_HPP_

#include <vector>

#include "hardware_interface/handle.hpp"
#include "joint_limits/joint_limiter_interface.hpp"
#include "joint_limits/joint_limits.hpp"
#include "rclcpp/duration.hpp"

namespace joint_limits
{
/// Implementation of simple joint limiter enforcing position, velocity, acceleration and effort limits without considering soft limits.
/**
 * This class implements a very simple position, velocity, acceleration and effort limits enforcing
 * policy, and tries to do this in hardware-agnostic way.
 * This enable its general use in the Resource Manager.
 */
class SimpleJointLimiter : public JointLimiterInterface<JointLimits>
{
public:
  SimpleJointLimiter(
    JointLimits & limits, std::vector<hardware_interface::StateInterface> & state_interfaces,
    std::vector<hardware_interface::CommandInterface *> & command_interfaces);

protected:
  void enforce_effort_limits(const rclcpp::Duration & period) override;
  void enforce_pos_vel_acc_limits(const rclcpp::Duration & period) override;
};

}  // namespace joint_limits

#endif  // JOINT_LIMITS__SIMPLE_JOINT_LIMITER_HPP_
