// Copyright (c) 2021, PickNik Inc.
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

/// \author Andy Zelenak, Denis Stogl

#ifndef RUCKIG_JOINT_LIMITER__RUCKIG_JOINT_LIMITER_HPP_
#define RUCKIG_JOINT_LIMITER__RUCKIG_JOINT_LIMITER_HPP_

#include <memory>

#include "joint_limits/joint_limiter_interface.hpp"
#include "joint_limits/joint_limits.hpp"
#include "joint_limits_enforcement_plugins/visibility_control.h"
#include "ruckig/input_parameter.hpp"
#include "ruckig/output_parameter.hpp"
#include "ruckig/ruckig.hpp"

namespace ruckig_joint_limiter
{
namespace  // utility namespace
{
constexpr double DEFAULT_MAX_VELOCITY = 5;       // rad/s
constexpr double DEFAULT_MAX_ACCELERATION = 10;  // rad/s^2
constexpr double DEFAULT_MAX_JERK = 20;          // rad/s^3
}  // namespace

template <typename LimitsType>
class RuckigJointLimiter : public joint_limits::JointLimiterInterface<joint_limits::JointLimits>
{
public:
  JOINT_LIMITS_ENFORCEMENT_PLUGINS_PUBLIC RuckigJointLimiter();

  JOINT_LIMITS_ENFORCEMENT_PLUGINS_PUBLIC bool on_init() override;

  JOINT_LIMITS_ENFORCEMENT_PLUGINS_PUBLIC bool on_configure(
    const trajectory_msgs::msg::JointTrajectoryPoint & current_joint_states) override;

  JOINT_LIMITS_ENFORCEMENT_PLUGINS_PUBLIC bool on_enforce(
    trajectory_msgs::msg::JointTrajectoryPoint & current_joint_states,
    trajectory_msgs::msg::JointTrajectoryPoint & desired_joint_states,
    const rclcpp::Duration & dt) override;

private:
  bool received_initial_state_ = false;
  // Ruckig algorithm
  std::shared_ptr<ruckig::Ruckig<0>> ruckig_;
  std::shared_ptr<ruckig::InputParameter<0>> ruckig_input_;
  std::shared_ptr<ruckig::OutputParameter<0>> ruckig_output_;
};

}  // namespace ruckig_joint_limiter

#endif  // RUCKIG_JOINT_LIMITER__RUCKIG_JOINT_LIMITER_HPP_
