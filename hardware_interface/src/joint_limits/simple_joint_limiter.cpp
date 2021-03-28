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

#include "joint_limits/simple_joint_limiter.hpp"

#include <algorithm>
#include <limits>
#include <stdexcept>
#include <vector>

#include "rcppmath/clamp.hpp"

namespace joint_limits
{
SimpleJointLimiter::SimpleJointLimiter(
  JointLimits & limits, std::vector<hardware_interface::StateInterface> & state_interfaces,
  std::vector<hardware_interface::CommandInterface *> & command_interfaces)
: JointLimiterInterface(limits, state_interfaces, command_interfaces)
{
  // Position state interface has to be always present and not virtual
  if (
    std::find_if(
      state_interfaces.begin(), state_interfaces.end(),
      [](const hardware_interface::StateInterface & interface) {
        return interface.get_name() == hardware_interface::HW_IF_POSITION;
      }) == state_interfaces.end())
  {
    throw std::runtime_error(
            "Simple joint limiter requires position state interface for joint '" + get_name() +
            "'.");
  }

  if (has_effort_command_) {
    if (!limits.has_velocity_limits) {
      throw std::runtime_error(
              "Cannot enforce limits on effort command interface for joint '" + get_name() +
              "'. It has no velocity limits specification.");
    }
    if (!limits.has_effort_limits) {
      throw std::runtime_error(
              "Cannot enforce limits on effort command interface for joint '" + get_name() +
              "'. It has no efforts limits specification.");
    }
    if (
      std::find_if(
        state_interfaces.begin(), state_interfaces.end(),
        [](const hardware_interface::StateInterface & interface) {
          return interface.get_name() == hardware_interface::HW_IF_VELOCITY;
        }) != state_interfaces.end())
    {
      has_velocity_state_ = true;
    }
  }

  if (limits_.has_position_limits) {
    min_position_limit_ = limits_.min_position;
    max_position_limit_ = limits_.max_position;
  } else {
    min_position_limit_ = -std::numeric_limits<double>::max();
    max_position_limit_ = std::numeric_limits<double>::max();
  }
  if (limits_.has_velocity_limits) {
    max_velocity_limit_ = limits_.max_velocity;
  } else {
    max_velocity_limit_ = std::numeric_limits<double>::max();
  }
  if (limits_.has_acceleration_limits) {
    max_acceleration_limit_ = limits_.max_velocity;
  } else {
    max_acceleration_limit_ = std::numeric_limits<double>::max();
  }
}

void SimpleJointLimiter::enforce_effort_limits(const rclcpp::Duration & period)
{
  double min_effort = -limits_.max_effort;
  double max_effort = limits_.max_effort;

  const double current_position = state_interfaces_[0].get_value();
  if (current_position < limits_.min_position) {
    min_effort = 0.0;
  } else if (current_position > limits_.max_position) {
    max_effort = 0.0;
  }

  double current_velocity = 0.0;
  if (has_velocity_state_) {
    current_velocity = state_interfaces_[1].get_value();
  } else {
    current_velocity = (current_position - previous_position_) / period.seconds();
    previous_position_ = current_position;
  }

  if (current_velocity < -max_velocity_limit_) {
    min_effort = 0.0;
  } else if (current_velocity > max_velocity_limit_) {
    min_effort = 0.0;
  }

  // Saturate effort command according to limits
  command_interfaces_[0]->set_value(
    rcppmath::clamp(command_interfaces_[0]->get_value(), min_effort, max_effort));
}

void SimpleJointLimiter::enforce_pos_vel_acc_limits(const rclcpp::Duration & period)
{
  double min_position, max_position;
  double min_velocity = -max_velocity_limit_;
  double max_velocity = max_velocity_limit_;
  double min_acceleration = -max_acceleration_limit_;
  double max_acceleration = max_acceleration_limit_;
  const double dt = period.seconds();

  // Make trivial dynamic clamping
  if (previous_position_ == min_position_limit_) {
    min_velocity = 0.0;
  } else if (previous_position_ == max_position_limit_) {
    max_velocity = 0.0;
  }
  if (previous_velocity_ <= min_velocity) {
    min_acceleration = 0.0;
  } else if (previous_velocity_ >= max_velocity) {
    max_acceleration = 0.0;
  }

  // enforce acceleration limits - set velocity constants
  // the velocity is based on the max acceleration times seconds since last update
  min_velocity = std::max(previous_velocity_ + min_acceleration * dt, min_velocity);
  max_velocity = std::min(previous_velocity_ + max_acceleration * dt, max_velocity);

  min_position = std::max(previous_position_ + min_velocity * dt, min_position_limit_);
  max_position = std::min(previous_position_ + max_velocity * dt, max_position_limit_);

  // Saturate acceleration command according to limits
  command_interfaces_[2]->set_value(
    rcppmath::clamp(command_interfaces_[2]->get_value(), min_acceleration, max_acceleration));

  // Saturate velocity command according to limits
  command_interfaces_[1]->set_value(
    rcppmath::clamp(command_interfaces_[1]->get_value(), min_velocity, max_velocity));
  // Cache variables
  previous_velocity_ = command_interfaces_[1]->get_value();

  // Saturate position command according to limits
  command_interfaces_[0]->set_value(
    rcppmath::clamp(command_interfaces_[0]->get_value(), min_position_limit_, max_position_limit_));
  // Cache variables
  previous_position_ = command_interfaces_[0]->get_value();
}

}  // namespace joint_limits
