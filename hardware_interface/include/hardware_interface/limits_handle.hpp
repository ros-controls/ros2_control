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

#ifndef HARDWARE_INTERFACE__LIMITS_HANDLE_HPP
#define HARDWARE_INTERFACE__LIMITS_HANDLE_HPP

#include <optional>
#include "hardware_interface/handle.hpp"
#include "joint_limits/joint_limits.hpp"
#include "rclcpp/duration.hpp"

namespace hardware_interface
{
class SaturationHandle
{
public:
  SaturationHandle() {}

  void enforce_limits(const rclcpp::Duration & period)
  {
    enforce_position_limits(period);
    enforce_velocity_limits(period);
  }

private:
  void enforce_position_limits(const rclcpp::Duration & period)
  {
    if (reference_position_.has_value())
    {
      // Initially, the previous command will be nan, set it to the current position value
      if (!std::isfinite(prev_pos_cmd_))
      {
        if (actual_position_.has_value())
        {
          prev_pos_cmd_ = actual_position_.value().get_value();
        }
        else
        {
          prev_pos_cmd_ = reference_position_->get_value();
        }
      }

      // Check if the joint has joint limits, if not set the limits to maximum
      double min_pos_limit, max_pos_limit;
      if (limits_.has_position_limits)
      {
        min_pos_limit = limits_.min_position;
        max_pos_limit = limits_.max_position;
      }
      else
      {
        min_pos_limit = std::numeric_limits<double>::min();
        max_pos_limit = std::numeric_limits<double>::max();
      }

      // Evalute and clamp the position command to the maximum reachable value by hardware
      double min_pos, max_pos;
      if (limits_.has_velocity_limits)
      {
        const double delta_pos = limits_.max_velocity * period.seconds();
        min_pos = std::max(prev_pos_cmd_ - delta_pos, min_pos_limit);
        max_pos = std::min(prev_pos_cmd_ + delta_pos, max_pos_limit);
      }
      else
      {
        min_pos = min_pos_limit;
        max_pos = max_pos_limit;
      }

      // Saturate position command according to limits
      const double cmd = std::clamp(reference_position_.value().get_value(), min_pos, max_pos);
      reference_position_->set_value(cmd);
      prev_pos_cmd_ = cmd;
    }
  }

  void enforce_velocity_limits(const rclcpp::Duration & period)
  {
    if (limits_.has_velocity_limits && reference_velocity_.has_value())
    {
      // Velocity bounds
      double vel_low;
      double vel_high;

      if (!std::isfinite(reference_velocity_->get_value()))
      {
        reference_velocity_->set_value(0.0);
      }
      if (!std::isfinite(prev_vel_cmd_))
      {
        prev_vel_cmd_ = 0.0;
      }

      if (limits_.has_acceleration_limits)
      {
        assert(period.seconds() > 0.0);
        const double dt = period.seconds();

        vel_low =
          std::max(prev_vel_cmd_ - fabs(limits_.max_deceleration) * dt, -limits_.max_velocity);
        vel_high = std::min(prev_vel_cmd_ + limits_.max_acceleration * dt, limits_.max_velocity);
      }
      else
      {
        vel_low = -limits_.max_velocity;
        vel_high = limits_.max_velocity;
      }

      // Saturate velocity command according to limits
      const double vel_cmd = std::clamp(reference_velocity_.value().get_value(), vel_low, vel_high);
      reference_velocity_->set_value(vel_cmd);
      prev_vel_cmd_ = vel_cmd;
    }
  }

  void enforce_effort_limits(const rclcpp::Duration & period)
  {
    if (limits_.has_effort_limits && reference_effort_.has_value())
    {
      double min_eff = -limits_.max_effort;
      double max_eff = limits_.max_effort;

      if (limits_.has_position_limits && actual_position_.has_value())
      {
        const double pos = actual_position_->get_value();
        if (pos < limits_.min_position)
        {
          min_eff = 0;
        }
        else if (pos > limits_.max_position)
        {
          max_eff = 0;
        }
      }

      if (limits_.has_velocity_limits && actual_velocity_.has_value())
      {
        const double vel = actual_velocity_->get_value();
        if (vel < -limits_.max_velocity)
        {
          min_eff = 0;
        }
        else if (vel > limits_.max_velocity)
        {
          max_eff = 0;
        }
      }
      // Saturate effort command according to limits
      const double eff_cmd = std::clamp(reference_effort_.value().get_value(), min_eff, max_eff);
      reference_effort_->set_value(eff_cmd);
    }
  }

  joint_limits::JointLimits limits_;
  double prev_pos_cmd_ = {std::numeric_limits<double>::quiet_NaN()};
  double prev_vel_cmd_ = {std::numeric_limits<double>::quiet_NaN()};

  std::optional<StateInterface> actual_position_ = std::nullopt;
  std::optional<StateInterface> actual_velocity_ = std::nullopt;
  std::optional<StateInterface> actual_effort_ = std::nullopt;
  std::optional<StateInterface> actual_acceleration_ = std::nullopt;

  std::optional<CommandInterface> reference_position_ = std::nullopt;
  std::optional<CommandInterface> reference_velocity_ = std::nullopt;
  std::optional<CommandInterface> reference_effort_ = std::nullopt;
  std::optional<CommandInterface> reference_acceleration_ = std::nullopt;
};

}  // namespace hardware_interface

#endif  // HARDWARE_INTERFACE__LIMITS_HANDLE_HPP
