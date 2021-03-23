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

#ifndef JOINT_LIMITS__JOINT_LIMITER_INTERFACE_HPP_
#define JOINT_LIMITS__JOINT_LIMITER_INTERFACE_HPP_

#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "joint_limits/joint_limits.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/rclcpp.hpp"

namespace joint_limits
{
// TODO(all): there is possibility to use something like LoanedCommandInterface, still having copy-constructor on this level makes thing way easier and more readable
// class LimiterCommandHandle

template <class LimitsType>
class JointLimiterInterface
{
public:
  JointLimiterInterface(
    LimitsType & limits, 
    std::vector<hardware_interface::StateInterface> & state_interfaces,
    // Here could be used "LimiterCommandHandle" with copy-constructor to avid raw-pointer access/storage
    std::vector<hardware_interface::CommandInterface *> & command_interfaces)
  : limits_(limits),
    has_effort_command_(false),
    has_velocity_state_(false),
    previous_position_(std::numeric_limits<double>::quiet_NaN()),
    previous_velocity_(0.0)
  {
    // At least one command interface has to be provided
    if (command_interfaces.size() == 0) {
      throw std::runtime_error("At least one command interface has to be provided.");
    }
    
    // Check if position-velocity-acceleration or effort commands are used
    if (
      std::find_if(
        command_interfaces.begin(), command_interfaces.end(),
        [](const hardware_interface::CommandInterface * interface) {
          return interface->get_name() == hardware_interface::HW_IF_EFFORT;
        }) != command_interfaces.end()) {
      has_effort_command_ = true;
      // Effort command interface has to be used as only interface
      if (command_interfaces.size() > 1) {
        // TODO(all): Do we need here specific exception type?
        throw std::runtime_error("Effort command interface has to be used alone.");
      }
    }

    for (const auto & interface_name : interface_order_) {
      for (const auto & interface : command_interfaces) {
        if (interface_name == interface->get_name()) {
          command_interfaces_.emplace_back(interface);
        } else {
          // Do not create 'effort' virtual interfaces if effort command interface is used
          if (!has_effort_command_ || interface_name != hardware_interface::HW_IF_EFFORT) {
            virtual_command_storage_.push_back(0.0);
            auto i = virtual_command_storage_.size() - 1;  // last element
            virtual_command_interfaces_.emplace_back(hardware_interface::CommandInterface(
              "joint_limiter_virtual", interface_name, &virtual_command_storage_[i]));
            command_interfaces_.emplace_back(&virtual_command_interfaces_.back());
          }
        }
      }
      for (const auto & interface : state_interfaces) {
        if (interface_name == interface.get_name()) {
          state_interfaces_.emplace_back(interface);
        } else {
          virtual_state_storage_.push_back(0.0);
          auto i = virtual_state_storage_.size() - 1;  // last element
          state_interfaces_.emplace_back(hardware_interface::StateInterface(
            "joint_limiter_virtual", interface_name, &virtual_state_storage_[i]));
        }
      }
    }
  }

  ~JointLimiterInterface() = default;

  /// Implementation of limit enforcing policy.
  virtual void enforce_limits(const rclcpp::Duration & period)
  {
    if (std::isnan(previous_position_)) {
      previous_position_ = state_interfaces_[0].get_value();
    }

    if (has_effort_command_) {
      enforce_effort_limits(period);
    } else {
      enforce_pos_vel_acc_limits(period);
    }
  }

  std::string get_name() const
  {
    // Hopefully some interface exists. Simply go through all of them and return first name
    for (const auto & interface : state_interfaces_) {
      return interface.get_name();
    }
    for (const auto & interface : command_interfaces_) {
      return interface->get_name();
    }
    return std::string();
  }

  /// Clear stored state, causing to reset next iteration.
  virtual void reset()
  {
    previous_position_ = std::numeric_limits<double>::quiet_NaN();
    previous_velocity_ = 0.0;
  }

protected:
  /// Limit enforcing policy for effort command interface.
  virtual void enforce_effort_limits(const rclcpp::Duration & period) = 0;

  /// Limit enforcing policy for position, velocity and acceleration command interfaces.
  virtual void enforce_pos_vel_acc_limits(const rclcpp::Duration & period) = 0;

  LimitsType & limits_;
  std::vector<hardware_interface::CommandInterface *> command_interfaces_;
  std::vector<hardware_interface::StateInterface> state_interfaces_;
  std::vector<hardware_interface::CommandInterface> virtual_command_interfaces_;
  std::vector<hardware_interface::StateInterface> virtual_state_interfaces_;
  std::vector<double> virtual_command_storage_;
  std::vector<double> virtual_state_storage_;

  bool has_effort_command_;
  bool has_velocity_state_;

  // stored states
  double previous_position_;
  double previous_velocity_;

  // min and max values
  double min_position_limit_, max_position_limit_;
  double max_velocity_limit_;
  double max_acceleration_limit_;

  std::vector<std::string> interface_order_ = {
    hardware_interface::HW_IF_POSITION, hardware_interface::HW_IF_VELOCITY,
    hardware_interface::HW_IF_ACCELERATION, hardware_interface::HW_IF_EFFORT};
};

}  // namespace joint_limits

#endif  // JOINT_LIMITS__JOINT_LIMITER_INTERFACE_HPP_
