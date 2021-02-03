// Copyright (c) 2021 PickNik, Inc.
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
//
// Author: Jafar Abdi, Denis Stogl

#include "fake_components/generic_system.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace fake_components
{

return_type GenericSystem::configure(const hardware_interface::HardwareInfo & info)
{
  if (configure_default(info) != return_type::OK) {
    return return_type::ERROR;
  }

  // Initialize storage for all joints, regardless of their existance
  hw_joint_commands_.resize(standard_interfaces_.size());
  hw_joint_states_.resize(standard_interfaces_.size());
  for (uint i = 0; i < standard_interfaces_.size(); i++) {
    hw_joint_commands_[i].resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
    hw_joint_states_[i].resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  }

  // Initialize with values from URDF
  for (uint i = 0; i < info_.joints.size(); i++) {
    const auto & joint = info_.joints[i];
    for (uint j = 0; j < standard_interfaces_.size(); j++) {
      auto it = joint.parameters.find("start_" + standard_interfaces_[j]);
      if (it != joint.parameters.end()) {
        hw_joint_commands_[j][i] = std::stod(it->second);
        hw_joint_states_[j][i] = std::stod(it->second);
      }
    }
  }

  status_ = hardware_interface::status::CONFIGURED;
  return hardware_interface::return_type::OK;
}

std::vector<hardware_interface::StateInterface> GenericSystem::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  // Joints' state interfaces
  for (uint i = 0; i < info_.joints.size(); i++) {
    const auto & joint = info_.joints[i];
    for (const auto & interface : joint.state_interfaces) {
      auto it = std::find(standard_interfaces_.begin(), standard_interfaces_.end(), interface.name);
      auto j = std::distance(standard_interfaces_.begin(), it);
      state_interfaces.emplace_back(joint.name, *it, &hw_joint_states_[j][i]);
    }
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> GenericSystem::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  // Joints' state interfaces
  for (uint i = 0; i < info_.joints.size(); i++) {
    const auto & joint = info_.joints[i];
    for (const auto & interface : joint.command_interfaces) {
      auto it = std::find(standard_interfaces_.begin(), standard_interfaces_.end(), interface.name);
      auto j = std::distance(standard_interfaces_.begin(), it);
      command_interfaces.emplace_back(joint.name, *it, &hw_joint_commands_[j][i]);
    }
  }

  return command_interfaces;
}

return_type GenericSystem::read()
{
  // only do loopback
  hw_joint_states_ = hw_joint_commands_;
  return return_type::OK;
}

}  // namespace fake_components

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(fake_components::GenericSystem, hardware_interface::SystemInterface)
PLUGINLIB_EXPORT_CLASS(fake_components::GenericRobot, hardware_interface::SystemInterface)
