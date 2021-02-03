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
#include <string>
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

  // Initialize storage for standard interfaces
  initialize_storage_vectors(hw_joint_commands_, hw_joint_states_, standard_interfaces_);

  // search for non-standard joint interfaces
  for (const auto & joint : info_.joints) {
    for (const auto & interface : joint.command_interfaces) {
      // add to list if non-standard interface
      if (std::find(standard_interfaces_.begin(), standard_interfaces_.end(), interface.name) ==
        standard_interfaces_.end())
      {
        other_interfaces_.push_back(interface.name);
      }
    }
    for (const auto & interface : joint.state_interfaces) {
      // add to list if non-standard interface
      if (std::find(standard_interfaces_.begin(), standard_interfaces_.end(), interface.name) ==
        standard_interfaces_.end())
      {
        other_interfaces_.push_back(interface.name);
      }
    }
  }

  // Initialize storage for non-standard interfaces
  initialize_storage_vectors(hw_other_commands_, hw_other_states_, other_interfaces_);

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
      if (it != standard_interfaces_.end()) {
        auto j = std::distance(standard_interfaces_.begin(), it);
        state_interfaces.emplace_back(joint.name, *it, &hw_joint_states_[j][i]);
      } else {
        auto it = std::find(other_interfaces_.begin(), other_interfaces_.end(), interface.name);
        if (it != other_interfaces_.end()) {
          auto j = std::distance(other_interfaces_.begin(), it);
          state_interfaces.emplace_back(joint.name, *it, &hw_other_states_[j][i]);
        }
      }
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
      if (it != standard_interfaces_.end()) {
        auto j = std::distance(standard_interfaces_.begin(), it);
        command_interfaces.emplace_back(joint.name, *it, &hw_joint_commands_[j][i]);
      } else {
        auto it = std::find(other_interfaces_.begin(), other_interfaces_.end(), interface.name);
        if (it != other_interfaces_.end()) {
          auto j = std::distance(other_interfaces_.begin(), it);
          command_interfaces.emplace_back(joint.name, *it, &hw_other_commands_[j][i]);
        }
      }
    }
  }

  return command_interfaces;
}

return_type GenericSystem::read()
{
  // only do loopback
  hw_joint_states_ = hw_joint_commands_;
  hw_other_states_ = hw_other_commands_;
  return return_type::OK;
}

// Private methods

void GenericSystem::initialize_storage_vectors(
  std::vector<std::vector<double>> & commands,
  std::vector<std::vector<double>> & states,
  const std::vector<std::string> & interfaces)
{
  // Initialize storage for all joints, regardless of their existance
  commands.resize(interfaces.size());
  states.resize(interfaces.size());
  for (uint i = 0; i < interfaces.size(); i++) {
    commands[i].resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
    states[i].resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  }

  // Initialize with values from URDF
  for (uint i = 0; i < info_.joints.size(); i++) {
    const auto & joint = info_.joints[i];
    for (uint j = 0; j < interfaces.size(); j++) {
      auto it = joint.parameters.find("initial_" + interfaces[j]);
      if (it != joint.parameters.end()) {
        commands[j][i] = std::stod(it->second);
        states[j][i] = std::stod(it->second);
      }
    }
  }
}

}  // namespace fake_components

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(fake_components::GenericSystem, hardware_interface::SystemInterface)
PLUGINLIB_EXPORT_CLASS(fake_components::GenericRobot, hardware_interface::SystemInterface)
