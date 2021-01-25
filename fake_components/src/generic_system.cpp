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

#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <rclcpp/rclcpp.hpp>

namespace fake_components
{

std::vector<hardware_interface::StateInterface> GenericSystem::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (uint i = 0; i < info_.joints.size(); i++) {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION, &current_positions[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &current_velocities[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &current_efforts[i]));
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> GenericSystem::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (uint i = 0; i < info_.joints.size(); i++) {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION, &command_positions[i]));
  }

  return command_interfaces;
}
hardware_interface::return_type
GenericSystem::configure(const hardware_interface::HardwareInfo &info)
{
  if (configure_default(info) != hardware_interface::return_type::OK) {
    return hardware_interface::return_type::ERROR;
  }

  // Default start position is zero
  command_positions.resize(info_.joints.size(), 0.0);
  current_positions.resize(info_.joints.size(), 0.0);
  current_velocities.resize(info_.joints.size(), 0.0);
  current_efforts.resize(info_.joints.size(), 0.0);

  for (size_t i = 0; i < info_.joints.size(); ++i) {
    const auto &joint = info_.joints.at(i);
    auto it = joint.parameters.find("start_position");
    if (it != joint.parameters.end()) {
      command_positions.at(i) = std::stod(it->second);
      current_positions.at(i) = std::stod(it->second);
    }
  }

  status_ = hardware_interface::status::CONFIGURED;
  return hardware_interface::return_type::OK;
}

} // namespace fake_components

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(fake_components::GenericSystem, hardware_interface::SystemInterface)
