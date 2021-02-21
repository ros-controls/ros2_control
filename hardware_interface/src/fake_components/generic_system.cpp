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

  // check if to create fake command interface for sensor
  auto it = info_.hardware_parameters.find("fake_sensor_commands");
  if (it != info_.hardware_parameters.end()) {
    fake_sensor_command_interfaces = it->second == "true";
  } else {
    fake_sensor_command_interfaces = false;
  }

  // Initialize storage for standard interfaces
  initialize_storage_vectors(joint_commands_, joint_states_, standard_interfaces_);
  // set all values without initial values to 0
  for (uint i = 0; i < info_.joints.size(); i++) {
    for (uint j = 0; j < standard_interfaces_.size(); j++) {
      if (std::isnan(joint_commands_[j][i])) {
        joint_commands_[j][i] = 0.0;
        joint_states_[j][i] = 0.0;
      }
    }
  }

  // search for non-standard joint interfaces
  for (const auto & joint : info_.joints) {
    for (const auto & interface : joint.command_interfaces) {
      // add to list if non-standard interface
      if (std::find(standard_interfaces_.begin(), standard_interfaces_.end(), interface.name) ==
        standard_interfaces_.end())
      {
        other_interfaces_.emplace_back(interface.name);
      }
    }
    for (const auto & interface : joint.state_interfaces) {
      // add to list if non-standard interface
      if (std::find(standard_interfaces_.begin(), standard_interfaces_.end(), interface.name) ==
        standard_interfaces_.end())
      {
        other_interfaces_.emplace_back(interface.name);
      }
    }
  }
  // Initialize storage for non-standard interfaces
  initialize_storage_vectors(other_commands_, other_states_, other_interfaces_);

  for (const auto & sensor : info_.sensors) {
    for (const auto & interface : sensor.state_interfaces) {
      sensor_interfaces_.emplace_back(interface.name);
    }
  }
  initialize_storage_vectors(sensor_fake_commands_, sensor_states_, sensor_interfaces_);

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
      // Add interface: if not in the standard list than use "other" interface list
      if (!get_interface(
          joint.name, standard_interfaces_, interface.name, i, joint_states_, state_interfaces))
      {
        if (!get_interface(
            joint.name, other_interfaces_, interface.name, i, other_states_, state_interfaces))
        {
          throw std::runtime_error(
                  "Interface is not found in the standard nor other list. "
                  "This should never happen!");
        }
      }
    }
  }

  // Sensor state interfaces
  for (uint i = 0; i < info_.sensors.size(); i++) {
    const auto & sensor = info_.sensors[i];
    for (const auto & interface : sensor.state_interfaces) {
      if (!get_interface(
          sensor.name, sensor_interfaces_, interface.name, i, sensor_states_, state_interfaces))
      {
        throw std::runtime_error(
                "Interface is not found in the standard nor other list. "
                "This should never happen!");
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
      // Add interface: if not in the standard list than use "other" interface list
      if (!get_interface(
          joint.name, standard_interfaces_, interface.name, i, joint_commands_, command_interfaces))
      {
        if (!get_interface(
            joint.name, other_interfaces_, interface.name, i, other_commands_, command_interfaces))
        {
          throw std::runtime_error(
                  "Interface is not found in the standard nor other list. "
                  "This should never happen!");
        }
      }
    }
  }

  // Fake sensor command interfaces
  if (fake_sensor_command_interfaces) {
    for (uint i = 0; i < info_.sensors.size(); i++) {
      const auto & sensor = info_.sensors[i];
      for (const auto & interface : sensor.state_interfaces) {
        if (!get_interface(
            sensor.name, sensor_interfaces_, interface.name, i,
            sensor_fake_commands_, command_interfaces))
        {
          throw std::runtime_error(
                  "Interface is not found in the standard nor other list. "
                  "This should never happen!");
        }
      }
    }
  }

  return command_interfaces;
}

return_type GenericSystem::read()
{
  // only do loopback
  joint_states_ = joint_commands_;
  other_states_ = other_commands_;
  if (fake_sensor_command_interfaces) {
    sensor_states_ = sensor_fake_commands_;
  }
  return return_type::OK;
}

// Private methods
template<typename HandleType>
bool GenericSystem::get_interface(
  const std::string & name,
  const std::vector<std::string> & interface_list,
  const std::string & interface_name,
  const uint vector_index,
  std::vector<std::vector<double>> & values,
  std::vector<HandleType> & interfaces)
{
  auto it = std::find(interface_list.begin(), interface_list.end(), interface_name);
  if (it != interface_list.end()) {
    auto j = std::distance(interface_list.begin(), it);
    interfaces.emplace_back(name, *it, &values[j][vector_index]);
    return true;
  }
  return false;
}

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
