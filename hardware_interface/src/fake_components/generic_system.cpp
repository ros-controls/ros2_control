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
#include <iterator>
#include <limits>
#include <set>
#include <string>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rcutils/logging_macros.h"

namespace fake_components
{
CallbackReturn GenericSystem::on_init(const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS)
  {
    return CallbackReturn::ERROR;
  }

  auto populate_non_standard_interfaces = [this](
                                            auto interface_list, auto & non_standard_interfaces) {
    for (const auto & interface : interface_list)
    {
      // add to list if non-standard interface
      if (
        std::find(standard_interfaces_.begin(), standard_interfaces_.end(), interface.name) ==
        standard_interfaces_.end())
      {
        if (
          std::find(
            non_standard_interfaces.begin(), non_standard_interfaces.end(), interface.name) ==
          non_standard_interfaces.end())
        {
          non_standard_interfaces.emplace_back(interface.name);
        }
      }
    }
  };

  // check if to create fake command interface for sensor
  auto it = info_.hardware_parameters.find("fake_sensor_commands");
  if (it != info_.hardware_parameters.end())
  {
    // TODO(anyone): change this to parse_bool() (see ros2_control#339)
    use_fake_sensor_command_interfaces_ = it->second == "true" || it->second == "True";
  }
  else
  {
    use_fake_sensor_command_interfaces_ = false;
  }

  // check if to create fake command interface for gpio
  it = info_.hardware_parameters.find("fake_gpio_commands");
  if (it != info_.hardware_parameters.end())
  {
    // TODO(anyone): change this to parse_bool() (see ros2_control#339)
    use_fake_gpio_command_interfaces_ = it->second == "true" || it->second == "True";
  }
  else
  {
    use_fake_gpio_command_interfaces_ = false;
  }

  // process parameters about state following
  position_state_following_offset_ = 0.0;
  custom_interface_with_following_offset_ = "";

  it = info_.hardware_parameters.find("position_state_following_offset");
  if (it != info_.hardware_parameters.end())
  {
    position_state_following_offset_ = std::stod(it->second);
    it = info_.hardware_parameters.find("custom_interface_with_following_offset");
    if (it != info_.hardware_parameters.end())
    {
      custom_interface_with_following_offset_ = it->second;
    }
  }
  // its extremlly unprobably that std::distance results int this value - therefore default
  index_custom_interface_with_following_offset_ = std::numeric_limits<size_t>::max();

  // Initialize storage for standard interfaces
  initialize_storage_vectors(joint_commands_, joint_states_, standard_interfaces_);
  // set all values without initial values to 0
  for (auto i = 0u; i < info_.joints.size(); i++)
  {
    for (auto j = 0u; j < standard_interfaces_.size(); j++)
    {
      if (std::isnan(joint_states_[j][i]))
      {
        joint_states_[j][i] = 0.0;
      }
    }
  }

  // Search for mimic joints
  for (auto i = 0u; i < info_.joints.size(); ++i)
  {
    const auto & joint = info_.joints.at(i);
    if (joint.parameters.find("mimic") != joint.parameters.cend())
    {
      const auto mimicked_joint_it = std::find_if(
        info_.joints.begin(), info_.joints.end(),
        [&mimicked_joint =
           joint.parameters.at("mimic")](const hardware_interface::ComponentInfo & joint_info) {
          return joint_info.name == mimicked_joint;
        });
      if (mimicked_joint_it == info_.joints.cend())
      {
        throw std::runtime_error(
          std::string("Mimicked joint '") + joint.parameters.at("mimic") + "' not found");
      }
      MimicJoint mimic_joint;
      mimic_joint.joint_index = i;
      mimic_joint.mimicked_joint_index = std::distance(info_.joints.begin(), mimicked_joint_it);
      auto param_it = joint.parameters.find("multiplier");
      if (param_it != joint.parameters.end())
      {
        mimic_joint.multiplier = std::stod(joint.parameters.at("multiplier"));
      }
      mimic_joints_.push_back(mimic_joint);
    }
  }

  // search for non-standard joint interfaces
  for (const auto & joint : info_.joints)
  {
    // populate non-standard command interfaces to other_interfaces_
    populate_non_standard_interfaces(joint.command_interfaces, other_interfaces_);

    // populate non-standard state interfaces to other_interfaces_
    populate_non_standard_interfaces(joint.state_interfaces, other_interfaces_);
  }

  // Initialize storage for non-standard interfaces
  initialize_storage_vectors(other_commands_, other_states_, other_interfaces_);

  // when following offset is used on custom interface then find its index
  if (!custom_interface_with_following_offset_.empty())
  {
    auto if_it = std::find(
      other_interfaces_.begin(), other_interfaces_.end(), custom_interface_with_following_offset_);
    if (if_it != other_interfaces_.end())
    {
      index_custom_interface_with_following_offset_ =
        std::distance(other_interfaces_.begin(), if_it);
      RCUTILS_LOG_INFO_NAMED(
        "fake_generic_system", "Custom interface with following offset '%s' found at index: %zu.",
        custom_interface_with_following_offset_.c_str(),
        index_custom_interface_with_following_offset_);
    }
    else
    {
      RCUTILS_LOG_WARN_NAMED(
        "fake_generic_system",
        "Custom interface with following offset '%s' does not exist. Offset will not be applied",
        custom_interface_with_following_offset_.c_str());
    }
  }

  for (const auto & sensor : info_.sensors)
  {
    for (const auto & interface : sensor.state_interfaces)
    {
      if (
        std::find(sensor_interfaces_.begin(), sensor_interfaces_.end(), interface.name) ==
        sensor_interfaces_.end())
      {
        sensor_interfaces_.emplace_back(interface.name);
      }
    }
  }
  initialize_storage_vectors(sensor_fake_commands_, sensor_states_, sensor_interfaces_);

  // search for gpio interfaces
  for (const auto & gpio : info_.gpios)
  {
    // populate non-standard command interfaces to gpio_interfaces_
    populate_non_standard_interfaces(gpio.command_interfaces, gpio_interfaces_);

    // populate non-standard state interfaces to gpio_interfaces_
    populate_non_standard_interfaces(gpio.state_interfaces, gpio_interfaces_);
  }

  // Fake gpio command interfaces
  if (use_fake_gpio_command_interfaces_)
  {
    initialize_storage_vectors(gpio_fake_commands_, gpio_states_, gpio_interfaces_);
  }
  // Real gpio command interfaces
  else
  {
    initialize_storage_vectors(gpio_commands_, gpio_states_, gpio_interfaces_);
  }

  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> GenericSystem::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  // Joints' state interfaces
  for (auto i = 0u; i < info_.joints.size(); i++)
  {
    const auto & joint = info_.joints[i];
    for (const auto & interface : joint.state_interfaces)
    {
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
  if (!populate_interfaces(
        info_.sensors, sensor_interfaces_, sensor_states_, state_interfaces, true))
  {
    throw std::runtime_error(
      "Interface is not found in the standard nor other list. This should never happen!");
  };

  // GPIO state interfaces
  if (!populate_interfaces(info_.gpios, gpio_interfaces_, gpio_states_, state_interfaces, true))
  {
    throw std::runtime_error("Interface is not found in the gpio list. This should never happen!");
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> GenericSystem::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  // Joints' state interfaces
  for (auto i = 0u; i < info_.joints.size(); i++)
  {
    const auto & joint = info_.joints[i];
    for (const auto & interface : joint.command_interfaces)
    {
      // Add interface: if not in the standard list than use "other" interface list
      if (!get_interface(
            joint.name, standard_interfaces_, interface.name, i, joint_commands_,
            command_interfaces))
      {
        if (!get_interface(
              joint.name, other_interfaces_, interface.name, i, other_commands_,
              command_interfaces))
        {
          throw std::runtime_error(
            "Interface is not found in the standard nor other list. "
            "This should never happen!");
        }
      }
    }
  }

  // Fake sensor command interfaces
  if (use_fake_sensor_command_interfaces_)
  {
    if (!populate_interfaces(
          info_.sensors, sensor_interfaces_, sensor_fake_commands_, command_interfaces, true))
    {
      throw std::runtime_error(
        "Interface is not found in the standard nor other list. This should never happen!");
    }
  }

  // Fake gpio command interfaces (consider all state interfaces for command interfaces)
  if (use_fake_gpio_command_interfaces_)
  {
    if (!populate_interfaces(
          info_.gpios, gpio_interfaces_, gpio_fake_commands_, command_interfaces, true))
    {
      throw std::runtime_error(
        "Interface is not found in the gpio list. This should never happen!");
    }
  }
  // GPIO command interfaces (real command interfaces)
  else
  {
    if (!populate_interfaces(
          info_.gpios, gpio_interfaces_, gpio_commands_, command_interfaces, false))
    {
      throw std::runtime_error(
        "Interface is not found in the gpio list. This should never happen!");
    }
  }

  return command_interfaces;
}

return_type GenericSystem::read()
{
  auto mirror_command_to_state = [](auto & states_, auto commands_, size_t start_index = 0) {
    for (size_t i = start_index; i < states_.size(); ++i)
    {
      for (size_t j = 0; j < states_[i].size(); ++j)
      {
        if (!std::isnan(commands_[i][j]))
        {
          states_[i][j] = commands_[i][j];
        }
      }
    }
  };

  // apply offset to positions only
  for (size_t j = 0; j < joint_states_[POSITION_INTERFACE_INDEX].size(); ++j)
  {
    if (!std::isnan(joint_commands_[POSITION_INTERFACE_INDEX][j]))
    {
      joint_states_[POSITION_INTERFACE_INDEX][j] =
        joint_commands_[POSITION_INTERFACE_INDEX][j] +
        (custom_interface_with_following_offset_.empty() ? position_state_following_offset_ : 0.0);
    }
  }

  // do loopback on all other interfaces - starts from 1 because 0 index is position interface
  mirror_command_to_state(joint_states_, joint_commands_, 1);

  for (const auto & mimic_joint : mimic_joints_)
  {
    for (auto i = 0u; i < joint_states_.size(); ++i)
    {
      joint_states_[i][mimic_joint.joint_index] =
        mimic_joint.multiplier * joint_states_[i][mimic_joint.mimicked_joint_index];
    }
  }

  for (size_t i = 0; i < other_states_.size(); ++i)
  {
    for (size_t j = 0; j < other_states_[i].size(); ++j)
    {
      if (
        i == index_custom_interface_with_following_offset_ &&
        !std::isnan(joint_commands_[POSITION_INTERFACE_INDEX][j]))
      {
        other_states_[i][j] =
          joint_commands_[POSITION_INTERFACE_INDEX][j] + position_state_following_offset_;
      }
      else if (!std::isnan(other_commands_[i][j]))
      {
        other_states_[i][j] = other_commands_[i][j];
      }
    }
  }

  if (use_fake_sensor_command_interfaces_)
  {
    mirror_command_to_state(sensor_states_, sensor_fake_commands_);
  }

  // do loopback on all gpio interfaces
  if (use_fake_gpio_command_interfaces_)
  {
    mirror_command_to_state(gpio_states_, gpio_fake_commands_);
  }
  else
  {
    mirror_command_to_state(gpio_states_, gpio_commands_);
  }

  return return_type::OK;
}

// Private methods
template <typename HandleType>
bool GenericSystem::get_interface(
  const std::string & name, const std::vector<std::string> & interface_list,
  const std::string & interface_name, const size_t vector_index,
  std::vector<std::vector<double>> & values, std::vector<HandleType> & interfaces)
{
  auto it = std::find(interface_list.begin(), interface_list.end(), interface_name);
  if (it != interface_list.end())
  {
    auto j = std::distance(interface_list.begin(), it);
    interfaces.emplace_back(name, *it, &values[j][vector_index]);
    return true;
  }
  return false;
}

void GenericSystem::initialize_storage_vectors(
  std::vector<std::vector<double>> & commands, std::vector<std::vector<double>> & states,
  const std::vector<std::string> & interfaces)
{
  // Initialize storage for all joints, regardless of their existence
  commands.resize(interfaces.size());
  states.resize(interfaces.size());
  for (auto i = 0u; i < interfaces.size(); i++)
  {
    commands[i].resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
    states[i].resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  }

  // Initialize with values from URDF
  for (auto i = 0u; i < info_.joints.size(); i++)
  {
    const auto & joint = info_.joints[i];
    for (const auto & interface : joint.state_interfaces)
    {
      auto it = std::find(interfaces.begin(), interfaces.end(), interface.name);

      // If interface name is found in the interfaces list
      if (it != interfaces.end())
      {
        auto index = std::distance(interfaces.begin(), it);

        // Check the initial_value param is used
        if (!interface.initial_value.empty())
        {
          states[index][i] = std::stod(interface.initial_value);
        }
        else
        {
          // Initialize the value in old way with warning message
          auto it2 = joint.parameters.find("initial_" + interface.name);
          if (it2 != joint.parameters.end())
          {
            states[index][i] = std::stod(it2->second);
            RCUTILS_LOG_WARN_NAMED(
              "fake_generic_system",
              "The usage of initial_%s has been deprecated. Please use 'initial_value' instead.",
              interface.name.c_str());
          }
        }
      }
    }
  }
}

template <typename InterfaceType>
bool GenericSystem::populate_interfaces(
  const std::vector<hardware_interface::ComponentInfo> & components,
  std::vector<std::string> & interface_names, std::vector<std::vector<double>> & storage,
  std::vector<InterfaceType> & target_interfaces, bool using_state_interfaces)
{
  for (auto i = 0u; i < components.size(); i++)
  {
    const auto & component = components[i];
    const auto interfaces =
      (using_state_interfaces) ? component.state_interfaces : component.command_interfaces;
    for (const auto & interface : interfaces)
    {
      if (!get_interface(
            component.name, interface_names, interface.name, i, storage, target_interfaces))
      {
        return false;
      }
    }
  }

  return true;
}
}  // namespace fake_components

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(fake_components::GenericSystem, hardware_interface::SystemInterface)
