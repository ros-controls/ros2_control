// Copyright 2020 ros2_control Development Team
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

#include "multi_interface_joint/multi_interface_joint.hpp"

#include <set>
#include <string>
#include <vector>

#include "./component_lists_management.hpp"

namespace multi_interface_joint
{

hardware_interface::return_type MultiInterfaceJoint::configure(
  const hardware_interface::components::ComponentInfo & joint_info)
{
  const size_t command_interfaces_size = joint_info.command_interfaces.size();
  const size_t state_interfaces_size = joint_info.state_interfaces.size();

  // fail if no interfaces at all are specified
  if (command_interfaces_size == state_interfaces_size && command_interfaces_size == 0u) {
    return hardware_interface::return_type::INTERFACE_NOT_PROVIDED;
  }

  auto has_duplicates = [](const std::vector<std::string> interfaces) -> bool
    {
      std::set<std::string> set(interfaces.begin(), interfaces.end());
      return set.size() != interfaces.size();
    };

  // fail if command interfaces has duplicates
  if (has_duplicates(joint_info.command_interfaces)) {
    return hardware_interface::return_type::INTERFACE_DUPLICATES;
  }

  // fail if state interfaces has duplicates
  if (has_duplicates(joint_info.state_interfaces)) {
    return hardware_interface::return_type::INTERFACE_DUPLICATES;
  }

  command_interfaces_ = joint_info.command_interfaces;
  command_values_.resize(command_interfaces_size);

  state_interfaces_ = joint_info.state_interfaces;
  state_values_.resize(state_interfaces_size);

  return hardware_interface::return_type::OK;
}

std::vector<std::string> MultiInterfaceJoint::get_command_interfaces() const
{
  return command_interfaces_;
}

std::vector<std::string> MultiInterfaceJoint::get_state_interfaces() const
{
  return state_interfaces_;
}

hardware_interface::return_type MultiInterfaceJoint::get_command(
  std::vector<double> & command,
  const std::vector<std::string> & interfaces) const
{
  return get_internal_values(command, interfaces, command_interfaces_, command_values_);
}

hardware_interface::return_type MultiInterfaceJoint::get_command(
  std::vector<double> & command) const
{
  return get_internal_values(command, command_values_);
}

hardware_interface::return_type MultiInterfaceJoint::set_command(
  const std::vector<double> & command,
  const std::vector<std::string> & interfaces)
{
  return set_internal_values(command, interfaces, command_interfaces_, command_values_);
}

hardware_interface::return_type MultiInterfaceJoint::set_command(
  const std::vector<double> & command)
{
  return set_internal_values(command, command_values_);
}

hardware_interface::return_type MultiInterfaceJoint::get_state(
  std::vector<double> & state,
  const std::vector<std::string> & interfaces) const
{
  return get_internal_values(state, interfaces, state_interfaces_, state_values_);
}

hardware_interface::return_type MultiInterfaceJoint::get_state(std::vector<double> & state) const
{
  return get_internal_values(state, state_values_);
}

hardware_interface::return_type MultiInterfaceJoint::set_state(
  const std::vector<double> & state,
  const std::vector<std::string> & interfaces)
{
  return set_internal_values(state, interfaces, state_interfaces_, state_values_);
}

hardware_interface::return_type MultiInterfaceJoint::set_state(const std::vector<double> & state)
{
  return set_internal_values(state, state_values_);
}

}  // namespace multi_interface_joint
