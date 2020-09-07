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

#include <string>
#include <vector>

#include "hardware_interface/components/joint.hpp"

#include "hardware_interface/components/component_info.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"

#include "./component_lists_management.hpp"

namespace hardware_interface
{
namespace components
{

return_type Joint::configure(const ComponentInfo & joint_info)
{
  info_ = joint_info;
  if (info_.command_interfaces.size() > 0) {
    commands_.resize(info_.command_interfaces.size());
  }
  if (info_.state_interfaces.size() > 0) {
    states_.resize(info_.state_interfaces.size());
  }
  return return_type::OK;
}

std::vector<std::string> Joint::get_command_interfaces() const
{
  return info_.command_interfaces;
}

std::vector<std::string> Joint::get_state_interfaces() const
{
  return info_.state_interfaces;
}

return_type Joint::get_command(
  std::vector<double> & command, const std::vector<std::string> & interfaces) const
{
  return get_internal_values(command, interfaces, info_.command_interfaces, commands_);
}

return_type Joint::get_command(std::vector<double> & command) const
{
  return get_internal_values(command, commands_);
}

return_type Joint::set_command(
  const std::vector<double> & command,
  const std::vector<std::string> & interfaces)
{
  return set_internal_values(command, interfaces, info_.command_interfaces, commands_);
}

return_type Joint::set_command(const std::vector<double> & command)
{
  return set_internal_values(command, commands_);
}

return_type Joint::get_state(
  std::vector<double> & state, const std::vector<std::string> & interfaces) const
{
  return get_internal_values(state, interfaces, info_.state_interfaces, states_);
}

return_type Joint::get_state(std::vector<double> & state) const
{
  return get_internal_values(state, states_);
}

return_type Joint::set_state(
  const std::vector<double> & state, const std::vector<std::string> & interfaces)
{
  return set_internal_values(state, interfaces, info_.state_interfaces, states_);
}

return_type Joint::set_state(const std::vector<double> & state)
{
  return set_internal_values(state, states_);
}

}  // namespace components
}  // namespace hardware_interface
