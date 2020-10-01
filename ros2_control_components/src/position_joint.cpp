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
#include "hardware_interface/components/component_lists_management.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"

using hardware_interface::components::ComponentInfo;
using hardware_interface::components::Joint;
using hardware_interface::return_type;

namespace ros2_control_components
{

class PositionJoint : public Joint
{
public:
  return_type configure(const ComponentInfo & joint_info) override
  {
    if (Joint::configure(joint_info) != return_type::OK) {
      return return_type::ERROR;
    }

    // has to provide exactly one command interface if defined
    if (info_.command_interfaces.size() > 1 || info_.state_interfaces.size() > 1) {
      return return_type::COMPONENT_TOO_MANY_INTERFACES;
    }

    if (info_.command_interfaces.size() == 1 &&
      info_.command_interfaces[0].compare(hardware_interface::HW_IF_POSITION))
    {
      return return_type::COMPONENT_WRONG_INTERFACE;
    }

    if (info_.state_interfaces.size() == 1 &&
      info_.state_interfaces[0].compare(hardware_interface::HW_IF_POSITION))
    {
      return return_type::COMPONENT_WRONG_INTERFACE;
    }

    // set default values is not interface defined in URDF.
    // Set state interface to default only if command interface is also not defined, otherwise
    // return error code.
    if (info_.command_interfaces.size() == 0) {
      info_.command_interfaces = {hardware_interface::HW_IF_POSITION};
      commands_.resize(1);
      if (info_.state_interfaces.size() == 0) {
        info_.state_interfaces = {hardware_interface::HW_IF_POSITION};
        states_.resize(1);
      } else {
        return return_type::COMPONENT_ONLY_STATE_DEFINED;
      }
    }

    if (info_.parameters.find("min") == info_.parameters.end()) {
      return return_type::COMPONENT_MISSING_PARAMETER;
    }
    if (info_.parameters.find("max") == info_.parameters.end()) {
      return return_type::COMPONENT_MISSING_PARAMETER;
    }
    lower_limits_.resize(1);
    lower_limits_[0] = stod(info_.parameters["min"]);
    upper_limits_.resize(1);
    upper_limits_[0] = stod(info_.parameters["max"]);

    return return_type::OK;
  }

  return_type set_command(
    const std::vector<double> & command,
    const std::vector<std::string> & interfaces) override
  {
    return hardware_interface::components::set_internal_values_with_limits(
      command, interfaces, info_.command_interfaces, commands_, lower_limits_, upper_limits_);
  }

  return_type set_command(const std::vector<double> & command) override
  {
    return hardware_interface::components::set_internal_values_with_limits(
      command, commands_, lower_limits_, upper_limits_);
  }

private:
  std::vector<double> lower_limits_, upper_limits_;
};

}  // namespace ros2_control_components

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  ros2_control_components::PositionJoint, hardware_interface::components::Joint)
