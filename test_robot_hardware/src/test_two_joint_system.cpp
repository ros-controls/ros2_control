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

#include <array>
#include <memory>
#include <vector>

#include "hardware_interface/components/base_interface.hpp"
#include "hardware_interface/components/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"

using hardware_interface::status;
using hardware_interface::return_type;
using hardware_interface::StateInterface;
using hardware_interface::CommandInterface;
using hardware_interface::components::BaseInterface;
using hardware_interface::components::SystemInterface;

namespace test_robot_hardware
{

class TestTwoJointSystem : public BaseInterface<SystemInterface>
{
  return_type configure(const hardware_interface::HardwareInfo & system_info) override
  {
    if (configure_default(system_info) != return_type::OK) {
      return return_type::ERROR;
    }

    // can only control two joint
    if (info_.joints.size() != 2) {return return_type::ERROR;}
    for (const auto & joint : info_.joints) {
      // can only control in position
      const auto & command_interfaces = joint.command_interfaces;
      if (command_interfaces.size() != 1) {return return_type::ERROR;}
      if (command_interfaces[0].name != hardware_interface::HW_IF_POSITION) {
        return return_type::ERROR;
      }
      // can only give feedback state for position and velocity
      const auto & state_interfaces = joint.state_interfaces;
      if (state_interfaces.size() != 1) {return return_type::ERROR;}
      if (state_interfaces[0].name != hardware_interface::HW_IF_POSITION) {
        return return_type::ERROR;
      }
    }

    fprintf(stderr, "TestTwoJointSystem configured successfully.\n");
    return return_type::OK;
  }

  std::vector<StateInterface> export_state_interfaces() override
  {
    std::vector<StateInterface> state_interfaces;
    for (auto i = 0u; i < info_.joints.size(); ++i) {
      state_interfaces.emplace_back(
        hardware_interface::StateInterface(
          info_.joints[i].name, hardware_interface::HW_IF_POSITION, &position_state_[i]));
    }

    return state_interfaces;
  }

  std::vector<CommandInterface> export_command_interfaces() override
  {
    std::vector<CommandInterface> command_interfaces;
    for (auto i = 0u; i < info_.joints.size(); ++i) {
      command_interfaces.emplace_back(
        hardware_interface::CommandInterface(
          info_.joints[i].name, hardware_interface::HW_IF_POSITION, &position_command_[i]));
    }

    return command_interfaces;
  }

  return_type start() override
  {
    return return_type::OK;
  }

  return_type stop() override
  {
    return return_type::OK;
  }

  return_type read() override
  {
    return return_type::OK;
  }

  return_type write() override
  {
    return return_type::OK;
  }

private:
  std::array<double, 2> position_command_ = {0.0, 0.0};
  std::array<double, 2> position_state_ = {0.0, 0.0};
};

}  // namespace test_robot_hardware

#include "pluginlib/class_list_macros.hpp"  // NOLINT
PLUGINLIB_EXPORT_CLASS(
  test_robot_hardware::TestTwoJointSystem, hardware_interface::components::SystemInterface)
