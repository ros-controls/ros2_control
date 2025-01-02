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
#include <vector>

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"

using hardware_interface::CommandInterface;
using hardware_interface::return_type;
using hardware_interface::StateInterface;
using hardware_interface::SystemInterface;

namespace test_hardware_components
{
class TestTwoJointSystem : public SystemInterface
{
  CallbackReturn on_init(const hardware_interface::HardwareInfo & system_info) override
  {
    if (SystemInterface::on_init(system_info) != CallbackReturn::SUCCESS)
    {
      return CallbackReturn::ERROR;
    }

    // can only control two joint
    if (get_hardware_info().joints.size() != 2)
    {
      return CallbackReturn::ERROR;
    }
    for (const auto & joint : get_hardware_info().joints)
    {
      // can only control in position
      const auto & command_interfaces = joint.command_interfaces;
      if (command_interfaces.size() != 1)
      {
        return CallbackReturn::ERROR;
      }
      if (command_interfaces[0].name != hardware_interface::HW_IF_POSITION)
      {
        return CallbackReturn::ERROR;
      }
      // can only give feedback state for position and velocity
      const auto & state_interfaces = joint.state_interfaces;
      if (state_interfaces.size() != 1)
      {
        return CallbackReturn::ERROR;
      }
      if (state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
      {
        return CallbackReturn::ERROR;
      }
    }

    fprintf(stderr, "TestTwoJointSystem configured successfully.\n");
    return CallbackReturn::SUCCESS;
  }

  std::vector<StateInterface> export_state_interfaces() override
  {
    std::vector<StateInterface> state_interfaces;
    for (auto i = 0u; i < get_hardware_info().joints.size(); ++i)
    {
      state_interfaces.emplace_back(hardware_interface::StateInterface(
        get_hardware_info().joints[i].name, hardware_interface::HW_IF_POSITION,
        &position_state_[i]));
    }

    return state_interfaces;
  }

  std::vector<CommandInterface> export_command_interfaces() override
  {
    std::vector<CommandInterface> command_interfaces;
    for (auto i = 0u; i < get_hardware_info().joints.size(); ++i)
    {
      command_interfaces.emplace_back(hardware_interface::CommandInterface(
        get_hardware_info().joints[i].name, hardware_interface::HW_IF_POSITION,
        &position_command_[i]));
    }

    return command_interfaces;
  }

  return_type read(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) override
  {
    return return_type::OK;
  }

  return_type write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) override
  {
    return return_type::OK;
  }

private:
  std::array<double, 2> position_command_ = {{0.0, 0.0}};
  std::array<double, 2> position_state_ = {{0.0, 0.0}};
};

}  // namespace test_hardware_components

#include "pluginlib/class_list_macros.hpp"  // NOLINT
PLUGINLIB_EXPORT_CLASS(
  test_hardware_components::TestTwoJointSystem, hardware_interface::SystemInterface)
