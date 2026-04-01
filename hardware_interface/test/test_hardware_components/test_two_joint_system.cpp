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
  CallbackReturn on_init(
    const hardware_interface::HardwareComponentInterfaceParams & params) override
  {
    if (SystemInterface::on_init(params) != CallbackReturn::SUCCESS)
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

  std::vector<StateInterface::ConstSharedPtr> on_export_state_interfaces() override
  {
    std::vector<StateInterface::ConstSharedPtr> state_interfaces;
    for (auto i = 0u; i < get_hardware_info().joints.size(); ++i)
    {
      position_state_interfaces_[i] = std::make_shared<StateInterface>(
        get_hardware_info().joints[i].name, hardware_interface::HW_IF_POSITION);
      state_interfaces.push_back(position_state_interfaces_[i]);
    }
    return state_interfaces;
  }

  std::vector<CommandInterface::SharedPtr> on_export_command_interfaces() override
  {
    std::vector<CommandInterface::SharedPtr> command_interfaces;
    for (auto i = 0u; i < get_hardware_info().joints.size(); ++i)
    {
      position_command_interfaces_[i] = std::make_shared<CommandInterface>(
        get_hardware_info().joints[i].name, hardware_interface::HW_IF_POSITION);
      command_interfaces.push_back(position_command_interfaces_[i]);
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
  std::array<StateInterface::SharedPtr, 2> position_state_interfaces_;
  std::array<CommandInterface::SharedPtr, 2> position_command_interfaces_;
};

}  // namespace test_hardware_components

#include "pluginlib/class_list_macros.hpp"  // NOLINT
PLUGINLIB_EXPORT_CLASS(
  test_hardware_components::TestTwoJointSystem, hardware_interface::SystemInterface)
