// Copyright 2021 Department of Engineering Cybernetics, NTNU
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
#include <string>
#include <vector>

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"

namespace test_hardware_components
{
class TestSystemCommandModes : public hardware_interface::SystemInterface
{
public:
  CallbackReturn on_init(const hardware_interface::HardwareInfo & system_info) override
  {
    if (hardware_interface::SystemInterface::on_init(system_info) != CallbackReturn::SUCCESS)
    {
      return CallbackReturn::ERROR;
    }

    // Can only control two joints
    if (info_.joints.size() != 2)
    {
      return CallbackReturn::ERROR;
    }
    for (const hardware_interface::ComponentInfo & joint : info_.joints)
    {
      // Can control in position or velocity
      const auto & command_interfaces = joint.command_interfaces;
      if (command_interfaces.size() != 2)
      {
        return CallbackReturn::ERROR;
      }
      for (const auto & command_interface : command_interfaces)
      {
        if (
          command_interface.name != hardware_interface::HW_IF_POSITION &&
          command_interface.name != hardware_interface::HW_IF_VELOCITY)
        {
          return CallbackReturn::ERROR;
        }
      }
      // Can give feedback state for position, velocity, and acceleration
      const auto & state_interfaces = joint.state_interfaces;
      if (state_interfaces.size() != 2)
      {
        return CallbackReturn::ERROR;
      }
      for (const auto & state_interface : state_interfaces)
      {
        if (
          state_interface.name != hardware_interface::HW_IF_POSITION &&
          state_interface.name != hardware_interface::HW_IF_VELOCITY)
        {
          return CallbackReturn::ERROR;
        }
      }
    }

    fprintf(stderr, "TestSystemCommandModes configured successfully.\n");
    return CallbackReturn::SUCCESS;
  }

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override
  {
    std::vector<hardware_interface::StateInterface> state_interfaces;
    for (auto i = 0u; i < info_.joints.size(); i++)
    {
      state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION, &position_state_[i]));
      state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &velocity_state_[i]));
      state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_ACCELERATION, &acceleration_state_[i]));
    }

    return state_interfaces;
  }

  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override
  {
    std::vector<hardware_interface::CommandInterface> command_interfaces;
    for (auto i = 0u; i < info_.joints.size(); i++)
    {
      command_interfaces.emplace_back(hardware_interface::CommandInterface(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION, &position_command_[i]));
      command_interfaces.emplace_back(hardware_interface::CommandInterface(
        info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &velocity_command_[i]));
    }

    return command_interfaces;
  }

  hardware_interface::return_type read(
    const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) override
  {
    return hardware_interface::return_type::OK;
  }

  hardware_interface::return_type write(
    const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) override
  {
    return hardware_interface::return_type::OK;
  }

  hardware_interface::return_type prepare_command_mode_switch(
    const std::vector<std::string> & start_interfaces,
    const std::vector<std::string> & stop_interfaces) override
  {
    acceleration_state_[0] += 1.0;

    // Starting interfaces
    start_modes_.clear();
    stop_modes_.clear();
    for (const auto & key : start_interfaces)
    {
      for (auto i = 0u; i < info_.joints.size(); i++)
      {
        if (key == info_.joints[i].name + "/" + hardware_interface::HW_IF_POSITION)
        {
          start_modes_.push_back(hardware_interface::HW_IF_POSITION);
        }
        if (key == info_.joints[i].name + "/" + hardware_interface::HW_IF_VELOCITY)
        {
          start_modes_.push_back(hardware_interface::HW_IF_VELOCITY);
        }
      }
    }
    // Example Criteria 1 - Starting: All interfaces must be given a new mode at the same time
    if (start_modes_.size() != 0 && start_modes_.size() != info_.joints.size())
    {
      return hardware_interface::return_type::ERROR;
    }

    // Stopping interfaces
    for (const auto & key : stop_interfaces)
    {
      for (auto i = 0u; i < info_.joints.size(); i++)
      {
        if (key.find(info_.joints[i].name) != std::string::npos)
        {
          stop_modes_.push_back(true);
        }
      }
    }
    // Example Criteria 2 - Stopping: All joints must have the same command mode
    if (stop_modes_.size() != 0 && stop_modes_.size() != 2 && stop_modes_[0] != stop_modes_[1])
    {
      return hardware_interface::return_type::ERROR;
    }
    return hardware_interface::return_type::OK;
  }

  hardware_interface::return_type perform_command_mode_switch(
    const std::vector<std::string> & start_interfaces,
    const std::vector<std::string> & /*stop_interfaces*/) override
  {
    acceleration_state_[0] += 100.0;
    // Test of failure in perform command mode switch
    // Fail if given an empty list.
    // This should never occur in a real system as the same start_interfaces list is sent to both
    // prepare and perform, and an error should be handled in prepare.
    if (start_interfaces.size() == 0)
    {
      return hardware_interface::return_type::ERROR;
    }
    return hardware_interface::return_type::OK;
  }

private:
  std::vector<std::string> start_modes_ = {"position", "position"};
  std::vector<bool> stop_modes_ = {false, false};

  std::array<double, 2> position_command_ = {0.0, 0.0};
  std::array<double, 2> velocity_command_ = {0.0, 0.0};
  std::array<double, 2> position_state_ = {0.0, 0.0};
  std::array<double, 2> velocity_state_ = {0.0, 0.0};
  std::array<double, 2> acceleration_state_ = {0.0, 0.0};
};

}  // namespace test_hardware_components

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  test_hardware_components::TestSystemCommandModes, hardware_interface::SystemInterface)
