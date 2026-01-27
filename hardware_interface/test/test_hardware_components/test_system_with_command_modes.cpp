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
#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"

namespace test_hardware_components
{
class TestSystemCommandModes : public hardware_interface::SystemInterface
{
public:
  CallbackReturn on_init(
    const hardware_interface::HardwareComponentInterfaceParams & params) override
  {
    if (hardware_interface::SystemInterface::on_init(params) != CallbackReturn::SUCCESS)
    {
      return CallbackReturn::ERROR;
    }

    // Can only control two joints
    if (get_hardware_info().joints.size() != 2)
    {
      return CallbackReturn::ERROR;
    }
    for (const hardware_interface::ComponentInfo & joint : get_hardware_info().joints)
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

  std::vector<hardware_interface::StateInterface::ConstSharedPtr> on_export_state_interfaces()
    override
  {
    std::vector<hardware_interface::StateInterface::ConstSharedPtr> state_interfaces;
    for (auto i = 0u; i < get_hardware_info().joints.size(); i++)
    {
      position_state_interfaces_[i] = std::make_shared<hardware_interface::StateInterface>(
        get_hardware_info().joints[i].name, hardware_interface::HW_IF_POSITION);
      velocity_state_interfaces_[i] = std::make_shared<hardware_interface::StateInterface>(
        get_hardware_info().joints[i].name, hardware_interface::HW_IF_VELOCITY);
      acceleration_state_interfaces_[i] = std::make_shared<hardware_interface::StateInterface>(
        get_hardware_info().joints[i].name, hardware_interface::HW_IF_ACCELERATION);
      position_state_interfaces_[i]->set_value(0.0);
      velocity_state_interfaces_[i]->set_value(0.0);
      acceleration_state_interfaces_[i]->set_value(0.0);
      state_interfaces.push_back(position_state_interfaces_[i]);
      state_interfaces.push_back(velocity_state_interfaces_[i]);
      state_interfaces.push_back(acceleration_state_interfaces_[i]);
    }

    return state_interfaces;
  }

  std::vector<hardware_interface::CommandInterface::SharedPtr> on_export_command_interfaces()
    override
  {
    std::vector<hardware_interface::CommandInterface::SharedPtr> command_interfaces;
    for (auto i = 0u; i < get_hardware_info().joints.size(); i++)
    {
      position_command_interfaces_[i] = std::make_shared<hardware_interface::CommandInterface>(
        get_hardware_info().joints[i].name, hardware_interface::HW_IF_POSITION);
      velocity_command_interfaces_[i] = std::make_shared<hardware_interface::CommandInterface>(
        get_hardware_info().joints[i].name, hardware_interface::HW_IF_VELOCITY);
      position_command_interfaces_[i]->set_value(0.0);
      velocity_command_interfaces_[i]->set_value(0.0);
      command_interfaces.push_back(position_command_interfaces_[i]);
      command_interfaces.push_back(velocity_command_interfaces_[i]);
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
    double accel = 0.0;
    (void)acceleration_state_interfaces_[0]->get_value(accel, true);
    (void)acceleration_state_interfaces_[0]->set_value(accel + 1.0, true);

    // Starting interfaces
    start_modes_.clear();
    stop_modes_.clear();
    for (const auto & key : start_interfaces)
    {
      for (auto i = 0u; i < get_hardware_info().joints.size(); i++)
      {
        if (key == get_hardware_info().joints[i].name + "/" + hardware_interface::HW_IF_POSITION)
        {
          start_modes_.push_back(hardware_interface::HW_IF_POSITION);
        }
        if (key == get_hardware_info().joints[i].name + "/" + hardware_interface::HW_IF_VELOCITY)
        {
          start_modes_.push_back(hardware_interface::HW_IF_VELOCITY);
        }
      }
    }
    // Example Criteria 1 - Starting: All interfaces must be given a new mode at the same time
    if (start_modes_.size() != 0 && start_modes_.size() != get_hardware_info().joints.size())
    {
      return hardware_interface::return_type::ERROR;
    }

    // Stopping interfaces
    for (const auto & key : stop_interfaces)
    {
      for (auto i = 0u; i < get_hardware_info().joints.size(); i++)
      {
        if (key.find(get_hardware_info().joints[i].name) != std::string::npos)
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
    double accel = 0.0;
    (void)acceleration_state_interfaces_[0]->get_value(accel, true);
    (void)acceleration_state_interfaces_[0]->set_value(accel + 100.0, true);
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

  std::array<hardware_interface::CommandInterface::SharedPtr, 2> position_command_interfaces_;
  std::array<hardware_interface::CommandInterface::SharedPtr, 2> velocity_command_interfaces_;
  std::array<hardware_interface::StateInterface::SharedPtr, 2> position_state_interfaces_;
  std::array<hardware_interface::StateInterface::SharedPtr, 2> velocity_state_interfaces_;
  std::array<hardware_interface::StateInterface::SharedPtr, 2> acceleration_state_interfaces_;
};

}  // namespace test_hardware_components

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  test_hardware_components::TestSystemCommandModes, hardware_interface::SystemInterface)
