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

#include "rcutils/logging_macros.h"

using hardware_interface::CommandInterface;
using hardware_interface::return_type;
using hardware_interface::StateInterface;
using hardware_interface::SystemInterface;

class TestSystem : public SystemInterface
{
  std::vector<StateInterface> export_state_interfaces() override
  {
    std::vector<StateInterface> state_interfaces;
    for (auto i = 0u; i < info_.joints.size(); ++i)
    {
      if (info_.joints[i].name != "configuration")
      {
        state_interfaces.emplace_back(hardware_interface::StateInterface(
          info_.joints[i].name, hardware_interface::HW_IF_POSITION, &position_state_[i]));
        state_interfaces.emplace_back(hardware_interface::StateInterface(
          info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &velocity_state_[i]));
        state_interfaces.emplace_back(hardware_interface::StateInterface(
          info_.joints[i].name, hardware_interface::HW_IF_ACCELERATION, &acceleration_state_[i]));
      }
    }

    if (info_.joints.size() > 2)
    {
      // Add configuration/max_tcp_jerk interface
      state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.joints[2].name, info_.joints[2].state_interfaces[0].name, &configuration_state_));
    }

    return state_interfaces;
  }

  std::vector<CommandInterface> export_command_interfaces() override
  {
    RCUTILS_LOG_INFO_NAMED("test_system", "Exporting configuration interfaces.");
    std::vector<CommandInterface> command_interfaces;
    for (auto i = 0u; i < info_.joints.size(); ++i)
    {
      if (info_.joints[i].name != "configuration")
      {
        command_interfaces.emplace_back(hardware_interface::CommandInterface(
          info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &velocity_command_[i]));
      }
    }
    // Add max_acceleration command interface
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[0].name, info_.joints[0].command_interfaces[1].name,
      &max_acceleration_command_));

    if (info_.joints.size() > 2)
    {
      // Add configuration/max_tcp_jerk interface
      command_interfaces.emplace_back(hardware_interface::CommandInterface(
        info_.joints[2].name, info_.joints[2].command_interfaces[0].name, &configuration_command_));
    }

    return command_interfaces;
  }

  return_type read(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) override
  {
    // simulate error on read
    if (velocity_command_[0] == 28282828)
    {
      // reset value to get out from error on the next call - simplifies CM tests
      velocity_command_[0] = 0.0;
      return return_type::ERROR;
    }
    return return_type::OK;
  }

  return_type write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) override
  {
    // simulate error on write
    if (velocity_command_[0] == 23232323)
    {
      // reset value to get out from error on the next call - simplifies CM tests
      velocity_command_[0] = 0.0;
      return return_type::ERROR;
    }
    return return_type::OK;
  }

private:
  std::array<double, 2> velocity_command_ = {0.0, 0.0};
  std::array<double, 2> position_state_ = {0.0, 0.0};
  std::array<double, 2> velocity_state_ = {0.0, 0.0};
  std::array<double, 2> acceleration_state_ = {0.0, 0.0};
  double max_acceleration_command_ = 0.0;
  double configuration_state_ = 0.0;
  double configuration_command_ = 0.0;
};

class TestUnitilizableSystem : public TestSystem
{
  CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override
  {
    SystemInterface::on_init(info);
    return CallbackReturn::ERROR;
  }
};

#include "pluginlib/class_list_macros.hpp"  // NOLINT
PLUGINLIB_EXPORT_CLASS(TestSystem, hardware_interface::SystemInterface)
PLUGINLIB_EXPORT_CLASS(TestUnitilizableSystem, hardware_interface::SystemInterface)
