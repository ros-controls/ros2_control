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

#include "hardware_interface/components/system_interface.hpp"

using hardware_interface::status;
using hardware_interface::return_type;
using hardware_interface::StateInterface;
using hardware_interface::CommandInterface;

class TestSystem : public hardware_interface::components::SystemInterface
{
  return_type configure(const hardware_interface::HardwareInfo & system_info) override
  {
    system_info_ = system_info;
    return return_type::OK;
  }

  std::vector<StateInterface> export_state_interfaces() override
  {
    std::vector<StateInterface> state_interfaces;
    for (auto i = 0u; i < system_info_.joints.size(); ++i) {
      state_interfaces.emplace_back(
        hardware_interface::StateInterface(
          system_info_.joints[i].name, "position", &position_state_[i]));
      state_interfaces.emplace_back(
        hardware_interface::StateInterface(
          system_info_.joints[i].name, "velocity", &velocity_state_[i]));
      state_interfaces.emplace_back(
        hardware_interface::StateInterface(
          system_info_.joints[i].name, "acceleration", &acceleration_state_[i]));
    }

    return state_interfaces;
  }

  std::vector<CommandInterface> export_command_interfaces() override
  {
    std::vector<CommandInterface> command_interfaces;
    for (auto i = 0u; i < system_info_.joints.size(); ++i) {
      command_interfaces.emplace_back(
        hardware_interface::CommandInterface(
          system_info_.joints[i].name, "velocity", &velocity_command_[i]));
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

  status get_status() const override
  {
    return status::UNKNOWN;
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
  std::array<double, 2> velocity_command_ = {0.0, 0.0};
  std::array<double, 2> position_state_ = {0.0, 0.0};
  std::array<double, 2> velocity_state_ = {0.0, 0.0};
  std::array<double, 2> acceleration_state_ = {0.0, 0.0};
  hardware_interface::HardwareInfo system_info_;
};

#include "pluginlib/class_list_macros.hpp"  // NOLINT
PLUGINLIB_EXPORT_CLASS(TestSystem, hardware_interface::components::SystemInterface)
