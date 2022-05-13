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

#include <memory>
#include <vector>

#include "hardware_interface/actuator_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"

using hardware_interface::ActuatorInterface;
using hardware_interface::CommandInterface;
using hardware_interface::return_type;
using hardware_interface::StateInterface;

namespace test_hardware_components
{
class TestSingleJointActuator : public ActuatorInterface
{
  CallbackReturn on_init(const hardware_interface::HardwareInfo & actuator_info) override
  {
    if (ActuatorInterface::on_init(actuator_info) != CallbackReturn::SUCCESS)
    {
      return CallbackReturn::ERROR;
    }

    // can only control one joint
    if (info_.joints.size() != 1)
    {
      return CallbackReturn::ERROR;
    }
    // can only control in position
    const auto & command_interfaces = info_.joints[0].command_interfaces;
    if (command_interfaces.size() != 1)
    {
      return CallbackReturn::ERROR;
    }
    if (command_interfaces[0].name != hardware_interface::HW_IF_POSITION)
    {
      return CallbackReturn::ERROR;
    }
    // can only give feedback state for position and velocity
    const auto & state_interfaces = info_.joints[0].state_interfaces;
    if (state_interfaces.size() < 1)
    {
      return CallbackReturn::ERROR;
    }
    for (const auto & state_interface : state_interfaces)
    {
      if (
        (state_interface.name != hardware_interface::HW_IF_POSITION) &&
        (state_interface.name != hardware_interface::HW_IF_VELOCITY))
      {
        return CallbackReturn::ERROR;
      }
    }
    fprintf(stderr, "TestSingleJointActuator configured successfully.\n");
    return CallbackReturn::SUCCESS;
  }

  std::vector<StateInterface> export_state_interfaces() override
  {
    std::vector<StateInterface> state_interfaces;

    const auto & joint_name = info_.joints[0].name;
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      joint_name, hardware_interface::HW_IF_POSITION, &position_state_));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      joint_name, hardware_interface::HW_IF_VELOCITY, &velocity_state_));

    return state_interfaces;
  }

  std::vector<CommandInterface> export_command_interfaces() override
  {
    std::vector<CommandInterface> command_interfaces;

    const auto & joint_name = info_.joints[0].name;
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      joint_name, hardware_interface::HW_IF_POSITION, &position_command_));

    return command_interfaces;
  }

  return_type read(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) override
  {
    return return_type::OK;
  }

  return_type write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) override
  {
    velocity_state_ = position_command_ - position_state_;
    position_state_ = position_command_;
    return return_type::OK;
  }

private:
  double position_state_ = 0.0;
  double velocity_state_ = 0.0;
  double position_command_ = 0.0;
};

}  // namespace test_hardware_components

#include "pluginlib/class_list_macros.hpp"  // NOLINT
PLUGINLIB_EXPORT_CLASS(
  test_hardware_components::TestSingleJointActuator, hardware_interface::ActuatorInterface)
