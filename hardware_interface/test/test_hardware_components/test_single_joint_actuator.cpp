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
  CallbackReturn on_init(
    const hardware_interface::HardwareComponentInterfaceParams & params) override
  {
    if (ActuatorInterface::on_init(params) != CallbackReturn::SUCCESS)
    {
      return CallbackReturn::ERROR;
    }

    // can only control one joint
    if (get_hardware_info().joints.size() != 1)
    {
      return CallbackReturn::ERROR;
    }
    // can only control in position
    const auto & command_interfaces = get_hardware_info().joints[0].command_interfaces;
    if (command_interfaces.size() != 1)
    {
      return CallbackReturn::ERROR;
    }
    if (command_interfaces[0].name != hardware_interface::HW_IF_POSITION)
    {
      return CallbackReturn::ERROR;
    }
    // can only give feedback state for position and velocity
    const auto & state_interfaces = get_hardware_info().joints[0].state_interfaces;
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

  std::vector<StateInterface::ConstSharedPtr> on_export_state_interfaces() override
  {
    const auto & joint_name = get_hardware_info().joints[0].name;
    position_state_interface_ =
      std::make_shared<StateInterface>(joint_name, hardware_interface::HW_IF_POSITION);
    velocity_state_interface_ =
      std::make_shared<StateInterface>(joint_name, hardware_interface::HW_IF_VELOCITY);
    return {position_state_interface_, velocity_state_interface_};
  }

  std::vector<CommandInterface::SharedPtr> on_export_command_interfaces() override
  {
    const auto & joint_name = get_hardware_info().joints[0].name;
    position_command_interface_ =
      std::make_shared<CommandInterface>(joint_name, hardware_interface::HW_IF_POSITION);
    return {position_command_interface_};
  }

  return_type read(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) override
  {
    return return_type::OK;
  }

  return_type write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) override
  {
    double position_state = 0.0;
    double position_command = 0.0;
    (void)position_state_interface_->get_value(position_state, true);
    (void)position_command_interface_->get_value(position_command, true);
    (void)velocity_state_interface_->set_value(position_command - position_state, true);
    (void)position_state_interface_->set_value(position_command, true);
    return return_type::OK;
  }

private:
  StateInterface::SharedPtr position_state_interface_;
  StateInterface::SharedPtr velocity_state_interface_;
  CommandInterface::SharedPtr position_command_interface_;
};

}  // namespace test_hardware_components

#include "pluginlib/class_list_macros.hpp"  // NOLINT
PLUGINLIB_EXPORT_CLASS(
  test_hardware_components::TestSingleJointActuator, hardware_interface::ActuatorInterface)
