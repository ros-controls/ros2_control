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
#include "ros2_control_test_assets/test_hardware_interface_constants.hpp"

using hardware_interface::ActuatorInterface;
using hardware_interface::CommandInterface;
using hardware_interface::return_type;
using hardware_interface::StateInterface;

class TestActuator : public ActuatorInterface
{
  CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override
  {
    if (ActuatorInterface::on_init(info) != CallbackReturn::SUCCESS)
    {
      return CallbackReturn::ERROR;
    }

    if (get_hardware_info().joints[0].state_interfaces[1].name == "does_not_exist")
    {
      return CallbackReturn::ERROR;
    }

    /*
     * a hardware can optional prove for incorrect info here.
     *
     * // can only control one joint
     * if (get_hardware_info().joints.size() != 1) {return CallbackReturn::ERROR;}
     * // can only control in position
     * if (get_hardware_info().joints[0].command_interfaces.size() != 1) {return
     * CallbackReturn::ERROR;}
     * // can only give feedback state for position and velocity
     * if (get_hardware_info().joints[0].state_interfaces.size() != 2) {return
     * CallbackReturn::ERROR;}
     */

    return CallbackReturn::SUCCESS;
  }

  std::vector<StateInterface> export_state_interfaces() override
  {
    std::vector<StateInterface> state_interfaces;
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      get_hardware_info().joints[0].name, get_hardware_info().joints[0].state_interfaces[0].name,
      &position_state_));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      get_hardware_info().joints[0].name, get_hardware_info().joints[0].state_interfaces[1].name,
      &velocity_state_));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      get_hardware_info().joints[0].name, "some_unlisted_interface", nullptr));

    return state_interfaces;
  }

  std::vector<CommandInterface> export_command_interfaces() override
  {
    std::vector<CommandInterface> command_interfaces;
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      get_hardware_info().joints[0].name, get_hardware_info().joints[0].command_interfaces[0].name,
      &velocity_command_));

    if (get_hardware_info().joints[0].command_interfaces.size() > 1)
    {
      command_interfaces.emplace_back(hardware_interface::CommandInterface(
        get_hardware_info().joints[0].name,
        get_hardware_info().joints[0].command_interfaces[1].name, &max_velocity_command_));
    }

    return command_interfaces;
  }

  hardware_interface::return_type prepare_command_mode_switch(
    const std::vector<std::string> & /*start_interfaces*/,
    const std::vector<std::string> & /*stop_interfaces*/) override
  {
    position_state_ += 1.0;
    return hardware_interface::return_type::OK;
  }

  hardware_interface::return_type perform_command_mode_switch(
    const std::vector<std::string> & /*start_interfaces*/,
    const std::vector<std::string> & /*stop_interfaces*/) override
  {
    position_state_ += 100.0;
    return hardware_interface::return_type::OK;
  }

  return_type read(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) override
  {
    // simulate error on read
    if (velocity_command_ == test_constants::READ_FAIL_VALUE)
    {
      // reset value to get out from error on the next call - simplifies CM
      // tests
      velocity_command_ = 0.0;
      return return_type::ERROR;
    }
    // simulate deactivate on read
    if (velocity_command_ == test_constants::READ_DEACTIVATE_VALUE)
    {
      return return_type::DEACTIVATE;
    }
    // The next line is for the testing purposes. We need value to be changed to
    // be sure that the feedback from hardware to controllers in the chain is
    // working as it should. This makes value checks clearer and confirms there
    // is no "state = command" line or some other mixture of interfaces
    // somewhere in the test stack.
    velocity_state_ = velocity_command_ / 2;
    return return_type::OK;
  }

  return_type write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) override
  {
    // simulate error on write
    if (velocity_command_ == test_constants::WRITE_FAIL_VALUE)
    {
      // reset value to get out from error on the next call - simplifies CM
      // tests
      velocity_command_ = 0.0;
      return return_type::ERROR;
    }
    // simulate deactivate on write
    if (velocity_command_ == test_constants::WRITE_DEACTIVATE_VALUE)
    {
      return return_type::DEACTIVATE;
    }
    return return_type::OK;
  }

private:
  double position_state_ = 0.0;
  double velocity_state_ = 0.0;
  double velocity_command_ = 0.0;
  double max_velocity_command_ = 0.0;
};

class TestUninitializableActuator : public TestActuator
{
  CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override
  {
    ActuatorInterface::on_init(info);
    return CallbackReturn::ERROR;
  }
};

#include "pluginlib/class_list_macros.hpp"  // NOLINT
PLUGINLIB_EXPORT_CLASS(TestActuator, hardware_interface::ActuatorInterface)
PLUGINLIB_EXPORT_CLASS(TestUninitializableActuator, hardware_interface::ActuatorInterface)
