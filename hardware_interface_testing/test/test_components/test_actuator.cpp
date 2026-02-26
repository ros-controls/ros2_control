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
#include "rclcpp/logging.hpp"
#include "ros2_control_test_assets/test_hardware_interface_constants.hpp"

using hardware_interface::ActuatorInterface;
using hardware_interface::CommandInterface;
using hardware_interface::return_type;
using hardware_interface::StateInterface;

class TestActuator : public ActuatorInterface
{
  CallbackReturn on_init(
    const hardware_interface::HardwareComponentInterfaceParams & params) override
  {
    if (ActuatorInterface::on_init(params) != CallbackReturn::SUCCESS)
    {
      return CallbackReturn::ERROR;
    }

    if (get_hardware_info().joints[0].state_interfaces[1].name == "does_not_exist")
    {
      return CallbackReturn::ERROR;
    }
    if (get_hardware_info().rw_rate == 0u)
    {
      RCLCPP_WARN(
        get_logger(),
        "Actuator hardware component '%s' from plugin '%s' failed to initialize as rw_rate is 0.",
        get_hardware_info().name.c_str(), get_hardware_info().hardware_plugin_name.c_str());
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

  std::vector<StateInterface::ConstSharedPtr> on_export_state_interfaces() override
  {
    std::vector<StateInterface::ConstSharedPtr> state_interfaces;
    position_state_ = std::make_shared<StateInterface>(
      get_hardware_info().joints[0].name, get_hardware_info().joints[0].state_interfaces[0].name);
    (void)position_state_->set_value(0.0, false);
    velocity_state_ = std::make_shared<StateInterface>(
      get_hardware_info().joints[0].name, get_hardware_info().joints[0].state_interfaces[1].name);
    (void)velocity_state_->set_value(0.0, false);
    unlisted_interface_ = std::make_shared<StateInterface>(
      get_hardware_info().joints[0].name, "some_unlisted_interface");
    state_interfaces.push_back(position_state_);
    state_interfaces.push_back(velocity_state_);
    state_interfaces.push_back(unlisted_interface_);

    return state_interfaces;
  }

  std::vector<CommandInterface::SharedPtr> on_export_command_interfaces() override
  {
    std::vector<CommandInterface::SharedPtr> command_interfaces;
    velocity_command_ = std::make_shared<CommandInterface>(
      get_hardware_info().joints[0].name, get_hardware_info().joints[0].command_interfaces[0].name);
    (void)velocity_command_->set_value(0.0, false);
    command_interfaces.push_back(velocity_command_);

    if (get_hardware_info().joints[0].command_interfaces.size() > 1)
    {
      max_velocity_command_ = std::make_shared<CommandInterface>(
        get_hardware_info().joints[0].name,
        get_hardware_info().joints[0].command_interfaces[1].name);
      (void)max_velocity_command_->set_value(0.0, false);
      command_interfaces.push_back(max_velocity_command_);
    }

    return command_interfaces;
  }

  hardware_interface::return_type prepare_command_mode_switch(
    const std::vector<std::string> & /*start_interfaces*/,
    const std::vector<std::string> & /*stop_interfaces*/) override
  {
    double pos = 0.0;
    (void)position_state_->get_value(pos, false);
    (void)position_state_->set_value(pos + 0.001, false);
    return hardware_interface::return_type::OK;
  }

  hardware_interface::return_type perform_command_mode_switch(
    const std::vector<std::string> & /*start_interfaces*/,
    const std::vector<std::string> & /*stop_interfaces*/) override
  {
    if (get_hardware_info().hardware_parameters.count("fail_on_perform_mode_switch"))
    {
      if (hardware_interface::parse_bool(
            get_hardware_info().hardware_parameters.at("fail_on_perform_mode_switch")))
      {
        return hardware_interface::return_type::ERROR;
      }
    }
    double pos = 0.0;
    (void)position_state_->get_value(pos, false);
    (void)position_state_->set_value(pos + 0.1, false);
    return hardware_interface::return_type::OK;
  }

  return_type read(const rclcpp::Time & /*time*/, const rclcpp::Duration & period) override
  {
    if (get_hardware_info().is_async)
    {
      std::this_thread::sleep_for(
        std::chrono::milliseconds(1000 / (3 * get_hardware_info().rw_rate)));
    }
    double vel_cmd = 0.0;
    (void)velocity_command_->get_value(vel_cmd, false);
    // simulate error on read
    if (vel_cmd == test_constants::READ_FAIL_VALUE)
    {
      // reset value to get out from error on the next call - simplifies CM
      // tests
      (void)velocity_command_->set_value(0.0, false);
      return return_type::ERROR;
    }
    // simulate deactivate on read
    if (vel_cmd == test_constants::READ_DEACTIVATE_VALUE)
    {
      return return_type::DEACTIVATE;
    }
    // The next line is for the testing purposes. We need value to be changed to
    // be sure that the feedback from hardware to controllers in the chain is
    // working as it should. This makes value checks clearer and confirms there
    // is no "state = command" line or some other mixture of interfaces
    // somewhere in the test stack.
    double vel_state = vel_cmd / 2;
    (void)velocity_state_->set_value(vel_state, false);
    double pos_state = 0.0;
    (void)position_state_->get_value(pos_state, false);
    (void)position_state_->set_value(pos_state + vel_state * period.seconds(), false);

    if (vel_cmd == test_constants::RESET_STATE_INTERFACES_VALUE)
    {
      (void)position_state_->set_value(0.0, false);
      (void)velocity_state_->set_value(0.0, false);
    }
    return return_type::OK;
  }

  return_type write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) override
  {
    if (get_hardware_info().is_async)
    {
      std::this_thread::sleep_for(
        std::chrono::milliseconds(1000 / (6 * get_hardware_info().rw_rate)));
    }
    double vel_cmd = 0.0;
    (void)velocity_command_->get_value(vel_cmd, false);
    // simulate error on write
    if (vel_cmd == test_constants::WRITE_FAIL_VALUE)
    {
      // reset value to get out from error on the next call - simplifies CM
      // tests
      (void)velocity_command_->set_value(0.0, false);
      return return_type::ERROR;
    }
    // simulate deactivate on write
    if (vel_cmd == test_constants::WRITE_DEACTIVATE_VALUE)
    {
      return return_type::DEACTIVATE;
    }
    return return_type::OK;
  }

private:
  StateInterface::SharedPtr position_state_;
  StateInterface::SharedPtr velocity_state_;
  CommandInterface::SharedPtr velocity_command_;
  CommandInterface::SharedPtr max_velocity_command_;
  StateInterface::SharedPtr unlisted_interface_;
};

class TestUninitializableActuator : public TestActuator
{
  CallbackReturn on_init(
    const hardware_interface::HardwareComponentInterfaceParams & params) override
  {
    ActuatorInterface::on_init(params);
    return CallbackReturn::ERROR;
  }
};

#include "pluginlib/class_list_macros.hpp"  // NOLINT
PLUGINLIB_EXPORT_CLASS(TestActuator, hardware_interface::ActuatorInterface)
PLUGINLIB_EXPORT_CLASS(TestUninitializableActuator, hardware_interface::ActuatorInterface)
