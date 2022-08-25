// Copyright 2020 Open Source Robotics Foundation, Inc.
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

#include "test_controller.hpp"

#include <memory>
#include <string>

#include "lifecycle_msgs/msg/state.hpp"

namespace test_controller
{
TestController::TestController()
: controller_interface::ControllerInterface(),
  cmd_iface_cfg_{controller_interface::interface_configuration_type::NONE}
{
}

controller_interface::InterfaceConfiguration TestController::command_interface_configuration() const
{
  if (
    get_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE ||
    get_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE)
  {
    return cmd_iface_cfg_;
  }
  else
  {
    throw std::runtime_error(
      "Can not get command interface configuration until the controller is configured.");
  }
}

controller_interface::InterfaceConfiguration TestController::state_interface_configuration() const
{
  if (
    get_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE ||
    get_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE)
  {
    return state_iface_cfg_;
  }
  else
  {
    throw std::runtime_error(
      "Can not get state interface configuration until the controller is configured.");
  }
}

controller_interface::return_type TestController::update(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  ++internal_counter;

  for (size_t i = 0; i < command_interfaces_.size(); ++i)
  {
    RCLCPP_INFO(
      get_node()->get_logger(), "Setting value of command interface '%s' to %f",
      command_interfaces_[i].get_name().c_str(), external_commands_for_testing_[i]);
    command_interfaces_[i].set_value(external_commands_for_testing_[i]);
  }

  return controller_interface::return_type::OK;
}

CallbackReturn TestController::on_init() { return CallbackReturn::SUCCESS; }

CallbackReturn TestController::on_configure(const rclcpp_lifecycle::State & /*previous_state*/)
{
  return CallbackReturn::SUCCESS;
}

CallbackReturn TestController::on_cleanup(const rclcpp_lifecycle::State & /*previous_state*/)
{
  if (simulate_cleanup_failure)
  {
    return CallbackReturn::FAILURE;
  }

  if (cleanup_calls)
  {
    (*cleanup_calls)++;
  }
  return CallbackReturn::SUCCESS;
}

void TestController::set_command_interface_configuration(
  const controller_interface::InterfaceConfiguration & cfg)
{
  cmd_iface_cfg_ = cfg;
  external_commands_for_testing_.resize(cmd_iface_cfg_.names.size(), 0.0);
}

void TestController::set_state_interface_configuration(
  const controller_interface::InterfaceConfiguration & cfg)
{
  state_iface_cfg_ = cfg;
}

}  // namespace test_controller

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(test_controller::TestController, controller_interface::ControllerInterface)
