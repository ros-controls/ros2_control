// Copyright (c) 2022, Stogl Robotics Consulting UG (haftungsbeschränkt)
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

#include "test_chainable_controller.hpp"

#include <memory>
#include <string>
#include <vector>

#include "lifecycle_msgs/msg/state.hpp"

namespace test_chainable_controller
{
TestChainableController::TestChainableController()
: controller_interface::ChainableControllerInterface(),
  cmd_iface_cfg_{controller_interface::interface_configuration_type::NONE},
  state_iface_cfg_{controller_interface::interface_configuration_type::NONE}
{
}

controller_interface::InterfaceConfiguration
TestChainableController::command_interface_configuration() const
{
  if (
    get_lifecycle_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE ||
    get_lifecycle_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE)
  {
    return cmd_iface_cfg_;
  }
  else
  {
    throw std::runtime_error(
      "Can not get command interface configuration until the controller is configured.");
  }
}

controller_interface::InterfaceConfiguration
TestChainableController::state_interface_configuration() const
{
  if (
    get_lifecycle_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE ||
    get_lifecycle_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE)
  {
    auto state_iface_cfg = state_iface_cfg_;
    if (imu_sensor_)
    {
      auto imu_interfaces = imu_sensor_->get_state_interface_names();
      state_iface_cfg.names.insert(
        state_iface_cfg.names.end(), imu_interfaces.begin(), imu_interfaces.end());
    }
    return state_iface_cfg_;
  }
  else
  {
    throw std::runtime_error(
      "Can not get state interface configuration until the controller is configured.");
  }
}

controller_interface::return_type TestChainableController::update_reference_from_subscribers(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  for (size_t i = 0; i < reference_interfaces_.size(); ++i)
  {
    RCLCPP_INFO(
      get_node()->get_logger(),
      "Value of reference interface '%s' before checking external input is %f",
      (std::string(get_node()->get_name()) + "/" + reference_interface_names_[i]).c_str(),
      reference_interfaces_[i]);
  }

  auto joint_commands = rt_command_ptr_.readFromRT();
  reference_interfaces_ = (*joint_commands)->data;
  for (size_t i = 0; i < reference_interfaces_.size(); ++i)
  {
    RCLCPP_INFO(
      get_node()->get_logger(),
      "Updated value of reference interface '%s' after applying external input is %f",
      (std::string(get_node()->get_name()) + "/" + reference_interface_names_[i]).c_str(),
      reference_interfaces_[i]);
  }

  return controller_interface::return_type::OK;
}

controller_interface::return_type TestChainableController::update_and_write_commands(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  ++internal_counter;

  for (size_t i = 0; i < command_interfaces_.size(); ++i)
  {
    command_interfaces_[i].set_value(reference_interfaces_[i] - state_interfaces_[i].get_value());
  }
  // If there is a command interface then integrate and set it to the exported state interface data
  for (size_t i = 0; i < exported_state_interface_names_.size() && i < command_interfaces_.size();
       ++i)
  {
    state_interfaces_values_[i] = command_interfaces_[i].get_value() * CONTROLLER_DT;
  }
  // If there is no command interface and if there is a state interface then just forward the same
  // value as in the state interface
  for (size_t i = 0; i < exported_state_interface_names_.size() && i < state_interfaces_.size() &&
                     command_interfaces_.empty();
       ++i)
  {
    state_interfaces_values_[i] = state_interfaces_[i].get_value();
  }

  return controller_interface::return_type::OK;
}

CallbackReturn TestChainableController::on_init() { return CallbackReturn::SUCCESS; }

CallbackReturn TestChainableController::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  joints_command_subscriber_ = get_node()->create_subscription<CmdType>(
    "~/commands", rclcpp::SystemDefaultsQoS(),
    [this](const CmdType::SharedPtr msg)
    {
      auto joint_commands = rt_command_ptr_.readFromNonRT();

      if (msg->data.size() != (*joint_commands)->data.size())
      {
        rt_command_ptr_.writeFromNonRT(msg);
      }
      else
      {
        RCLCPP_ERROR_THROTTLE(
          get_node()->get_logger(), *get_node()->get_clock(), 1000,
          "command size (%zu) does not match number of reference interfaces (%zu)",
          (*joint_commands)->data.size(), reference_interfaces_.size());
      }
    });

  auto msg = std::make_shared<CmdType>();
  msg->data.resize(reference_interfaces_.size());
  rt_command_ptr_.writeFromNonRT(msg);

  return CallbackReturn::SUCCESS;
}

CallbackReturn TestChainableController::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  if (!is_in_chained_mode())
  {
    auto msg = rt_command_ptr_.readFromRT();
    (*msg)->data = reference_interfaces_;
  }

  return CallbackReturn::SUCCESS;
}

CallbackReturn TestChainableController::on_cleanup(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  joints_command_subscriber_.reset();
  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
TestChainableController::on_export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  for (size_t i = 0; i < exported_state_interface_names_.size(); ++i)
  {
    state_interfaces.push_back(hardware_interface::StateInterface(
      get_node()->get_name(), exported_state_interface_names_[i], &state_interfaces_values_[i]));
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
TestChainableController::on_export_reference_interfaces()
{
  std::vector<hardware_interface::CommandInterface> reference_interfaces;

  for (size_t i = 0; i < reference_interface_names_.size(); ++i)
  {
    reference_interfaces.push_back(hardware_interface::CommandInterface(
      get_node()->get_name(), reference_interface_names_[i], &reference_interfaces_[i]));
  }

  return reference_interfaces;
}

void TestChainableController::set_command_interface_configuration(
  const controller_interface::InterfaceConfiguration & cfg)
{
  cmd_iface_cfg_ = cfg;
}

void TestChainableController::set_state_interface_configuration(
  const controller_interface::InterfaceConfiguration & cfg)
{
  state_iface_cfg_ = cfg;
}

void TestChainableController::set_reference_interface_names(
  const std::vector<std::string> & reference_interface_names)
{
  reference_interface_names_ = reference_interface_names;

  reference_interfaces_.resize(reference_interface_names.size(), 0.0);
}

void TestChainableController::set_exported_state_interface_names(
  const std::vector<std::string> & state_interface_names)
{
  exported_state_interface_names_ = state_interface_names;

  state_interfaces_values_.resize(exported_state_interface_names_.size(), 0.0);
}

void TestChainableController::set_imu_sensor_name(const std::string & name)
{
  if (!name.empty())
  {
    imu_sensor_ = std::make_unique<semantic_components::IMUSensor>(name);
  }
}

std::vector<double> TestChainableController::get_state_interface_data() const
{
  std::vector<double> state_intr_data;
  for (const auto & interface : state_interfaces_)
  {
    state_intr_data.push_back(interface.get_value());
  }
  return state_intr_data;
}
}  // namespace test_chainable_controller

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  test_chainable_controller::TestChainableController,
  controller_interface::ChainableControllerInterface)
