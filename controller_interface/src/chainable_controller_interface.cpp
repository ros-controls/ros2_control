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

#include "controller_interface/chainable_controller_interface.hpp"

#include <vector>

#include "hardware_interface/types/lifecycle_state_names.hpp"
#include "lifecycle_msgs/msg/state.hpp"

namespace controller_interface
{
ChainableControllerInterface::ChainableControllerInterface() : ControllerInterfaceBase() {}

bool ChainableControllerInterface::is_chainable() const { return true; }

return_type ChainableControllerInterface::update(
  const rclcpp::Time & time, const rclcpp::Duration & period)
{
  return_type ret = return_type::ERROR;

  if (!is_in_chained_mode())
  {
    ret = update_reference_from_subscribers();
    if (ret != return_type::OK)
    {
      return ret;
    }
  }

  ret = update_and_write_commands(time, period);

  return ret;
}

std::vector<hardware_interface::CommandInterface>
ChainableControllerInterface::export_reference_interfaces()
{
  auto reference_interfaces = on_export_reference_interfaces();

  // check if the "reference_interfaces_" variable is resized to number of interfaces
  if (reference_interfaces_.size() != reference_interfaces.size())
  {
    // TODO(destogl): Should here be "FATAL"? It is fatal in terms of controller but not for the
    // framework
    RCLCPP_FATAL(
      get_node()->get_logger(),
      "The internal storage for reference values 'reference_interfaces_' variable has size '%zu', "
      "but it is expected to have the size '%zu' equal to the number of exported reference "
      "interfaces. No reference interface will be exported. Please correct and recompile "
      "the controller with name '%s' and try again.",
      reference_interfaces_.size(), reference_interfaces.size(), get_node()->get_name());
    reference_interfaces.clear();
  }

  // check if the names of the reference interfaces begin with the controller's name
  for (const auto & interface : reference_interfaces)
  {
    if (interface.get_prefix_name() != get_node()->get_name())
    {
      RCLCPP_FATAL(
        get_node()->get_logger(),
        "The name of the interface '%s' does not begin with the controller's name. This is "
        "mandatory "
        " for reference interfaces. No reference interface will be exported. Please correct and "
        "recompile the controller with name '%s' and try again.",
        interface.get_name().c_str(), get_node()->get_name());
      reference_interfaces.clear();
      break;
    }
  }

  return reference_interfaces;
}

bool ChainableControllerInterface::set_chained_mode(bool chained_mode)
{
  bool result = false;

  if (get_state().id() != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE)
  {
    result = on_set_chained_mode(chained_mode);

    if (result)
    {
      in_chained_mode_ = chained_mode;
    }
  }
  else
  {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "Can not change controller's chained mode because it is no in '%s' state. "
      "Current state is '%s'.",
      hardware_interface::lifecycle_state_names::UNCONFIGURED, get_state().label().c_str());
  }

  return result;
}

bool ChainableControllerInterface::is_in_chained_mode() const { return in_chained_mode_; }

bool ChainableControllerInterface::on_set_chained_mode(bool /*chained_mode*/) { return true; }

}  // namespace controller_interface
