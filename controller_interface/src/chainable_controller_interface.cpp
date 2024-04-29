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
    ret = update_reference_from_subscribers(time, period);
    if (ret != return_type::OK)
    {
      return ret;
    }
  }

  ret = update_and_write_commands(time, period);

  return ret;
}

std::vector<std::shared_ptr<hardware_interface::CommandInterface>>
ChainableControllerInterface::export_reference_interfaces()
{
  auto reference_interfaces = on_export_reference_interfaces();
  std::vector<std::shared_ptr<hardware_interface::CommandInterface>> reference_interfaces_ptrs_vec;
  reference_interfaces_ptrs_vec.reserve(reference_interfaces.size());

  // BEGIN (Handle export change): for backward compatibility
  // check if the "reference_interfaces_" variable is resized to number of interfaces
  if (reference_interfaces_.size() != reference_interfaces.size())
  {
    // TODO(destogl): Should here be "FATAL"? It is fatal in terms of controller but not for the
    // framework
    std::string error_msg =
      "The internal storage for reference values 'reference_interfaces_' variable has size '" +
      std::to_string(reference_interfaces_.size()) + "', but it is expected to have the size '" +
      std::to_string(reference_interfaces.size()) +
      "' equal to the number of exported reference interfaces. Please correct and recompile the "
      "controller with name '" +
      get_node()->get_name() + "' and try again.";
    throw std::runtime_error(error_msg);
  }
  // END

  // check if the names of the reference interfaces begin with the controller's name
  const auto ref_interface_size = reference_interfaces.size();
  for (auto & interface : reference_interfaces)
  {
    if (interface.get_prefix_name() != get_node()->get_name())
    {
      std::string error_msg = "The name of the interface " + interface.get_name() +
                              " does not begin with the controller's name. This is mandatory for "
                              "reference interfaces. Please "
                              "correct and recompile the controller with name " +
                              get_node()->get_name() + " and try again.";
      throw std::runtime_error(error_msg);
    }

    std::shared_ptr<hardware_interface::CommandInterface> interface_ptr =
      std::make_shared<hardware_interface::CommandInterface>(std::move(interface));
    reference_interfaces_ptrs_vec.push_back(interface_ptr);
    reference_interfaces_ptrs_.insert(std::make_pair(interface_ptr->get_name(), interface_ptr));
  }

  if (reference_interfaces_ptrs_.size() != ref_interface_size)
  {
    std::string error_msg =
      "The internal storage for reference ptrs 'reference_interfaces_ptrs_' variable has size '" +
      std::to_string(reference_interfaces_ptrs_.size()) +
      "', but it is expected to have the size '" + std::to_string(ref_interface_size) +
      "' equal to the number of exported reference interfaces. Please correct and recompile the "
      "controller with name '" +
      get_node()->get_name() + "' and try again.";
    throw std::runtime_error(error_msg);
  }

  return reference_interfaces_ptrs_vec;
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
