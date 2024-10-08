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

#include "controller_interface/helpers.hpp"
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

std::vector<hardware_interface::StateInterface::ConstSharedPtr>
ChainableControllerInterface::export_state_interfaces()
{
  auto state_interfaces = on_export_state_interfaces();
  std::vector<hardware_interface::StateInterface::ConstSharedPtr> state_interfaces_ptrs_vec;
  state_interfaces_ptrs_vec.reserve(state_interfaces.size());
  ordered_exported_state_interfaces_.reserve(state_interfaces.size());
  exported_state_interface_names_.reserve(state_interfaces.size());

  // check if the names of the controller state interfaces begin with the controller's name
  for (const auto & interface : state_interfaces)
  {
    if (interface.get_prefix_name() != get_node()->get_name())
    {
      std::string error_msg =
        "The prefix of the interface '" + interface.get_prefix_name() +
        "' does not equal the controller's name '" + get_node()->get_name() +
        "'. This is mandatory for state interfaces. No state interface will be exported. Please "
        "correct and recompile the controller with name '" +
        get_node()->get_name() + "' and try again.";
      throw std::runtime_error(error_msg);
    }
    auto state_interface = std::make_shared<hardware_interface::StateInterface>(interface);
    const auto interface_name = state_interface->get_interface_name();
    auto [it, succ] = exported_state_interfaces_.insert({interface_name, state_interface});
    // either we have name duplicate which we want to avoid under all circumstances since interfaces
    // need to be uniquely identify able or something else really went wrong. In any case abort and
    // inform cm by throwing exception
    if (!succ)
    {
      std::string error_msg =
        "Could not insert StateInterface<" + interface_name +
        "> into exported_state_interfaces_ map. Check if you export duplicates. The "
        "map returned iterator with interface_name<" +
        it->second->get_name() +
        ">. If its a duplicate adjust exportation of InterfacesDescription so that all the "
        "interface names are unique.";
      exported_state_interfaces_.clear();
      exported_state_interface_names_.clear();
      state_interfaces_ptrs_vec.clear();
      throw std::runtime_error(error_msg);
    }
    ordered_exported_state_interfaces_.push_back(state_interface);
    add_element_to_list(exported_state_interface_names_, interface_name);
    state_interfaces_ptrs_vec.push_back(
      std::const_pointer_cast<const hardware_interface::StateInterface>(state_interface));
  }

  if (exported_state_interfaces_.size() != state_interfaces.size())
  {
    std::string error_msg =
      "The internal storage for state interface ptrs 'exported_state_interfaces_' variable has "
      "size '" +
      std::to_string(exported_state_interfaces_.size()) +
      "', but it is expected to have the size '" + std::to_string(state_interfaces.size()) +
      "' equal to the number of exported reference interfaces. Please correct and recompile the "
      "controller with name '" +
      get_node()->get_name() + "' and try again.";
    throw std::runtime_error(error_msg);
  }

  return state_interfaces_ptrs_vec;
}

std::vector<hardware_interface::CommandInterface::SharedPtr>
ChainableControllerInterface::export_reference_interfaces()
{
  auto reference_interfaces = on_export_reference_interfaces();
  std::vector<hardware_interface::CommandInterface::SharedPtr> reference_interfaces_ptrs_vec;
  reference_interfaces_ptrs_vec.reserve(reference_interfaces.size());
  exported_reference_interface_names_.reserve(reference_interfaces.size());
  ordered_exported_reference_interfaces_.reserve(reference_interfaces.size());

  // BEGIN (Handle export change): for backward compatibility
  // check if the "reference_interfaces_" variable is resized to number of interfaces
  if (reference_interfaces_.size() != reference_interfaces.size())
  {
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

    hardware_interface::CommandInterface::SharedPtr reference_interface =
      std::make_shared<hardware_interface::CommandInterface>(std::move(interface));
    const auto interface_name = reference_interface->get_interface_name();
    // check the exported interface name is unique
    auto [it, succ] = exported_reference_interfaces_.insert({interface_name, reference_interface});
    // either we have name duplicate which we want to avoid under all circumstances since interfaces
    // need to be uniquely identify able or something else really went wrong. In any case abort and
    // inform cm by throwing exception
    if (!succ)
    {
      std::string error_msg =
        "Could not insert Reference interface<" + interface_name +
        "> into reference_interfaces_ map. Check if you export duplicates. The "
        "map returned iterator with interface_name<" +
        it->second->get_name() +
        ">. If its a duplicate adjust exportation of InterfacesDescription so that all the "
        "interface names are unique.";
      reference_interfaces_.clear();
      exported_reference_interface_names_.clear();
      reference_interfaces_ptrs_vec.clear();
      throw std::runtime_error(error_msg);
    }
    ordered_exported_reference_interfaces_.push_back(reference_interface);
    add_element_to_list(exported_reference_interface_names_, interface_name);
    reference_interfaces_ptrs_vec.push_back(reference_interface);
  }

  if (exported_reference_interfaces_.size() != ref_interface_size)
  {
    std::string error_msg =
      "The internal storage for exported reference ptrs 'exported_reference_interfaces_' variable "
      "has size '" +
      std::to_string(exported_reference_interfaces_.size()) +
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

  if (get_lifecycle_state().id() != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE)
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
      hardware_interface::lifecycle_state_names::UNCONFIGURED,
      get_lifecycle_state().label().c_str());
  }

  return result;
}

bool ChainableControllerInterface::is_in_chained_mode() const { return in_chained_mode_; }

bool ChainableControllerInterface::on_set_chained_mode(bool /*chained_mode*/) { return true; }

std::vector<hardware_interface::StateInterface>
ChainableControllerInterface::on_export_state_interfaces()
{
  state_interfaces_values_.resize(exported_state_interface_names_.size(), 0.0);
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (size_t i = 0; i < exported_state_interface_names_.size(); ++i)
  {
    state_interfaces.emplace_back(
      get_node()->get_name(), exported_state_interface_names_[i], &state_interfaces_values_[i]);
  }
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
ChainableControllerInterface::on_export_reference_interfaces()
{
  reference_interfaces_.resize(exported_reference_interface_names_.size(), 0.0);
  std::vector<hardware_interface::CommandInterface> reference_interfaces;
  for (size_t i = 0; i < exported_reference_interface_names_.size(); ++i)
  {
    reference_interfaces.emplace_back(hardware_interface::CommandInterface(
      get_node()->get_name(), exported_reference_interface_names_[i], &reference_interfaces_[i]));
  }
  return reference_interfaces;
}

}  // namespace controller_interface
