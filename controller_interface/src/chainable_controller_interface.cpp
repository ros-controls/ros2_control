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

#include <fmt/compile.h>

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
  // Reset internal state before calling the export method so that stale names from a previous
  // configure cycle (e.g. after a failed activation that skipped on_cleanup) do not cause ghost
  // interfaces.
  exported_state_interfaces_.clear();
  exported_state_interface_names_.clear();
  ordered_exported_state_interfaces_.clear();

  const auto state_interfaces_list = on_export_state_interfaces_list();
  std::vector<hardware_interface::StateInterface::ConstSharedPtr> state_interfaces_ptrs_vec;
  state_interfaces_ptrs_vec.reserve(state_interfaces_list.size());
  ordered_exported_state_interfaces_.reserve(state_interfaces_list.size());
  exported_state_interface_names_.reserve(state_interfaces_list.size());

  for (const auto & interface_ptr : state_interfaces_list)
  {
    if (interface_ptr->get_prefix_name().find(get_node()->get_name()) != 0)
    {
      const std::string error_msg = fmt::format(
        FMT_COMPILE(
          "The prefix of the interface '{}' should begin with the controller's name '{}'. "
          "This is mandatory for state interfaces. No state interface will be exported. "
          "Please correct and recompile the controller with name '{}' and try again."),
        interface_ptr->get_prefix_name(), get_node()->get_name(), get_node()->get_name());
      throw std::runtime_error(error_msg);
    }
    const auto interface_name = interface_ptr->get_name();
    auto [it, succ] = exported_state_interfaces_.insert({interface_name, interface_ptr});
    if (!succ)
    {
      std::string error_msg = fmt::format(
        FMT_COMPILE(
          "Could not insert StateInterface<{}> into exported_state_interfaces_ map. "
          "Check if you export duplicates. The map returned iterator with interface_name<{}>. "
          "If its a duplicate adjust exportation of InterfacesDescription so that all the "
          "interface names are unique."),
        interface_name, it->second->get_name());
      exported_state_interfaces_.clear();
      exported_state_interface_names_.clear();
      state_interfaces_ptrs_vec.clear();
      throw std::runtime_error(error_msg);
    }
    ros2_control::add_item(ordered_exported_state_interfaces_, interface_ptr);
    ros2_control::add_item(exported_state_interface_names_, interface_name);
    state_interfaces_ptrs_vec.push_back(interface_ptr);
  }

  if (exported_state_interfaces_.size() != state_interfaces_list.size())
  {
    std::string error_msg = fmt::format(
      FMT_COMPILE(
        "The internal storage for state interface ptrs 'exported_state_interfaces_' variable has "
        "size '{}', but it is expected to have the size '{}' equal to the number of exported "
        "state interfaces. Please correct and recompile the controller with name '{}' and try "
        "again."),
      exported_state_interfaces_.size(), state_interfaces_list.size(), get_node()->get_name());
    throw std::runtime_error(error_msg);
  }

  return state_interfaces_ptrs_vec;
}

std::vector<hardware_interface::CommandInterface::SharedPtr>
ChainableControllerInterface::export_reference_interfaces()
{
  // Reset internal state before calling the export method so that stale names from a previous
  // configure cycle (e.g. after a failed activation that skipped on_cleanup) do not cause ghost
  // interfaces.
  exported_reference_interfaces_.clear();
  exported_reference_interface_names_.clear();
  ordered_exported_reference_interfaces_.clear();

  const auto reference_interfaces_list = on_export_reference_interfaces_list();
  std::vector<hardware_interface::CommandInterface::SharedPtr> reference_interfaces_ptrs_vec;
  reference_interfaces_ptrs_vec.reserve(reference_interfaces_list.size());
  exported_reference_interface_names_.reserve(reference_interfaces_list.size());
  ordered_exported_reference_interfaces_.reserve(reference_interfaces_list.size());

  for (const auto & interface_ptr : reference_interfaces_list)
  {
    if (interface_ptr->get_prefix_name().find(get_node()->get_name()) != 0)
    {
      std::string error_msg = fmt::format(
        FMT_COMPILE(
          "The prefix of the interface '{}' should begin with the controller's name '{}'. "
          "This is mandatory for reference interfaces. Please correct and recompile the "
          "controller with name '{}' and try again."),
        interface_ptr->get_prefix_name(), get_node()->get_name(), get_node()->get_name());
      throw std::runtime_error(error_msg);
    }

    const auto interface_name = interface_ptr->get_name();
    auto [it, succ] = exported_reference_interfaces_.insert({interface_name, interface_ptr});
    if (!succ)
    {
      std::string error_msg = fmt::format(
        FMT_COMPILE(
          "Could not insert Reference interface<{}> into exported_reference_interfaces_ map. "
          "Check if you export duplicates. The map returned iterator with interface_name<{}>. "
          "If its a duplicate adjust exportation of InterfacesDescription so that all the "
          "interface names are unique."),
        interface_name, it->second->get_name());
      exported_reference_interface_names_.clear();
      reference_interfaces_ptrs_vec.clear();
      throw std::runtime_error(error_msg);
    }
    ros2_control::add_item(ordered_exported_reference_interfaces_, interface_ptr);
    ros2_control::add_item(exported_reference_interface_names_, interface_name);
    reference_interfaces_ptrs_vec.push_back(interface_ptr);
  }

  if (exported_reference_interfaces_.size() != reference_interfaces_list.size())
  {
    std::string error_msg = fmt::format(
      FMT_COMPILE(
        "The internal storage for exported reference ptrs 'exported_reference_interfaces_' "
        "variable has size '{}', but it is expected to have the size '{}' equal to the number of "
        "exported reference interfaces. Please correct and recompile the controller with name '{}' "
        "and try again."),
      exported_reference_interfaces_.size(), reference_interfaces_list.size(),
      get_node()->get_name());
    throw std::runtime_error(error_msg);
  }

  return reference_interfaces_ptrs_vec;
}

bool ChainableControllerInterface::set_chained_mode(bool chained_mode)
{
  bool result = false;

  if (get_lifecycle_id() != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE)
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

std::vector<hardware_interface::StateInterface::SharedPtr>
ChainableControllerInterface::on_export_state_interfaces_list()
{
  // return empty vector by default.
  return {};
}

std::vector<hardware_interface::CommandInterface::SharedPtr>
ChainableControllerInterface::on_export_reference_interfaces_list()
{
  // return empty vector by default.
  return {};
}

}  // namespace controller_interface
