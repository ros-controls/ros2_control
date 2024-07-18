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

#include "controller_interface/controller_interface_base.hpp"

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "lifecycle_msgs/msg/state.hpp"

namespace controller_interface
{
return_type ControllerInterfaceBase::init(
  const std::string & controller_name, const std::string & urdf, unsigned int cm_update_rate,
  const std::string & node_namespace, const rclcpp::NodeOptions & node_options)
{
  urdf_ = urdf;
  node_ = std::make_shared<rclcpp_lifecycle::LifecycleNode>(
    controller_name, node_namespace, node_options,
    false);  // disable LifecycleNode service interfaces

  auto retrieve_remap_interface = [this](
                                    const std::string & remap_namespace,
                                    std::map<std::string, std::string> & remappings) -> bool
  {
    if (!node_->has_parameter(remap_namespace))
    {
      node_->declare_parameters(remap_namespace, remappings);
    }
    return node_->get_parameters(remap_namespace, remappings);
  };

  try
  {
    auto_declare<int>("update_rate", cm_update_rate);
    auto_declare<bool>("is_async", false);
    if (
      retrieve_remap_interface("remap.state_interfaces", state_interfaces_remap_) &&
      !state_interfaces_remap_.empty())
    {
      RCLCPP_WARN(
        node_->get_logger(),
        "The controller : %s will be able to remap the following state interfaces:",
        controller_name.c_str());
      for (const auto & [key, value] : state_interfaces_remap_)
      {
        RCLCPP_INFO(node_->get_logger(), "\t'%s' to '%s'", key.c_str(), value.c_str());
      }
    }
    if (
      retrieve_remap_interface("remap.command_interfaces", command_interfaces_remap_) &&
      !command_interfaces_remap_.empty())
    {
      RCLCPP_WARN(
        node_->get_logger(),
        "The controller : %s will be able to remap the following command interfaces:",
        controller_name.c_str());
      for (const auto & [key, value] : command_interfaces_remap_)
      {
        RCLCPP_INFO(node_->get_logger(), "\t'%s' to '%s'", key.c_str(), value.c_str());
      }
    }
  }
  catch (const std::exception & e)
  {
    fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
    return return_type::ERROR;
  }

  switch (on_init())
  {
    case LifecycleNodeInterface::CallbackReturn::SUCCESS:
      break;
    case LifecycleNodeInterface::CallbackReturn::ERROR:
    case LifecycleNodeInterface::CallbackReturn::FAILURE:
      return return_type::ERROR;
  }

  node_->register_on_configure(
    std::bind(&ControllerInterfaceBase::on_configure, this, std::placeholders::_1));

  node_->register_on_cleanup(
    std::bind(&ControllerInterfaceBase::on_cleanup, this, std::placeholders::_1));

  node_->register_on_activate(
    std::bind(&ControllerInterfaceBase::on_activate, this, std::placeholders::_1));

  node_->register_on_deactivate(
    std::bind(&ControllerInterfaceBase::on_deactivate, this, std::placeholders::_1));

  node_->register_on_shutdown(
    std::bind(&ControllerInterfaceBase::on_shutdown, this, std::placeholders::_1));

  node_->register_on_error(
    std::bind(&ControllerInterfaceBase::on_error, this, std::placeholders::_1));

  return return_type::OK;
}

const rclcpp_lifecycle::State & ControllerInterfaceBase::configure()
{
  // TODO(destogl): this should actually happen in "on_configure" but I am not sure how to get
  // overrides correctly in combination with std::bind. The goal is to have the following calls:
  // 1. CM: controller.get_node()->configure()
  // 2. LifecycleNode: ControllerInterfaceBase::on_configure()
  // 3. ControllerInterfaceBase: <controller>::on_configure()
  // Then we don't need to do state-machine related checks.
  //
  // Other solution is to add check into the LifecycleNode if a transition is valid to trigger
  if (get_lifecycle_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED)
  {
    update_rate_ = static_cast<unsigned int>(get_node()->get_parameter("update_rate").as_int());
    is_async_ = get_node()->get_parameter("is_async").as_bool();
  }

  return get_node()->configure();
}

void ControllerInterfaceBase::assign_interfaces(
  std::vector<hardware_interface::LoanedCommandInterface> && command_interfaces,
  std::vector<hardware_interface::LoanedStateInterface> && state_interfaces)
{
  command_interfaces_ = std::forward<decltype(command_interfaces)>(command_interfaces);
  state_interfaces_ = std::forward<decltype(state_interfaces)>(state_interfaces);
}

void ControllerInterfaceBase::release_interfaces()
{
  command_interfaces_.clear();
  state_interfaces_.clear();
}

const rclcpp_lifecycle::State & ControllerInterfaceBase::get_lifecycle_state() const
{
  return node_->get_current_state();
}

std::shared_ptr<rclcpp_lifecycle::LifecycleNode> ControllerInterfaceBase::get_node()
{
  if (!node_.get())
  {
    throw std::runtime_error("Lifecycle node hasn't been initialized yet!");
  }
  return node_;
}

std::shared_ptr<const rclcpp_lifecycle::LifecycleNode> ControllerInterfaceBase::get_node() const
{
  if (!node_.get())
  {
    throw std::runtime_error("Lifecycle node hasn't been initialized yet!");
  }
  return node_;
}

unsigned int ControllerInterfaceBase::get_update_rate() const { return update_rate_; }

bool ControllerInterfaceBase::is_async() const { return is_async_; }

const std::string & ControllerInterfaceBase::get_robot_description() const { return urdf_; }

const std::map<std::string, std::string> &
controller_interface::ControllerInterfaceBase::get_state_interfaces_remap() const
{
  return state_interfaces_remap_;
}

const std::map<std::string, std::string> &
controller_interface::ControllerInterfaceBase::get_command_interfaces_remap() const
{
  return command_interfaces_remap_;
}

}  // namespace controller_interface
