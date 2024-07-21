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

#include "hardware_interface/types/lifecycle_state_names.hpp"
#include "lifecycle_msgs/msg/state.hpp"

namespace controller_interface
{
return_type ControllerInterfaceBase::init(
  const std::string & controller_name, const std::string & namespace_,
  const rclcpp::NodeOptions & node_options)
{
  node_ = std::make_shared<rclcpp_lifecycle::LifecycleNode>(
    controller_name, namespace_, node_options, false);  // disable LifecycleNode service interfaces

  try
  {
    auto_declare<int>("update_rate", 0);
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
  if (get_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED)
  {
    update_rate_ = static_cast<unsigned int>(get_node()->get_parameter("update_rate").as_int());
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

const rclcpp_lifecycle::State & ControllerInterfaceBase::get_state() const
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

std::shared_ptr<rclcpp_lifecycle::LifecycleNode> ControllerInterfaceBase::get_node() const
{
  if (!node_.get())
  {
    throw std::runtime_error("Lifecycle node hasn't been initialized yet!");
  }
  return node_;
}

unsigned int ControllerInterfaceBase::get_update_rate() const { return update_rate_; }

}  // namespace controller_interface
