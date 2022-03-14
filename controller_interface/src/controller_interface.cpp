// Copyright 2017 Open Source Robotics Foundation, Inc.
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

#include "controller_interface/controller_interface.hpp"

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "hardware_interface/types/lifecycle_state_names.hpp"

namespace controller_interface
{
return_type ControllerInterface::init(const std::string & controller_name)
{
  node_ = std::make_shared<rclcpp_lifecycle::LifecycleNode>(
    controller_name,
    rclcpp::NodeOptions()
      .allow_undeclared_parameters(true)
      .automatically_declare_parameters_from_overrides(true),
    false);  // disable LifecycleNode service interfaces

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
    std::bind(&ControllerInterface::on_configure, this, std::placeholders::_1));

  node_->register_on_cleanup(
    std::bind(&ControllerInterface::on_cleanup, this, std::placeholders::_1));

  node_->register_on_activate(
    std::bind(&ControllerInterface::on_activate, this, std::placeholders::_1));

  node_->register_on_deactivate(
    std::bind(&ControllerInterface::on_deactivate, this, std::placeholders::_1));

  node_->register_on_shutdown(
    std::bind(&ControllerInterface::on_shutdown, this, std::placeholders::_1));

  node_->register_on_error(std::bind(&ControllerInterface::on_error, this, std::placeholders::_1));

  return return_type::OK;
}

const rclcpp_lifecycle::State & ControllerInterface::configure()
{
  update_rate_ = node_->get_parameter("update_rate").as_int();

  return node_->configure();
}

void ControllerInterface::assign_interfaces(
  std::vector<hardware_interface::LoanedCommandInterface> && command_interfaces,
  std::vector<hardware_interface::LoanedStateInterface> && state_interfaces)
{
  command_interfaces_ = std::forward<decltype(command_interfaces)>(command_interfaces);
  state_interfaces_ = std::forward<decltype(state_interfaces)>(state_interfaces);
}

void ControllerInterface::release_interfaces()
{
  command_interfaces_.clear();
  state_interfaces_.clear();
}

const rclcpp_lifecycle::State & ControllerInterface::get_state() const
{
  return node_->get_current_state();
}

std::shared_ptr<rclcpp_lifecycle::LifecycleNode> ControllerInterface::get_node()
{
  if (!node_.get())
  {
    throw std::runtime_error("Lifecycle node hasn't been initialized yet!");
  }
  return node_;
}

unsigned int ControllerInterface::get_update_rate() const { return update_rate_; }

}  // namespace controller_interface
