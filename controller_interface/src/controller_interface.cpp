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
#include <lifecycle_msgs/msg/state.hpp>

#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace controller_interface
{

return_type
ControllerInterface::init(const std::string & controller_name)
{
  node_ = std::make_shared<rclcpp::Node>(
    controller_name,
    rclcpp::NodeOptions().allow_undeclared_parameters(true));
  lifecycle_state_ = rclcpp_lifecycle::State(
    lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED, "unconfigured");
  return return_type::OK;
}

return_type
ControllerInterface::init(const std::string & controller_name, rclcpp::NodeOptions & node_options)
{
  node_ = std::make_shared<rclcpp::Node>(
    controller_name,
    node_options.allow_undeclared_parameters(true));
  lifecycle_state_ = rclcpp_lifecycle::State(
    lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED, "unconfigured");
  return return_type::OK;
}

const rclcpp_lifecycle::State & ControllerInterface::configure()
{
  if (lifecycle_state_.id() == lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED) {
    switch (on_configure(lifecycle_state_)) {
      case LifecycleNodeInterface::CallbackReturn::SUCCESS:
        lifecycle_state_ = rclcpp_lifecycle::State(
          lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE,
          "inactive");
        break;
      case LifecycleNodeInterface::CallbackReturn::ERROR:
        on_error(lifecycle_state_);
        lifecycle_state_ = rclcpp_lifecycle::State(
          lifecycle_msgs::msg::State::PRIMARY_STATE_FINALIZED, "finalized");
        break;
      case LifecycleNodeInterface::CallbackReturn::FAILURE:
        break;
    }
  }
  return lifecycle_state_;
}

const rclcpp_lifecycle::State & ControllerInterface::cleanup()
{
  switch (on_cleanup(lifecycle_state_)) {
    case LifecycleNodeInterface::CallbackReturn::SUCCESS:
      lifecycle_state_ = rclcpp_lifecycle::State(
        lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED, "unconfigured");
      break;
    case LifecycleNodeInterface::CallbackReturn::ERROR:
      on_error(lifecycle_state_);
      lifecycle_state_ = rclcpp_lifecycle::State(
        lifecycle_msgs::msg::State::PRIMARY_STATE_FINALIZED, "finalized");
      break;
    case LifecycleNodeInterface::CallbackReturn::FAILURE:
      break;
  }
  return lifecycle_state_;
}
const rclcpp_lifecycle::State & ControllerInterface::deactivate()
{
  switch (on_deactivate(lifecycle_state_)) {
    case LifecycleNodeInterface::CallbackReturn::SUCCESS:
      lifecycle_state_ = rclcpp_lifecycle::State(
        lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE, "inactive");
      break;
    case LifecycleNodeInterface::CallbackReturn::ERROR:
      on_error(lifecycle_state_);
      lifecycle_state_ = rclcpp_lifecycle::State(
        lifecycle_msgs::msg::State::PRIMARY_STATE_FINALIZED, "finalized");
      break;
    case LifecycleNodeInterface::CallbackReturn::FAILURE:
      break;
  }
  return lifecycle_state_;
}
const rclcpp_lifecycle::State & ControllerInterface::activate()
{
  if (lifecycle_state_.id() == lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE) {
    switch (on_activate(lifecycle_state_)) {
      case LifecycleNodeInterface::CallbackReturn::SUCCESS:
        lifecycle_state_ = rclcpp_lifecycle::State(
          lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE, "active");
        break;
      case LifecycleNodeInterface::CallbackReturn::ERROR:
        on_error(lifecycle_state_);
        lifecycle_state_ = rclcpp_lifecycle::State(
          lifecycle_msgs::msg::State::PRIMARY_STATE_FINALIZED, "finalized");
        break;
      case LifecycleNodeInterface::CallbackReturn::FAILURE:
        break;
    }
  }
  return lifecycle_state_;
}

const rclcpp_lifecycle::State & ControllerInterface::shutdown()
{
  switch (on_activate(lifecycle_state_)) {
    case LifecycleNodeInterface::CallbackReturn::SUCCESS:
      lifecycle_state_ = rclcpp_lifecycle::State(
        lifecycle_msgs::msg::State::PRIMARY_STATE_FINALIZED, "finalized");
      break;
    case LifecycleNodeInterface::CallbackReturn::ERROR:
      on_error(lifecycle_state_);
      lifecycle_state_ = rclcpp_lifecycle::State(
        lifecycle_msgs::msg::State::PRIMARY_STATE_FINALIZED, "finalized");
      break;
    case LifecycleNodeInterface::CallbackReturn::FAILURE:
      break;
  }
  return lifecycle_state_;
}

const rclcpp_lifecycle::State & ControllerInterface::get_current_state() const
{
  return lifecycle_state_;
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

std::shared_ptr<rclcpp::Node>
ControllerInterface::get_node()
{
  if (!node_.get()) {
    throw std::runtime_error("Node hasn't been initialized yet!");
  }
  return node_;
}

}  // namespace controller_interface
