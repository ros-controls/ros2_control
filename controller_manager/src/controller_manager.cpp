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

#include "controller_manager/controller_manager.hpp"

#include <memory>
#include <string>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "controller_manager/controller_loader_pluginlib.hpp"

#include "lifecycle_msgs/msg/state.hpp"

#include "rcutils/logging_macros.h"

namespace controller_manager
{
ControllerManager::ControllerManager(
  std::shared_ptr<hardware_interface::RobotHardware> hw,
  std::shared_ptr<rclcpp::executor::Executor> executor,
  const std::string & manager_node_name)
: rclcpp::Node(manager_node_name),
  hw_(hw),
  executor_(executor)
{
  // add pluginlib loader by default
  loaders_.push_back(std::make_shared<ControllerLoaderPluginlib>());
}

std::shared_ptr<controller_interface::ControllerInterface>
ControllerManager::load_controller(
  const std::string & controller_name,
  const std::string & controller_type)
{
  RCUTILS_LOG_INFO("going to load controller %s\n", controller_name.c_str());

  std::shared_ptr<controller_interface::ControllerInterface> controller =
    loaders_[0]->create(controller_type);

  return add_controller_impl(controller, controller_name);
}

std::vector<std::shared_ptr<controller_interface::ControllerInterface>>
ControllerManager::get_loaded_controller() const
{
  return loaded_controllers_;
}

std::shared_ptr<controller_interface::ControllerInterface>
ControllerManager::add_controller_impl(
  std::shared_ptr<controller_interface::ControllerInterface> controller,
  const std::string & controller_name)
{
  controller->init(hw_, controller_name);
  executor_->add_node(controller->get_lifecycle_node()->get_node_base_interface());

  loaded_controllers_.emplace_back(controller);
  return loaded_controllers_.back();
}

controller_interface::controller_interface_ret_t
ControllerManager::update()
{
  auto ret = controller_interface::CONTROLLER_INTERFACE_RET_SUCCESS;
  for (auto loaded_controller : loaded_controllers_) {
    auto controller_ret = loaded_controller->update();
    if (controller_ret != controller_interface::CONTROLLER_INTERFACE_RET_SUCCESS) {
      ret = controller_ret;
    }
  }

  return ret;
}

controller_interface::controller_interface_ret_t
ControllerManager::configure() const
{
  auto ret = controller_interface::CONTROLLER_INTERFACE_RET_SUCCESS;
  for (auto loaded_controller : loaded_controllers_) {
    auto controller_state = loaded_controller->get_lifecycle_node()->configure();
    if (controller_state.id() != lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE) {
      ret = controller_interface::CONTROLLER_INTERFACE_RET_ERROR;
    }
  }

  return ret;
}

controller_interface::controller_interface_ret_t
ControllerManager::activate() const
{
  auto ret = controller_interface::CONTROLLER_INTERFACE_RET_SUCCESS;
  for (auto loaded_controller : loaded_controllers_) {
    auto controller_state = loaded_controller->get_lifecycle_node()->activate();
    if (controller_state.id() != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
      ret = controller_interface::CONTROLLER_INTERFACE_RET_ERROR;
    }
  }

  return ret;
}

controller_interface::controller_interface_ret_t
ControllerManager::deactivate() const
{
  auto ret = controller_interface::CONTROLLER_INTERFACE_RET_SUCCESS;
  for (auto loaded_controller : loaded_controllers_) {
    auto controller_state = loaded_controller->get_lifecycle_node()->deactivate();
    if (controller_state.id() != lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE) {
      ret = controller_interface::CONTROLLER_INTERFACE_RET_ERROR;
    }
  }

  return ret;
}

controller_interface::controller_interface_ret_t
ControllerManager::cleanup() const
{
  auto ret = controller_interface::CONTROLLER_INTERFACE_RET_SUCCESS;
  for (auto loaded_controller : loaded_controllers_) {
    auto controller_state = loaded_controller->get_lifecycle_node()->cleanup();
    if (controller_state.id() != lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED) {
      ret = controller_interface::CONTROLLER_INTERFACE_RET_ERROR;
    }
  }

  return ret;
}

}  // namespace controller_manager
