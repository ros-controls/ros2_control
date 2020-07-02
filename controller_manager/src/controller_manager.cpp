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

#include "rclcpp/rclcpp.hpp"

namespace controller_manager
{

ControllerManager::ControllerManager(
  std::shared_ptr<hardware_interface::RobotHardware> hw,
  std::shared_ptr<rclcpp::Executor> executor,
  const std::string & manager_node_name)
: rclcpp::Node(manager_node_name),
  hw_(hw),
  executor_(executor),
  // add pluginlib loader by default
  loaders_({std::make_shared<ControllerLoaderPluginlib>()})
{
}

std::shared_ptr<controller_interface::ControllerInterface>
ControllerManager::load_controller(
  const std::string & controller_name,
  const std::string & controller_type)
{
  RCLCPP_INFO(get_logger(), "Loading controller '%s'\n", controller_name.c_str());

  auto it = std::find_if(
    loaders_.cbegin(), loaders_.cend(),
    [&](auto loader)
    {return loader->is_available(controller_type);});

  std::shared_ptr<controller_interface::ControllerInterface> controller(nullptr);
  if (it != loaders_.cend()) {
    controller = (*it)->create(controller_type);
  } else {
    const std::string error_msg("Loader for controller '" + controller_name + "' not found\n");
    RCLCPP_ERROR(get_logger(), "%s", error_msg.c_str());
    throw std::runtime_error(error_msg);
  }

  return add_controller_impl(controller, controller_name);
}

std::vector<std::shared_ptr<controller_interface::ControllerInterface>>
ControllerManager::get_loaded_controllers() const
{
  return loaded_controllers_;
}

void ControllerManager::register_controller_loader(ControllerLoaderInterfaceSharedPtr loader)
{
  loaders_.push_back(loader);
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

controller_interface::return_type
ControllerManager::update()
{
  auto ret = controller_interface::return_type::SUCCESS;
  for (auto loaded_controller : loaded_controllers_) {
    auto controller_ret = loaded_controller->update();
    if (controller_ret != controller_interface::return_type::SUCCESS) {
      ret = controller_ret;
    }
  }

  return ret;
}

controller_interface::return_type
ControllerManager::configure() const
{
  auto ret = controller_interface::return_type::SUCCESS;
  for (auto loaded_controller : loaded_controllers_) {
    auto controller_state = loaded_controller->get_lifecycle_node()->configure();
    if (controller_state.id() != lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE) {
      ret = controller_interface::return_type::ERROR;
    }
  }

  return ret;
}

controller_interface::return_type
ControllerManager::activate() const
{
  auto ret = controller_interface::return_type::SUCCESS;
  for (auto loaded_controller : loaded_controllers_) {
    auto controller_state = loaded_controller->get_lifecycle_node()->activate();
    if (controller_state.id() != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
      ret = controller_interface::return_type::ERROR;
    }
  }

  return ret;
}

controller_interface::return_type
ControllerManager::deactivate() const
{
  auto ret = controller_interface::return_type::SUCCESS;
  for (auto loaded_controller : loaded_controllers_) {
    auto controller_state = loaded_controller->get_lifecycle_node()->deactivate();
    if (controller_state.id() != lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE) {
      ret = controller_interface::return_type::ERROR;
    }
  }

  return ret;
}

controller_interface::return_type
ControllerManager::cleanup() const
{
  auto ret = controller_interface::return_type::SUCCESS;
  for (auto loaded_controller : loaded_controllers_) {
    auto controller_state = loaded_controller->get_lifecycle_node()->cleanup();
    if (controller_state.id() != lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED) {
      ret = controller_interface::return_type::ERROR;
    }
  }

  return ret;
}

}  // namespace controller_manager
