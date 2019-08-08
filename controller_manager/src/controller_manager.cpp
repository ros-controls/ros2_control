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

#include "ament_index_cpp/get_resource.hpp"
#include "ament_index_cpp/get_resources.hpp"

#include "class_loader/class_loader.hpp"

#include "controller_interface/controller_interface.hpp"

#include "lifecycle_msgs/msg/state.hpp"

#include "rcutils/format_string.h"
#include "rcutils/logging_macros.h"
#include "rcutils/split.h"
#include "rcutils/types.h"

#ifdef __clang__
// TODO(dirk-thomas) custom implementation until we can use libc++ 3.9
namespace fs
{
class path
{
public:
  explicit path(const std::string & p)
  : path_(p)
  {}
  bool is_absolute()
  {
    return path_[0] == '/';
  }

private:
  std::string path_;
};
}  // namespace fs
#else
# include <experimental/filesystem>
namespace fs = std::experimental::filesystem;
#endif

namespace controller_manager
{

static constexpr const char * resource_index = "ros_controllers";

namespace
{

std::string
parse_library_path(
  const std::string & package_name,
  const std::string & class_name,
  const rclcpp::Logger logger)  // should be const ref when possible
{
  // get node plugin resource from package
  std::string content;
  std::string base_path;
  if (!ament_index_cpp::get_resource(resource_index, package_name, content, &base_path)) {
    auto error_msg = std::string("unable to load resource for package ") + package_name;
    RCLCPP_ERROR(logger, error_msg);
    throw std::runtime_error(error_msg);
  }

  auto allocator = rcutils_get_default_allocator();
  rcutils_string_array_t controller_array = rcutils_get_zero_initialized_string_array();
  rcutils_split(content.c_str(), '\n', allocator, &controller_array);
  if (controller_array.size == 0) {
    auto error_msg = std::string("no ros controllers found in package " + package_name);
    RCLCPP_ERROR(logger, error_msg);
    throw std::runtime_error(error_msg);
  }

  std::string library_path = "";
  bool controller_is_available = false;
  for (auto i = 0u; i < controller_array.size; ++i) {
    rcutils_string_array_t controller_details = rcutils_get_zero_initialized_string_array();
    rcutils_split(controller_array.data[i], ';', allocator, &controller_details);
    if (controller_details.size != 2) {
      auto error_msg = std::string("package resource content has wrong format") +
        " - should be <class_name>;<library_path> but is " + controller_array.data[i];
      RCLCPP_ERROR(logger, error_msg);
      throw std::runtime_error(error_msg);
    }

    if (strcmp(controller_details.data[0], class_name.c_str()) == 0) {
      library_path = controller_details.data[1];
      if (!fs::path(library_path).is_absolute()) {
        library_path = base_path + "/" + controller_details.data[1];
      }
      controller_is_available = true;
      break;
    }
  }

  if (!controller_is_available) {
    auto error_msg = std::string("couldn't find controller class ") + class_name;
    RCLCPP_ERROR(logger, error_msg);
    throw std::runtime_error(error_msg);
  }

  return library_path;
}

}  // namespace

ControllerManager::ControllerManager(
  std::shared_ptr<hardware_interface::RobotHardware> hw,
  std::shared_ptr<rclcpp::executor::Executor> executor,
  const std::string & manager_node_name)
: rclcpp::Node(manager_node_name),
  hw_(hw),
  executor_(executor)
{}

std::shared_ptr<controller_interface::ControllerInterface>
ControllerManager::load_controller(
  const std::string & package_name,
  const std::string & class_name,
  const std::string & controller_name)
{
  auto library_path = parse_library_path(package_name, class_name, this->get_logger());

  RCLCPP_INFO(this->get_logger(), "going to load controller %s from library %s",
    class_name.c_str(), library_path.c_str());

  // let possible exceptions escalate
  auto loader = std::make_shared<class_loader::ClassLoader>(library_path);
  std::shared_ptr<controller_interface::ControllerInterface> controller =
    loader->createInstance<controller_interface::ControllerInterface>(class_name);
  loaders_.push_back(loader);

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
