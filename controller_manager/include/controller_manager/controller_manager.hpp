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

#ifndef CONTROLLER_MANAGER__CONTROLLER_MANAGER_HPP_
#define CONTROLLER_MANAGER__CONTROLLER_MANAGER_HPP_

#include <memory>
#include <string>
#include <tuple>
#include <vector>

#include "controller_interface/controller_interface.hpp"

#include "controller_manager/controller_loader_interface.hpp"
#include "controller_manager/visibility_control.h"

#include "hardware_interface/robot_hardware.hpp"

#include "rclcpp/executor.hpp"
#include "rclcpp/node.hpp"

namespace controller_manager
{

class ControllerManager : public rclcpp::Node
{
public:
  CONTROLLER_MANAGER_PUBLIC
  ControllerManager(
    std::shared_ptr<hardware_interface::RobotHardware> hw,
    std::shared_ptr<rclcpp::Executor> executor,
    const std::string & name = "controller_manager");

  CONTROLLER_MANAGER_PUBLIC
  virtual
  ~ControllerManager() = default;

  CONTROLLER_MANAGER_PUBLIC
  std::shared_ptr<controller_interface::ControllerInterface>
  load_controller(
    const std::string & controller_name,
    const std::string & controller_type);

  CONTROLLER_MANAGER_PUBLIC
  std::vector<std::shared_ptr<controller_interface::ControllerInterface>>
  get_loaded_controllers() const;

  [[deprecated(
    "get_loaded_controller is deprecated, it has been renamed to get_loaded_controllers")]]
  CONTROLLER_MANAGER_PUBLIC
  std::vector<std::shared_ptr<controller_interface::ControllerInterface>>
  get_loaded_controller() const;

  CONTROLLER_MANAGER_PUBLIC
  void register_controller_loader(ControllerLoaderInterfaceSharedPtr loader);

  template<
    typename T,
    typename std::enable_if<std::is_convertible<
      T *, controller_interface::ControllerInterface *>::value, T>::type * = nullptr>
  std::shared_ptr<controller_interface::ControllerInterface>
  add_controller(std::shared_ptr<T> controller, std::string controller_name)
  {
    return add_controller_impl(controller, controller_name);
  }

  CONTROLLER_MANAGER_PUBLIC
  controller_interface::return_type
  update();

  CONTROLLER_MANAGER_PUBLIC
  controller_interface::return_type
  configure() const;

  CONTROLLER_MANAGER_PUBLIC
  controller_interface::return_type
  activate() const;

  CONTROLLER_MANAGER_PUBLIC
  controller_interface::return_type
  deactivate() const;

  CONTROLLER_MANAGER_PUBLIC
  controller_interface::return_type
  cleanup() const;

protected:
  CONTROLLER_MANAGER_PUBLIC
  std::shared_ptr<controller_interface::ControllerInterface>
  add_controller_impl(
    std::shared_ptr<controller_interface::ControllerInterface> controller,
    const std::string & controller_name);

private:
  std::shared_ptr<hardware_interface::RobotHardware> hw_;
  std::shared_ptr<rclcpp::Executor> executor_;
  std::vector<ControllerLoaderInterfaceSharedPtr> loaders_;
  std::vector<std::shared_ptr<controller_interface::ControllerInterface>> loaded_controllers_;
};

}  // namespace controller_manager

#endif  // CONTROLLER_MANAGER__CONTROLLER_MANAGER_HPP_
