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

#ifndef CONTROLLER_INTERFACE__CONTROLLER_INTERFACE_HPP_
#define CONTROLLER_INTERFACE__CONTROLLER_INTERFACE_HPP_

#include <memory>
#include <string>
#include <vector>

#include "controller_interface/visibility_control.h"

#include "hardware_interface/loaned_command_interface.hpp"
#include "hardware_interface/loaned_state_interface.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

namespace controller_interface
{

enum class return_type : std::uint8_t
{
  SUCCESS = 0,
  ERROR = 1,
};

enum class configuration_type : std::uint8_t
{
  ALL = 0,
  INDIVIDUAL = 1,
  NONE = 2,
};

struct InterfaceConfiguration
{
  configuration_type type;
  std::vector<std::string> names = {};
};

class ControllerInterface : public rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface
{
public:
  CONTROLLER_INTERFACE_PUBLIC
  ControllerInterface() = default;

  CONTROLLER_INTERFACE_PUBLIC
  virtual
  ~ControllerInterface() = default;

  CONTROLLER_INTERFACE_PUBLIC
  virtual
  InterfaceConfiguration command_interface_configuration() const = 0;

  CONTROLLER_INTERFACE_PUBLIC
  virtual
  InterfaceConfiguration state_interface_configuration() const = 0;

  CONTROLLER_INTERFACE_PUBLIC
  void assign_interfaces(
    std::vector<hardware_interface::LoanedCommandInterface> && command_interfaces,
    std::vector<hardware_interface::LoanedStateInterface> && state_interfaces);

  CONTROLLER_INTERFACE_PUBLIC
  void release_interfaces();

  CONTROLLER_INTERFACE_PUBLIC
  virtual
  return_type
  init(const std::string & controller_name);

  CONTROLLER_INTERFACE_PUBLIC
  virtual
  return_type
  update() = 0;

  CONTROLLER_INTERFACE_PUBLIC
  std::shared_ptr<rclcpp_lifecycle::LifecycleNode>
  get_lifecycle_node();

protected:
  std::vector<hardware_interface::LoanedCommandInterface> command_interfaces_;
  std::vector<hardware_interface::LoanedStateInterface> state_interfaces_;
  std::shared_ptr<rclcpp_lifecycle::LifecycleNode> lifecycle_node_;
};

using ControllerInterfaceSharedPtr = std::shared_ptr<ControllerInterface>;

}  // namespace controller_interface

#endif  // CONTROLLER_INTERFACE__CONTROLLER_INTERFACE_HPP_
