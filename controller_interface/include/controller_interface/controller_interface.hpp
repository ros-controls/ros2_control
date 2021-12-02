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
#include "lifecycle_msgs/msg/state.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"

namespace controller_interface
{
// TODO(karsten1987): Remove clang pragma within Galactic
#ifdef __clang__
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wc++17-extensions"
#endif
enum class return_type : std::uint8_t
{
  OK = 0,
  ERROR = 1,
  SUCCESS [[deprecated("Use controller_interface::return_type::OK instead.")]] = OK
};
#ifdef __clang__
#pragma clang diagnostic pop
#endif

/// Indicating which interfaces are to be claimed.
/**
 * One might either claim all available command/state interfaces,
 * specifying a set of individual interfaces,
 * or none at all.
 */
enum class interface_configuration_type : std::uint8_t
{
  ALL = 0,
  INDIVIDUAL = 1,
  NONE = 2,
};

/// Configuring what command/state interfaces to claim.
struct InterfaceConfiguration
{
  interface_configuration_type type;
  std::vector<std::string> names = {};
};

class ControllerInterface : public rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface
{
public:
  CONTROLLER_INTERFACE_PUBLIC
  ControllerInterface() = default;

  CONTROLLER_INTERFACE_PUBLIC
  virtual ~ControllerInterface() = default;

  CONTROLLER_INTERFACE_PUBLIC
  virtual InterfaceConfiguration command_interface_configuration() const = 0;

  CONTROLLER_INTERFACE_PUBLIC
  virtual InterfaceConfiguration state_interface_configuration() const = 0;

  CONTROLLER_INTERFACE_PUBLIC
  void assign_interfaces(
    std::vector<hardware_interface::LoanedCommandInterface> && command_interfaces,
    std::vector<hardware_interface::LoanedStateInterface> && state_interfaces);

  CONTROLLER_INTERFACE_PUBLIC
  void release_interfaces();

  CONTROLLER_INTERFACE_PUBLIC
  virtual LifecycleNodeInterface::CallbackReturn on_init() = 0;

  CONTROLLER_INTERFACE_PUBLIC
  virtual return_type init(const std::string & controller_name);

  CONTROLLER_INTERFACE_PUBLIC
  virtual return_type update(const rclcpp::Time & time, const rclcpp::Duration & period) = 0;

  CONTROLLER_INTERFACE_PUBLIC
  std::shared_ptr<rclcpp::Node> get_node();

  /// Declare and initialize a parameter with a type.
  /**
   *
   * Wrapper function for templated node's declare_parameter() which checks if
   * parameter is already declared.
   * For use in all components that inherit from ControllerInterface
   */
  template <typename ParameterT>
  auto auto_declare(const std::string & name, const ParameterT & default_value)
  {
    if (!node_->has_parameter(name))
    {
      return node_->declare_parameter<ParameterT>(name, default_value);
    }
    else
    {
      return node_->get_parameter(name).get_value<ParameterT>();
    }
  }

  /**
   * The methods below are a substitute to the LifecycleNode methods with the same name.
   * The Life cycle is shown in ROS2 design document:
   * https://design.ros2.org/articles/node_lifecycle.html
   * We cannot use a LifecycleNode because it would expose change-state services to the rest
   * of the ROS system.
   * Only the Controller Manager should have possibility to change state of a controller.
   *
   * Hopefully in the future we can use a LifecycleNode where we disable modifications from the outside.
   */
  CONTROLLER_INTERFACE_PUBLIC
  const rclcpp_lifecycle::State & configure();

  CONTROLLER_INTERFACE_PUBLIC
  const rclcpp_lifecycle::State & cleanup();

  CONTROLLER_INTERFACE_PUBLIC
  const rclcpp_lifecycle::State & deactivate();

  CONTROLLER_INTERFACE_PUBLIC
  const rclcpp_lifecycle::State & activate();

  CONTROLLER_INTERFACE_PUBLIC
  const rclcpp_lifecycle::State & shutdown();

  CONTROLLER_INTERFACE_PUBLIC
  const rclcpp_lifecycle::State & get_state() const;

  CONTROLLER_INTERFACE_PUBLIC
  unsigned int get_update_rate() const;

protected:
  std::vector<hardware_interface::LoanedCommandInterface> command_interfaces_;
  std::vector<hardware_interface::LoanedStateInterface> state_interfaces_;
  std::shared_ptr<rclcpp::Node> node_;
  rclcpp_lifecycle::State lifecycle_state_;
  unsigned int update_rate_ = 0;
};

using ControllerInterfaceSharedPtr = std::shared_ptr<ControllerInterface>;

}  // namespace controller_interface

#endif  // CONTROLLER_INTERFACE__CONTROLLER_INTERFACE_HPP_
