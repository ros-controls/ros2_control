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

#ifndef CONTROLLER_INTERFACE__CONTROLLER_INTERFACE_BASE_HPP_
#define CONTROLLER_INTERFACE__CONTROLLER_INTERFACE_BASE_HPP_

#include <memory>
#include <string>
#include <vector>

#include "controller_interface/visibility_control.h"

#include "hardware_interface/handle.hpp"
#include "hardware_interface/loaned_command_interface.hpp"
#include "hardware_interface/loaned_state_interface.hpp"

#include "rclcpp/version.h"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

namespace controller_interface
{
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

enum class return_type : std::uint8_t
{
  OK = 0,
  ERROR = 1,
};

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

/**
 * Base interface class  for an controller. The interface may not be used to implement a controller.
 * The class provides definitions for `ControllerInterface` and `ChainableControllerInterface`
 * that should be implemented and extended for a specific controller.
 */
class ControllerInterfaceBase : public rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface
{
public:
  CONTROLLER_INTERFACE_PUBLIC
  ControllerInterfaceBase() = default;

  CONTROLLER_INTERFACE_PUBLIC
  virtual ~ControllerInterfaceBase() = default;

  /// Get configuration for controller's required command interfaces.
  /**
   * Method used by the controller_manager to get the set of command interfaces used by the
   * controller. Each controller can use individual method to determine interface names that in
   * simples case have the following format: `<joint>/<interface>`. The method is called only in
   * `inactive` or `active` state, i.e., `on_configure` has to be called first. The configuration is
   * used to check if controller can be activated and to claim interfaces from hardware. The claimed
   * interfaces are populated in the \ref ControllerInterfaceBase::command_interfaces_
   * "command_interfaces_" member.
   *
   * \returns configuration of command interfaces.
   */
  CONTROLLER_INTERFACE_PUBLIC
  virtual InterfaceConfiguration command_interface_configuration() const = 0;

  /// Get configuration for controller's required state interfaces.
  /**
   * Method used by the controller_manager to get the set of state interface used by the controller.
   * Each controller can use individual method to determine interface names that in simples case
   * have the following format: `<joint>/<interface>`.
   * The method is called only in `inactive` or `active` state, i.e., `on_configure` has to be
   * called first.
   * The configuration is used to check if controller can be activated and to claim interfaces from
   * hardware.
   * The claimed interfaces are populated in the
   * \ref ControllerInterfaceBase::state_interface_ "state_interface_" member.
   *
   * \returns configuration of state interfaces.
   */
  CONTROLLER_INTERFACE_PUBLIC
  virtual InterfaceConfiguration state_interface_configuration() const = 0;

  /// Method that assigns the Loaned interfaces to the controller.
  /**
   * Method used by the controller_manager to assign the interfaces to the controller.
   * \note When this method is overridden, the user has to also implement the `release_interfaces`
   * method by overriding it to release the interfaces.
   *
   * \param[in] command_interfaces vector of command interfaces to be assigned to the controller.
   * \param[in] state_interfaces vector of state interfaces to be assigned to the controller.
   */
  CONTROLLER_INTERFACE_PUBLIC
  virtual void assign_interfaces(
    std::vector<hardware_interface::LoanedCommandInterface> && command_interfaces,
    std::vector<hardware_interface::LoanedStateInterface> && state_interfaces);

  /// Method that releases the Loaned interfaces from the controller.
  /**
   * Method used by the controller_manager to release the interfaces from the controller.
   */
  CONTROLLER_INTERFACE_PUBLIC
  virtual void release_interfaces();

  CONTROLLER_INTERFACE_PUBLIC
  return_type init(
    const std::string & controller_name, const std::string & urdf, unsigned int cm_update_rate,
    const std::string & node_namespace, const rclcpp::NodeOptions & node_options);

  /// Custom configure method to read additional parameters for controller-nodes
  /*
   * Override default implementation for configure of LifecycleNode to get parameters.
   */
  CONTROLLER_INTERFACE_PUBLIC
  const rclcpp_lifecycle::State & configure();

  /// Extending interface with initialization method which is individual for each controller
  CONTROLLER_INTERFACE_PUBLIC
  virtual CallbackReturn on_init() = 0;

  /**
   * Control step update. Command interfaces are updated based on on reference inputs and current
   * states.
   * **The method called in the (real-time) control loop.**
   *
   * \param[in] time The time at the start of this control loop iteration
   * \param[in] period The measured time taken by the last control loop iteration
   * \returns return_type::OK if update is successfully, otherwise return_type::ERROR.
   */
  CONTROLLER_INTERFACE_PUBLIC
  virtual return_type update(const rclcpp::Time & time, const rclcpp::Duration & period) = 0;

  CONTROLLER_INTERFACE_PUBLIC
  std::shared_ptr<rclcpp_lifecycle::LifecycleNode> get_node();

  CONTROLLER_INTERFACE_PUBLIC
  std::shared_ptr<const rclcpp_lifecycle::LifecycleNode> get_node() const;

  CONTROLLER_INTERFACE_PUBLIC
  const rclcpp_lifecycle::State & get_lifecycle_state() const;

  CONTROLLER_INTERFACE_PUBLIC
  unsigned int get_update_rate() const;

  CONTROLLER_INTERFACE_PUBLIC
  bool is_async() const;

  CONTROLLER_INTERFACE_PUBLIC
  const std::string & get_robot_description() const;

  /**
   * Method used by the controller_manager for base NodeOptions to instantiate the Lifecycle node
   * of the controller upon loading the controller.
   *
   * \note The controller_manager will modify these NodeOptions in case a params file is passed
   * by the spawner to load the controller parameters or when controllers are loaded in simulation
   * (see ros2_control#1311, ros2_controllers#698 , ros2_controllers#795,ros2_controllers#966 for
   * more details)
   *
   * @returns NodeOptions required for the configuration of the controller lifecycle node
   */
  CONTROLLER_INTERFACE_PUBLIC
  virtual rclcpp::NodeOptions define_custom_node_options() const
  {
    rclcpp::NodeOptions node_options;
// \note The versions conditioning is added here to support the source-compatibility with Humble
#if RCLCPP_VERSION_MAJOR >= 21
    node_options.enable_logger_service(true);
#else
    node_options.allow_undeclared_parameters(true);
    node_options.automatically_declare_parameters_from_overrides(true);
#endif
    return node_options;
  }

  /// Declare and initialize a parameter with a type.
  /**
   *
   * Wrapper function for templated node's declare_parameter() which checks if
   * parameter is already declared.
   * For use in all components that inherit from ControllerInterfaceBase
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

  // Methods for chainable controller types with default values so we can put all controllers into
  // one list in Controller Manager

  /// Get information if a controller is chainable.
  /**
   * Get information if a controller is chainable.
   *
   * \returns true is controller is chainable and false if it is not.
   */
  CONTROLLER_INTERFACE_PUBLIC
  virtual bool is_chainable() const = 0;

  /**
   * Export interfaces for a chainable controller that can be used as command interface of other
   * controllers.
   *
   * \returns list of command interfaces for preceding controllers.
   */
  CONTROLLER_INTERFACE_PUBLIC
  virtual std::vector<hardware_interface::CommandInterface::SharedPtr>
  export_reference_interfaces() = 0;

  /**
   * Export interfaces for a chainable controller that can be used as state interface by other
   * controllers.
   *
   * \returns list of state interfaces for preceding controllers.
   */
  CONTROLLER_INTERFACE_PUBLIC
  virtual std::vector<hardware_interface::StateInterface::ConstSharedPtr>
  export_state_interfaces() = 0;

  /**
   * Set chained mode of a chainable controller. This method triggers internal processes to switch
   * a chainable controller to "chained" mode and vice-versa. Setting controller to "chained" mode
   * usually involves the usage of the controller's reference interfaces by the other
   * controllers
   *
   * \returns true if mode is switched successfully and false if not.
   */
  CONTROLLER_INTERFACE_PUBLIC
  virtual bool set_chained_mode(bool chained_mode) = 0;

  /// Get information if a controller is currently in chained mode.
  /**
   * Get information about controller if it is currently used in chained mode. In chained mode only
   * internal interfaces are available and all subscribers are expected to be disabled. This
   * prevents concurrent writing to controller's inputs from multiple sources.
   *
   * \returns true is controller is in chained mode and false if it is not.
   */
  CONTROLLER_INTERFACE_PUBLIC
  virtual bool is_in_chained_mode() const = 0;

protected:
  std::vector<hardware_interface::LoanedCommandInterface> command_interfaces_;
  std::vector<hardware_interface::LoanedStateInterface> state_interfaces_;

private:
  std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node_;
  unsigned int update_rate_ = 0;
  bool is_async_ = false;
  std::string urdf_ = "";
};

using ControllerInterfaceBaseSharedPtr = std::shared_ptr<ControllerInterfaceBase>;

}  // namespace controller_interface

#endif  // CONTROLLER_INTERFACE__CONTROLLER_INTERFACE_BASE_HPP_
