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
#include <utility>
#include <vector>

#include "realtime_tools/async_function_handler.hpp"

#include "hardware_interface/handle.hpp"
#include "hardware_interface/introspection.hpp"
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

/**
 * @brief Indicate which interfaces are to be claimed.
 *
 * A controller may choose to:
 * - Claim all available command/state interfaces.
 * - Specify a set of individual interfaces to be claimed.
 * - Claim no interfaces at all.
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

struct ControllerUpdateStats
{
  void reset()
  {
    total_triggers = 0;
    failed_triggers = 0;
  }

  unsigned int total_triggers;
  unsigned int failed_triggers;
};

/**
 * Struct to store the status of the controller update method.
 * The status contains information if the update was triggered successfully, the result of the
 * update method and the execution duration of the update method. The status is used to provide
 * feedback to the controller_manager.
 * @var successful: true if the update was triggered successfully, false if not.
 * @var result: return_type::OK if update is successfully, otherwise return_type::ERROR.
 * @var execution_time: duration of the execution of the update method.
 * @var period: period of the update method.
 */
struct ControllerUpdateStatus
{
  bool successful = true;
  return_type result = return_type::OK;
  std::optional<std::chrono::nanoseconds> execution_time = std::nullopt;
  std::optional<rclcpp::Duration> period = std::nullopt;
};

/**
 * Base interface class  for an controller. The interface may not be used to implement a controller.
 * The class provides definitions for `ControllerInterface` and `ChainableControllerInterface`
 * that should be implemented and extended for a specific controller.
 */
class ControllerInterfaceBase : public rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface
{
public:
  ControllerInterfaceBase() = default;

  virtual ~ControllerInterfaceBase();

  /**
 * @brief Retrieve the configuration for the controller's required command interfaces.
 *
 * This method is used by the `controller_manager` to obtain the set of command interfaces 
 * utilized by the controller. Each controller may use its own method to determine interface 
 * names, which in the simplest case follow the format: `<joint>/<interface>`. 
 *
 * This method is called only when the controller is in the `inactive` or `active` state, 
 * meaning `on_configure` must be called first. The configuration is used to verify whether 
 * the controller can be activated and to claim interfaces from the hardware. 
 *
 * The claimed interfaces are populated in the 
 * `ControllerInterfaceBase::command_interfaces_` member.
 *
 * @return Configuration of command interfaces.
 */

  virtual InterfaceConfiguration command_interface_configuration() const = 0;

  /**
 * @brief Retrieve the configuration for the controller's required state interfaces.
 *
 * This method is used by the `controller_manager` to obtain the set of state interfaces 
 * utilized by the controller. Each controller may use its own method to determine interface 
 * names, which in the simplest case follow the format: `<joint>/<interface>`. 
 *
 * This method is called only when the controller is in the `inactive` or `active` state, 
 * meaning `on_configure` must be called first. The configuration is used to verify whether 
 * the controller can be activated and to claim interfaces from the hardware. 
 *
 * The claimed interfaces are populated in the 
 * `ControllerInterfaceBase::state_interfaces_` member.
 *
 * @return Configuration of state interfaces.
 */

  virtual InterfaceConfiguration state_interface_configuration() const = 0;

  /**
 * @brief Assign loaned interfaces to the controller.
 *
 * This method is used by the `controller_manager` to assign command and state interfaces 
 * to the controller.
 *
 * @note When overriding this method, the user must also override the `release_interfaces` 
 *       method to properly release the assigned interfaces.
 *
 * @param[in] command_interfaces Vector of command interfaces to be assigned to the controller.
 * @param[in] state_interfaces Vector of state interfaces to be assigned to the controller.
 */

  virtual void assign_interfaces(
    std::vector<hardware_interface::LoanedCommandInterface> && command_interfaces,
    std::vector<hardware_interface::LoanedStateInterface> && state_interfaces);

 /**
 * @brief Release loaned interfaces from the controller.
 *
 * This method is used by the `controller_manager` to release the interfaces assigned 
 * to the controller.
 */

  virtual void release_interfaces();

  return_type init(
    const std::string & controller_name, const std::string & urdf, unsigned int cm_update_rate,
    const std::string & node_namespace, const rclcpp::NodeOptions & node_options);

  /**
 * @brief Custom configure method to read additional parameters for controller nodes.
 *
 * Overrides the default `LifecycleNode` configuration method to retrieve parameters.
 */

  const rclcpp_lifecycle::State & configure();

  /// Extending interface with initialization method which is individual for each controller
  virtual CallbackReturn on_init() = 0;

  /**
   * Control step update. Command interfaces are updated based on on reference inputs and current
   * states.
   * **The method called in the (real-time) control loop.**
   *
   * \param[in] time The time at the start of this control loop iteration
   * \param[in] period The measured time since the last control loop iteration
   * \returns return_type::OK if update is successfully, otherwise return_type::ERROR.
   */
  virtual return_type update(const rclcpp::Time & time, const rclcpp::Duration & period) = 0;

  /**
   * Trigger update method. This method is used by the controller_manager to trigger the update
   * method of the controller.
   * The method is used to trigger the update method of the controller synchronously or
   * asynchronously, based on the controller configuration.
   * **The method called in the (real-time) control loop.**
   *
   * \param[in] time The time at the start of this control loop iteration
   * \param[in] period The measured time taken by the last control loop iteration
   * \returns ControllerUpdateStatus. The status contains information if the update was triggered
   * successfully, the result of the update method and the execution duration of the update method.
   */
  ControllerUpdateStatus trigger_update(const rclcpp::Time & time, const rclcpp::Duration & period);

  std::shared_ptr<rclcpp_lifecycle::LifecycleNode> get_node();

  std::shared_ptr<const rclcpp_lifecycle::LifecycleNode> get_node() const;

  const rclcpp_lifecycle::State & get_lifecycle_state() const;

  unsigned int get_update_rate() const;

  bool is_async() const;

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

  /**
 * @brief Declare and initialize a parameter with a specified type.
 *
 * Wrapper function for the templated `declare_parameter()` method of the node, 
 * which checks if the parameter is already declared.
 *
 * This function is intended for use in all components that inherit from 
 * `ControllerInterfaceBase`.
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

  /**
 * @brief Check whether the controller is chainable.
 *
 * Determines if the controller supports chaining with other controllers.
 *
 * @return `true` if the controller is chainable, `false` otherwise.
 */

  virtual bool is_chainable() const = 0;

  /**
   * Export interfaces for a chainable controller that can be used as command interface of other
   * controllers.
   *
   * \returns list of command interfaces for preceding controllers.
   */
  virtual std::vector<hardware_interface::CommandInterface::SharedPtr>
  export_reference_interfaces() = 0;

  /**
   * Export interfaces for a chainable controller that can be used as state interface by other
   * controllers.
   *
   * \returns list of state interfaces for preceding controllers.
   */
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
  virtual bool set_chained_mode(bool chained_mode) = 0;

  /**
 * @brief Check whether the controller is currently in chained mode.
 *
 * Determines if the controller is operating in chained mode. In this mode, only internal 
 * interfaces are available, and all subscribers are expected to be disabled. This prevents 
 * concurrent writing to the controller's inputs from multiple sources.
 *
 * @return `true` if the controller is in chained mode, `false` otherwise.
 */

  virtual bool is_in_chained_mode() const = 0;

  /**
   * Method to wait for any running async update cycle to finish after finishing the current cycle.
   * This is needed to be called before deactivating the controller by the controller_manager, so
   * that the interfaces still exist when the controller finishes its cycle and then it's exits.
   *
   * \note **The method is not real-time safe and shouldn't be called in the control loop.**
   *
   * If the controller is running in async mode, the method will wait for the current async update
   * to finish. If the controller is not running in async mode, the method will do nothing.
   */
  void wait_for_trigger_update_to_finish();

  std::string get_name() const;

 /**
 * @brief Enable or disable introspection of the controller.
 *
 * Controls whether introspection is enabled for the controller.
 *
 * @param[in] enable Set to `true` to enable introspection, `false` to disable it.
 */

  void enable_introspection(bool enable);

protected:
  std::vector<hardware_interface::LoanedCommandInterface> command_interfaces_;
  std::vector<hardware_interface::LoanedStateInterface> state_interfaces_;

private:
  std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node_;
  std::unique_ptr<realtime_tools::AsyncFunctionHandler<return_type>> async_handler_;
  unsigned int update_rate_ = 0;
  bool is_async_ = false;
  std::string urdf_ = "";
  ControllerUpdateStats trigger_stats_;

protected:
  pal_statistics::RegistrationsRAII stats_registrations_;
};

using ControllerInterfaceBaseSharedPtr = std::shared_ptr<ControllerInterfaceBase>;

}  // namespace controller_interface

#endif  // CONTROLLER_INTERFACE__CONTROLLER_INTERFACE_BASE_HPP_
