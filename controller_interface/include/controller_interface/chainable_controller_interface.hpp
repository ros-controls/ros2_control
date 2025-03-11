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

#ifndef CONTROLLER_INTERFACE__CHAINABLE_CONTROLLER_INTERFACE_HPP_
#define CONTROLLER_INTERFACE__CHAINABLE_CONTROLLER_INTERFACE_HPP_

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "controller_interface/controller_interface_base.hpp"
#include "hardware_interface/handle.hpp"

namespace controller_interface
{
/**
 * @brief Virtual class for integrating a controller that can be preceded by other controllers.
 *
 * This is a specialization of the ControllerInterface class that enforces the implementation 
 * of methods specific to "chainable" controllers. A chainable controller can be preceded 
 * by another controller, such as an inner controller in a control cascade.
 */

class ChainableControllerInterface : public ControllerInterfaceBase
{
public:
  ChainableControllerInterface();

  virtual ~ChainableControllerInterface() = default;

  /**
   * Control step update. Command interfaces are updated based on on reference inputs and current
   * states.
   * **The method called in the (real-time) control loop.**
   *
   * \param[in] time The time at the start of this control loop iteration
   * \param[in] period The measured time taken by the last control loop iteration
   * \returns return_type::OK if update is successfully, otherwise return_type::ERROR.
   */
  return_type update(const rclcpp::Time & time, const rclcpp::Duration & period) final;

  bool is_chainable() const final;

  std::vector<hardware_interface::StateInterface::ConstSharedPtr> export_state_interfaces() final;

  std::vector<hardware_interface::CommandInterface::SharedPtr> export_reference_interfaces() final;

  bool set_chained_mode(bool chained_mode) final;

  bool is_in_chained_mode() const final;

protected:
/**
 * @brief Virtual method to export read-only chainable interfaces.
 *
 * Each chainable controller must implement this method to export all its state (read-only) 
 * interfaces. This method serves the same purpose as the `export_state_interfaces` method 
 * in `hardware_interface::SystemInterface` or `hardware_interface::ActuatorInterface`.
 *
 * @return List of StateInterfaces that other controllers can use as inputs.
 */

  virtual std::vector<hardware_interface::StateInterface> on_export_state_interfaces();

  /**
 * @brief Virtual method to export read/write chainable interfaces.
 *
 * Each chainable controller must implement this method to export all input (command) interfaces. 
 * This method serves the same purpose as the `export_command_interface` method in 
 * `hardware_interface::SystemInterface` or `hardware_interface::ActuatorInterface`.
 *
 * @return List of CommandInterfaces that other controllers can use as their outputs.
 */

  virtual std::vector<hardware_interface::CommandInterface> on_export_reference_interfaces();

 /**
 * @brief Virtual method to switch the chained mode of a chainable controller.
 *
 * Each chainable controller must implement this method to switch between "chained" and "external" 
 * modes. In "chained" mode, all external interfaces, such as subscribers and service servers, 
 * are disabled to prevent potential concurrency in input commands.
 *
 * @param[in] flag Indicates whether to switch to or from chained mode.
 * @return True if the controller successfully switched between "chained" and "external" modes.
 * @note The default implementation returns true, so this method does not need to be overridden 
 *       if the controller can always switch modes.
 */

  virtual bool on_set_chained_mode(bool chained_mode);

  /**
 * @brief Update the reference from input topics when not in chained mode.
 *
 * Each chainable controller must implement this method to update the reference 
 * from subscribers when not operating in chained mode.
 *
 * @return `return_type::OK` if the update is successful, otherwise `return_type::ERROR`.
 */

  virtual return_type update_reference_from_subscribers(
    const rclcpp::Time & time, const rclcpp::Duration & period) = 0;

  /**
 * @brief Execute controller calculations and update command interfaces.
 *
 * This is the update method for chainable controllers. It is valid to assume that 
 * `reference_interfaces_` hold the values needed for command calculations in the current 
 * control step. This method is called after `update_reference_from_subscribers` if the controller 
 * is not in chained mode.
 *
 * @return `return_type::OK` if the calculation and writing to the interface are successful, 
 *         otherwise `return_type::ERROR`.
 */

  virtual return_type update_and_write_commands(
    const rclcpp::Time & time, const rclcpp::Duration & period) = 0;

  /// Storage of values for state interfaces
  std::vector<std::string> exported_state_interface_names_;
  std::vector<hardware_interface::StateInterface::SharedPtr> ordered_exported_state_interfaces_;
  std::unordered_map<std::string, hardware_interface::StateInterface::SharedPtr>
    exported_state_interfaces_;
  // BEGIN (Handle export change): for backward compatibility
  std::vector<double> state_interfaces_values_;
  // END

  /// Storage of values for reference interfaces
  std::vector<std::string> exported_reference_interface_names_;
  // BEGIN (Handle export change): for backward compatibility
  std::vector<double> reference_interfaces_;
  // END
  std::vector<hardware_interface::CommandInterface::SharedPtr>
    ordered_exported_reference_interfaces_;
  std::unordered_map<std::string, hardware_interface::CommandInterface::SharedPtr>
    exported_reference_interfaces_;

private:
  /// A flag marking if a chainable controller is currently preceded by another controller.
  bool in_chained_mode_ = false;
};

}  // namespace controller_interface

#endif  // CONTROLLER_INTERFACE__CHAINABLE_CONTROLLER_INTERFACE_HPP_
