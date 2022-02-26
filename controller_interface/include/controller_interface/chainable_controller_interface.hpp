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

#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "controller_interface/visibility_control.h"
#include "hardware_interface/handle.hpp"

namespace controller_interface
{
/// Virtual class to implement when integrating a controller that can be preceded by another
/// controllers.
/**
 * Specialization of ControllerInterface class to force implementation of methods specific for
 * "chainable" controller, i.e., controller that can be preceded by an another controller, for
 * example inner controller of an control cascade.
 *
 */
class ChainableControllerInterface : public ControllerInterface
{
public:
  CONTROLLER_INTERFACE_PUBLIC
  ChainableControllerInterface() = default;

  CONTROLLER_INTERFACE_PUBLIC
  virtual ~ChainableControllerInterface() = default;

  CONTROLLER_INTERFACE_PUBLIC
  return_type update(const rclcpp::Time & time, const rclcpp::Duration & period) final;

  CONTROLLER_INTERFACE_PUBLIC
  bool is_chainable() const final;

  CONTROLLER_INTERFACE_PUBLIC
  std::vector<hardware_interface::CommandInterface> export_reference_interfaces() final;

  CONTROLLER_INTERFACE_PUBLIC
  bool set_chained_mode(bool chained_mode) final;

  CONTROLLER_INTERFACE_PUBLIC
  bool is_in_chained_mode() const final;

protected:
  /// Virtual method that each chainable controller should implement to export its chainable
  /// interfaces.
  /**
   * Each chainable controller implements this methods where all input (command) interfaces are
   * exported. The method has the same meaning as `export_command_interface` method from
   * hardware_interface::SystemInterface or hardware_interface::ActuatorInterface.
   *
   * \returns list of CommandInterfaces that other controller can use as their outputs.
   */
  virtual std::vector<hardware_interface::CommandInterface> on_export_reference_interfaces() = 0;

  /// Virtual method that each chainable controller should implement to switch chained mode.
  /**
   * Each chainable controller implements this methods to switch between "chained" and "external"
   * mode. In "chained" mode all external interfaces like subscriber and service servers are
   * disabled to avoid potential concurrency in input commands.
   *
   * \param[in] flag marking a switch to or from chained mode.
   *
   * \returns true if controller successfully switched between "chained" and "external" mode.
   */
  virtual bool on_set_chained_mode(bool chained_mode);

  /// Update reference from input topics when not in chained mode.
  /**
   * Each chainable controller implements this method to update reference from subscribers when not
   * in chained mode.
   *
   * \returns return_type::OK if update is successfully, otherwise return_type::ERROR.
   */
  virtual return_type update_reference_from_subscribers() = 0;

  /// Execute calculations of the controller and update command interfaces.
  /**
   * Update method for chainable controllers.
   * In this method is valid to assume that \reference_interfaces_ hold the values for calculation
   * of the commands in the current control step.
   * This means that this method is called after \update_reference_from_subscribers if controller is
   * not in chained mode.
   *
   * \returns return_type::OK if calculation and writing of interface is successfully, otherwise
   * return_type::ERROR.
   */
  virtual return_type update_and_write_commands(
    const rclcpp::Time & time, const rclcpp::Duration & period) = 0;

  /// Storage of values for reference interfaces
  std::vector<double> reference_interfaces_;

private:
  /// A flag marking is a chainable controller is currently preceded by another controller.
  bool in_chained_mode_ = false;
};

}  // namespace controller_interface

#endif  // CONTROLLER_INTERFACE__CHAINABLE_CONTROLLER_INTERFACE_HPP_
