// Copyright 2020 ros2_control Development Team
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

#ifndef HARDWARE_INTERFACE__ACTUATOR_INTERFACE_HPP_
#define HARDWARE_INTERFACE__ACTUATOR_INTERFACE_HPP_

#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_status_values.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

namespace hardware_interface
{
/// Virtual Class to implement when integrating a 1 DoF actuator into ros2_control.
/**
  * The typical examples are conveyors or motors.
  */
class ActuatorInterface
{
public:
  ActuatorInterface() = default;

  virtual ~ActuatorInterface() = default;

  /// Initialization of the actuator from data parsed from the robot's URDF.
  /**
   * \param[in] actuator_info structure with data from URDF.
   * \returns CallbackReturn::SUCCESS if required data are provided and can be parsed.
   * \returns CallbackReturn::ERROR if any error happens or data are missing.
   */
  virtual CallbackReturn on_init(const HardwareInfo & actuator_info) = 0;

  /// Configuration of the hardware and start communication with it.
  /**
   * Configuration of the hardware. It start communication with hardware and configures it.
   * After this method finishes states can be read, and non-movement hardware interfaces commanded.
   *
   * Hardware interfaces for movement (will NOT be available) are:
   * HW_IF_POSITION, HW_IF_VELOCITY, HW_IF_ACCELERATION, and HW_IF_EFFORT.
   *
   * \returns LifecycleNodeInterface::CallbackReturn::SUCCESS if configuration is successful.
   * \returns LifecycleNodeInterface::CallbackReturn::FAILURE if configuration fails and can be
   * tried again.
   * \returns LifecycleNodeInterface::CallbackReturn::ERROR critical error has happened that
   * should be managed in "on_error" method.
   */
  virtual CallbackReturn on_configure() { return CallbackReturn::SUCCESS; }

  /// Cleanup hardware configuration and stop communication with it.
  /**
   * Cleanup of the hardware. Communication with hardware is stopped.
   * Interfaces are not available anymore.
   *
   * \returns LifecycleNodeInterface::CallbackReturn::SUCCESS if everything worked as expected.
   * \returns LifecycleNodeInterface::CallbackReturn::FAILURE if cleanup fails and can be
   * tried again.
   * \returns LifecycleNodeInterface::CallbackReturn::ERROR critical error has happened that
   * should be managed in "on_error" method.
   */
  virtual CallbackReturn on_cleanup() { return CallbackReturn::SUCCESS; }

  /// Shutdown hardware interface and prepare plugin for destruction.
  /**
   * Shutdown hardware interface.
   * Cleanup memory allocations and prepare plugin for clean unloading/destruction.
   *
   * \param[in] previous_state state from which the method is called.
   * \returns LifecycleNodeInterface::CallbackReturn::SUCCESS if everything worked as expected.
   * \returns LifecycleNodeInterface::CallbackReturn::ERROR critical error has happened that
   * should be managed in "on_error" method.
   */
  virtual CallbackReturn on_shutdown(const rclcpp_lifecycle::State & /*previous_state*/)
  {
    return CallbackReturn::SUCCESS;
  }

  /// Activate hardware power and enable its movement.
  /**
   * Activate power circuits of the hardware and prepare everything for commanding movements, e.g.,
   * disable brakes.
   * Command interfaces for movement has to be available after this and movement commands have to
   * be accepted.
   *
   * Hardware interfaces for movement are:
   * HW_IF_POSITION, HW_IF_VELOCITY, HW_IF_ACCELERATION, and HW_IF_EFFORT.
   *
   * \returns LifecycleNodeInterface::CallbackReturn::SUCCESS if everything worked as expected.
   * \returns LifecycleNodeInterface::CallbackReturn::FAILURE if activation fails and can be
   * tried again.
   * \returns LifecycleNodeInterface::CallbackReturn::ERROR critical error has happened that
   * should be managed in "on_error" method.
   */
  virtual CallbackReturn on_activate() = 0;

  /// Deactivate hardware power and disable its movement.
  /**
   * Deactivate power circuits of the hardware and disable possibility to move, e.g., activate
   * breaks.
   * Command interface for movement are disabled and movement commands are ignored
   *
   * Hardware interfaces for movement are:
   * HW_IF_POSITION, HW_IF_VELOCITY, HW_IF_ACCELERATION, and HW_IF_EFFORT.
   *
   * \returns LifecycleNodeInterface::CallbackReturn::SUCCESS if everything worked as expected.
   * \returns LifecycleNodeInterface::CallbackReturn::FAILURE if deactivation fails and can be
   * tried again.
   * \returns LifecycleNodeInterface::CallbackReturn::ERROR critical error has happened that
   * should be managed in "on_error" method.
   */
  virtual CallbackReturn on_deactivate() = 0;

  /// Deal with critical errors that happen on state changes.
  /**
   * Deal with critical errors that happen on state changes.
   * The method is called from any state.
   *
   * \param[in] previous_state state from which the method is called.
   * \returns LifecycleNodeInterface::CallbackReturn::SUCCESS if everything worked as expected.
   * \returns LifecycleNodeInterface::CallbackReturn::FAILURE if any error happens.
   */
  virtual CallbackReturn on_error(const rclcpp_lifecycle::State & /*previous_state*/)
  {
    return CallbackReturn::SUCCESS;
  }

  /// Exports all state interfaces for this actuator.
  /**
   * The state interfaces have to be created and transferred according
   * to the actuator info passed in for the configuration.
   *
   * Note the ownership over the state interfaces is transferred to the caller.
   *
   * \return vector of state interfaces
   */
  virtual std::vector<StateInterface> export_state_interfaces() = 0;

  /// Exports all command interfaces for this actuator.
  /**
   * The command interfaces have to be created and transferred according
   * to the actuator info passed in for the configuration.
   *
   * Note the ownership over the state interfaces is transferred to the caller.
   *
   * \return vector of command interfaces
   */
  virtual std::vector<CommandInterface> export_command_interfaces() = 0;

  /// Prepare for a new command interface switch.
  /**
   * Prepare for any mode-switching required by the new command interface combination.
   *
   * \note This is a non-realtime evaluation of whether a set of command interface claims are
   * possible, and call to start preparing data structures for the upcoming switch that will occur.
   * \note All starting and stopping interface keys are passed to all components, so the function should
   * return return_type::OK by default when given interface keys not relevant for this component.
   * \param[in] start_interfaces vector of string identifiers for the command interfaces starting.
   * \param[in] stop_interfaces vector of string identifiers for the command interfacs stopping.
   * \return return_type::OK if the new command interface combination can be prepared,
   * or if the interface key is not relevant to this system. Returns return_type::ERROR otherwise.
   */
  virtual return_type prepare_command_mode_switch(
    const std::vector<std::string> & /*start_interfaces*/,
    const std::vector<std::string> & /*stop_interfaces*/)
  {
    return return_type::OK;
  }

  // Perform switching to the new command interface.
  /**
   * Perform the mode-switching for the new command interface combination.
   *
   * \note This is part of the realtime update loop, and should be fast.
   * \note All starting and stopping interface keys are passed to all components, so the function should
   * return return_type::OK by default when given interface keys not relevant for this component.
   * \param[in] start_interfaces vector of string identifiers for the command interfaces starting.
   * \param[in] stop_interfaces vector of string identifiers for the command interfacs stopping.
   * \return return_type::OK if the new command interface combination can be switched to,
   * or if the interface key is not relevant to this system. Returns return_type::ERROR otherwise.
   */
  virtual return_type perform_command_mode_switch(
    const std::vector<std::string> & /*start_interfaces*/,
    const std::vector<std::string> & /*stop_interfaces*/)
  {
    return return_type::OK;
  }

  /// Get name of the actuator hardware.
  /**
   * \return name.
   */
  virtual std::string get_name() const = 0;

  /// Read the current state values from the actuator.
  /**
   * The data readings from the physical hardware has to be updated
   * and reflected accordingly in the exported state interfaces.
   * That is, the data pointed by the interfaces shall be updated.
   *
   * \return return_type::OK if the read was successful, return_type::ERROR otherwise.
   */
  virtual return_type read() = 0;

  /// Write the current command values to the actuator.
  /**
   * The physical hardware shall be updated with the latest value from
   * the exported command interfaces.
   *
   * \return return_type::OK if the read was successful, return_type::ERROR otherwise.
   */
  virtual return_type write() = 0;
};

}  // namespace hardware_interface
#endif  // HARDWARE_INTERFACE__ACTUATOR_INTERFACE_HPP_
