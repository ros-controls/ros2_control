// Copyright 2020 - 2021 ros2_control Development Team
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
#include "hardware_interface/types/lifecycle_state_names.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

namespace hardware_interface
{
/// Virtual Class to implement when integrating a 1 DoF actuator into ros2_control.
/**
 * The typical examples are conveyors or motors.
 *
 * Methods return values have type
 * rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn with the following
 * meaning:
 *
 * \returns CallbackReturn::SUCCESS method execution was successful.
 * \returns CallbackReturn::FAILURE method execution has failed and and can be called again.
 * \returns CallbackReturn::ERROR critical error has happened that should be managed in
 * "on_error" method.
 *
 * The hardware ends after each method in a state with the following meaning:
 *
 * UNCONFIGURED (on_init, on_cleanup):
 *   Hardware is initialized but communication is not started and therefore no interface is
 * available.
 *
 * INACTIVE (on_configure, on_deactivate):
 *   Communication with the hardware is started and it is configured.
 *   States can be read and non-movement hardware interfaces commanded.
 *   Hardware interfaces for movement will NOT be available.
 *   Those interfaces are: HW_IF_POSITION, HW_IF_VELOCITY, HW_IF_ACCELERATION, and HW_IF_EFFORT.
 *
 * FINALIZED (on_shutdown):
 *   Hardware interface is ready for unloading/destruction.
 *   Allocated memory is cleaned up.
 *
 * ACTIVE (on_activate):
 *   Power circuits of hardware are active and hardware can be moved, e.g., brakes are disabled.
 *   Command interfaces for movement are available and have to be accepted.
 *   Those interfaces are: HW_IF_POSITION, HW_IF_VELOCITY, HW_IF_ACCELERATION, and HW_IF_EFFORT.
 */

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class ActuatorInterface : public rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface
{
public:
  ActuatorInterface()
  : lifecycle_state_(rclcpp_lifecycle::State(
      lifecycle_msgs::msg::State::PRIMARY_STATE_UNKNOWN, lifecycle_state_names::UNKNOWN))
  {
  }

  /// ActuatorInterface copy constructor is actively deleted.
  /**
   * Hardware interfaces are having a unique ownership and thus can't be copied in order to avoid
   * failed or simultaneous access to hardware.
   */
  ActuatorInterface(const ActuatorInterface & other) = delete;

  ActuatorInterface(ActuatorInterface && other) = default;

  virtual ~ActuatorInterface() = default;

  /// Initialization of the hardware interface from data parsed from the robot's URDF.
  /**
   * \param[in] hardware_info structure with data from URDF.
   * \returns CallbackReturn::SUCCESS if required data are provided and can be parsed.
   * \returns CallbackReturn::ERROR if any error happens or data are missing.
   */
  virtual CallbackReturn on_init(const HardwareInfo & hardware_info)
  {
    info_ = hardware_info;
    return CallbackReturn::SUCCESS;
  };

  /// Exports all state interfaces for this hardware interface.
  /**
   * The state interfaces have to be created and transferred according
   * to the hardware info passed in for the configuration.
   *
   * Note the ownership over the state interfaces is transferred to the caller.
   *
   * \return vector of state interfaces
   */
  virtual std::vector<StateInterface> export_state_interfaces() = 0;

  /// Exports all command interfaces for this hardware interface.
  /**
   * The command interfaces have to be created and transferred according
   * to the hardware info passed in for the configuration.
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
   * \note All starting and stopping interface keys are passed to all components, so the function
   * should return return_type::OK by default when given interface keys not relevant for this
   * component. \param[in] start_interfaces vector of string identifiers for the command interfaces
   * starting. \param[in] stop_interfaces vector of string identifiers for the command interfaces
   * stopping. \return return_type::OK if the new command interface combination can be prepared, or
   * if the interface key is not relevant to this system. Returns return_type::ERROR otherwise.
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
   * \note All starting and stopping interface keys are passed to all components, so the function
   * should return return_type::OK by default when given interface keys not relevant for this
   * component. \param[in] start_interfaces vector of string identifiers for the command interfaces
   * starting. \param[in] stop_interfaces vector of string identifiers for the command interfaces
   * stopping. \return return_type::OK if the new command interface combination can be switched to,
   * or if the interface key is not relevant to this system. Returns return_type::ERROR otherwise.
   */
  virtual return_type perform_command_mode_switch(
    const std::vector<std::string> & /*start_interfaces*/,
    const std::vector<std::string> & /*stop_interfaces*/)
  {
    return return_type::OK;
  }

  /// Read the current state values from the actuator.
  /**
   * The data readings from the physical hardware has to be updated
   * and reflected accordingly in the exported state interfaces.
   * That is, the data pointed by the interfaces shall be updated.
   *
   * \param[in] time The time at the start of this control loop iteration
   * \param[in] period The measured time taken by the last control loop iteration
   * \return return_type::OK if the read was successful, return_type::ERROR otherwise.
   */
  virtual return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) = 0;

  /// Write the current command values to the actuator.
  /**
   * The physical hardware shall be updated with the latest value from
   * the exported command interfaces.
   *
   * \param[in] time The time at the start of this control loop iteration
   * \param[in] period The measured time taken by the last control loop iteration
   * \return return_type::OK if the read was successful, return_type::ERROR otherwise.
   */
  virtual return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) = 0;

  /// Get name of the actuator hardware.
  /**
   * \return name.
   */
  virtual std::string get_name() const { return info_.name; }

  /// Get life-cycle state of the actuator hardware.
  /**
   * \return state.
   */
  const rclcpp_lifecycle::State & get_state() const { return lifecycle_state_; }

  /// Set life-cycle state of the actuator hardware.
  /**
   * \return state.
   */
  void set_state(const rclcpp_lifecycle::State & new_state) { lifecycle_state_ = new_state; }

protected:
  HardwareInfo info_;
  rclcpp_lifecycle::State lifecycle_state_;
};

}  // namespace hardware_interface
#endif  // HARDWARE_INTERFACE__ACTUATOR_INTERFACE_HPP_
