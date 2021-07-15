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

#ifndef HARDWARE_INTERFACE__SYSTEM_INTERFACE_HPP_
#define HARDWARE_INTERFACE__SYSTEM_INTERFACE_HPP_

#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_status_values.hpp"
#include "hardware_interface/visibility_control.h"

namespace hardware_interface
{

/// Virtual Class to implement when integrating a complex system into ros2_control.
/**
* The common examples for these types of hardware are multi-joint systems with or without sensors
* such as industrial or humanoid robots.
*/
class SystemInterface
{
public:
  SystemInterface() = default;

  virtual
  ~SystemInterface() = default;

  /// Configuration of the system from data parsed from the robot's URDF.
  /**
   * \param[in] system_info structure with data from URDF.
   * \return return_type::OK if required data are provided and can be parsed,
   * return_type::ERROR otherwise.
   */
  virtual
  return_type configure(const HardwareInfo & system_info) = 0;

  /// Exports all state interfaces for this system.
  /**
   * The state interfaces have to be created and transfered according
   * to the system info passed in for the configuration.
   *
   * Note the ownership over the state interfaces is transfered to the caller.
   *
   *
   * \return vector of state interfaces
   */
  virtual
  std::vector<StateInterface> export_state_interfaces() = 0;

  /// Exports all command interfaces for this system.
  /**
   * The command interfaces have to be created and transfered according
   * to the system info passed in for the configuration.
   *
   * Note the ownership over the state interfaces is transfered to the caller.
   *
   * \return vector of command interfaces
   */
  virtual
  std::vector<CommandInterface> export_command_interfaces() = 0;

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
  virtual
  return_type prepare_command_mode_switch(
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
  virtual
  return_type perform_command_mode_switch(
    const std::vector<std::string> & /*start_interfaces*/,
    const std::vector<std::string> & /*stop_interfaces*/)
  {
    return return_type::OK;
  }

  /// Start exchange data with the hardware.
  /**
   * \return return_type:OK if everything worked as expected, return_type::ERROR otherwise.
   */
  virtual
  return_type start() = 0;

  /// Stop exchange data with the hardware.
  /**
   * \return return_type:OK if everything worked as expected, return_type::ERROR otherwise.
   */
  virtual
  return_type stop() = 0;

  /// Get name of the system hardware.
  /**
   * \return name.
   */
  virtual
  std::string get_name() const = 0;

  /// Get current state of the system hardware.
  /**
   * \return current status.
   */
  virtual
  status get_status() const = 0;

  /// Read the current state values from the actuators and sensors within the system.
  /**
   * The data readings from the physical hardware has to be updated
   * and reflected accordingly in the exported state interfaces.
   * That is, the data pointed by the interfaces shall be updated.
   *
   * \return return_type::OK if the read was successful, return_type::ERROR otherwise.
   */
  virtual
  return_type read() = 0;

  /// Write the current command values to the actuator within the system.
  /**
   * The physical hardware shall be updated with the latest value from
   * the exported command interfaces.
   *
   * \return return_type::OK if the read was successful, return_type::ERROR otherwise.
   */
  virtual
  return_type write() = 0;
};

}  // namespace hardware_interface
#endif  // HARDWARE_INTERFACE__SYSTEM_INTERFACE_HPP_
