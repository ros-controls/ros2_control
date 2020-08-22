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

#ifndef HARDWARE_INTERFACE__COMPONENTS__JOINT_HPP_
#define HARDWARE_INTERFACE__COMPONENTS__JOINT_HPP_

#include <string>
#include <vector>

#include "hardware_interface/components/component_info.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/visibility_control.h"

namespace hardware_interface
{
namespace components
{

/**
 * \brief Base Class for the "Joint" component used as a basic building block for a robot.
 * A joint is always 1-DoF and can have one or more interfaces (e.g., position, velocity, etc.)
 * A joint has to be able to receive command(s) and optionally can provide its state(s).
 */
class Joint
{
public:
  HARDWARE_INTERFACE_PUBLIC
  Joint() = default;

  HARDWARE_INTERFACE_PUBLIC
  virtual
  ~Joint() = default;

  /**
   * \brief Configure base joint class based on the description in the robot's URDF file.
   *
   * \param joint_info structure with data from URDF.
   * \return return_type::OK if required data are provided and is successfully parsed,
   * return_type::ERROR otherwise.
   */
  HARDWARE_INTERFACE_PUBLIC
  return_type configure(const ComponentInfo & joint_info);

  /**
   * \brief Provide the list of command interfaces configured for the joint.
   *
   * \return string list with command interfaces.
   */
  HARDWARE_INTERFACE_PUBLIC
  std::vector<std::string> get_command_interfaces() const;

  /**
   * \brief Provide the list of state interfaces configured for the joint.
   *
   * \return string list with state interfaces.
   */
  HARDWARE_INTERFACE_PUBLIC
  std::vector<std::string> get_state_interfaces() const;

  /**
   * \brief Get command list from the joint. This function is used in the write function of the
   * actuator or system hardware. The parameters command and interfaces have the same order,
   * and number of elements. Using the interfaces list, the hardware can choose which values to
   * provide.
   *
   * \param command list of doubles with commands for the hardware.
   * \param interfaces list of interfaces on which commands have to set.
   * \return return_type::INTERFACE_VALUE_SIZE_NOT_EQUAL if command and interfaces arguments do not
   * have the same length; return_type::INTERFACE_NOT_FOUND if one of provided interfaces is not
   * defined for the joint; return return_type::INTERFACE_NOT_PROVIDED if the list of interfaces
   * is empty; return_type::OK otherwise.
   */
  HARDWARE_INTERFACE_EXPORT
  return_type get_command(
    std::vector<double> & command,
    const std::vector<std::string> & interfaces) const;

  /**
   * \brief Get complete command list for the joint. This function is used by the hardware to get
   * complete command for it. The hardware valus have the same order as interfaces which
   * can be recived by get_hardware_interfaces() function. Return value is used for API consistency.
   *
   * \param command list of doubles with commands for the hardware.
   * \return return_type::OK always.
   */
  HARDWARE_INTERFACE_EXPORT
  return_type get_command(std::vector<double> & command) const;

  /**
   * \brief Set command list for the joint. This function is used by the controller to set the goal
   * values for the hardware. The parameters command, and interfaces have the same order and number
   * of elements. Using the interfaces list, the controller can choose which values to set.
   *
   * \param command list of doubles with commands for the hardware.
   * \param interfaces list of interfaces on which commands have to be provided.
   * \return return_type::INTERFACE_VALUE_SIZE_NOT_EQUAL if command and interfaces arguments do not
   * have the same length; return_type::COMMAND_OUT_OF_LIMITS if one of the command values is out
   * of limits; return_type::INTERFACE_NOT_FOUND if one of provided interfaces is not
   * defined for the joint; return_type::OK otherwise.
   *
   * \todo The error handling in this function could lead to incosistant command or state variables
   * for different interfaces. This should be changed in the future.
   * (see: https://github.com/ros-controls/ros2_control/issues/129)
   */
  HARDWARE_INTERFACE_EXPORT
  return_type set_command(
    const std::vector<double> & command,
    const std::vector<std::string> & interfaces);

  /**
   * \brief Get complete state list from the joint. This function is used by the hardware to get
   * complete command for it. The hardware valus have the same order as interfaces which
   * can be recived by get_hardware_interfaces() function.
   *
   * \param command list of doubles with commands for the hardware.
   * \return return_type::INTERFACE_VALUE_SIZE_NOT_EQUAL is command size is not equal to number of
   * joint's command interfaces; return_type::COMMAND_OUT_OF_LIMITS if one of the command values is out
   * of limits; return_type::OK otherwise.
   */
  HARDWARE_INTERFACE_EXPORT
  return_type set_command(const std::vector<double> & command);

  /**
   * \brief Get state list from the joint. This function is used by the controller to get the
   * actual state of the hardware. The parameters state, and interfaces have the same order and
   * number of elements. Using the interfaces list, the controller can choose which values to get.
   *
   * \param state list of doubles with states of the hardware.
   * \param interfaces list of interfaces on which states have to be provided.
   * \return return_type::INTERFACE_VALUE_SIZE_NOT_EQUAL if state and interfaces arguments do not
   * have the same length; return_type::INTERFACE_NOT_FOUND if one of provided interfaces is not
   * defined for the joint; return return_type::INTERFACE_NOT_PROVIDED if the list of interfaces
   * is empty; return_type::OK otherwise.
   */
  HARDWARE_INTERFACE_EXPORT
  return_type get_state(
    std::vector<double> & state,
    const std::vector<std::string> & interfaces) const;

  /**
   * \brief Get complete state list from the joint. This function is used by the controller to get
   * complete actual state of the hardware. The state values have the same order as interfaces which
   * can be recived by get_state_interfaces() function. Return value is used for API consistency.
   *
   * \param state list of doubles with states of the hardware.
   * \return return_type::OK always.
   */
  HARDWARE_INTERFACE_EXPORT
  return_type get_state(std::vector<double> & state) const;

  /**
   * \brief Set state list for the joint. This function is used by the hardware to set its actual
   * state. The parameters state, and interfaces have the same order and number of elements. Using
   * the interfaces list, the hardware can choose which values to set.
   *
   * \param state list of doubles with states of the hardware.
   * \param interfaces list of interfaces on which states have to be provided.
   * \return return_type::INTERFACE_VALUE_SIZE_NOT_EQUAL if state and interfaces arguments do not
   * have the same length; return_type::INTERFACE_NOT_FOUND if one of provided interfaces is not
   * defined for the joint; return_type::OK otherwise.
   */
  HARDWARE_INTERFACE_EXPORT
  return_type set_state(
    const std::vector<double> & state,
    const std::vector<std::string> & interfaces);

  /**
   * \brief Set complete state list from the joint.This function is used by the hardware to set its
   * complete actual state. The state values have the same order as interfaces which can be recived
   * by get_state_interfaces() function.
   *
   * \param state list of doubles with states from the hardware.
   * \return return_type::INTERFACE_VALUE_SIZE_NOT_EQUAL is command size is not equal to number of
   * joint's state interfaces, return_type::OK otherwise.
   */
  HARDWARE_INTERFACE_EXPORT
  return_type set_state(const std::vector<double> & state);

protected:
  ComponentInfo info_;
  std::vector<double> commands_;
  std::vector<double> states_;
};

}  // namespace components
}  // namespace hardware_interface
#endif  // HARDWARE_INTERFACE__COMPONENTS__JOINT_HPP_
