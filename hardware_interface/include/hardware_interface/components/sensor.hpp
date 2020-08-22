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

#ifndef HARDWARE_INTERFACE__COMPONENTS__SENSOR_HPP_
#define HARDWARE_INTERFACE__COMPONENTS__SENSOR_HPP_

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
 * \brief Base Class for "Sensor" component used as a basic build block for the devices which
 * provide data. A sensor can have one or more interfaces (e.g., force, acceleration, etc.) to
 * provide states for. The list of state interfaces defines this.
 */
class Sensor
{
public:
  HARDWARE_INTERFACE_PUBLIC
  Sensor() = default;

  HARDWARE_INTERFACE_PUBLIC
  virtual
  ~Sensor() = default;

  /**
   * \brief Configure base sensor class based on the description in the robot's URDF file.
   *
   * \param joint_info structure with data from URDF.
   * \return return_type::OK if required data are provided and is successfully parsed,
   * return_type::ERROR otherwise.
   */
  HARDWARE_INTERFACE_PUBLIC
  return_type configure(const ComponentInfo & joint_info);

  /**
   * \brief Provide the list of state interfaces configured for the sensor.
   *
   * \return string list with state interfaces.
   */
  HARDWARE_INTERFACE_PUBLIC
  std::vector<std::string> get_state_interfaces();

  /**
   * \brief Get state list from the sensor. This function is used by the controller to get the
   * actual state of the hardware. The parameters state, and interfaces have the same order and
   * number of elements. Using the interfaces list, the controller can choose which values to get.
   *
   * \param state list of doubles with states of the hardware.
   * \param interfaces list of interfaces on which states have to be provided.
   * \return return_type::INTERFACE_VALUE_SIZE_NOT_EQUAL if state and interfaces arguments do not
   * have the same length; return_type::INTERFACE_NOT_FOUND if one of provided interfaces is not
   * defined for the sensor; return return_type::INTERFACE_NOT_PROVIDED if the list of interfaces
   * is empty; return_type::OK otherwise.
   */
  HARDWARE_INTERFACE_EXPORT
  return_type get_state(
    std::vector<double> & state,
    const std::vector<std::string> & interfaces) const;

  /**
   * \brief Get complete state list from the sensor. This function is used by the controller to get
   * complete actual state of the hardware. The state values have the same order as interfaces which
   * can be recived by get_state_interfaces() function. Return value is used for API consistency.
   *
   * \param state list of doubles with states of the hardware.
   * \return return_type::OK always.
   */
  HARDWARE_INTERFACE_EXPORT
  return_type get_state(std::vector<double> & state) const;

  /**
   * \brief Set state list for the sensor. This function is used by the hardware to set its actual
   * state. The parameters state, and interfaces have the same order and number of elements. Using
   * the interfaces list, the hardware can choose which values to set.
   *
   * \param state list of doubles with states of the hardware.
   * \param interfaces list of interfaces on which states have to be provided.
   * \return return_type::INTERFACE_VALUE_SIZE_NOT_EQUAL if state and interfaces arguments do not
   * have the same length; return_type::INTERFACE_NOT_FOUND if one of provided interfaces is not
   * defined for the sensor; return_type::OK otherwise.
   */
  HARDWARE_INTERFACE_EXPORT
  return_type set_state(
    const std::vector<double> & state,
    const std::vector<std::string> & interfaces);

  /**
   * \brief Set complete state list from the sensor.This function is used by the hardware to set its
   * complete actual state. The state values have the same order as interfaces which can be recived
   * by get_state_interfaces() function.
   *
   * \param state list of doubles with states from the hardware.
   * \return return_type::INTERFACE_VALUE_SIZE_NOT_EQUAL is state size is not equal to number of
   * sensor's state interfaces, return_type::OK otherwise.
   */
  HARDWARE_INTERFACE_EXPORT
  return_type set_state(const std::vector<double> & state);

protected:
  ComponentInfo info_;
  std::vector<double> states_;
};

}  // namespace components
}  // namespace hardware_interface
#endif  // HARDWARE_INTERFACE__COMPONENTS__SENSOR_HPP_
