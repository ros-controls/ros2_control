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

#ifndef HARDWARE_INTERFACE__HELPERS__COMPONENT_LISTS_MANAGEMENT_HPP_
#define HARDWARE_INTERFACE__HELPERS__COMPONENT_LISTS_MANAGEMENT_HPP_

#include <string>
#include <vector>

#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/visibility_control.h"

namespace hardware_interface
{

namespace helpers
{

/**
  * \brief Get values for queried_interfaces from the int_values. int_values data structure matches
  * int_interfaces vector.
  *
  * \param values values to return.
  * \param queried_interfaces interfaces for which values are queried.
  * \param int_interfaces full list of interfaces of a component.
  * \param int_values internal values of a component.
  * \return return_type::INTERFACE_VALUE_SIZE_NOT_EQUAL if values and queried_interfaces arguments
  * do not have the same length; return_type::INTERFACE_NOT_FOUND if one of queried_interfaces is
  * not defined in int_interfaces; return return_type::INTERFACE_NOT_PROVIDED if queried_interfaces
  * list is is empty; return_type::OK otherwise.
  */
return_type get_internal_values(
  std::vector<double> & values, const std::vector<std::string> & queried_interfaces,
  const std::vector<std::string> & int_interfaces, const std::vector<double> & int_values);

/**
 * \brief Set all internal values to to other vector;
 *
 * \param values output list of values.
 * \param int_values internal values of the component.
 */
void get_internal_values(
  std::vector<double> & values, const std::vector<double> & int_values);

/**
 * \brief set values for queried_interfaces to the int_values. int_values data structure matches
 * int_interfaces vector.
 *
 * \param values values to set.
 * \param queried_interfaces interfaces for which values are queried.
 * \param int_interfaces full list of interfaces of a component.
 * \param int_values internal values of a component.
 * \return return return_type::INTERFACE_NOT_PROVIDED if
 * queried_interfaces list is is empty; return_type::INTERFACE_VALUE_SIZE_NOT_EQUAL if values and
 * queried_interfaces arguments do not have the same length; return_type::INTERFACE_NOT_FOUND if
 * one of queried_interfaces is not defined in int_interfaces; return_type::OK otherwise.
 *
 * \todo The error handling in this function could lead to incosistant command or state variables
 * for different interfaces. This should be changed in the future.
 * (see: https://github.com/ros-controls/ros2_control/issues/129)
 */
return_type set_internal_values(
  const std::vector<double> & values, const std::vector<std::string> & queried_interfaces,
  const std::vector<std::string> & int_interfaces, std::vector<double> & int_values);

/**
 * \brief set all values to compoenents internal values.
 *
 * \param values values to set.
 * \param int_values internal values of a component.
 * \return return_type::INTERFACE_VALUE_SIZE_NOT_EQUAL if the size of the arguments is not equal,
 * return_type::OK otherwise.
 */
return_type set_internal_values(
  const std::vector<double> & values, std::vector<double> & int_values);

}  // namespace helpers

}  // namespace hardware_interface
#endif  // HARDWARE_INTERFACE__HELPERS__COMPONENT_LISTS_MANAGEMENT_HPP_
