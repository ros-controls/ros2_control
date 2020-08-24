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

#ifndef COMPONENTS__COMPONENT_LISTS_MANAGEMENT_HPP_
#define COMPONENTS__COMPONENT_LISTS_MANAGEMENT_HPP_

#include <algorithm>
#include <string>
#include <vector>

#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/visibility_control.h"

namespace hardware_interface
{
namespace components
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
inline return_type get_internal_values(
  std::vector<double> & values, const std::vector<std::string> & queried_interfaces,
  const std::vector<std::string> & int_interfaces, const std::vector<double> & int_values)
{
  if (queried_interfaces.size() == 0) {
    return return_type::INTERFACE_NOT_PROVIDED;
  }

  for (const auto & interface : queried_interfaces) {
    auto it = std::find(
      int_interfaces.begin(), int_interfaces.end(), interface);
    if (it != int_interfaces.end()) {
      values.push_back(int_values[std::distance(int_interfaces.begin(), it)]);
    } else {
      values.clear();
      return return_type::INTERFACE_NOT_FOUND;
    }
  }
  return return_type::OK;
}

/**
 * \brief Set all internal values to to other vector. Return value is used for API consistency.
 *
 * \param values output list of values.
 * \param int_values internal values of the component.
 * \return return_type::OK always.
 */
inline return_type get_internal_values(
  std::vector<double> & values, const std::vector<double> & int_values)
{
  values.clear();
  for (const auto & int_value : int_values) {
    values.push_back(int_value);
  }
  return return_type::OK;
}

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
inline return_type set_internal_values(
  const std::vector<double> & values, const std::vector<std::string> & queried_interfaces,
  const std::vector<std::string> & int_interfaces, std::vector<double> & int_values)
{
  if (queried_interfaces.size() == 0) {
    return return_type::INTERFACE_NOT_PROVIDED;
  }
  if (values.size() != queried_interfaces.size()) {
    return return_type::INTERFACE_VALUE_SIZE_NOT_EQUAL;
  }

  for (auto q_it = queried_interfaces.begin(); q_it != queried_interfaces.end(); ++q_it) {
    auto it = std::find(int_interfaces.begin(), int_interfaces.end(), *q_it);
    if (it != int_interfaces.end()) {
      int_values[std::distance(int_interfaces.begin(), it)] =
        values[std::distance(queried_interfaces.begin(), q_it)];
    } else {
      return return_type::INTERFACE_NOT_FOUND;
    }
  }
  return return_type::OK;
}

/**
 * \brief set all values to compoenents internal values.
 *
 * \param values values to set.
 * \param int_values internal values of a component.
 * \return return_type::INTERFACE_VALUE_SIZE_NOT_EQUAL if the size of the arguments is not equal,
 * return_type::OK otherwise.
 */
inline return_type set_internal_values(
  const std::vector<double> & values, std::vector<double> & int_values)
{
  if (values.size() == int_values.size()) {
    int_values = values;
  } else {
    return return_type::INTERFACE_VALUE_SIZE_NOT_EQUAL;
  }
  return return_type::OK;
}

}  // namespace components
}  // namespace hardware_interface
#endif  // COMPONENTS__COMPONENT_LISTS_MANAGEMENT_HPP_
