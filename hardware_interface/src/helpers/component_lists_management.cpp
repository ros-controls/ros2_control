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

#include <algorithm>
#include <string>
#include <vector>

#include "hardware_interface/helpers/component_lists_management.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"

namespace hardware_interface
{

namespace helpers
{

return_type get_internal_values(
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

void get_internal_values(
  std::vector<double> & values, const std::vector<double> & int_values)
{
  values.clear();
  for (const auto & int_value : int_values) {
    values.push_back(int_value);
  }
}

return_type set_internal_values(
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

return_type set_internal_values(
  const std::vector<double> & values, std::vector<double> & int_values)
{
  if (values.size() == int_values.size()) {
    for (uint i = 0; i < int_values.size(); i++) {
      int_values[i] = values[i];
    }
  } else {
    return return_type::INTERFACE_VALUE_SIZE_NOT_EQUAL;
  }
  return return_type::OK;
}

}  // namespace helpers
}  // namespace hardware_interface
