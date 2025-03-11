// Copyright (c) 2021, Stogl Robotics Consulting UG (haftungsbeschränkt)
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

#ifndef CONTROLLER_INTERFACE__HELPERS_HPP_
#define CONTROLLER_INTERFACE__HELPERS_HPP_

#include <functional>
#include <string>
#include <vector>

namespace controller_interface
{
/**
 * @brief Reorder interfaces with references according to joint names or full interface names.
 *
 * Reorders and verifies that all expected interfaces are provided for the specified joints. 
 * Populates `ordered_interfaces` with references from `unordered_interfaces` in the same 
 * order as specified in `ordered_names`.
 *
 * @param[in] unordered_interfaces Vector of loaned unordered state or command interfaces.
 * @param[in] ordered_names Vector of ordered names used to sort `unordered_interfaces`. 
 *                          The values can be either a list of joint names or full interface names.  
 *                          If joint names are used, `interface_type` must be specified.  
 *                          If full interface names are used, `interface_type` should be an empty string (`""`).
 * @param[in] interface_type Specifies the interface type when ordering by joint names.
 * @param[out] ordered_interfaces Vector to store the ordered interfaces.
 * @return `true` if all interfaces or joints in `ordered_names` are found, otherwise `false`.
 */

template <typename T>
bool get_ordered_interfaces(
  std::vector<T> & unordered_interfaces, const std::vector<std::string> & ordered_names,
  const std::string & interface_type, std::vector<std::reference_wrapper<T>> & ordered_interfaces)
{
  ordered_interfaces.reserve(ordered_names.size());
  for (const auto & name : ordered_names)
  {
    for (auto & interface : unordered_interfaces)
    {
      if (!interface_type.empty())
      {
        // check case where:
        // (<joint> == <joint> AND <interface> == <interface>) OR <joint>/<interface> == 'full name'
        if (
          ((name == interface.get_prefix_name()) &&
           (interface_type == interface.get_interface_name())) ||
          ((name + "/" + interface_type) == interface.get_name()))
        {
          ordered_interfaces.push_back(std::ref(interface));
        }
      }
      else
      {
        if (name == interface.get_name())
        {
          ordered_interfaces.push_back(std::ref(interface));
        }
      }
    }
  }

  return ordered_names.size() == ordered_interfaces.size();
}

inline bool interface_list_contains_interface_type(
  const std::vector<std::string> & interface_type_list, const std::string & interface_type)
{
  return std::find(interface_type_list.begin(), interface_type_list.end(), interface_type) !=
         interface_type_list.end();
}

template <typename T>
void add_element_to_list(std::vector<T> & list, const T & element)
{
  if (std::find(list.begin(), list.end(), element) == list.end())
  {
    // Only add to the list if it doesn't exist
    list.push_back(element);
  }
}

}  // namespace controller_interface

#endif  // CONTROLLER_INTERFACE__HELPERS_HPP_
