// Copyright (c) 2024, Sherpa Mobile Robotics
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

#ifndef SEMANTIC_COMPONENTS__SEMANTIC_COMPONENT_COMMAND_INTERFACE_HPP_
#define SEMANTIC_COMPONENTS__SEMANTIC_COMPONENT_COMMAND_INTERFACE_HPP_

#include <string>
#include <vector>

#include "controller_interface/helpers.hpp"
#include "hardware_interface/loaned_command_interface.hpp"

namespace semantic_components
{
template <typename MessageInputType>
class SemanticComponentCommandInterface
{
public:
  SemanticComponentCommandInterface(
    const std::string & name, const std::vector<std::string> & interface_names)
  : name_(name), interface_names_(interface_names)
  {
    assert(interface_names.size() > 0);
    command_interfaces_.reserve(interface_names.size());
  }

  virtual ~SemanticComponentCommandInterface() = default;

  /**
 * @brief Assign loaned command interfaces from the hardware.
 *
 * Assigns the provided loaned command interfaces when the controller starts.
 *
 * @param[in] command_interfaces A vector of command interfaces provided to the controller.
 */

  bool assign_loaned_command_interfaces(
    std::vector<hardware_interface::LoanedCommandInterface> & command_interfaces)
  {
    return controller_interface::get_ordered_interfaces(
      command_interfaces, interface_names_, "", command_interfaces_);
  }

  /// Release loaned command interfaces from the hardware.
  void release_interfaces() { command_interfaces_.clear(); }

  /**
 * @brief Define command interface names for the component.
 *
 * This function should be used in `command_interface_configuration()` of a controller 
 * to provide standardized command interface names for a semantic component.
 *
 * @note The default implementation defines command interfaces as `"name/NR"`, 
 * where `NR` is a number from 0 to the size of the values.
 *
 * @return A list of strings containing the command interface names for the semantic component.
 */

  const std::vector<std::string> & get_command_interface_names() const { return interface_names_; }

  /**
 * @brief Retrieve all values.
 *
 * @return true if all values are successfully retrieved, false otherwise 
 * (e.g., due to invalid size or failure of 
 * `hardware_interface::LoanedCommandInterface::set_value`).
 */

  bool set_values(const std::vector<double> & values)
  {
    // check we have sufficient memory
    if (values.size() != command_interfaces_.size())
    {
      return false;
    }
    // set values
    bool all_set = true;
    for (auto i = 0u; i < values.size(); ++i)
    {
      all_set &= command_interfaces_[i].get().set_value(values[i]);
    }
    return all_set;
  }

 /**
 * @brief Set values from MessageInputType.
 *
 * @return true if all values were set successfully, false otherwise.
 */

  virtual bool set_values_from_message(const MessageInputType & /* message */) = 0;

protected:
  std::string name_;
  std::vector<std::string> interface_names_;
  std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>>
    command_interfaces_;
};

}  // namespace semantic_components

#endif  // SEMANTIC_COMPONENTS__SEMANTIC_COMPONENT_COMMAND_INTERFACE_HPP_
