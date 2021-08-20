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

#ifndef SEMANTIC_COMPONENTS__SEMANTIC_COMPONENT_INTERFACE_HPP_
#define SEMANTIC_COMPONENTS__SEMANTIC_COMPONENT_INTERFACE_HPP_

#include <string>
#include <vector>

#include "controller_interface/helpers.hpp"
#include "hardware_interface/loaned_state_interface.hpp"

namespace semantic_components
{
template <typename MessageReturnType>
class SemanticComponentInterface
{
public:
  explicit SemanticComponentInterface(const std::string & name, size_t size = 0)
  {
    name_ = name;
    interface_names_.reserve(size);
    state_interfaces_.reserve(size);
  }

  ~SemanticComponentInterface() = default;

  /// Assign loaned state interfaces from the hardware.
  /**
   * Assign loaned state interfaces on the controller start.
   *
   * \param[in] state_interfaces vector of interfaces provided by the controller.
   */
  bool assign_loaned_state_interfaces(
    std::vector<hardware_interface::LoanedStateInterface> & state_interfaces)
  {
    return controller_interface::get_ordered_interfaces(
      state_interfaces, interface_names_, "", state_interfaces_);
  }

  /// Release loaned interfaces from the hardware.
  void release_interfaces() { state_interfaces_.clear(); }

  /// Definition of state interface names for the component.
  /**
   * The function should be used in "state_interface_configuration()" of a controller to provide
   * standardized interface names semantic component.
   *
   * \default Default implementation defined state interfaces as "name/NR" where NR is number
   * from 0 to size of values;
   * \return list of strings with state interface names for the semantic component.
   */
  virtual std::vector<std::string> get_state_interface_names()
  {
    if (interface_names_.empty())
    {
      for (auto i = 0u; i < interface_names_.capacity(); ++i)
      {
        interface_names_.emplace_back(name_ + "/" + std::to_string(i + 1));
      }
    }
    return interface_names_;
  }

  /// Return all values.
  /**
   * \return true if it gets all the values, else false
   */
  bool get_values(std::vector<double> & values) const
  {
    // check we have sufficient memory
    if (values.capacity() != state_interfaces_.size())
    {
      return false;
    }
    // insert all the values
    for (size_t i = 0; i < state_interfaces_.size(); ++i)
    {
      values.emplace_back(state_interfaces_[i].get().get_value());
    }
    return true;
  }

  /// Return values as MessageReturnType
  /**
   * \return false by default
   */
  bool get_values_as_message(MessageReturnType & /* message */) { return false; }

protected:
  std::string name_;
  std::vector<std::string> interface_names_;
  std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>> state_interfaces_;
};

}  // namespace semantic_components

#endif  // SEMANTIC_COMPONENTS__SEMANTIC_COMPONENT_INTERFACE_HPP_
