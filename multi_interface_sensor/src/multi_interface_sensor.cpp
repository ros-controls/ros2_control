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

#include "multi_interface_sensor/multi_interface_sensor.hpp"

#include <set>
#include <string>
#include <vector>

#include "./component_lists_management.hpp"

namespace multi_interface_sensor
{

hardware_interface::return_type MultiInterfaceSensor::configure(
  const hardware_interface::components::ComponentInfo & sensor_info)
{
  const size_t state_interfaces_size = sensor_info.state_interfaces.size();

  // fail if no interfaces at all are specified
  if (state_interfaces_size == 0u) {
    return hardware_interface::return_type::INTERFACE_NOT_PROVIDED;
  }

  auto has_duplicates = [](const std::vector<std::string> interfaces) -> bool
    {
      std::set<std::string> set(interfaces.begin(), interfaces.end());
      return set.size() != interfaces.size();
    };

  // fail if state interfaces has duplicates
  if (has_duplicates(sensor_info.state_interfaces)) {
    return hardware_interface::return_type::INTERFACE_DUPLICATES;
  }

  state_interfaces_ = sensor_info.state_interfaces;
  state_values_.resize(state_interfaces_size);

  return hardware_interface::return_type::OK;
}

std::vector<std::string> MultiInterfaceSensor::get_state_interfaces() const
{
  return state_interfaces_;
}

hardware_interface::return_type MultiInterfaceSensor::get_state(
  std::vector<double> & state,
  const std::vector<std::string> & interfaces) const
{
  return get_internal_values(state, interfaces, state_interfaces_, state_values_);
}

hardware_interface::return_type MultiInterfaceSensor::get_state(std::vector<double> & state) const
{
  return get_internal_values(state, state_values_);
}

hardware_interface::return_type MultiInterfaceSensor::set_state(
  const std::vector<double> & state,
  const std::vector<std::string> & interfaces)
{
  return set_internal_values(state, interfaces, state_interfaces_, state_values_);
}

hardware_interface::return_type MultiInterfaceSensor::set_state(const std::vector<double> & state)
{
  return set_internal_values(state, state_values_);
}

}  // namespace multi_interface_sensor
