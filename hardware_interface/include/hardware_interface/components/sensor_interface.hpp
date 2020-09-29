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

#ifndef HARDWARE_INTERFACE__COMPONENTS__SENSOR_INTERFACE_HPP_
#define HARDWARE_INTERFACE__COMPONENTS__SENSOR_INTERFACE_HPP_

#include <string>
#include <vector>

#include "hardware_interface/components/component_info.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/visibility_control.h"

namespace hardware_interface
{
namespace components
{

class SensorInterface
{
public:
  HARDWARE_INTERFACE_PUBLIC
  SensorInterface() = default;

  HARDWARE_INTERFACE_PUBLIC
  virtual
  ~SensorInterface() = default;

  HARDWARE_INTERFACE_PUBLIC
  virtual
  return_type configure(const ComponentInfo & sensor_info) = 0;

  HARDWARE_INTERFACE_PUBLIC
  virtual
  std::vector<std::string> get_state_interfaces() const = 0;

  HARDWARE_INTERFACE_PUBLIC
  virtual
  return_type get_state(
    std::vector<double> & state,
    const std::vector<std::string> & interfaces) const = 0;

  HARDWARE_INTERFACE_PUBLIC
  virtual
  return_type get_state(std::vector<double> & state) const = 0;

  HARDWARE_INTERFACE_PUBLIC
  virtual
  return_type set_state(
    const std::vector<double> & state,
    const std::vector<std::string> & interfaces) = 0;

  HARDWARE_INTERFACE_PUBLIC
  virtual
  return_type set_state(const std::vector<double> & state) = 0;
};

}  // namespace components
}  // namespace hardware_interface
#endif  // HARDWARE_INTERFACE__COMPONENTS__SENSOR_INTERFACE_HPP_
