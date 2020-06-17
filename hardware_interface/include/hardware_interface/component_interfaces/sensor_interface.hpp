// Copyright 2020 ROS2-Control Development Team
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

#ifndef HARDWARE_INTERFACE__COMPONENT_INTERFACES__SENSOR_INTERFACE_HPP_
#define HARDWARE_INTERFACE__COMPONENT_INTERFACES__SENSOR_INTERFACE_HPP_

#include <string>

#include "hardware_interface/component_info.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/visibility_control.h"

namespace hardware_interface
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
  hardware_interface_ret_t configure(const ComponentInfo & sensor_info) = 0;

  HARDWARE_INTERFACE_PUBLIC
  virtual
  std::string get_interface_name() const = 0;

  HARDWARE_INTERFACE_PUBLIC
  virtual
  hardware_interface_ret_t start() = 0;

  HARDWARE_INTERFACE_PUBLIC
  virtual
  hardware_interface_ret_t stop() = 0;

  HARDWARE_INTERFACE_PUBLIC
  virtual
  bool is_started() const = 0;

  HARDWARE_INTERFACE_PUBLIC
  virtual
  hardware_interface_ret_t read(double & data) = 0;
};

}  // namespace hardware_interface
#endif  // HARDWARE_INTERFACE__COMPONENT_INTERFACES__SENSOR_INTERFACE_HPP_
