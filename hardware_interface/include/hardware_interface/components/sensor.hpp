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

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_status_values.hpp"
#include "hardware_interface/visibility_control.h"

namespace hardware_interface
{
namespace components
{

class SensorInterface;

class Sensor final
{
public:
  Sensor() = default;

  HARDWARE_INTERFACE_PUBLIC
  explicit Sensor(std::unique_ptr<SensorInterface> impl);

  Sensor(Sensor && other) = default;

  ~Sensor() = default;

  HARDWARE_INTERFACE_PUBLIC
  return_type configure(const HardwareInfo & sensor_info);

  HARDWARE_INTERFACE_PUBLIC
  std::vector<StateInterface> export_state_interfaces();

  HARDWARE_INTERFACE_PUBLIC
  return_type start();

  HARDWARE_INTERFACE_PUBLIC
  return_type stop();

  HARDWARE_INTERFACE_PUBLIC
  std::string get_name() const;

  HARDWARE_INTERFACE_PUBLIC
  status get_status() const;

  HARDWARE_INTERFACE_PUBLIC
  return_type read();

private:
  std::unique_ptr<SensorInterface> impl_;
};

}  // namespace components
}  // namespace hardware_interface
#endif  // HARDWARE_INTERFACE__COMPONENTS__SENSOR_HPP_
