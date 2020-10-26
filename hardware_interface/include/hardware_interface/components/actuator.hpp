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

#ifndef HARDWARE_INTERFACE__COMPONENTS__ACTUATOR_HPP_
#define HARDWARE_INTERFACE__COMPONENTS__ACTUATOR_HPP_

#include <memory>

#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_status_values.hpp"
#include "hardware_interface/visibility_control.h"

namespace hardware_interface
{
namespace components
{

class ActuatorInterface;

class Actuator final
{
public:
  Actuator() = default;

  explicit Actuator(std::unique_ptr<ActuatorInterface> impl);

  ~Actuator() = default;

  return_type configure(const HardwareInfo & actuator_info);

  return_type start();

  return_type stop();

  status get_status() const;

private:
  std::unique_ptr<ActuatorInterface> impl_;
};

}  // namespace components
}  // namespace hardware_interface
#endif  // HARDWARE_INTERFACE__COMPONENTS__ACTUATOR_HPP_
