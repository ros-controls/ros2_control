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

#ifndef HARDWARE_INTERFACE__ACTUATOR_HARDWARE_HPP_
#define HARDWARE_INTERFACE__ACTUATOR_HARDWARE_HPP_

#include <memory>

#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_status_values.hpp"
#include "hardware_interface/visibility_control.h"

namespace hardware_interface
{

namespace components
{
class Joint;
}  // namespace components
class ActuatorHardwareInterface;

class ActuatorHardware final
{
public:
  ActuatorHardware() = default;

  explicit ActuatorHardware(std::unique_ptr<ActuatorHardwareInterface> impl);

  ~ActuatorHardware() = default;

  return_type configure(const HardwareInfo & actuator_info);

  return_type start();

  return_type stop();

  hardware_interface_status get_status() const;

  return_type read_joint(std::shared_ptr<components::Joint> joint);

  return_type write_joint(const std::shared_ptr<components::Joint> joint);

private:
  std::unique_ptr<ActuatorHardwareInterface> impl_;
};

}  // namespace hardware_interface
#endif  // HARDWARE_INTERFACE__ACTUATOR_HARDWARE_HPP_
