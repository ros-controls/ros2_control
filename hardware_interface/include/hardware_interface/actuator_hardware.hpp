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

#include <algorithm>
#include <memory>
#include <string>
#include <utility>

#include "hardware_interface/hardware_and_component_info.hpp"
#include "hardware_interface/actuator_hardware_interface.hpp"
#include "hardware_interface/joint.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_status_values.hpp"
#include "hardware_interface/visibility_control.h"

namespace hardware_interface
{

class ActuatorHardware final
{
public:
  ActuatorHardware() = default;

  explicit ActuatorHardware(std::unique_ptr<ActuatorHardwareInterface> impl)
  : impl_(std::move(impl))
  {}

  ~ActuatorHardware() = default;

  HARDWARE_INTERFACE_PUBLIC
  return_type configure(const HardwareInfo & actuator_info)
  {
    return impl_->configure(actuator_info);
  }

  HARDWARE_INTERFACE_PUBLIC
  return_type start()
  {
    return impl_->start();
  }

  HARDWARE_INTERFACE_PUBLIC
  return_type stop()
  {
    return impl_->stop();
  }

  HARDWARE_INTERFACE_PUBLIC
  hardware_interface_status get_status()
  {
    return impl_->get_status();
  }

  HARDWARE_INTERFACE_PUBLIC
  return_type read_joint(Joint & joint)
  {
    return impl_->read_joint(joint);
  }

  HARDWARE_INTERFACE_PUBLIC
  return_type write(const Joint & joint)
  {
    return impl_->write(joint);
  }

private:
  std::unique_ptr<ActuatorHardwareInterface> impl_;
};

}  // namespace hardware_interface
#endif  // HARDWARE_INTERFACE__ACTUATOR_HARDWARE_HPP_
