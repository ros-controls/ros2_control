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

#ifndef HARDWARE_INTERFACE__SYSTEM_HARDWARE_HPP_
#define HARDWARE_INTERFACE__SYSTEM_HARDWARE_HPP_

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "hardware_interface/component_info.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_hardware_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_status_values.hpp"
#include "hardware_interface/visibility_control.h"

namespace hardware_interface
{

class SystemHardware final
{
public:
  explicit SystemHardware(std::unique_ptr<SystemHardwareInterface> impl)
  : impl_(std::move(impl))
  {}

  virtual ~SystemHardware() = default;

  HARDWARE_INTERFACE_PUBLIC
  return_type configure(const HardwareInfo & system_info)
  {
    return impl_->configure(system_info);
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
  hardware_interface_status get_status() const
  {
    return impl_->get_status();
  }

  HARDWARE_INTERFACE_PUBLIC
  return_type read_sensors(std::vector<std::shared_ptr<Sensor>> & sensors)
  {
    return impl_->read_sensors(sensors);
  }

  HARDWARE_INTERFACE_PUBLIC
  return_type read_joints(std::vector<std::shared_ptr<Joint>> & joints)
  {
    return impl_->read_joints(joints);
  }

  HARDWARE_INTERFACE_PUBLIC
  return_type write(const std::vector<std::shared_ptr<Joint>> & joints)
  {
    return impl_->write(joints);
  }

private:
  std::unique_ptr<SystemHardwareInterface> impl_;
};

}  // namespace hardware_interface
#endif  // HARDWARE_INTERFACE__SYSTEM_HARDWARE_HPP_
