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

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "hardware_interface/system_hardware.hpp"

#include "hardware_interface/components/component_info.hpp"
#include "hardware_interface/components/joint.hpp"
#include "hardware_interface/components/sensor.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_hardware_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_status_values.hpp"
#include "hardware_interface/visibility_control.h"

namespace hardware_interface
{

SystemHardware::SystemHardware(std::unique_ptr<SystemHardwareInterface> impl)
: impl_(std::move(impl))
{}

return_type SystemHardware::configure(const HardwareInfo & system_info)
{
  return impl_->configure(system_info);
}

return_type SystemHardware::start()
{
  return impl_->start();
}

return_type SystemHardware::stop()
{
  return impl_->stop();
}

hardware_interface_status SystemHardware::get_status() const
{
  return impl_->get_status();
}

return_type SystemHardware::read_sensors(std::vector<std::shared_ptr<components::Sensor>> & sensors)
{
  return impl_->read_sensors(sensors);
}

return_type SystemHardware::read_joints(std::vector<std::shared_ptr<components::Joint>> & joints)
{
  return impl_->read_joints(joints);
}

return_type SystemHardware::write_joints(
  const std::vector<std::shared_ptr<components::Joint>> & joints)
{
  return impl_->write_joints(joints);
}

}  // namespace hardware_interface
