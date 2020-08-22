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

#include <algorithm>
#include <memory>
#include <string>
#include <utility>

#include "hardware_interface/actuator_hardware.hpp"

#include "hardware_interface/actuator_hardware_interface.hpp"
#include "hardware_interface/components/component_info.hpp"
#include "hardware_interface/components/joint.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_status_values.hpp"
#include "hardware_interface/visibility_control.h"

namespace hardware_interface
{

ActuatorHardware::ActuatorHardware(std::unique_ptr<ActuatorHardwareInterface> impl)
: impl_(std::move(impl))
{}

return_type ActuatorHardware::configure(const HardwareInfo & actuator_info)
{
  return impl_->configure(actuator_info);
}

return_type ActuatorHardware::start()
{
  return impl_->start();
}

return_type ActuatorHardware::stop()
{
  return impl_->stop();
}

hardware_interface_status ActuatorHardware::get_status() const
{
  return impl_->get_status();
}

return_type ActuatorHardware::read_joint(std::shared_ptr<components::Joint> joint)
{
  return impl_->read_joint(joint);
}

return_type ActuatorHardware::write_joint(const std::shared_ptr<components::Joint> joint)
{
  return impl_->write_joint(joint);
}

}  // namespace hardware_interface
