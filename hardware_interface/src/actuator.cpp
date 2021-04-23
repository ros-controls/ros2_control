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
#include <vector>

#include "hardware_interface/actuator.hpp"
#include "hardware_interface/actuator_interface.hpp"

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_status_values.hpp"

namespace hardware_interface
{

Actuator::Actuator(std::unique_ptr<ActuatorInterface> impl)
: impl_(std::move(impl))
{}

return_type Actuator::configure(const HardwareInfo & actuator_info)
{
  return impl_->configure(actuator_info);
}

std::vector<StateInterface> Actuator::export_state_interfaces()
{
  // TODO(karsten1987): Might be worth to do some brief sanity check here
  return impl_->export_state_interfaces();
}

std::vector<CommandInterface> Actuator::export_command_interfaces()
{
  // TODO(karsten1987): Might be worth to do some brief sanity check here
  return impl_->export_command_interfaces();
}

return_type Actuator::prepare_command_mode_switch(
  const std::vector<std::string> & start_interfaces,
  const std::vector<std::string> & stop_interfaces)
{
  return impl_->prepare_command_mode_switch(start_interfaces, stop_interfaces);
}

return_type Actuator::perform_command_mode_switch(
  const std::vector<std::string> & start_interfaces,
  const std::vector<std::string> & stop_interfaces)
{
  return impl_->perform_command_mode_switch(start_interfaces, stop_interfaces);
}

return_type Actuator::start()
{
  return impl_->start();
}

return_type Actuator::stop()
{
  return impl_->stop();
}

std::string Actuator::get_name() const
{
  return impl_->get_name();
}

status Actuator::get_status() const
{
  return impl_->get_status();
}

return_type Actuator::read()
{
  return impl_->read();
}

return_type Actuator::write()
{
  return impl_->write();
}

}  // namespace hardware_interface
