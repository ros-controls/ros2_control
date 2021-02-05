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

#include "hardware_interface/system.hpp"

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_status_values.hpp"

namespace hardware_interface
{

System::System(std::unique_ptr<SystemInterface> impl)
: impl_(std::move(impl))
{}

return_type System::configure(const HardwareInfo & system_info)
{
  return impl_->configure(system_info);
}

std::vector<StateInterface> System::export_state_interfaces()
{
  return impl_->export_state_interfaces();
}

std::vector<CommandInterface> System::export_command_interfaces()
{
  return impl_->export_command_interfaces();
}

return_type System::start()
{
  return impl_->start();
}

return_type System::stop()
{
  return impl_->stop();
}

std::string System::get_name() const
{
  return impl_->get_name();
}

status System::get_status() const
{
  return impl_->get_status();
}

return_type System::read()
{
  return impl_->read();
}

return_type System::write()
{
  return impl_->write();
}

return_type System::accept_command_resource_claim(const std::vector<std::string> & interfaces)
{
  return impl_->accept_command_resource_claim(interfaces);
}

}  // namespace hardware_interface
