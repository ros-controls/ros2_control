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

#include "hardware_interface/components/system.hpp"

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/components/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_status_values.hpp"

namespace hardware_interface
{
namespace components
{

System::System(std::unique_ptr<SystemInterface> impl)
: impl_(std::move(impl))
{}

return_type System::configure(const HardwareInfo & system_info)
{
  return impl_->configure(system_info);
}

std::vector<StateHandle> System::export_state_handles()
{
  return impl_->export_state_handles();
}

std::vector<CommandHandle> System::export_command_handles()
{
  return impl_->export_command_handles();
}

return_type System::start()
{
  return impl_->start();
}

return_type System::stop()
{
  return impl_->stop();
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

}  // namespace components
}  // namespace hardware_interface
