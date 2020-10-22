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

#include "hardware_interface/components/sensor.hpp"

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/components/sensor_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_status_values.hpp"

namespace hardware_interface
{
namespace components
{

Sensor::Sensor(std::unique_ptr<SensorInterface> impl)
: impl_(std::move(impl))
{}

return_type Sensor::configure(const HardwareInfo & sensor_info)
{
  return impl_->configure(sensor_info);
}

std::vector<StateHandle> Sensor::export_state_handles()
{
  return impl_->export_state_handles();
}

return_type Sensor::start()
{
  return impl_->start();
}

return_type Sensor::stop()
{
  return impl_->stop();
}

status Sensor::get_status() const
{
  return impl_->get_status();
}

return_type Sensor::read()
{
  return impl_->read();
}

}  // namespace components
}  // namespace hardware_interface
