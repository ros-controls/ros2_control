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
#include "hardware_interface/components/component_info.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"

#include "./component_lists_management.hpp"

namespace hardware_interface
{
namespace components
{

Sensor::Sensor(std::shared_ptr<SensorInterface> impl, Deleter deleter)
: impl_(std::move(impl)),
  deleter_(deleter)
{}

Sensor::~Sensor()
{
  if (deleter_) {deleter_();}
}

return_type Sensor::configure(const ComponentInfo & sensor_info)
{
  return impl_->configure(sensor_info);
}

std::vector<std::string> Sensor::get_state_interfaces() const
{
  return impl_->get_state_interfaces();
}

return_type Sensor::get_state(
  std::vector<double> & state, const std::vector<std::string> & interfaces) const
{
  return impl_->get_state(state, interfaces);
}

return_type Sensor::get_state(std::vector<double> & state) const
{
  return impl_->get_state(state);
}

return_type Sensor::set_state(
  const std::vector<double> & state, const std::vector<std::string> & interfaces)
{
  return impl_->set_state(state, interfaces);
}

return_type Sensor::set_state(const std::vector<double> & state)
{
  return impl_->set_state(state);
}

}  // namespace components
}  // namespace hardware_interface
