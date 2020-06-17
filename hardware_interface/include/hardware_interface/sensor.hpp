// Copyright 2020 ROS2-Control Development Team
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


#ifndef HARDWARE_INTERFACE__SENSOR_HPP_
#define HARDWARE_INTERFACE__SENSOR_HPP_

#include <memory>
#include <string>
#include <utility>

#include "hardware_interface/component_info.hpp"
#include "hardware_interface/component_interfaces/sensor_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/visibility_control.h"

namespace hardware_interface
{

class Sensor final
{
public:
  explicit Sensor(std::unique_ptr<SensorInterface> impl)
  : impl_(std::move(impl))
  {}

  virtual ~Sensor() = default;

  HARDWARE_INTERFACE_PUBLIC
  hardware_interface_ret_t configure(const ComponentInfo & sensor_info)
  {
    return impl_->configure(sensor_info);
  }

  HARDWARE_INTERFACE_PUBLIC
  hardware_interface_ret_t start()
  {
    return impl_->start();
  }

  HARDWARE_INTERFACE_PUBLIC
  hardware_interface_ret_t stop()
  {
    return impl_->stop();
  }

  HARDWARE_INTERFACE_PUBLIC
  hardware_interface_ret_t read(double & data)
  {
    return impl_->read(data);
  }

  HARDWARE_INTERFACE_PUBLIC
  std::string get_interface_name()
  {
    return impl_->get_interface_name();
  }

private:
  std::unique_ptr<SensorInterface> impl_;
};

}  // namespace hardware_interface
#endif  // HARDWARE_INTERFACE__SENSOR_HPP_
