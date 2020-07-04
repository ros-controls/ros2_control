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

#ifndef HARDWARE_INTERFACE__ACTUATOR_HPP_
#define HARDWARE_INTERFACE__ACTUATOR_HPP_

#include <algorithm>
#include <memory>
#include <string>
#include <utility>

#include "hardware_interface/component_info.hpp"
#include "hardware_interface/component_interfaces/actuator_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/visibility_control.h"

namespace hardware_interface
{

class Actuator final
{
public:
  Actuator() = default;

  explicit Actuator(std::unique_ptr<ActuatorInterface> & impl)
  : impl_(std::move(impl))
  {}

  ~Actuator() = default;

  HARDWARE_INTERFACE_PUBLIC
  return_type configure(const ComponentInfo & actuator_info)
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
  bool is_started() const
  {
    return impl_->is_started();
  }

  HARDWARE_INTERFACE_PUBLIC
  return_type read(double & data)
  {
    return impl_->read(data);
  }

  HARDWARE_INTERFACE_PUBLIC
  return_type write(const double & data)
  {
    return impl_->write(data);
  }

  HARDWARE_INTERFACE_PUBLIC
  std::string get_interface_name() const
  {
    return impl_->get_interface_name();
  }

private:
  std::unique_ptr<ActuatorInterface> impl_;
};

}  // namespace hardware_interface
#endif  // HARDWARE_INTERFACE__ACTUATOR_HPP_
