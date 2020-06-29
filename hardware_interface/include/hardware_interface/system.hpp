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

#ifndef HARDWARE_INTERFACE__SYSTEM_HPP_
#define HARDWARE_INTERFACE__SYSTEM_HPP_

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "hardware_interface/component_info.hpp"
#include "hardware_interface/component_interfaces/system_interface.hpp"
#include "hardware_interface/visibility_control.h"

namespace hardware_interface
{

class System final
{
public:
  explicit System(std::unique_ptr<SystemInterface> impl)
  : impl_(std::move(impl))
  {}

  virtual ~System() = default;

  HARDWARE_INTERFACE_PUBLIC
  hardware_interface_ret_t configure(const ComponentInfo & system_info)
  {
    return impl_->configure(system_info);
  }

  HARDWARE_INTERFACE_PUBLIC
  std::vector<std::string> get_interface_names()
  {
    return impl_->get_interface_names();
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
  bool is_started()
  {
    return impl_->is_started();
  }

  HARDWARE_INTERFACE_PUBLIC
  hardware_interface_ret_t read(std::vector<double> & data)
  {
    return impl_->read(data);
  }

  HARDWARE_INTERFACE_PUBLIC
  hardware_interface_ret_t write(const std::vector<double> & data)
  {
    return impl_->write(data);
  }

private:
  std::unique_ptr<SystemInterface> impl_;
};

typedef System Robot;

}  // namespace hardware_interface
#endif  // HARDWARE_INTERFACE__SYSTEM_HPP_
