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

#ifndef HARDWARE_INTERFACE__BASE_INTERFACE_HPP_
#define HARDWARE_INTERFACE__BASE_INTERFACE_HPP_

#include <string>

#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_status_values.hpp"

namespace hardware_interface
{
template <class InterfaceType>
class BaseInterface : public InterfaceType
{
public:
  return_type configure(const HardwareInfo & info) override
  {
    info_ = info;
    status_ = status::CONFIGURED;
    return return_type::OK;
  }

  return_type configure_default(const HardwareInfo & info)
  {
    return BaseInterface<InterfaceType>::configure(info);
  }

  std::string get_name() const final { return info_.name; }

  status get_status() const final { return status_; }

protected:
  HardwareInfo info_;
  status status_;
};

}  // namespace hardware_interface

#endif  // HARDWARE_INTERFACE__BASE_INTERFACE_HPP_
