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

#ifndef ROS2_CONTROL_COMPONENTS__BASE_INTERFACE_HPP_
#define ROS2_CONTROL_COMPONENTS__BASE_INTERFACE_HPP_

#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/types/hardware_interface_status_values.hpp"

namespace ros2_control_components
{

template<class InterfaceType>
class BaseInterface : public InterfaceType
{
public:
  hardware_interface::return_type configure(const hardware_interface::HardwareInfo & info) override
  {
    info_ = info;
    status_ = hardware_interface::status::CONFIGURED;
    return hardware_interface::return_type::OK;
  }

  std::string get_name() const
  {
    return info_.name;
  }

  hardware_interface::status get_status() const final
  {
    return status_;
  }

protected:
  hardware_interface::HardwareInfo info_;
  hardware_interface::status status_;
};

}  // namespace ros2_control_components

#endif  // ROS2_CONTROL_COMPONENTS__BASE_INTERFACE_HPP_
