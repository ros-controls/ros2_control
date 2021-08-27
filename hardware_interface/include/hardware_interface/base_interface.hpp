// Copyright 2021 ros2_control Development Team
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
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

namespace hardware_interface
{
template <class InterfaceType>
class BaseInterface : public InterfaceType
{
public:
  CallbackReturn on_init(const HardwareInfo & info) override
  {
    info_ = info;
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_init_default(const HardwareInfo & info)
  {
    return BaseInterface<InterfaceType>::on_init(info);
  }

  std::string get_name() const final { return info_.name; }

protected:
  HardwareInfo info_;
};

}  // namespace hardware_interface

#endif  // HARDWARE_INTERFACE__BASE_INTERFACE_HPP_
