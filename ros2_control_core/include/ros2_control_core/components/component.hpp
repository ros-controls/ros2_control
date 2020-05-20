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


#ifndef ROS2_CONTROL_CORE__COMPONENTS_BASE_COMPONENT_HPP_
#define ROS2_CONTROL_CORE__COMPONENTS_BASE_COMPONENT_HPP_

#include <string>

#include "ros2_control_core/visibility_control.h"

#include "ros2_control_core/ros2_control_types.h"

#include "ros2_control_core/hardware/component_hardware.hpp"

namespace ros2_control_core_components
{

template < typename ComponentDescriptionType, typename ComponentHardwareType >
class Component
{
public:
  ROS2_CONTROL_CORE_PUBLIC Component() = default;

  ROS2_CONTROL_CORE_PUBLIC Component(std::string name){
    this->name = name;
  };

  ROS2_CONTROL_CORE_PUBLIC virtual ~Component() = default;

  ROS2_CONTROL_CORE_PUBLIC ros2_control_types::return_type init(ComponentDescriptionType description_in);

  // TODO: Remove is not used...
//   ROS2_CONTROL_CORE_PUBLIC virtual ros2_control_types::return_type init(std::string name, ros2_control_types::HardwareDescription hardware_description);

  ROS2_CONTROL_CORE_PUBLIC virtual ros2_control_types::return_type recover() = 0;

  ROS2_CONTROL_CORE_PUBLIC virtual ros2_control_types::return_type stop() = 0;

protected:
  std::string name;
  ComponentDescriptionType description;
  ComponentHardwareType hardware;
};

}  // namespace ros2_control_core_components

#endif  // ROS2_CONTROL_CORE__COMPONENTS_BASE_COMPONENT_HPP_
