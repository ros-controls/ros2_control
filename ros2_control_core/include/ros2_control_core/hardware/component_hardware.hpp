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


#ifndef ROS2_CONTROL_CORE__HARDWARE_COMPONENT_HARDWARE_HPP_
#define ROS2_CONTROL_CORE__HARDWARE_COMPONENT_HARDWARE_HPP_

#include "ros2_control_core/visibility_control.h"

#include "ros2_control_core/ros2_control_types.h"

#include "ros2_control_core/communication_interface/hardware_communication_interface.hpp"

namespace ros2_control_core_hardware
{

template < typename ComponentHardwareDescriptionType >
class ComponentHardware
{
public:
  ROS2_CONTROL_CORE_PUBLIC ComponentHardware() = default;

  ROS2_CONTROL_CORE_PUBLIC virtual ~ComponentHardware() = default;

  ROS2_CONTROL_CORE_PUBLIC ros2_control_types::return_type init(ComponentHardwareDescriptionType description);

protected:
  ComponentHardwareDescriptionType description;
  ros2_control_core_communication_interface::HardwareCommunicationInterface communication_interface;

};

}  // namespace ros2_control_core_hardware

#endif  // ROS2_CONTROL_CORE__HARDWARE_COMPONENT_HARDWARE_HPP_
