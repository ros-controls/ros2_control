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


#ifndef ROS2_CONTROL_CORE__COMPONENTS_ACTUATOR_HPP_
#define ROS2_CONTROL_CORE__COMPONENTS_ACTUATOR_HPP_

#include <string>

#include "ros2_control_core/visibility_control.h"

#include "ros2_control_core/ros2_control_types.h"

#include "ros2_control_core/components/simple_component.hpp"

#include "ros2_control_core/hardware/actuator_hardware.hpp"


namespace ros2_control_core_components
{

class Actuator : private SimpleComponent< ros2_control_types::ActuatorDescription, ros2_control_core_hardware::ActuatorHardware >
{
public:
  ROS2_CONTROL_CORE_PUBLIC Actuator() = default;

  ROS2_CONTROL_CORE_PUBLIC virtual ~Actuator() = default;

  ROS2_CONTROL_CORE_PUBLIC virtual ros2_control_types::return_type read() = 0;

  ROS2_CONTROL_CORE_PUBLIC virtual ros2_control_types::return_type write() = 0;

  template < class ActuatorDataType >
  ROS2_CONTROL_CORE_PUBLIC ros2_control_types::return_type read(ActuatorDataType& data);

  template < class ActuatorDataType >
  ROS2_CONTROL_CORE_PUBLIC ros2_control_types::return_type write(ActuatorDataType& data);

  template < class ActuatorDataType >
  ROS2_CONTROL_CORE_PUBLIC ros2_control_types::return_type set_value(ActuatorDataType& data);

  template < typename ActuatorDataType >
  ROS2_CONTROL_CORE_PUBLIC ros2_control_types::return_type get_value(ActuatorDataType& data);

  ROS2_CONTROL_CORE_PUBLIC virtual ros2_control_types::return_type claim(std::string claimer_id) = 0;

  ROS2_CONTROL_CORE_PUBLIC virtual ros2_control_types::return_type unclaim(std::string claimer_id) = 0;

  ROS2_CONTROL_CORE_PUBLIC virtual bool isClaimed() = 0;

  ROS2_CONTROL_CORE_PUBLIC virtual bool canRead() = 0;

protected:
  bool can_read = false;
  std::string claimer;

};

}  // namespace ros2_control_core_components

#endif  // ROS2_CONTROL_CORE__COMPONENTS_ACTUATOR_HPP_
