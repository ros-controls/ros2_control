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


#ifndef ROBOT_CONTROL_COMPONENTS__ACTUATOR_HPP_
#define ROBOT_CONTROL_COMPONENTS__ACTUATOR_HPP_

#include <algorithm>
#include <string>

#include "control_msgs/msg/interface_value.hpp"

#include "rclcpp/macros.hpp"
#include "rclcpp/rclcpp.hpp"

#include "robot_control_components/visibility_control.h"


namespace robot_control_components
{

class Actuator
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(Actuator)

  ROS2_CONTROL_CORE_PUBLIC Actuator() = default;

  ROS2_CONTROL_CORE_PUBLIC virtual ~Actuator() = default;

};

}  // namespace robot_control_components

#endif  // ROBOT_CONTROL_COMPONENTS__ACTUATOR_HPP_
