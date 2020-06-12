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


#ifndef ROBOT_CONTROL_COMPONENTS__ROBOT_HPP_
#define ROBOT_CONTROL_COMPONENTS__ROBOT_HPP_

#include <string>
#include <vector>

#include "control_msgs/msg/dynamic_joint_state.hpp"
#include "control_msgs/msg/interface_value.hpp"

#include "rclcpp/macros.hpp"
#include "rclcpp/rclcpp.hpp"

#include "robot_control_components/visibility_control.h"


namespace robot_control_components
{

class Robot
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(Robot)

  ROS2_CONTROL_CORE_PUBLIC Robot() = default;

  ROS2_CONTROL_CORE_PUBLIC ~Robot() = default;

};

}  // namespace robot_control_components

#endif  // ROBOT_CONTROL_COMPONENTS__ROBOT_HPP_
