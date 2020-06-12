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

/* This header must be included by all rclcpp headers which declare symbols
 * which are defined in the rclcpp library. When not building the rclcpp
 * library, i.e. when using the headers in other package's code, the contents
 * of this header change the visibility of certain symbols which the rclcpp
 * library cannot have, but the consuming code must have inorder to link.
 */

#ifndef ROBOT_CONTROL_COMPONENTS__ROS2_CONTROL_TYPES_H_
#define ROBOT_CONTROL_COMPONENTS__ROS2_CONTROL_TYPES_H_

#include <map>
#include <string>

#include "rclcpp/rclcpp.hpp"

#include "robot_control_components/visibility_control.h"

#include "control_msgs/msg/interface_value.hpp"


// TODO: Do we need "ROS2_CONTROL_CORE_PUBLIC" before the variables?

namespace robot_control_components
{

using components_ret_t = int;
static constexpr components_ret_t ROS2C_RETURN_OK = 0;
static constexpr components_ret_t ROS2C_RETURN_ERROR = 1;

static constexpr components_ret_t ROS2C_RETURN_ACTUATOR_CLAIMED_ERROR = 10;
static constexpr components_ret_t ROS2C_RETURN_ACTUATOR_ALREADY_CLAIMED = 11;
static constexpr components_ret_t ROS2C_RETURN_ACTUATOR_NOT_CLAIMED = 11;
static constexpr components_ret_t ROS2C_RETURN_ACTUATOR_UNATHORIZED_UNCLAIM = 13;
static constexpr components_ret_t ROS2C_RETURN_ACTUATOR_CAN_NOT_READ = 20;

// TODO: Check if needed or lifecycle component can do this
// using component_state_type = uint;
// static constexpr component_state_type ROS2C_COMPONENT_STATE_LOADED = 1;
// static constexpr component_state_type ROS2C_COMPONENT_STATE_CONFIGURED = 2;
// static constexpr component_state_type ROS2C_COMPONENT_STATE_INITIALIZED = 3;
// static constexpr component_state_type ROS2C_COMPONENT_STATE_STARTED = 4;
// static constexpr component_state_type ROS2C_COMPONENT_STATE_STOPPED = 5;
// static constexpr component_state_type ROS2C_COMPONENT_STATE_HALTED = 6;


}  // namespace robot_control_components

#endif  // ROBOT_CONTROL_COMPONENTS__ROS2_CONTROL_TYPES_H_
