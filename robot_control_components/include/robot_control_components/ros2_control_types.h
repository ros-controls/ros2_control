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

#ifndef ROBOT_CONTROL_COMPONENTS__ROS2_CONTROL_TYPES_H_
#define ROBOT_CONTROL_COMPONENTS__ROS2_CONTROL_TYPES_H_

#include "robot_control_components/visibility_control.h"

namespace robot_control_components
{
using components_ret_t = int;
static constexpr components_ret_t ROS2C_RETURN_OK = 0;
static constexpr components_ret_t ROS2C_RETURN_ERROR = 1;

static constexpr components_ret_t ROS2C_RETURN_ACTUATOR_CLAIMED_ERROR = 10;
static constexpr components_ret_t ROS2C_RETURN_ACTUATOR_ALREADY_CLAIMED = 11;
static constexpr components_ret_t ROS2C_RETURN_ACTUATOR_NOT_CLAIMED = 11;
static constexpr components_ret_t ROS2C_RETURN_ACTUATOR_UNATHORIZED_UNCLAIM = 13;
static constexpr components_ret_t ROS2C_RETURN_ACTUATOR_NON_CLAIMED_WRITE = 15;

static constexpr components_ret_t ROS2C_RETURN_ACTUATOR_CAN_NOT_READ = 20;


constexpr const auto ROS2C_INTERFACE_POSITION = "position";
constexpr const auto ROS2C_INTERFACE_VELOCITY = "velocity";
constexpr const auto ROS2C_INTERFACE_EFFORT = "effor";

}  // namespace robot_control_components
#endif  // ROBOT_CONTROL_COMPONENTS__ROS2_CONTROL_TYPES_H_
