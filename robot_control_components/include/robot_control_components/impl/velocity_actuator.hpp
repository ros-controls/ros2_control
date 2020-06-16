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


#ifndef ROBOT_CONTROL_COMPONENTS__IMPL_VELOCITY_ACTUATOR_HPP_
#define ROBOT_CONTROL_COMPONENTS__IMPL_VELOCITY_ACTUATOR_HPP_


#include "rclcpp/macros.hpp"

#include "robot_control_components/ros2_control_types.h"
#include "robot_control_components/actuator.hpp"
#include "robot_control_components/visibility_control.h"


using namespace robot_control_components;

namespace robot_control_components_actuators
{

class VelocityActuator : public Actuator
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(VelocityActuator);

  ROBOT_CONTROL_COMPONENTS_PUBLIC VelocityActuator() : Actuator() {
    valid_interface_names_.resize(1);
    valid_interface_names_.push_back(ROS2C_INTERFACE_VELOCITY);
  };

  ROBOT_CONTROL_COMPONENTS_PUBLIC ~VelocityActuator() = default;

};

}  // namespace robot_control_components_sensors

#endif  // ROBOT_CONTROL_COMPONENTS__IMPL_VELOCITY_ACTUATOR_HPP_

