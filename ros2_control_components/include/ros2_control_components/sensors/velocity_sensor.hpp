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


#ifndef ROS2_CONTROL_COMPONENTS__SENSORS_VELOCITY_SENSOR_HPP_
#define ROS2_CONTROL_COMPONENTS__SENSORS_VELOCITY_SENSOR_HPP_

#include "rclcpp/macros.hpp"

#include "ros2_control_components/visibility_control.h"
#include "ros2_control_core/components/sensor.hpp"

namespace ros2_control_components_sensors
{

class VelocitySensor : public ros2_control_core_components::Sensor
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(VelocitySensor);

  ROS2_CONTROL_COMPONENTS_PUBLIC VelocitySensor() = default;

  ROS2_CONTROL_COMPONENTS_PUBLIC ~VelocitySensor() = default;

  ROS2_CONTROL_COMPONENTS_PUBLIC ros2_control_types::return_type configure(const std::string parameters_path, const rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr logging_interface, const rclcpp::node_interfaces::NodeParametersInterface::SharedPtr parameters_interface, const rclcpp::node_interfaces::NodeServicesInterface::SharedPtr services_interface);

};

}  // namespace ros2_control_components_sensors

#endif  // ROS2_CONTROL_COMPONENTS__SENSORS_VELOCITY_SENSOR_HPP_
