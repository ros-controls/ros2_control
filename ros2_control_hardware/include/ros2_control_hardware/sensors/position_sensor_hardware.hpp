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


#ifndef ROS2_CONTROL_HARDWARE__SENSORS_POSITION_SENSOR_HPP_
#define ROS2_CONTROL_HARDWARE__SENSORS_POSITION_SENSOR_HPP_

#include "rclcpp/macros.hpp"

#include "ros2_control_hardware/visibility_control.h"
#include "ros2_control_core/hardware/sensor_hardware.hpp"

namespace ros2_control_hardware_sensors
{

class PositionSensorHardware : public ros2_control_core_hardware::SensorHardware
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(PositionSensorHardware);

  ROS2_CONTROL_HARDWARE_PUBLIC PositionSensorHardware() = default;

  ROS2_CONTROL_HARDWARE_PUBLIC ~PositionSensorHardware() = default;

  ROS2_CONTROL_HARDWARE_PUBLIC ros2_control_types::return_type configure(const std::string parameters_path, const rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr logging_interface, const rclcpp::node_interfaces::NodeParametersInterface::SharedPtr parameters_interface, const rclcpp::node_interfaces::NodeServicesInterface::SharedPtr services_interface);

};

}  // namespace ros2_control_hardware_sensors

#endif  // ROS2_CONTROL_HARDWARE__SENSORS_POSITION_SENSOR_HPP_
