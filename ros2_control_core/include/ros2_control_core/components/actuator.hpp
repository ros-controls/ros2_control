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

#include "control_msgs/msg/interface_value.hpp"

#include "rclcpp/macros.hpp"
#include "rclcpp/rclcpp.hpp"

#include "ros2_control_core/components/simple_component.hpp"
#include "ros2_control_core/hardware/actuator_hardware.hpp"

#include "ros2_control_core/ros2_control_types.h"
#include "ros2_control_core/visibility_control.h"


namespace ros2_control_core_components
{

class Actuator : public SimpleComponent< ros2_control_core_hardware::ActuatorHardware >
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(Actuator)

  ROS2_CONTROL_CORE_PUBLIC Actuator() = default;

  ROS2_CONTROL_CORE_PUBLIC virtual ~Actuator() = default;

  ROS2_CONTROL_CORE_PUBLIC ros2_control_types::return_type configure(const std::string parameters_path, const rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr logging_interface, const rclcpp::node_interfaces::NodeParametersInterface::SharedPtr parameters_interface, const rclcpp::node_interfaces::NodeServicesInterface::SharedPtr services_interface);

//   ROS2_CONTROL_CORE_PUBLIC virtual ros2_control_types::return_type read() = 0;

//   ROS2_CONTROL_CORE_PUBLIC virtual ros2_control_types::return_type write() = 0;

  ROS2_CONTROL_CORE_PUBLIC ros2_control_types::return_type read(const control_msgs::msg::InterfaceValue data);

  ROS2_CONTROL_CORE_PUBLIC ros2_control_types::return_type write(const control_msgs::msg::InterfaceValue::SharedPtr data);

//   ROS2_CONTROL_CORE_PUBLIC ros2_control_types::return_type set_value(const control_msgs::msg::InterfaceValue::SharedPtr data);

//   ROS2_CONTROL_CORE_PUBLIC ros2_control_types::return_type get_value(const control_msgs::msg::InterfaceValue::SharedPtr data);

  ROS2_CONTROL_CORE_PUBLIC ros2_control_types::return_type claim(const std::string claimer_id);

  ROS2_CONTROL_CORE_PUBLIC ros2_control_types::return_type unclaim(const std::string claimer_id);

  ROS2_CONTROL_CORE_PUBLIC bool isClaimed();

  ROS2_CONTROL_CORE_PUBLIC bool canRead();

protected:
  bool can_read = false;
  std::string claimer = "";
};

}  // namespace ros2_control_core_components

#endif  // ROS2_CONTROL_CORE__COMPONENTS_ACTUATOR_HPP_
