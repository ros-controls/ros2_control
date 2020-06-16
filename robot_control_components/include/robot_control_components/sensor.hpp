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


#ifndef ROBOT_CONTROL_COMPONENTS__SENSOR_HPP_
#define ROBOT_CONTROL_COMPONENTS__SENSOR_HPP_

#include <string>
#include <vector>

#include "control_msgs/msg/interface_value.hpp"

#include "rclcpp/macros.hpp"
#include "rclcpp/rclcpp.hpp"

#include "robot_control_components/component_info.hpp"
#include "robot_control_components/ros2_control_types.h"
#include "robot_control_components/visibility_control.h"

namespace robot_control_components
{

class Sensor
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(Sensor);

  Sensor() = default;

  virtual ~Sensor() = default;

  ROBOT_CONTROL_COMPONENTS_PUBLIC components_ret_t configure(ComponentInfo sensor_info);

  ROBOT_CONTROL_COMPONENTS_PUBLIC components_ret_t init();

  ROBOT_CONTROL_COMPONENTS_PUBLIC components_ret_t recover();

  ROBOT_CONTROL_COMPONENTS_PUBLIC components_ret_t start();

  ROBOT_CONTROL_COMPONENTS_PUBLIC components_ret_t stop();

  ROBOT_CONTROL_COMPONENTS_PUBLIC components_ret_t read();

  ROBOT_CONTROL_COMPONENTS_PUBLIC components_ret_t get_value(control_msgs::msg::InterfaceValue * data);

  ROBOT_CONTROL_COMPONENTS_PUBLIC std::vector<std::string> get_interface_names();

protected:
  std::string name_;
  std::string type_;
  bool has_hardware_;
  bool read_async_;

  std::vector<std::string> valid_interface_names_;
  std::vector<std::string> interface_names_;
  control_msgs::msg::InterfaceValue data_;
};

}  // namespace robot_control_components

#endif  // ROBOT_CONTROL_COMPONENTS__SENSOR_HPP_
