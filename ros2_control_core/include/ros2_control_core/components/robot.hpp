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


#ifndef ROS2_CONTROL_CORE__COMPONENTS_ROBOT_HPP_
#define ROS2_CONTROL_CORE__COMPONENTS_ROBOT_HPP_

#include <string>
#include <vector>

#include "rclcpp/macros.hpp"
#include "rclcpp/rclcpp.hpp"

#include "ros2_control_core/ros2_control_types.h"
#include "ros2_control_core/ros2_control_utils.hpp"
#include "ros2_control_core/visibility_control.h"

#include "ros2_control_core/components/component.hpp"
#include "ros2_control_core/components/actuator.hpp"
#include "ros2_control_core/components/sensor.hpp"

#include "ros2_control_core/hardware/robot_hardware.hpp"

#include "control_msgs/msg/dynamic_joint_state.hpp"



namespace ros2_control_core_components
{

class Robot : public Component< ros2_control_core_hardware::RobotHardware >
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(Robot)

  ROS2_CONTROL_CORE_PUBLIC Robot() = default;

  ROS2_CONTROL_CORE_PUBLIC ~Robot() = default;

  ROS2_CONTROL_CORE_PUBLIC ros2_control_types::return_type configure(const std::string parameters_path, const rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr logging_interface,  const rclcpp::node_interfaces::NodeParametersInterface::SharedPtr parameters_interface, const rclcpp::node_interfaces::NodeServicesInterface::SharedPtr services_interface);

  ROS2_CONTROL_CORE_PUBLIC ros2_control_types::return_type init();

  ROS2_CONTROL_CORE_PUBLIC ros2_control_types::return_type recover();

  ROS2_CONTROL_CORE_PUBLIC ros2_control_types::return_type read();

  ROS2_CONTROL_CORE_PUBLIC ros2_control_types::return_type write();

  ROS2_CONTROL_CORE_PUBLIC ros2_control_types::return_type stop();


protected:
  control_msgs::msg::DynamicJointState joint_states_;

  std::vector<std::string> joint_names_;
  std::vector<std::string> robot_names_;
  std::vector<std::string> tool_actuator_names_;
  std::vector<std::string> tool_sensor_names_;
  bool has_robots_;
  bool has_tools_;

  std::map<std::string, Actuator::SharedPtr> actuators_;
  std::map<std::string, Sensor::SharedPtr> sensors_;
  std::map<std::string, Robot::SharedPtr> robots_;

  std::map<std::string, Actuator::SharedPtr> tool_actuators_;
  std::map<std::string, Sensor::SharedPtr> tool_sensors_;

private:

  ros2_control_types::return_type configure_components(std::string parameters_prefix, std::vector<std::string> name_list, std::map<std::string, auto> component_list)
  {
    ros2_control_types::return_type ret = ros2_control_types::ROS2C_RETURN_OK;
    ros2_control_types::return_type temp_ret;
    for (auto name: name_list)
    {
      if (component_list[name])
      {
        temp_ret = component_list[name]->configure(parameters_prefix + "." + name, logging_interface_, parameters_interface_, services_interface_);
        if (temp_ret != ros2_control_types::ROS2C_RETURN_OK) {
          ret = temp_ret;
          RCLCPP_ERROR(logging_interface_->get_logger(), "%s: could not be configured!", (parameters_prefix + "." + name).c_str());
        }
      }
      else
      {
        ret = ros2_control_types::ROS2C_RETURN_ERROR;
        RCLCPP_ERROR(logging_interface_->get_logger(), "Component '%s' could not be configured because the refrerence is NULL!", name.c_str());
      }
    }
    return ret;
  };

  ros2_control_types::return_type init_components(const std::map<std::string, auto> component_list){
    ros2_control_types::return_type ret = ros2_control_types::ROS2C_RETURN_OK;
    ros2_control_types::return_type temp_ret;
    for (auto element: component_list)
    {
      auto component = element.second;
      if (component)
      {
        temp_ret = component->init();
        if (temp_ret != ros2_control_types::ROS2C_RETURN_OK) {
          ret = temp_ret;
        }
      }
      else
      {
        RCLCPP_ERROR(logging_interface_->get_logger(), "Component '%s' could not be initalized because the refrerence is NULL!", element.first.c_str());
      }
    }
    return ret;
  };

};

}  // namespace ros2_control_core_components

#endif  // ROS2_CONTROL_CORE__COMPONENTS_ROBOT_HPP_
