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

#include "ros2_control_core/loaders_pluginlib.hpp"
#include "ros2_control_core/ros2_control_types.h"
#include "ros2_control_core/visibility_control.h"

#include "ros2_control_core/components/component.hpp"
#include "ros2_control_core/components/actuator.hpp"
#include "ros2_control_core/components/sensor.hpp"

#include "ros2_control_core/hardware/robot_hardware.hpp"

#include "control_msgs/msg/dynamic_joint_state.hpp"



namespace ros2_control_core_components
{

class Robot : protected Component< ros2_control_core_hardware::RobotHardware >
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(Robot)

  ROS2_CONTROL_CORE_PUBLIC Robot() = default;

  ROS2_CONTROL_CORE_PUBLIC Robot(std::string name, const rclcpp::Node::SharedPtr node, std::string param_base_path);

  ROS2_CONTROL_CORE_PUBLIC Robot(const std::string parameters_path, const rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr logging_interface,  const rclcpp::node_interfaces::NodeParametersInterface::SharedPtr parameters_interface, const rclcpp::node_interfaces::NodeServicesInterface::SharedPtr services_interface);

  ROS2_CONTROL_CORE_PUBLIC ~Robot() = default;

  ROS2_CONTROL_CORE_PUBLIC ros2_control_types::return_type init(ros2_control_types::RobotDescription description);

  ROS2_CONTROL_CORE_PUBLIC ros2_control_types::return_type recover();

  ROS2_CONTROL_CORE_PUBLIC ros2_control_types::return_type read();

  ROS2_CONTROL_CORE_PUBLIC ros2_control_types::return_type write();

  ROS2_CONTROL_CORE_PUBLIC ros2_control_types::return_type stop();


protected:
  control_msgs::msg::DynamicJointState joint_states_;

  std::vector<std::string> joints_;
  bool has_robots_;
  bool has_tools_;

  std::map<std::string, Actuator::SharedPtr> actuators_;
  std::map<std::string, Sensor::SharedPtr> sensors_;
  std::map<std::string, Robot::SharedPtr> robots_;

  std::map<std::string, Actuator::SharedPtr> tool_actuators_;
  std::map<std::string, Sensor::SharedPtr> tool_sensors_;



  template<typename T>
  std::shared_ptr<T> load_component_from_parameter(std::string parameter_name, const rclcpp::node_interfaces::NodeParametersInterface::SharedPtr parameters_interface, ros2_control_core::ROS2ControlLoaderPluginlib<T> class_loader, rclcpp::Logger logger)
  {
    std::shared_ptr<T> component;
    std::string class_name;
    bool class_available;

    parameters_interface->declare_parameter(parameter_name);
    class_name = parameters_interface->get_parameter(parameter_name).as_string();
    class_available = class_loader.is_available(class_name);
    if (class_available)
    {
      component = class_loader.create(class_name);
    }
    else
    {
      RCLCPP_WARN(logger, "Robot %s class is _not_ available.", class_name.c_str());
    }
    return component;
  };

  template<typename T>
  std::map<std::string, std::shared_ptr<T>> loadSubComponents(std::string parameters_prefix, const rclcpp::node_interfaces::NodeParametersInterface::SharedPtr parameters_interface, std::vector<std::string> name_list, ros2_control_core::ROS2ControlLoaderPluginlib<T> class_loader, rclcpp::Logger logger)
  {
    std::map<std::string, std::shared_ptr<T>> loaded_components;
    for (auto name: name_list)
    {
      loaded_components[name] = load_component_from_parameter<T>(parameters_prefix + "." + name + ".type", parameters_interface, class_loader, logger);
    }
    return loaded_components;
  };
};

}  // namespace ros2_control_core_components

#endif  // ROS2_CONTROL_CORE__COMPONENTS_ROBOT_HPP_
