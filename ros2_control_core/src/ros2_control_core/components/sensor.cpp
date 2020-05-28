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

#include "ros2_control_core/components/sensor.hpp"

using namespace ros2_control_core_components;

ros2_control_types::return_type Sensor::configure(const std::string parameters_path, const rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr logging_interface, const rclcpp::node_interfaces::NodeParametersInterface::SharedPtr parameters_interface, const rclcpp::node_interfaces::NodeServicesInterface::SharedPtr services_interface)
{
  ros2_control_types::return_type ret = ros2_control_types::ROS2C_RETURN_OK;
  ret = SimpleComponent::configure(parameters_path, "Sensor", logging_interface, parameters_interface, services_interface);

  if (has_hardware_)
  {
    ros2_control_utils::ROS2ControlLoaderPluginlib<ros2_control_core_hardware::SensorHardware> hw_loader("ros2_control_hardware", "ros2_control_core_hardware::SensorHardware");
    load_and_configure_hardware(hw_loader);
  }

  return ret;
}

ros2_control_types::return_type Sensor::read(const control_msgs::msg::InterfaceValue data)
{
  return ros2_control_types::ROS2C_RETURN_OK;
}
