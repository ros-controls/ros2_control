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


#ifndef ROS2_CONTROL_CORE__COMMUNICATION_INTERFACE_HARDWARE_COMMUNICATION_INTERFACE_HPP_
#define ROS2_CONTROL_CORE__COMMUNICATION_INTERFACE_HARDWARE_COMMUNICATION_INTERFACE_HPP_

#include "ros2_control_core/ros2_control_types.h"
#include "ros2_control_core/visibility_control.h"


namespace ros2_control_core_communication_interface
{

class HardwareCommunicationInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(HardwareCommunicationInterface);

  ROS2_CONTROL_CORE_PUBLIC HardwareCommunicationInterface() = default;

  ROS2_CONTROL_CORE_PUBLIC virtual ~HardwareCommunicationInterface() = default;

  ROS2_CONTROL_CORE_PUBLIC ros2_control_types::return_type configure(const std::string parameters_path, const rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr logging_interface,  const rclcpp::node_interfaces::NodeParametersInterface::SharedPtr parameters_interface, const rclcpp::node_interfaces::NodeServicesInterface::SharedPtr services_interface)
  {
    parameters_path_ = parameters_path;
    logging_interface_ = logging_interface;
    parameters_interface_ = parameters_interface;
    //FIXME:DEBUG
    RCLCPP_INFO(logging_interface_->get_logger(), "%s is configured!", parameters_path_.c_str());

    //TODO: Add call to library configure is needed
    return ros2_control_types::ROS2C_RETURN_OK;
  };

  ROS2_CONTROL_CORE_PUBLIC virtual ros2_control_types::return_type init() = 0;

protected:
  std::string parameters_path_;

  rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr logging_interface_;
  rclcpp::node_interfaces::NodeParametersInterface::SharedPtr parameters_interface_;
};

}  // ros2_control_core_hardware

#endif  // ROS2_CONTROL_CORE__COMMUNICATION_INTERFACE_HARDWARE_COMMUNICATION_INTERFACE_HPP_
