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

#include "ros2_control_core/components/actuator.hpp"

using namespace ros2_control_core_components;

ros2_control_types::return_type Actuator::configure(const std::string parameters_path, const rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr logging_interface, const rclcpp::node_interfaces::NodeParametersInterface::SharedPtr parameters_interface, const rclcpp::node_interfaces::NodeServicesInterface::SharedPtr services_interface)
{
  ros2_control_types::return_type ret = ros2_control_types::ROS2C_RETURN_OK;
  ret = SimpleComponent::configure(parameters_path, "Actuator", logging_interface, parameters_interface, services_interface);

  parameters_interface_->declare_parameter(parameters_path_ + ".can_read");

  return ret;
}


ros2_control_types::return_type Actuator::read(const control_msgs::msg::InterfaceValue data)
{
  return ros2_control_types::ROS2C_RETURN_OK;
}

ros2_control_types::return_type Actuator::write(const control_msgs::msg::InterfaceValue::SharedPtr data)
{
  return ros2_control_types::ROS2C_RETURN_OK;
}

ros2_control_types::return_type Actuator::claim(std::string claimer_id)
{
  ros2_control_types::return_type ret = ros2_control_types::ROS2C_RETURN_OK;

  if (!isClaimed()) {
    claimer = claimer_id;
  }
  else {
    ret = ros2_control_types::ROS2C_RETURN_ACTUATOR_ALREADY_CLAIMED;
  }

  return ret;
}


ros2_control_types::return_type Actuator::unclaim(std::string claimer_id)
{
  ros2_control_types::return_type ret = ros2_control_types::ROS2C_RETURN_OK;
  if (isClaimed()) {
    if (claimer.compare(claimer_id) == 0) {
      claimer = "";
    }
    else {
      ret = ros2_control_types::ROS2C_RETURN_ACTUATOR_UNATHORIZED_UNCLAIM;
    }
  }
  else {
    ret = ros2_control_types::ROS2C_RETURN_ACTUATOR_NOT_CLAIMED;
  }

  return ret;
}


bool Actuator::isClaimed()
{
  return !claimer.empty();
}


bool Actuator::canRead()
{
  return can_read;
}
