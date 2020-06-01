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


#ifndef ROS2_CONTROL_CORE__HARDWARE_COMPONENT_HARDWARE_HPP_
#define ROS2_CONTROL_CORE__HARDWARE_COMPONENT_HARDWARE_HPP_

#include "rclcpp/macros.hpp"
#include "rclcpp/rclcpp.hpp"

#include "ros2_control_core/communication_interface/hardware_communication_interface.hpp"

#include "ros2_control_core/ros2_control_types.h"
#include "ros2_control_core/ros2_control_utils.hpp"
#include "ros2_control_core/visibility_control.h"


namespace ros2_control_core_hardware
{

class ComponentHardware
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(ComponentHardware);

  ROS2_CONTROL_CORE_PUBLIC ComponentHardware() = default;

  ROS2_CONTROL_CORE_PUBLIC virtual ~ComponentHardware() = default;

  ROS2_CONTROL_CORE_PUBLIC ros2_control_types::return_type configure(const std::string parameters_path, const rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr logging_interface, const rclcpp::node_interfaces::NodeParametersInterface::SharedPtr parameters_interface, const rclcpp::node_interfaces::NodeServicesInterface::SharedPtr services_interface)
  {
    parameters_path_ = parameters_path;
    logging_interface_ = logging_interface;
    parameters_interface_ = parameters_interface;
    parameters_interface_->declare_parameter(parameters_path + ".has_communication_interface", rclcpp::ParameterValue(false));
    has_communication_interface_ = parameters_interface_->get_parameter(parameters_path + ".has_communication_interface").as_bool();

    if (has_communication_interface_)
    {
      ros2_control_utils::ROS2ControlLoaderPluginlib<ros2_control_core_communication_interface::HardwareCommunicationInterface> class_loader("ros2_control_core","ros2_control_core_communication_interface::HardwareCommunicationInterface");
      communication_interface_ = ros2_control_utils::load_component_from_parameter<ros2_control_core_communication_interface::HardwareCommunicationInterface>(parameters_path_ + ".HardwareCommunicationInterface.type", parameters_interface_, class_loader, logging_interface->get_logger());
      if (communication_interface_)
      {
        communication_interface_->configure(parameters_path_ + ".HardwareCommunicationInterface", logging_interface_, parameters_interface_, services_interface);
        //FIXME: Here is the "right" init called from the loaded class...
        communication_interface_->init();
      }
      else
      {
        RCLCPP_FATAL(logging_interface->get_logger(), "Communicaiton interface has to be defined if 'has_communication_interface=True'!");
      }
    }
    configure_hardware();
    //FIXME:DEBUG
    RCLCPP_INFO(logging_interface_->get_logger(), "Configuration for ComponentHardware '%s' is finished.", parameters_path_.c_str());
  };

  ROS2_CONTROL_CORE_PUBLIC ros2_control_types::return_type init()
  {
    ros2_control_types::return_type ret = ros2_control_types::ROS2C_RETURN_OK;
    if (has_communication_interface_)
    {
      //FIXME: Error initalizing interface... the virtual function is called.... why?
      //FIXME:DEBUG
      RCLCPP_INFO(logging_interface_->get_logger(), "%s calling communication init.", parameters_path_.c_str());
      ret = communication_interface_->init();
      //FIXME:DEBUG
      RCLCPP_INFO(logging_interface_->get_logger(), "%s calling 'init_hardware'.", parameters_path_.c_str());
      ret = init_hardware();
    }
    else
    {
      //FIXME:DEBUG
      RCLCPP_INFO(logging_interface_->get_logger(), "Initalizing %s ComponentHardware without communication interface.", parameters_path_.c_str());

      ret = init_hardware();

      //FIXME:DEBUG
      RCLCPP_INFO(logging_interface_->get_logger(), "%s ComponentHardware is initalized.", parameters_path_.c_str());
    }
    return ret;
  };

  ROS2_CONTROL_CORE_PUBLIC ros2_control_types::return_type configure_hardware(){
    RCLCPP_WARN(logging_interface_->get_logger(), "Configuration for %s ComponentHardware not implemented. Is this intended?", parameters_path_.c_str());
  };

  ROS2_CONTROL_CORE_PUBLIC ros2_control_types::return_type init_hardware(){
    RCLCPP_WARN(logging_interface_->get_logger(), "Initialization for %s ComponentHardware not implemented. Is this intended?", parameters_path_.c_str());
  };

  //TODO: This should be also implemented...
//   ROS2_CONTROL_CORE_PUBLIC virtual ros2_control_types::return_type halt_hardware() = 0;
//   ROS2_CONTROL_CORE_PUBLIC virtual ros2_control_types::return_type recover_hardware() = 0;
//   ROS2_CONTROL_CORE_PUBLIC virtual ros2_control_types::return_type start_hardware() = 0;
//   ROS2_CONTROL_CORE_PUBLIC virtual ros2_control_types::return_type stop_hardware() = 0;

protected:
  bool has_communication_interface_;
  std::string parameters_path_;
  rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr logging_interface_;
  rclcpp::node_interfaces::NodeParametersInterface::SharedPtr parameters_interface_;

  ros2_control_core_communication_interface::HardwareCommunicationInterface::SharedPtr communication_interface_;

// FIXME: make this matching over templates to have only specific component
// Introduce BaseComponentHardware
  std::vector<std::string> valid_interface_names_;
};

}  // namespace ros2_control_core_hardware

#endif  // ROS2_CONTROL_CORE__HARDWARE_COMPONENT_HARDWARE_HPP_
