// Copyright 2017 Open Source Robotics Foundation, Inc.
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

#include "hardware_interface/resource_manager.hpp"

#include "hardware_interface/utils/component_parser.hpp"
#include "hardware_interface/utils/ros2_control_utils.hpp"
#include "rclcpp/rclcpp.hpp"

namespace hardware_interface
{

hardware_interface_ret_t ResourceManager::parse_system_from_urdf(const std::string & urdf_string)
{
  try {
    system_info_ = utils::parse_robot_from_urdf(urdf_string);
  } catch (const std::runtime_error & e) {
    RCLCPP_FATAL(logger_, "Error while parsing URDF: " + std::string(e.what()));
    return HW_RET_ERROR;
  }

  if (system_info_.type == robotType) {
    // Check if robot_hardware exists
    if (!system_info_.hardware_class_type.empty()) {
      RobotHardwareLoaderType robot_loader = RobotHardwareLoaderType("hardware_interface", "hardware_interface::RobotHardware");
      if (robot_loader.is_available(system_info_.hardware_class_type)) {
        RCLCPP_DEBUG(logger_, "SystemInterface class %s found.", system_info_.hardware_class_type.c_str());
        robot_hardware_ = robot_loader.create_unique(system_info_.hardware_class_type);
      }
      else {
        RCLCPP_FATAL(logger_, "SystemInterface class %s is not available! Exiting.", system_info_.hardware_class_type.c_str());
      }
    }
    // TODO: A robot is a system
//     if (!system_info_.hardware_class_type.empty()) {
//       SystemInterfaceLoaderType sys_loader = SystemInterfaceLoaderType("hardware_interface", "hardware_interface::RobotHardware");
//       if (sys_loader.is_available(system_info_.hardware_class_type)) {
//         RCLCPP_DEBUG(logger_, "SystemInterface class %s found.", system_info_.hardware_class_type.c_str());
//         system_itf_ = sys_loader.create_unique(system_info_.hardware_class_type);
//       }
//       else {
//         RCLCPP_FATAL(logger_, "SystemInterface class %s is not available! Exiting.", system_info_.hardware_class_type.c_str());
//       }
//     }
  }

  //TODO: Implement here for actuators and sensors

  ActuatorInterfaceLoaderType actuator_loader = ActuatorInterfaceLoaderType("hardware_interface", "hardware_interface::ActuatorInterface");
  SensorInterfaceLoaderType sensor_loader = SensorInterfaceLoaderType("hardware_interface", "hardware_interface::SensorInterface");

  std::shared_ptr<ActuatorInterface> actuator_itf;
  std::shared_ptr<SensorInterface> sensor_itf;

  for (auto comp_info : system_info_.subcomponents) {
    if (comp_info.type == actuatorType) {
      if (actuator_loader.is_available(system_info_.class_type)) {
        RCLCPP_DEBUG(logger_, "ActuatorInterface class %s found.", comp_info.class_type.c_str());
        actuator_itf = actuator_loader.create_shared(comp_info.class_type);
        actuators_[comp_info.joint] = Actuator(actuator_itf);
      }
      else {
        RCLCPP_FATAL(logger_, "ActuatorInterface class %s is not available! Exiting.", comp_info.class_type.c_str());
      }
    }
    else if (comp_info.type == sensorType) {
      if (sensor_loader.is_available(system_info_.class_type)) {
        RCLCPP_DEBUG(logger_, "SensorInterface class %s found.", comp_info.class_type.c_str());
        sensor_itf = sensor_loader.create_shared(comp_info.class_type);
        sensors_[comp_info.joint] = Sensor(sensor_itf);
      }
      else {
        RCLCPP_FATAL(logger_, "SensorInterface class %s is not available! Exiting.", comp_info.class_type.c_str());
      }

    }
  }

  return HW_RET_OK;
}

hardware_interface_ret_t ResourceManager::init_system()
{
  hardware_interface_ret_t ret = HW_RET_OK;


  return ret;
}

}  // namespace hardware_interface
