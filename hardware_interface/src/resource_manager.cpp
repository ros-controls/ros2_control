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

#include "hardware_interface/resource_manager.hpp"

#include <map>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "hardware_interface/actuator_hardware_interface.hpp"
#include "hardware_interface/actuator_hardware.hpp"
#include "hardware_interface/component_parser.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/robot_hardware_interface.hpp"
#include "hardware_interface/sensor_hardware_interface.hpp"
#include "hardware_interface/sensor_hardware.hpp"
#include "hardware_interface/system_hardware_interface.hpp"
#include "hardware_interface/system_hardware.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "pluginlib/class_loader.hpp"
#include "rclcpp/rclcpp.hpp"

namespace resource_manager
{

ResourceManager::ResourceManager()
{
  actuator_loader_.reset(new pluginlib::ClassLoader<hardware_interface::ActuatorHardwareInterface>(
      "hardware_interface", "hardware_interface::ActuatorHardwareInterface"));
  sensor_loader_.reset(new pluginlib::ClassLoader<hardware_interface::SensorHardwareInterface>(
      "hardware_interface", "hardware_interface::SensorHardwareInterface"));
  system_loader_.reset(new pluginlib::ClassLoader<hardware_interface::SystemHardwareInterface>(
      "hardware_interface", "hardware_interface::SystemHardwareInterface"));

  joint_loader_.reset(new pluginlib::ClassLoader<hardware_interface::components::Joint>(
      "hardware_interface", "hardware_interface::components::Joint"));
}

//  Non real-time safe functions
return_type
ResourceManager::load_and_configure_resources_from_urdf(std::string urdf_string)
{
  std::vector<hardware_interface::HardwareInfo> hardware_info_list =
    hardware_interface::parse_control_resources_from_urdf(urdf_string);

  return_type ret = return_type::OK;
  for (auto hardware_info : hardware_info_list) {
    RCLCPP_INFO(rclcpp::get_logger("ros2_control_ResourceManager"),
      "Loading hardware plugin: " + hardware_info.hardware_class_type);
    if (!hardware_info.type.compare("system")) {
      // TODO(anyone): this here is really not nice...
      std::unique_ptr<hardware_interface::SystemHardwareInterface> sys_hw_if;
      sys_hw_if.reset(system_loader_->createUnmanagedInstance(hardware_info.hardware_class_type));
      std::shared_ptr<hardware_interface::SystemHardware> system_hw =
        std::make_shared<hardware_interface::SystemHardware>(std::move(sys_hw_if));
      ret = system_hw->configure(hardware_info);
      if (ret != return_type::OK) {
        return ret;
      }
      systems_.push_back(system_hw);
      // TODO(anyone): do the same for loading sensors and actuators hardware
    } else {
      RCLCPP_FATAL(rclcpp::get_logger("ros2_control_ResourceManager"),
        "hardware type not recognized");
      return return_type::ERROR;
    }

    std::vector<std::shared_ptr<hardware_interface::components::Joint>> joints;
    for (auto joint_info : hardware_info.joints) {
      RCLCPP_INFO(rclcpp::get_logger("ros2_control_ResourceManager"),
        "Loading joint plugin: " + joint_info.class_type);
      std::shared_ptr<hardware_interface::components::Joint> joint =
        joint_loader_->createSharedInstance(joint_info.class_type);
      ret = joint->configure(joint_info);
      if (ret != hardware_interface::return_type::OK) {
        return ret;
      }
      joints.push_back(joint);
      joint_to_hardware_mapping_[joint_info.name] = hardware_info.name;
    }
    joint_components_[hardware_info.name] = joints;

    // TODO(anyone): add implementation for sensors
  }

  RCLCPP_INFO(rclcpp::get_logger("ros2_control_ResourceManager"),
    "All hardware and component plugins loaded and configured successfully.");
  return return_type::OK;
}

return_type ResourceManager::start_all_resources()
{
  return_type ret = return_type::OK;
  for (auto system : systems_) {
    ret = system->start();
    if (ret != return_type::OK) {
      return ret;
    }
    // initial read of joints
    ret = system->read_joints(joint_components_[system->get_name()]);
    if (ret != return_type::OK) {
      return ret;
    }
    // TODO(anyone): add support to read sensors of a system is they exist
  }
  // TODO(anyone): add sensors and actuators
  return return_type::OK;
}

return_type ResourceManager::stop_all_resources()
{
  return_type ret = return_type::OK;
  for (auto system : systems_) {
    ret = system->stop();
    if (ret != return_type::OK) {
      return ret;
    }
    // TODO(anyone): add support to read sensors of a system is they exist
  }
  // TODO(anyone): add sensors and actuators
  return return_type::OK;
}

//  Real-time safe functions (at least the goal is to be...)
return_type ResourceManager::read_all_resources()
{
  return_type ret = return_type::OK;
  for (auto system : systems_) {
    ret = system->read_joints(joint_components_[system->get_name()]);
    if (ret != return_type::OK) {
      return ret;
    }
    // TODO(anyone): add support to read sensors of a system is they exist
  }
  // TODO(anyone): add sensors and actuators
  return ret;
}

return_type ResourceManager::write_all_resources()
{
  return_type ret = return_type::OK;
  for (auto system : systems_) {
    ret = system->write_joints(joint_components_[system->get_name()]);
    if (ret != return_type::OK) {
      return ret;
    }
    // TODO(anyone): add support to read sensors of a system is they exist
  }
  // TODO(anyone): add sensors and actuators
  return ret;
}

}  //  namespace resource_manager
