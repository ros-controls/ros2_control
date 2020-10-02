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
  actuator_loader_.reset(
    new pluginlib::ClassLoader<hardware_interface::ActuatorHardwareInterface>(
      "hardware_interface", "hardware_interface::ActuatorHardwareInterface"));
  sensor_loader_.reset(
    new pluginlib::ClassLoader<hardware_interface::SensorHardwareInterface>(
      "hardware_interface", "hardware_interface::SensorHardwareInterface"));
  system_loader_.reset(
    new pluginlib::ClassLoader<hardware_interface::SystemHardwareInterface>(
      "hardware_interface", "hardware_interface::SystemHardwareInterface"));

  joint_loader_.reset(
    new pluginlib::ClassLoader<hardware_interface::components::Joint>(
      "hardware_interface", "hardware_interface::components::Joint"));
}

//  No real-time safe functions
return_type
ResourceManager::load_and_configure_resources_from_urdf(std::string urdf_string)
{
  std::vector<hardware_interface::HardwareInfo> hardware_info_list =
    hardware_interface::parse_control_resources_from_urdf(urdf_string);

  return_type ret = return_type::OK;
  for (auto hardware_info : hardware_info_list) {
    RCLCPP_INFO(
      rclcpp::get_logger("ros2_control_ResourceManager"),
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
      RCLCPP_FATAL(
        rclcpp::get_logger("ros2_control_ResourceManager"),
        "hardware type not recognized");
      return return_type::ERROR;
    }

    std::vector<std::shared_ptr<hardware_interface::components::Joint>> joints;
    for (auto joint_info : hardware_info.joints) {
      RCLCPP_INFO(
        rclcpp::get_logger("ros2_control_ResourceManager"),
        "Loading joint plugin: " + joint_info.class_type);
      std::shared_ptr<hardware_interface::components::Joint> joint =
        joint_loader_->createSharedInstance(joint_info.class_type);
      ret = joint->configure(joint_info);
      if (ret != hardware_interface::return_type::OK) {
        return ret;
      }
      joints.push_back(joint);
      command_interfaces_[joint_info.name] = joint->get_command_interfaces();
      // TODO(anyone): add checking if Joint has state interfaces at all
      state_interfaces_[joint_info.name] = joint->get_state_interfaces();
      joint_components_[joint_info.name] = joint;
      claimed_command_interfaces_[joint_info.name] = std::vector<std::string>();
    }
    joint_components_for_hardware_[hardware_info.name] = joints;

    // TODO(anyone): add implementation for sensors
  }

  RCLCPP_INFO(
    rclcpp::get_logger("ros2_control_ResourceManager"),
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
    ret = system->read_joints(joint_components_for_hardware_[system->get_name()]);
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
    ret = system->read_joints(joint_components_for_hardware_[system->get_name()]);
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
    ret = system->write_joints(joint_components_for_hardware_[system->get_name()]);
    if (ret != return_type::OK) {
      return ret;
    }
    // TODO(anyone): add support to read sensors of a system is they exist
  }
  // TODO(anyone): add sensors and actuators
  return ret;
}

return_type ResourceManager::check_command_interfaces(
  const std::string & joint_name, const std::vector<std::string> & interfaces) const
{
  // Check joint existance
  if (command_interfaces_.find(joint_name) == command_interfaces_.end()) {
    // TODO(all): Do we need to return dedicated code?
    RCLCPP_ERROR(
      rclcpp::get_logger("ros2_control_ResourceManager"),
      "There is no command interface for " + joint_name);
    return return_type::INTERFACE_NOT_FOUND;
  }

  // Check interface existance
  for (const auto & interface : interfaces) {
    if (std::find(
        command_interfaces_.at(joint_name).cbegin(),
        command_interfaces_.at(joint_name).cend(),
        interface) == command_interfaces_.at(joint_name).cend())
    {
      RCLCPP_ERROR(
        rclcpp::get_logger("ros2_control_ResourceManager"),
        "There is no command interface '" + interface + "' found for " + joint_name);
      return return_type::INTERFACE_NOT_PROVIDED;
    }
  }

  return return_type::OK;
}

return_type ResourceManager::check_state_interfaces()
{
  // TODO(anyone): the same logic as for command interfaces
  return return_type::ERROR;
}

return_type ResourceManager::claim_command_handle(
  const std::string & joint_name, const std::vector<std::string> & interfaces,
  std::shared_ptr<hardware_interface::components::Joint> & command_handle)
{
  // Check joint existance
  if (joint_components_.find(joint_name) == joint_components_.end()) {
    // TODO(all): Do we need to return dedicated code?
    RCLCPP_ERROR(
      rclcpp::get_logger("ros2_control_ResourceManager"),
      "There is no command handle interface for " + joint_name);
    return return_type::INTERFACE_NOT_FOUND;
  }

  // check for each interface if already claimed
  for (const auto & interface : interfaces) {
    if (std::find(
        claimed_command_interfaces_.at(joint_name).cbegin(),
        claimed_command_interfaces_.at(joint_name).cend(),
        interface) != claimed_command_interfaces_.at(joint_name).cend())
    {
      RCLCPP_ERROR(
        rclcpp::get_logger("ros2_control_ResourceManager"),
        "The interface '" + interface + "' for " + joint_name + " is already claimed");
      return return_type::ALREADY_CLAIMED;
    }
  }

  command_handle = joint_components_[joint_name];
  // TODO(anyone) this could be done with `insert`...
  for (const auto & interface : interfaces) {
    claimed_command_interfaces_[joint_name].push_back(interface);
  }
  return return_type::OK;
}


}  // namespace resource_manager
