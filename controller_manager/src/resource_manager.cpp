// Copyright 2020 Open Source Robotics Foundation, Inc.
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

#include <memory>
#include <string>
#include <vector>
#include <utility>

#include "hardware_interface/components/component_info.hpp"
#include "hardware_interface/component_parser.hpp"
#include "hardware_interface/hardware_resources/actuator_hardware.hpp"
#include "hardware_interface/hardware_resources/hardware_info.hpp"
#include "hardware_interface/hardware_resources/sensor_hardware.hpp"
#include "hardware_interface/hardware_resources/system_hardware.hpp"

#include "pluginlib/class_loader.hpp"

#include "./resource_manager.hpp"

namespace controller_manager
{

class ResourceStorage
{
  static constexpr const char * pkg_name = "hardware_interface";

  static constexpr const char * joint_component_interface_name =
    "hardware_interface::components::Joint";
  static constexpr const char * sensor_component_interface_name =
    "hardware_interface::components::Sensor";

  static constexpr const char * actuator_interface_name =
    "hardware_interface::hardware_resources::ActuatorHardwareInterface";
  static constexpr const char * sensor_interface_name =
    "hardware_interface::hardware_resources::SensorHardwareInterface";
  static constexpr const char * system_interface_name =
    "hardware_interface::hardware_resources::SystemHardwareInterface";

public:
  ResourceStorage()
  : joint_component_loader_(pkg_name, joint_component_interface_name),
    sensor_component_loader_(pkg_name, sensor_component_interface_name),
    actuator_loader_(pkg_name, actuator_interface_name),
    sensor_loader_(pkg_name, sensor_interface_name),
    system_loader_(pkg_name, system_interface_name)
  {}

  ~ResourceStorage() = default;

  void initialize_joint_component(
    const hardware_interface::components::ComponentInfo & component_info)
  {
    joint_components_.emplace_back(
      std::unique_ptr<hardware_interface::components::Joint>(
        joint_component_loader_.createUnmanagedInstance(component_info.class_type)));
    joint_components_.back()->configure(component_info);
  }

  void initialize_sensor_component(
    const hardware_interface::components::ComponentInfo & component_info)
  {
    sensor_components_.emplace_back(
      std::unique_ptr<hardware_interface::components::Sensor>(
        sensor_component_loader_.createUnmanagedInstance(component_info.class_type)));
    sensor_components_.back()->configure(component_info);
  }

  template<class HardwareT, class HardwareInterfaceT>
  void initialize_hardware(
    const hardware_interface::hardware_resources::HardwareInfo & hardware_info,
    pluginlib::ClassLoader<HardwareInterfaceT> & loader,
    std::vector<HardwareT> & container)
  {
    // hardware_class_type has to match class name in plugin xml description
    // TODO(karsten1987) extract package from hardware_class_type
    // e.g.: <package_vendor>/<system_type>
    auto interface = std::unique_ptr<HardwareInterfaceT>(
      loader.createUnmanagedInstance(hardware_info.hardware_class_type));
    HardwareT actuator(std::move(interface));
    container.emplace_back(std::move(actuator));
    container.back().configure(hardware_info);
  }

  void initialize_actuator(
    const hardware_interface::hardware_resources::HardwareInfo & hardware_info)
  {
    initialize_hardware<hardware_interface::hardware_resources::ActuatorHardware,
      hardware_interface::hardware_resources::ActuatorHardwareInterface>(
      hardware_info, actuator_loader_, actuators_);
  }

  void initialize_sensor(const hardware_interface::hardware_resources::HardwareInfo & hardware_info)
  {
    initialize_hardware<hardware_interface::hardware_resources::SensorHardware,
      hardware_interface::hardware_resources::SensorHardwareInterface>(
      hardware_info, sensor_loader_, sensors_);
  }

  void initialize_system(const hardware_interface::hardware_resources::HardwareInfo & hardware_info)
  {
    initialize_hardware<hardware_interface::hardware_resources::SystemHardware,
      hardware_interface::hardware_resources::SystemHardwareInterface>(
      hardware_info, system_loader_, systems_);
  }

  // components plugins
  pluginlib::ClassLoader<hardware_interface::components::Joint> joint_component_loader_;
  pluginlib::ClassLoader<hardware_interface::components::Sensor> sensor_component_loader_;

  std::vector<std::unique_ptr<hardware_interface::components::Joint>> joint_components_;
  std::vector<std::unique_ptr<hardware_interface::components::Sensor>> sensor_components_;

  // hardware plugins
  pluginlib::ClassLoader<hardware_interface::hardware_resources::ActuatorHardwareInterface>
  actuator_loader_;
  pluginlib::ClassLoader<hardware_interface::hardware_resources::SensorHardwareInterface>
  sensor_loader_;
  pluginlib::ClassLoader<hardware_interface::hardware_resources::SystemHardwareInterface>
  system_loader_;

  std::vector<hardware_interface::hardware_resources::ActuatorHardware> actuators_;
  std::vector<hardware_interface::hardware_resources::SensorHardware> sensors_;
  std::vector<hardware_interface::hardware_resources::SystemHardware> systems_;
};

ResourceManager::ResourceManager()
: resource_storage_(std::make_unique<ResourceStorage>())
{}

ResourceManager::~ResourceManager() = default;

ResourceManager::ResourceManager(const std::string & urdf)
: resource_storage_(std::make_unique<ResourceStorage>())
{
  const std::string system_type = "system";
  const std::string sensor_type = "sensor";
  const std::string actuator_type = "actuator";

  auto hardware_info = hardware_interface::parse_control_resources_from_urdf(urdf);

  for (const auto & hardware : hardware_info) {
    if (hardware.type == actuator_type) {
      resource_storage_->initialize_actuator(hardware);
    }
    if (hardware.type == sensor_type) {
      resource_storage_->initialize_sensor(hardware);
    }
    if (hardware.type == system_type) {
      resource_storage_->initialize_system(hardware);
    }

    for (const auto & joint : hardware.joints) {
      resource_storage_->initialize_joint_component(joint);
    }

    for (const auto & sensor : hardware.sensors) {
      resource_storage_->initialize_sensor_component(sensor);
    }
  }
}

size_t ResourceManager::joint_components_size() const
{
  return resource_storage_->joint_components_.size();
}

size_t ResourceManager::sensor_components_size() const
{
  return resource_storage_->sensor_components_.size();
}

size_t ResourceManager::actuator_interfaces_size() const
{
  return resource_storage_->actuators_.size();
}

size_t ResourceManager::sensor_interfaces_size() const
{
  return resource_storage_->sensors_.size();
}

size_t ResourceManager::system_interfaces_size() const
{
  return resource_storage_->systems_.size();
}
}  // namespace controller_manager
