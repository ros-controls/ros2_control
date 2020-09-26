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
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/actuator_hardware.hpp"
#include "hardware_interface/sensor_hardware.hpp"
#include "hardware_interface/system_hardware.hpp"

#include "pluginlib/class_loader.hpp"

#include "./resource_manager.hpp"

namespace controller_manager
{

class ResourceStorage
{
  static constexpr const char * pkg_name = "hardware_interface";
  static constexpr const char * actuator_interface_name =
    "hardware_interface::ActuatorHardwareInterface";
  static constexpr const char * sensor_interface_name =
    "hardware_interface::SensorHardwareInterface";
  static constexpr const char * system_interface_name =
    "hardware_interface::SystemHardwareInterface";

public:
  ResourceStorage()
  : actuator_loader_(pkg_name, actuator_interface_name),
    sensor_loader_(pkg_name, sensor_interface_name),
    system_loader_(pkg_name, system_interface_name)
  {}

  ~ResourceStorage() = default;

  template<class HardwareT, class HardwareInterfaceT>
  void initialize_hardware(
    const hardware_interface::HardwareInfo & hardware_info,
    pluginlib::ClassLoader<HardwareInterfaceT> & loader,
    std::vector<HardwareT> & container)
  {
    // hardware_class_type has to match class name in plugin xml description
    auto interface = std::unique_ptr<HardwareInterfaceT>(
      loader.createUnmanagedInstance(hardware_info.hardware_class_type));
    HardwareT actuator(std::move(interface));
    container.emplace_back(std::move(actuator));
    container.back().configure(hardware_info);
  }

  void initialize_actuator(const hardware_interface::HardwareInfo & hardware_info)
  {
    using HardwareT = hardware_interface::ActuatorHardware;
    using HardwareInterfaceT = hardware_interface::ActuatorHardwareInterface;
    initialize_hardware<HardwareT, HardwareInterfaceT>(
      hardware_info, actuator_loader_, actuators_);
  }

  void initialize_sensor(const hardware_interface::HardwareInfo & hardware_info)
  {
    using HardwareT = hardware_interface::SensorHardware;
    using HardwareInterfaceT = hardware_interface::SensorHardwareInterface;
    initialize_hardware<HardwareT, HardwareInterfaceT>(
      hardware_info, sensor_loader_, sensors_);
  }

  void initialize_system(const hardware_interface::HardwareInfo & hardware_info)
  {
    using HardwareT = hardware_interface::SystemHardware;
    using HardwareInterfaceT = hardware_interface::SystemHardwareInterface;
    initialize_hardware<HardwareT, HardwareInterfaceT>(
      hardware_info, system_loader_, systems_);
  }

  pluginlib::ClassLoader<hardware_interface::ActuatorHardwareInterface> actuator_loader_;
  pluginlib::ClassLoader<hardware_interface::SensorHardwareInterface> sensor_loader_;
  pluginlib::ClassLoader<hardware_interface::SystemHardwareInterface> system_loader_;

  std::vector<hardware_interface::ActuatorHardware> actuators_;
  std::vector<hardware_interface::SensorHardware> sensors_;
  std::vector<hardware_interface::SystemHardware> systems_;
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
  }
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
