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

#include "hardware_interface/components/actuator.hpp"
#include "hardware_interface/components/actuator_interface.hpp"
#include "hardware_interface/components/sensor.hpp"
#include "hardware_interface/components/sensor_interface.hpp"
#include "hardware_interface/components/system.hpp"
#include "hardware_interface/components/system_interface.hpp"
#include "hardware_interface/component_parser.hpp"
#include "hardware_interface/hardware_info.hpp"

#include "pluginlib/class_loader.hpp"

#include "./resource_manager.hpp"

namespace controller_manager
{

class ResourceStorage
{
  static constexpr const char * pkg_name = "hardware_interface";

  static constexpr const char * actuator_interface_name =
    "hardware_interface::components::ActuatorInterface";
  static constexpr const char * sensor_interface_name =
    "hardware_interface::components::SensorInterface";
  static constexpr const char * system_interface_name =
    "hardware_interface::components::SystemInterface";

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
    // TODO(karsten1987) extract package from hardware_class_type
    // e.g.: <package_vendor>/<system_type>
    auto interface = std::unique_ptr<HardwareInterfaceT>(
      loader.createUnmanagedInstance(hardware_info.hardware_class_type));
    HardwareT actuator(std::move(interface));
    container.emplace_back(std::move(actuator));
    container.back().configure(hardware_info);
  }

  void initialize_actuator(const hardware_interface::HardwareInfo & hardware_info)
  {
    initialize_hardware<hardware_interface::components::Actuator,
      hardware_interface::components::ActuatorInterface>(
      hardware_info, actuator_loader_, actuators_);
  }

  void initialize_sensor(const hardware_interface::HardwareInfo & hardware_info)
  {
    initialize_hardware<hardware_interface::components::Sensor,
      hardware_interface::components::SensorInterface>(
      hardware_info, sensor_loader_, sensors_);
  }

  void initialize_system(const hardware_interface::HardwareInfo & hardware_info)
  {
    initialize_hardware<hardware_interface::components::System,
      hardware_interface::components::SystemInterface>(
      hardware_info, system_loader_, systems_);
  }

  // hardware plugins
  pluginlib::ClassLoader<hardware_interface::components::ActuatorInterface> actuator_loader_;
  pluginlib::ClassLoader<hardware_interface::components::SensorInterface> sensor_loader_;
  pluginlib::ClassLoader<hardware_interface::components::SystemInterface> system_loader_;

  std::vector<hardware_interface::components::Actuator> actuators_;
  std::vector<hardware_interface::components::Sensor> sensors_;
  std::vector<hardware_interface::components::System> systems_;
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
