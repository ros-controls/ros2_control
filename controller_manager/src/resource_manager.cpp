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
#include <unordered_map>
#include <utility>

#include "hardware_interface/actuator_hardware.hpp"
#include "hardware_interface/components/component_info.hpp"
#include "hardware_interface/components/joint.hpp"
#include "hardware_interface/components/sensor.hpp"
#include "hardware_interface/component_parser.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/sensor_hardware.hpp"
#include "hardware_interface/system_hardware.hpp"

#include "pluginlib/class_loader.hpp"

#include "./resource_manager.hpp"

namespace controller_manager
{

using ComponentDeleter = hardware_interface::components::Deleter;

class ResourceStorage
{
  static constexpr auto pkg_name = "hardware_interface";

  static constexpr auto joint_component_interface_name =
    "hardware_interface::components::JointInterface";
  static constexpr auto sensor_component_interface_name =
    "hardware_interface::components::SensorInterface";

  static constexpr auto actuator_interface_name =
    "hardware_interface::ActuatorHardwareInterface";
  static constexpr auto sensor_interface_name =
    "hardware_interface::SensorHardwareInterface";
  static constexpr auto system_interface_name =
    "hardware_interface::SystemHardwareInterface";

public:
  ResourceStorage()
  : joint_component_loader_(pkg_name, joint_component_interface_name),
    sensor_component_loader_(pkg_name, sensor_component_interface_name),
    actuator_loader_(pkg_name, actuator_interface_name),
    sensor_loader_(pkg_name, sensor_interface_name),
    system_loader_(pkg_name, system_interface_name)
  {}

  ~ResourceStorage() = default;

  template<class ComponentT, class ComponentInterfaceT>
  void initialize_component(
    const hardware_interface::components::ComponentInfo & component_info,
    pluginlib::ClassLoader<ComponentInterfaceT> & loader,
    std::unordered_map<std::string, ComponentT> & container,
    ComponentDeleter deleter)
  {
    // hardware_class_type has to match class name in plugin xml description
    // TODO(karsten1987) extract package from hardware_class_type
    // e.g.: <package_vendor>/<system_type>
    // TODO(dstoegl) why is it class_type here and "hardware_class_type" in hardware_info?
    auto interface = loader.createSharedInstance(component_info.class_type);
    container.emplace(std::make_pair(component_info.name, ComponentT(interface, deleter)));
    container.at(component_info.name).configure(component_info);
  }

  void initialize_joint_component(
    const hardware_interface::components::ComponentInfo & component_info,
    ComponentDeleter deleter)
  {
    initialize_component<
      hardware_interface::components::Joint,
      hardware_interface::components::JointInterface>(
      component_info, joint_component_loader_, joint_components_, deleter);
  }

  void initialize_sensor_component(
    const hardware_interface::components::ComponentInfo & component_info,
    ComponentDeleter deleter)
  {
    initialize_component<
      hardware_interface::components::Sensor,
      hardware_interface::components::SensorInterface>(
      component_info, sensor_component_loader_, sensor_components_, deleter);
  }

  template<class HardwareT, class HardwareInterfaceT>
  void initialize_hardware(
    const hardware_interface::HardwareInfo & hardware_info,
    pluginlib::ClassLoader<HardwareInterfaceT> & loader,
    std::unordered_map<std::string, HardwareT> & container)
  {
    // hardware_class_type has to match class name in plugin xml description
    // TODO(karsten1987) extract package from hardware_class_type
    // e.g.: <package_vendor>/<system_type>
    auto interface = std::unique_ptr<HardwareInterfaceT>(
      loader.createUnmanagedInstance(hardware_info.hardware_class_type));
    container.emplace(std::make_pair(hardware_info.name, HardwareT(std::move(interface))));
    container.at(hardware_info.name).configure(hardware_info);
  }

  void initialize_actuator(const hardware_interface::HardwareInfo & hardware_info)
  {
    initialize_hardware<
      hardware_interface::ActuatorHardware,
      hardware_interface::ActuatorHardwareInterface>(
      hardware_info, actuator_loader_, actuators_);
  }

  void initialize_sensor(const hardware_interface::HardwareInfo & hardware_info)
  {
    initialize_hardware<
      hardware_interface::SensorHardware,
      hardware_interface::SensorHardwareInterface>(
      hardware_info, sensor_loader_, sensors_);
  }

  void initialize_system(const hardware_interface::HardwareInfo & hardware_info)
  {
    initialize_hardware<
      hardware_interface::SystemHardware,
      hardware_interface::SystemHardwareInterface>(
      hardware_info, system_loader_, systems_);
  }

  // components plugins
  pluginlib::ClassLoader<hardware_interface::components::JointInterface> joint_component_loader_;
  pluginlib::ClassLoader<hardware_interface::components::SensorInterface> sensor_component_loader_;

  std::unordered_map<std::string, hardware_interface::components::Joint> joint_components_;
  std::unordered_map<std::string, hardware_interface::components::Sensor> sensor_components_;

  // hardware plugins
  pluginlib::ClassLoader<hardware_interface::ActuatorHardwareInterface> actuator_loader_;
  pluginlib::ClassLoader<hardware_interface::SensorHardwareInterface> sensor_loader_;
  pluginlib::ClassLoader<hardware_interface::SystemHardwareInterface> system_loader_;

  std::unordered_map<std::string, hardware_interface::ActuatorHardware> actuators_;
  std::unordered_map<std::string, hardware_interface::SensorHardware> sensors_;
  std::unordered_map<std::string, hardware_interface::SystemHardware> systems_;
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
      claimed_resource_map_.insert({joint.name, false});
      auto deleter = std::bind(&ResourceManager::release_component, this, joint.name);
      resource_storage_->initialize_joint_component(joint, deleter);
      // TODO(karsten1987) Verify that parser warns when sensor and joint
      // component are called the same.
    }

    for (const auto & sensor : hardware.sensors) {
      claimed_resource_map_.insert({sensor.name, false});
      auto deleter = std::bind(&ResourceManager::release_component, this, sensor.name);
      resource_storage_->initialize_sensor_component(sensor, deleter);
    }
  }
}

void ResourceManager::release_component(const std::string & component_id)
{
  std::lock_guard<decltype(resource_lock_)> lg(resource_lock_);
  claimed_resource_map_[component_id] = false;
}

size_t ResourceManager::joint_components_size() const
{
  return resource_storage_->joint_components_.size();
}

std::vector<std::string> ResourceManager::joint_components_name() const
{
  std::vector<std::string> ret;
  ret.reserve(resource_storage_->joint_components_.size());
  for (auto const & component : resource_storage_->joint_components_) {
    ret.emplace_back(std::get<0>(component));
  }

  return ret;
}

size_t ResourceManager::sensor_components_size() const
{
  return resource_storage_->sensor_components_.size();
}

std::vector<std::string> ResourceManager::sensor_components_name() const
{
  std::vector<std::string> ret;
  ret.reserve(resource_storage_->sensor_components_.size());
  for (const auto & component : resource_storage_->sensor_components_) {
    ret.emplace_back(std::get<0>(component));
  }

  return ret;
}

size_t ResourceManager::actuator_interfaces_size() const
{
  return resource_storage_->actuators_.size();
}

std::vector<std::string> ResourceManager::actuator_interfaces_name() const
{
  std::vector<std::string> ret;
  ret.reserve(resource_storage_->actuators_.size());
  for (const auto & hardware : resource_storage_->actuators_) {
    ret.emplace_back(std::get<0>(hardware));
  }

  return ret;
}

size_t ResourceManager::sensor_interfaces_size() const
{
  return resource_storage_->sensors_.size();
}

std::vector<std::string> ResourceManager::sensor_interfaces_name() const
{
  std::vector<std::string> ret;
  ret.reserve(resource_storage_->sensors_.size());
  for (const auto & hardware : resource_storage_->sensors_) {
    ret.emplace_back(std::get<0>(hardware));
  }

  return ret;
}

size_t ResourceManager::system_interfaces_size() const
{
  return resource_storage_->systems_.size();
}

std::vector<std::string> ResourceManager::system_interfaces_name() const
{
  std::vector<std::string> ret;
  ret.reserve(resource_storage_->systems_.size());
  for (const auto & hardware : resource_storage_->systems_) {
    ret.emplace_back(std::get<0>(hardware));
  }

  return ret;
}

hardware_interface::components::Sensor
ResourceManager::claim_sensor(const std::string & sensor_id)
{
  std::lock_guard<decltype(resource_lock_)> lg(resource_lock_);
  if (!sensor_exists(sensor_id)) {
    throw std::runtime_error(std::string("sensor id") + sensor_id + " doesn't exist");
  }
  if (sensor_is_claimed(sensor_id)) {
    throw std::runtime_error(std::string("sensor id") + sensor_id + " is already claimed");
  }

  claimed_resource_map_[sensor_id] = true;
  return resource_storage_->sensor_components_[sensor_id];
}

bool ResourceManager::sensor_exists(const std::string & sensor_id) const
{
  return resource_storage_->sensor_components_.find(sensor_id) !=
         resource_storage_->sensor_components_.end();
}

bool ResourceManager::sensor_is_claimed(const std::string & sensor_id) const
{
  std::lock_guard<decltype(resource_lock_)> lg(resource_lock_);
  if (!sensor_exists(sensor_id)) {return false;}

  return claimed_resource_map_.at(sensor_id);
}
}  // namespace controller_manager
