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

#include <iostream>
#include <memory>
#include <string>
#include <vector>
#include <unordered_map>
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
  }

  template<class HardwareT>
  void import_state_interfaces(HardwareT & hardware)
  {
    auto interfaces = hardware.export_state_interfaces();
    for (auto i = 0u; i < interfaces.size(); ++i) {
      auto key = interfaces[i].get_name() + "/" + interfaces[i].get_interface_name();
      state_interface_map_.emplace(
        std::make_pair(key, std::move(interfaces[i])));
    }
  }

  template<class HardwareT>
  void import_command_interfaces(HardwareT & hardware)
  {
    auto interfaces = hardware.export_command_interfaces();
    for (auto i = 0u; i < interfaces.size(); ++i) {
      auto key = interfaces[i].get_name() + "/" + interfaces[i].get_interface_name();
      command_interface_map_.emplace(
        std::make_pair(key, std::move(interfaces[i])));
    }
  }

  void initialize_actuator(const hardware_interface::HardwareInfo & hardware_info)
  {
    initialize_hardware<hardware_interface::components::Actuator,
      hardware_interface::components::ActuatorInterface>(
      hardware_info, actuator_loader_, actuators_);
    if (hardware_interface::return_type::OK != actuators_.back().configure(hardware_info)) {
      throw std::runtime_error(std::string("failed to configure ") + hardware_info.name);
    }
    import_state_interfaces(actuators_.back());
    import_command_interfaces(actuators_.back());
  }

  void initialize_sensor(const hardware_interface::HardwareInfo & hardware_info)
  {
    initialize_hardware<hardware_interface::components::Sensor,
      hardware_interface::components::SensorInterface>(
      hardware_info, sensor_loader_, sensors_);
    sensors_.back().configure(hardware_info);
    import_state_interfaces(sensors_.back());
  }

  void initialize_system(const hardware_interface::HardwareInfo & hardware_info)
  {
    initialize_hardware<hardware_interface::components::System,
      hardware_interface::components::SystemInterface>(
      hardware_info, system_loader_, systems_);
    systems_.back().configure(hardware_info);
    import_state_interfaces(systems_.back());
    import_command_interfaces(systems_.back());
  }

  // hardware plugins
  pluginlib::ClassLoader<hardware_interface::components::ActuatorInterface> actuator_loader_;
  pluginlib::ClassLoader<hardware_interface::components::SensorInterface> sensor_loader_;
  pluginlib::ClassLoader<hardware_interface::components::SystemInterface> system_loader_;

  std::vector<hardware_interface::components::Actuator> actuators_;
  std::vector<hardware_interface::components::Sensor> sensors_;
  std::vector<hardware_interface::components::System> systems_;

  std::unordered_map<std::string, hardware_interface::StateInterface> state_interface_map_;
  std::unordered_map<std::string, hardware_interface::CommandInterface> command_interface_map_;
};

ResourceManager::~ResourceManager() = default;

ResourceManager::ResourceManager(const std::string & urdf, bool validate_interfaces)
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

  auto validate_storage = [this, &hardware_info]() -> void
    {
      std::vector<std::string> missing_state_keys = {};
      std::vector<std::string> missing_command_keys = {};

      for (const auto & hardware : hardware_info) {
        for (const auto & joint : hardware.joints) {
          for (const auto & state_interface : joint.state_interfaces) {
            if (!state_interface_exists(joint.name + "/" + state_interface.name)) {
              missing_state_keys.emplace_back(joint.name + "/" + state_interface.name);
            }
          }
          for (const auto & command_interface : joint.command_interfaces) {
            if (!command_interface_exists(joint.name + "/" + command_interface.name)) {
              missing_state_keys.emplace_back(joint.name + "/" + command_interface.name);
            }
          }
        }
        for (const auto & sensor : hardware.sensors) {
          for (const auto & state_interface : sensor.state_interfaces) {
            if (!state_interface_exists(sensor.name + "/" + state_interface.name)) {
              missing_state_keys.emplace_back(sensor.name + "/" + state_interface.name);
            }
          }
        }
      }

      if (!missing_state_keys.empty() || !missing_command_keys.empty()) {
        std::string err_msg = "wrong state or command interface configuration.\n";
        err_msg += "missing state interfaces:\n";
        for (const auto & missing_key : missing_state_keys) {
          err_msg += missing_key + "\t";
        }
        err_msg += "\nmissing command interfaces:\n";
        for (const auto & missing_key : missing_command_keys) {
          err_msg += missing_key + "\t";
        }

        throw std::runtime_error(err_msg);
      }
    };

  // throw on missing state and command interfaces, not specified keys are being ignored
  if (validate_interfaces) {
    validate_storage();
  }

  claimed_command_interface_map_.reserve(resource_storage_->command_interface_map_.size());
  for (auto & command_interface_it : resource_storage_->command_interface_map_) {
    const auto & interface_name = std::get<0>(command_interface_it);
    claimed_command_interface_map_[interface_name] = false;
    //// set deleter to command handle
    //hardware_interface::CommandInterface::Deleter d = std::bind(
    //  &ResourceManager::release_command_interface, this, interface_name);
    //std::get<1>(command_interface_it).set_deleter(std::move(d));
  }
}

// make this a lambda?
void ResourceManager::release_command_interface(const std::string & key)
{
  std::lock_guard<decltype(resource_lock_)> lg(resource_lock_);
  std::cout << "releasing " << key << std::endl;
  claimed_command_interface_map_[key] = false;
}

std::vector<std::string> ResourceManager::state_interface_keys() const
{
  std::vector<std::string> keys;
  for (const auto & item : resource_storage_->state_interface_map_) {
    keys.push_back(std::get<0>(item));
  }
  return keys;
}

bool ResourceManager::state_interface_exists(const std::string & key) const
{
  return resource_storage_->state_interface_map_.find(key) !=
         resource_storage_->state_interface_map_.end();
}

bool ResourceManager::command_interface_is_claimed(const std::string & key) const
{
  if (!command_interface_exists(key)) {
    return false;
  }

  std::lock_guard<decltype(resource_lock_)> lg(resource_lock_);
  return claimed_command_interface_map_.at(key);
}

LoanedCommandInterface ResourceManager::claim_command_interface(const std::string & key)
{
  if (!command_interface_exists(key)) {
    throw std::runtime_error(
            std::string("command interface with key") + key + " does not exist");
  }

  if (command_interface_is_claimed(key)) {
    throw std::runtime_error(
            std::string("command interface with key") + key + " is already claimed");
  }

  std::lock_guard<decltype(resource_lock_)> lg(resource_lock_);
  claimed_command_interface_map_[key] = true;
  return LoanedCommandInterface(
    resource_storage_->command_interface_map_.at(key),
    std::bind(&ResourceManager::release_command_interface, this, key));
}

std::vector<std::string> ResourceManager::command_interface_keys() const
{
  std::vector<std::string> keys;
  for (const auto & item : resource_storage_->command_interface_map_) {
    keys.push_back(std::get<0>(item));
  }
  return keys;
}

bool ResourceManager::command_interface_exists(const std::string & key) const
{
  return resource_storage_->command_interface_map_.find(key) !=
         resource_storage_->command_interface_map_.end();
}

size_t ResourceManager::actuator_components_size() const
{
  return resource_storage_->actuators_.size();
}

size_t ResourceManager::sensor_components_size() const
{
  return resource_storage_->sensors_.size();
}

size_t ResourceManager::system_components_size() const
{
  return resource_storage_->systems_.size();
}
}  // namespace controller_manager
