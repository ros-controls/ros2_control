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

#include "hardware_interface/resource_manager.hpp"

#include <map>
#include <memory>
#include <sstream>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "hardware_interface/actuator.hpp"
#include "hardware_interface/actuator_interface.hpp"
#include "hardware_interface/component_parser.hpp"
#include "hardware_interface/sensor.hpp"
#include "hardware_interface/sensor_interface.hpp"
#include "hardware_interface/system.hpp"
#include "hardware_interface/system_interface.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "pluginlib/class_loader.hpp"
#include "rcutils/logging_macros.h"

namespace hardware_interface
{
class ResourceStorage
{
  static constexpr const char * pkg_name = "hardware_interface";

  static constexpr const char * actuator_interface_name = "hardware_interface::ActuatorInterface";
  static constexpr const char * sensor_interface_name = "hardware_interface::SensorInterface";
  static constexpr const char * system_interface_name = "hardware_interface::SystemInterface";

public:
  ResourceStorage()
  : actuator_loader_(pkg_name, actuator_interface_name),
    sensor_loader_(pkg_name, sensor_interface_name),
    system_loader_(pkg_name, system_interface_name)
  {
  }

  // A new generic method to do initialization checks
  template <class HardwareT>
  void check_hardware_initialization_and_configure(
    HardwareT & hardware, const HardwareInfo & hardware_info)
  {
    if (
      hardware.initialize(hardware_info).id() !=
      lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED)
    {
      throw std::runtime_error(std::string("Failed to initialize '") + hardware_info.name + "'");
    }
    if (hardware.configure().id() != lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE)
    {
      throw std::runtime_error(std::string("Failed to configure '") + hardware_info.name + "'");
    }
  }

  template <class HardwareT, class HardwareInterfaceT>
  void initialize_hardware(
    const HardwareInfo & hardware_info, pluginlib::ClassLoader<HardwareInterfaceT> & loader,
    std::vector<HardwareT> & container)
  {
    // hardware_class_type has to match class name in plugin xml description
    // TODO(karsten1987) extract package from hardware_class_type
    // e.g.: <package_vendor>/<system_type>
    auto interface = std::unique_ptr<HardwareInterfaceT>(
      loader.createUnmanagedInstance(hardware_info.hardware_class_type));
    HardwareT hardware(std::move(interface));
    container.emplace_back(std::move(hardware));
    hardware_status_map_.emplace(
      std::make_pair(container.back().get_name(), container.back().get_state()));
  }

  template <class HardwareT>
  void import_state_interfaces(HardwareT & hardware)
  {
    auto interfaces = hardware.export_state_interfaces();
    for (auto i = 0u; i < interfaces.size(); ++i)
    {
      auto key = interfaces[i].get_name() + "/" + interfaces[i].get_interface_name();
      state_interface_map_.emplace(std::make_pair(key, std::move(interfaces[i])));
    }
  }

  template <class HardwareT>
  void import_command_interfaces(
    HardwareT & hardware, std::unordered_map<std::string, bool> & claimed_command_interface_map)
  {
    auto interfaces = hardware.export_command_interfaces();
    for (auto i = 0u; i < interfaces.size(); ++i)
    {
      auto key = interfaces[i].get_name() + "/" + interfaces[i].get_interface_name();
      command_interface_map_.emplace(std::make_pair(key, std::move(interfaces[i])));
      claimed_command_interface_map.emplace(std::make_pair(key, false));
    }
  }

  void initialize_actuator(
    const HardwareInfo & hardware_info,
    std::unordered_map<std::string, bool> & claimed_command_interface_map)
  {
    initialize_hardware<Actuator, ActuatorInterface>(hardware_info, actuator_loader_, actuators_);
    check_hardware_initialization_and_configure(actuators_.back(), hardware_info);
    import_state_interfaces(actuators_.back());
    import_command_interfaces(actuators_.back(), claimed_command_interface_map);
  }

  void initialize_sensor(const HardwareInfo & hardware_info)
  {
    initialize_hardware<Sensor, SensorInterface>(hardware_info, sensor_loader_, sensors_);
    check_hardware_initialization_and_configure(sensors_.back(), hardware_info);
    import_state_interfaces(sensors_.back());
  }

  void initialize_system(
    const HardwareInfo & hardware_info,
    std::unordered_map<std::string, bool> & claimed_command_interface_map)
  {
    initialize_hardware<System, SystemInterface>(hardware_info, system_loader_, systems_);
    check_hardware_initialization_and_configure(systems_.back(), hardware_info);
    import_state_interfaces(systems_.back());
    import_command_interfaces(systems_.back(), claimed_command_interface_map);
  }

  void initialize_actuator(
    std::unique_ptr<ActuatorInterface> actuator, const HardwareInfo & hardware_info,
    std::unordered_map<std::string, bool> & claimed_command_interface_map)
  {
    this->actuators_.emplace_back(Actuator(std::move(actuator)));
    check_hardware_initialization_and_configure(actuators_.back(), hardware_info);
    import_state_interfaces(actuators_.back());
    import_command_interfaces(actuators_.back(), claimed_command_interface_map);
  }

  void initialize_sensor(
    std::unique_ptr<SensorInterface> sensor, const HardwareInfo & hardware_info)
  {
    this->sensors_.emplace_back(Sensor(std::move(sensor)));
    check_hardware_initialization_and_configure(sensors_.back(), hardware_info);
    import_state_interfaces(sensors_.back());
  }

  void initialize_system(
    std::unique_ptr<SystemInterface> system, const HardwareInfo & hardware_info,
    std::unordered_map<std::string, bool> & claimed_command_interface_map)
  {
    this->systems_.emplace_back(System(std::move(system)));
    check_hardware_initialization_and_configure(systems_.back(), hardware_info);
    import_state_interfaces(systems_.back());
    import_command_interfaces(systems_.back(), claimed_command_interface_map);
  }

  // hardware plugins
  pluginlib::ClassLoader<ActuatorInterface> actuator_loader_;
  pluginlib::ClassLoader<SensorInterface> sensor_loader_;
  pluginlib::ClassLoader<SystemInterface> system_loader_;

  std::vector<Actuator> actuators_;
  std::vector<Sensor> sensors_;
  std::vector<System> systems_;

  std::unordered_map<std::string, rclcpp_lifecycle::State> hardware_status_map_;

  std::map<std::string, StateInterface> state_interface_map_;
  std::map<std::string, CommandInterface> command_interface_map_;
};

ResourceManager::ResourceManager() : resource_storage_(std::make_unique<ResourceStorage>()) {}

ResourceManager::~ResourceManager() = default;

ResourceManager::ResourceManager(const std::string & urdf, bool validate_interfaces)
: resource_storage_(std::make_unique<ResourceStorage>())
{
  load_urdf(urdf, validate_interfaces);
}

void ResourceManager::load_urdf(const std::string & urdf, bool validate_interfaces)
{
  const std::string system_type = "system";
  const std::string sensor_type = "sensor";
  const std::string actuator_type = "actuator";

  const auto hardware_info = hardware_interface::parse_control_resources_from_urdf(urdf);
  for (const auto & hardware : hardware_info)
  {
    if (hardware.type == actuator_type)
    {
      resource_storage_->initialize_actuator(hardware, claimed_command_interface_map_);
    }
    if (hardware.type == sensor_type)
    {
      resource_storage_->initialize_sensor(hardware);
    }
    if (hardware.type == system_type)
    {
      resource_storage_->initialize_system(hardware, claimed_command_interface_map_);
    }
  }

  // throw on missing state and command interfaces, not specified keys are being ignored
  if (validate_interfaces)
  {
    validate_storage(hardware_info);
  }
}

void ResourceManager::release_command_interface(const std::string & key)
{
  std::lock_guard<decltype(resource_lock_)> lg(resource_lock_);
  claimed_command_interface_map_[key] = false;
}

LoanedStateInterface ResourceManager::claim_state_interface(const std::string & key)
{
  if (!state_interface_exists(key))
  {
    throw std::runtime_error(std::string("State interface with key '") + key + "' does not exist");
  }

  return LoanedStateInterface(resource_storage_->state_interface_map_.at(key));
}

std::vector<std::string> ResourceManager::state_interface_keys() const
{
  std::vector<std::string> keys;
  for (const auto & item : resource_storage_->state_interface_map_)
  {
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
  if (!command_interface_exists(key))
  {
    return false;
  }

  std::lock_guard<decltype(resource_lock_)> lg(resource_lock_);
  return claimed_command_interface_map_.at(key);
}

LoanedCommandInterface ResourceManager::claim_command_interface(const std::string & key)
{
  if (!command_interface_exists(key))
  {
    throw std::runtime_error(std::string("Command interface with '") + key + "' does not exist");
  }

  std::lock_guard<decltype(resource_lock_)> lg(resource_lock_);
  if (command_interface_is_claimed(key))
  {
    throw std::runtime_error(
      std::string("Command interface with '") + key + "' is already claimed");
  }

  claimed_command_interface_map_[key] = true;
  return LoanedCommandInterface(
    resource_storage_->command_interface_map_.at(key),
    std::bind(&ResourceManager::release_command_interface, this, key));
}

std::vector<std::string> ResourceManager::command_interface_keys() const
{
  std::vector<std::string> keys;
  for (const auto & item : resource_storage_->command_interface_map_)
  {
    keys.push_back(std::get<0>(item));
  }
  return keys;
}

bool ResourceManager::command_interface_exists(const std::string & key) const
{
  return resource_storage_->command_interface_map_.find(key) !=
         resource_storage_->command_interface_map_.end();
}

void ResourceManager::import_component(std::unique_ptr<ActuatorInterface> actuator)
{
  resource_storage_->actuators_.emplace_back(Actuator(std::move(actuator)));
  resource_storage_->import_state_interfaces(resource_storage_->actuators_.back());
  resource_storage_->import_command_interfaces(
    resource_storage_->actuators_.back(), claimed_command_interface_map_);
}

size_t ResourceManager::actuator_components_size() const
{
  return resource_storage_->actuators_.size();
}

void ResourceManager::import_component(std::unique_ptr<SensorInterface> sensor)
{
  resource_storage_->sensors_.emplace_back(Sensor(std::move(sensor)));
  resource_storage_->import_state_interfaces(resource_storage_->sensors_.back());
}

size_t ResourceManager::sensor_components_size() const
{
  return resource_storage_->sensors_.size();
}

void ResourceManager::import_component(std::unique_ptr<SystemInterface> system)
{
  resource_storage_->systems_.emplace_back(System(std::move(system)));
  resource_storage_->import_state_interfaces(resource_storage_->systems_.back());
  resource_storage_->import_command_interfaces(
    resource_storage_->systems_.back(), claimed_command_interface_map_);
}

void ResourceManager::import_component(
  std::unique_ptr<ActuatorInterface> actuator, const HardwareInfo & hardware_info)
{
  resource_storage_->initialize_actuator(
    std::move(actuator), hardware_info, claimed_command_interface_map_);
}

void ResourceManager::import_component(
  std::unique_ptr<SensorInterface> sensor, const HardwareInfo & hardware_info)
{
  resource_storage_->initialize_sensor(std::move(sensor), hardware_info);
}

void ResourceManager::import_component(
  std::unique_ptr<SystemInterface> system, const HardwareInfo & hardware_info)
{
  resource_storage_->initialize_system(
    std::move(system), hardware_info, claimed_command_interface_map_);
}

size_t ResourceManager::system_components_size() const
{
  return resource_storage_->systems_.size();
}

std::unordered_map<std::string, rclcpp_lifecycle::State> ResourceManager::get_components_states()
{
  for (auto & component : resource_storage_->actuators_)
  {
    resource_storage_->hardware_status_map_[component.get_name()] = component.get_state();
  }
  for (auto & component : resource_storage_->sensors_)
  {
    resource_storage_->hardware_status_map_[component.get_name()] = component.get_state();
  }
  for (auto & component : resource_storage_->systems_)
  {
    resource_storage_->hardware_status_map_[component.get_name()] = component.get_state();
  }

  return resource_storage_->hardware_status_map_;
}

std::string interfaces_to_string(
  const std::vector<std::string> & start_interfaces,
  const std::vector<std::string> & stop_interfaces)
{
  std::stringstream ss;
  ss << "Start interfaces: " << std::endl << "[" << std::endl;
  for (const auto & start_if : start_interfaces)
  {
    ss << "  " << start_if << std::endl;
  }
  ss << "]" << std::endl;
  ss << "Stop interfaces: " << std::endl << "[" << std::endl;
  for (const auto & stop_if : stop_interfaces)
  {
    ss << "  " << stop_if << std::endl;
  }
  ss << "]" << std::endl;
  return ss.str();
}

bool ResourceManager::prepare_command_mode_switch(
  const std::vector<std::string> & start_interfaces,
  const std::vector<std::string> & stop_interfaces)
{
  for (auto & component : resource_storage_->actuators_)
  {
    if (return_type::OK != component.prepare_command_mode_switch(start_interfaces, stop_interfaces))
    {
      RCUTILS_LOG_ERROR_NAMED(
        "resource_manager", "Component '%s' did not accept new command resource combination: \n %s",
        component.get_name().c_str(),
        interfaces_to_string(start_interfaces, stop_interfaces).c_str());
      return false;
    }
  }
  for (auto & component : resource_storage_->systems_)
  {
    if (return_type::OK != component.prepare_command_mode_switch(start_interfaces, stop_interfaces))
    {
      RCUTILS_LOG_ERROR_NAMED(
        "resource_manager", "Component '%s' did not accept new command resource combination: \n %s",
        component.get_name().c_str(),
        interfaces_to_string(start_interfaces, stop_interfaces).c_str());
      return false;
    }
  }
  return true;
}

bool ResourceManager::perform_command_mode_switch(
  const std::vector<std::string> & start_interfaces,
  const std::vector<std::string> & stop_interfaces)
{
  for (auto & component : resource_storage_->actuators_)
  {
    if (return_type::OK != component.perform_command_mode_switch(start_interfaces, stop_interfaces))
    {
      RCUTILS_LOG_ERROR_NAMED(
        "resource_manager", "Component '%s' could not perform switch",
        component.get_name().c_str());
      return false;
    }
  }
  for (auto & component : resource_storage_->systems_)
  {
    if (return_type::OK != component.perform_command_mode_switch(start_interfaces, stop_interfaces))
    {
      RCUTILS_LOG_ERROR_NAMED(
        "resource_manager", "Component '%s' could not perform switch",
        component.get_name().c_str());
      return false;
    }
  }
  return true;
}

void ResourceManager::start_components()
{
  for (auto & component : resource_storage_->actuators_)
  {
    component.activate();
  }
  for (auto & component : resource_storage_->sensors_)
  {
    component.activate();
  }
  for (auto & component : resource_storage_->systems_)
  {
    component.activate();
  }
}

void ResourceManager::stop_components()
{
  for (auto & component : resource_storage_->actuators_)
  {
    component.deactivate();
  }
  for (auto & component : resource_storage_->sensors_)
  {
    component.deactivate();
  }
  for (auto & component : resource_storage_->systems_)
  {
    component.deactivate();
  }
}

void ResourceManager::read()
{
  for (auto & component : resource_storage_->actuators_)
  {
    component.read();
  }
  for (auto & component : resource_storage_->sensors_)
  {
    component.read();
  }
  for (auto & component : resource_storage_->systems_)
  {
    component.read();
  }
}

void ResourceManager::write()
{
  for (auto & component : resource_storage_->actuators_)
  {
    component.write();
  }
  for (auto & component : resource_storage_->systems_)
  {
    component.write();
  }
}

void ResourceManager::validate_storage(
  const std::vector<hardware_interface::HardwareInfo> & hardware_info) const
{
  std::vector<std::string> missing_state_keys = {};
  std::vector<std::string> missing_command_keys = {};

  for (const auto & hardware : hardware_info)
  {
    for (const auto & joint : hardware.joints)
    {
      for (const auto & state_interface : joint.state_interfaces)
      {
        if (!state_interface_exists(joint.name + "/" + state_interface.name))
        {
          missing_state_keys.emplace_back(joint.name + "/" + state_interface.name);
        }
      }
      for (const auto & command_interface : joint.command_interfaces)
      {
        if (!command_interface_exists(joint.name + "/" + command_interface.name))
        {
          missing_command_keys.emplace_back(joint.name + "/" + command_interface.name);
        }
      }
    }
    for (const auto & sensor : hardware.sensors)
    {
      for (const auto & state_interface : sensor.state_interfaces)
      {
        if (!state_interface_exists(sensor.name + "/" + state_interface.name))
        {
          missing_state_keys.emplace_back(sensor.name + "/" + state_interface.name);
        }
      }
    }
  }

  if (!missing_state_keys.empty() || !missing_command_keys.empty())
  {
    std::string err_msg = "Wrong state or command interface configuration.\n";
    err_msg += "missing state interfaces:\n";
    for (const auto & missing_key : missing_state_keys)
    {
      err_msg += "' " + missing_key + " '" + "\t";
    }
    err_msg += "\nmissing command interfaces:\n";
    for (const auto & missing_key : missing_command_keys)
    {
      err_msg += "' " + missing_key + " '" + "\t";
    }

    throw std::runtime_error(err_msg);
  }
}

}  // namespace hardware_interface
