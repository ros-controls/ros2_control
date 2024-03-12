// Copyright 2020 ros2_control Development Team
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

#ifndef HARDWARE_INTERFACE__HARDWARE_INFO_HPP_
#define HARDWARE_INTERFACE__HARDWARE_INFO_HPP_

#include <iostream>
#include <string>
#include <unordered_map>
#include <vector>

namespace hardware_interface
{

template <typename T>
std::ostream & operator<<(std::ostream & os, const std::vector<T> & input)
{
  os << "<vector>[";
  for (auto const & i : input)
  {
    os << i << ", ";
  }
  os << "]";
  return os;
}

template <typename K, typename V>
std::ostream & operator<<(std::ostream & os, const std::unordered_map<K, V> & input)
{
  os << "<map>[";
  for (auto const & [key, value] : input)
  {
    os << "{" << key << ", " << value << "}, ";
  }
  os << "]";
  return os;
}

/**
 * This structure stores information about components defined for a specific hardware
 * in robot's URDF.
 */
struct InterfaceInfo
{
  /**
   * Name of the command interfaces that can be set, e.g. "position", "velocity", etc.
   * Used by joints and GPIOs.
   */
  std::string name;
  /// (Optional) Minimal allowed values of the interface.
  std::string min;
  /// (Optional) Maximal allowed values of the interface.
  std::string max;
  /// (Optional) Initial value of the interface.
  std::string initial_value;
  /// (Optional) The datatype of the interface, e.g. "bool", "int". Used by GPIOs.
  std::string data_type;
  /// (Optional) If the handle is an array, the size of the array. Used by GPIOs.
  int size;

  friend std::ostream & operator<<(std::ostream & os, const InterfaceInfo & interface_info)
  {
    os << "<InterfaceInfo{{name:" << interface_info.name << "}, {min: " << interface_info.min
       << "}, {max: " << interface_info.max << "}, {initial_value: " << interface_info.initial_value
       << "}, {size: " << interface_info.size << "}}>";
    return os;
  }
};

/**
 * This structure stores information about an interface for a specific hardware which should be
 * instantiated internally.
 */
struct InterfaceDescription
{
  InterfaceDescription(const std::string & prefix_name, const InterfaceInfo & interface_info)
  : prefix_name(prefix_name), interface_info(interface_info)
  {
  }

  /**
   * Name of the interface defined by the user.
   */
  std::string prefix_name;

  InterfaceInfo interface_info;

  friend std::ostream & operator<<(std::ostream & os, const InterfaceDescription & interface_descr)
  {
    os << "<InterfaceDescription{{prefix_name:" << interface_descr.prefix_name
       << "}, {interface_type: " << interface_descr.interface_info << "}}>";
    return os;
  }

  // InterfaceInfo interface_info;
};

/**
 * This structure stores information about components defined for a specific hardware
 * in robot's URDF.
 */
struct ComponentInfo
{
  /// Name of the component.
  std::string name;
  /// Type of the component: sensor, joint, or GPIO.
  std::string type;

  /**
   * Name of the command interfaces that can be set, e.g. "position", "velocity", etc.
   * Used by joints and GPIOs.
   */
  std::vector<InterfaceInfo> command_interfaces;
  /**
   * Name of the state interfaces that can be read, e.g. "position", "velocity", etc.
   * Used by joints, sensors and GPIOs.
   */
  std::vector<InterfaceInfo> state_interfaces;
  /// (Optional) Key-value pairs of component parameters, e.g. min/max values or serial number.
  std::unordered_map<std::string, std::string> parameters;

  friend std::ostream & operator<<(std::ostream & os, const ComponentInfo & component_info)
  {
    os << "<ComponentInfo{{name:" << component_info.name << "}, {type: " << component_info.type
       << "}, {command_interfaces: " << component_info.command_interfaces
       << "}, {state_interfaces: " << component_info.state_interfaces
       << "}, {parameters: " << component_info.parameters << "}}>";
    return os;
  }
};

/// Contains semantic info about a given joint loaded from URDF for a transmission
struct JointInfo
{
  std::string name;
  std::vector<std::string> state_interfaces;
  std::vector<std::string> command_interfaces;
  std::string role;
  double mechanical_reduction = 1.0;
  double offset = 0.0;

  friend std::ostream & operator<<(std::ostream & os, const JointInfo & joint_info)
  {
    os << "<JointInfo{{name:" << joint_info.name
       << "}, {state_interfaces: " << joint_info.state_interfaces
       << "}, {command_interfaces: " << joint_info.command_interfaces
       << "}, {role: " << joint_info.role
       << "}, {mechanical_reduction: " << joint_info.mechanical_reduction
       << "}, {offset: " << joint_info.offset << "}}>";
    return os;
  }
};

/// Contains semantic info about a given actuator loaded from URDF for a transmission
struct ActuatorInfo
{
  std::string name;
  std::vector<std::string> state_interfaces;
  std::vector<std::string> command_interfaces;
  std::string role;
  double mechanical_reduction = 1.0;
  double offset = 0.0;

  friend std::ostream & operator<<(std::ostream & os, const ActuatorInfo & actuator_info)
  {
    os << "<ActuatorInfo{{name:" << actuator_info.name
       << "}, {state_interfaces: " << actuator_info.state_interfaces
       << "}, {command_interfaces: " << actuator_info.command_interfaces
       << "}, {role: " << actuator_info.role
       << "}, {mechanical_reduction: " << actuator_info.mechanical_reduction
       << "}, {offset: " << actuator_info.offset << "}}>";
    return os;
  }
};

/// Contains semantic info about a given transmission loaded from URDF
struct TransmissionInfo
{
  std::string name;
  std::string type;
  std::vector<JointInfo> joints;
  std::vector<ActuatorInfo> actuators;
  /// (Optional) Key-value pairs of custom parameters
  std::unordered_map<std::string, std::string> parameters;

  friend std::ostream & operator<<(std::ostream & os, const TransmissionInfo & transmission_info)
  {
    os << "<TransmissionInfo{{name:" << transmission_info.name
       << "}, {type: " << transmission_info.type << "}, {joints: " << transmission_info.joints
       << "}, {actuators: " << transmission_info.actuators
       << "}, {parameters: " << transmission_info.parameters << "}}>";
    return os;
  }
};

/// This structure stores information about hardware defined in a robot's URDF.
struct HardwareInfo
{
  /// Name of the hardware.
  std::string name;
  /// Type of the hardware: actuator, sensor or system.
  std::string type;
  /// Component is async
  bool is_async;
  /// Name of the pluginlib plugin of the hardware that will be loaded.
  std::string hardware_plugin_name;
  /// (Optional) Key-value pairs for hardware parameters.
  std::unordered_map<std::string, std::string> hardware_parameters;
  /**
   * Map of joints provided by the hardware where the key is the joint name.
   * Required for Actuator and System Hardware.
   */
  std::vector<ComponentInfo> joints;
  /**
   * Map of sensors provided by the hardware where the key is the joint or link name.
   * Required for Sensor and optional for System Hardware.
   */
  std::vector<ComponentInfo> sensors;
  /**
   * Map of GPIO provided by the hardware where the key is a descriptive name of the GPIO.
   * Optional for any hardware components.
   */
  std::vector<ComponentInfo> gpios;
  /**
   * Map of transmissions to calculate ration between joints and physical actuators.
   * Optional for Actuator and System Hardware.
   */
  std::vector<TransmissionInfo> transmissions;
  /**
   * The XML contents prior to parsing
   */
  std::string original_xml;

  friend std::ostream & operator<<(std::ostream & os, const HardwareInfo & hardware_info)
  {
    os << "<HardwareInfo{{name:" << hardware_info.name << "}, {type: " << hardware_info.type
       << "}, {is_async: " << hardware_info.is_async
       << "}, {hardware_plugin_name: " << hardware_info.hardware_plugin_name
       << "}, {hardware_parameters: " << hardware_info.hardware_parameters
       << "}, {joints: " << hardware_info.joints << "}, {sensors: " << hardware_info.sensors
       << "}, {gpios: " << hardware_info.gpios
       << "}, {transmissions: " << hardware_info.transmissions
       << "}, {original_xml: " << hardware_info.original_xml << "}}}";
    return os;
  }
};

}  // namespace hardware_interface
#endif  // HARDWARE_INTERFACE__HARDWARE_INFO_HPP_
