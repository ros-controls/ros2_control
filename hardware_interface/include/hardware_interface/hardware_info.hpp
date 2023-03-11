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

#include <string>
#include <unordered_map>
#include <vector>

namespace hardware_interface
{
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
};

/// This structure stores information about hardware defined in a robot's URDF.
struct HardwareInfo
{
  /// Name of the hardware.
  std::string name;
  /// Type of the hardware: actuator, sensor or system.
  std::string type;
  /// Class of the hardware that will be dynamically loaded.
  std::string hardware_class_type;
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
};

}  // namespace hardware_interface
#endif  // HARDWARE_INTERFACE__HARDWARE_INFO_HPP_
