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

#include "joint_limits/joint_limits.hpp"

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
  /// (Optional) The datatype of the interface, e.g. "bool", "int".
  std::string data_type;
  /// (Optional) If the handle is an array, the size of the array.
  int size;
  /// (Optional) enable or disable the limits for the command interfaces
  bool enable_limits;
  /// (Optional) Key-value pairs of command/stateInterface parameters. This is
  /// useful for drivers that operate on protocols like modbus, where each
  /// interface needs own address(register), datatype, etc.
  std::unordered_map<std::string, std::string> parameters;
};

/// @brief This structure stores information about a joint that is mimicking another joint
struct MimicJoint
{
  std::size_t joint_index;
  std::size_t mimicked_joint_index;
  double multiplier = 1.0;
  double offset = 0.0;
};

/// @brief This enum is used to store the mimic attribute of a joint
enum class MimicAttribute
{
  NOT_SET,
  TRUE,
  FALSE
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

  ///  Hold the value of the mimic attribute if given, NOT_SET otherwise
  MimicAttribute is_mimic = MimicAttribute::NOT_SET;

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

/**
 * This structure stores information about an interface for a specific hardware which should be
 * instantiated internally.
 */
struct InterfaceDescription
{
  InterfaceDescription(const std::string & prefix_name_in, const InterfaceInfo & interface_info_in)
  : prefix_name(prefix_name_in),
    interface_info(interface_info_in),
    interface_name(prefix_name + "/" + interface_info.name)
  {
  }

  /**
   * Name of the interface defined by the user.
   */
  std::string prefix_name;

  /**
   * Information about the Interface type (position, velocity,...) as well as limits and so on.
   */
  InterfaceInfo interface_info;

  /**
   * Name of the interface
   */
  std::string interface_name;

  std::string get_prefix_name() const { return prefix_name; }

  std::string get_interface_name() const { return interface_info.name; }

  std::string get_name() const { return interface_name; }
};

/// This structure stores information about hardware defined in a robot's URDF.
struct HardwareInfo
{
  /// Name of the hardware.
  std::string name;
  /// Type of the hardware: actuator, sensor or system.
  std::string type;
  ///  Hardware group to which the hardware belongs.
  std::string group;
  /// Component is async
  bool is_async;
  /// Name of the pluginlib plugin of the hardware that will be loaded.
  std::string hardware_plugin_name;
  /// (Optional) Key-value pairs for hardware parameters.
  std::unordered_map<std::string, std::string> hardware_parameters;
  /**
   * Vector of joints provided by the hardware.
   * Required for Actuator and System Hardware.
   */
  std::vector<ComponentInfo> joints;
  /**
   * Vector of mimic joints.
   */
  std::vector<MimicJoint> mimic_joints;
  /**
   * Vector of sensors provided by the hardware.
   * Required for Sensor and optional for System Hardware.
   */
  std::vector<ComponentInfo> sensors;
  /**
   * Vector of GPIOs provided by the hardware.
   * Optional for any hardware components.
   */
  std::vector<ComponentInfo> gpios;
  /**
   * Vector of transmissions to calculate ratio between joints and physical actuators.
   * Optional for Actuator and System Hardware.
   */
  std::vector<TransmissionInfo> transmissions;
  /**
   * The XML contents prior to parsing
   */
  std::string original_xml;
  /**
   * The URDF parsed limits of the hardware components joint command interfaces
   */
  std::unordered_map<std::string, joint_limits::JointLimits> limits;

  /**
   * Map of software joint limits used for clamping the command where the key is the joint name.
   * Optional If not specified or less restrictive than the JointLimits uses the previous
   * JointLimits.
   */
  std::unordered_map<std::string, joint_limits::SoftJointLimits> soft_limits;
};

}  // namespace hardware_interface
#endif  // HARDWARE_INTERFACE__HARDWARE_INFO_HPP_
