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
 * \brief This structure stores information about components defined for a specific hardware
 * in robot's URDF.
 */
struct InterfaceInfo
{
  /**
   * \brief name of the command interfaces that can be set, e.g. "position", "velocity", etc.
   * Used by joints.
   */
  std::string name;
  /**
   * \brief (optional) minimal allowed values of the interface.
   */
  std::string min;
  /**
   * \brief (optional) maximal allowed values of the interface.
   */
  std::string max;
};

/**
 * \brief This structure stores information about components defined for a specific hardware
 * in robot's URDF.
 */
struct ComponentInfo
{
  /**
   * \brief name of the component.
   */
  std::string name;
  /**
   * \brief type of the component: sensor or joint.
   */
  std::string type;
  /**
   * \brief name of the command interfaces that can be set, e.g. "position", "velocity", etc.
   * Used by joints.
   */
  std::vector<InterfaceInfo> command_interfaces;
  /**
   * \brief name of the state interfaces that can be read, e.g. "position", "velocity", etc.
   * Used by Joints and Sensors.
   */
  std::vector<InterfaceInfo> state_interfaces;
  /**
   * \brief (optional) key-value pairs of component parameters, e.g. min/max values or serial number.
   */
  std::unordered_map<std::string, std::string> parameters;
};
/**
 * \brief This structure stores information about hardware defined in a robot's URDF.
 */
struct HardwareInfo
{
  /**
   * \brief name of the hardware.
   */
  std::string name;
  /**
   * \brief type of the hardware: actuator, sensor or system.
   */
  std::string type;
  /**
   * \brief class of the hardware that will be dynamically loaded.
   */
  std::string hardware_class_type;
  /**
   * \brief (optional) key-value pairs for hardware parameters.
   */
  std::unordered_map<std::string, std::string> hardware_parameters;
  /**
   * \brief map of joints provided by the hardware where the key is the joint name.
   * Required for Actuator and System Hardware.
   */
  std::vector<ComponentInfo> joints;
  /**
   * \brief map of sensors provided by the hardware where the key is the joint or link name.
   * Required for Sensor and optional for System Hardware.
   */
  std::vector<ComponentInfo> sensors;
  /**
   * \brief map of transmissions to calcualte ration between joints and physical actuators.
   * Optional for Actuator and System Hardware.
   */
  std::vector<ComponentInfo> transmissions;
};

}  // namespace hardware_interface
#endif  // HARDWARE_INTERFACE__HARDWARE_INFO_HPP_
