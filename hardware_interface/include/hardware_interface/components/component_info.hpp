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

#ifndef HARDWARE_INTERFACE__COMPONENTS__COMPONENT_INFO_HPP_
#define HARDWARE_INTERFACE__COMPONENTS__COMPONENT_INFO_HPP_

#include <string>
#include <unordered_map>
#include <vector>

namespace hardware_interface
{
namespace components
{

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
   * \brief type of the component: sensor or actuator.
   */
  std::string type;
  /**
   * \brief component's class, which will be dynamically loaded.
   */
  std::string class_type;
  /**
   * \brief name of the command interfaces that can be set, e.g. "position", "velocity", etc.
   * Used by joints.
   */
  std::vector<std::string> command_interfaces;
  /**
   * \brief name of the state interfaces that can be read, e.g. "position", "velocity", etc.
   * Used by Joints and Sensors.
   */
  std::vector<std::string> state_interfaces;
  /**
   * \brief (optional) key-value pairs of components parameters.
   */
  std::unordered_map<std::string, std::string> parameters;
};

}  // namespace components
}  // namespace hardware_interface
#endif  // HARDWARE_INTERFACE__COMPONENTS__COMPONENT_INFO_HPP_
