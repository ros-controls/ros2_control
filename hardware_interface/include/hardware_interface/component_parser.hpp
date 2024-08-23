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

#ifndef HARDWARE_INTERFACE__COMPONENT_PARSER_HPP_
#define HARDWARE_INTERFACE__COMPONENT_PARSER_HPP_

#include <string>
#include <vector>

#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/visibility_control.h"

namespace hardware_interface
{
/// Search XML snippet from URDF for information about a control component.
/**
 * \param[in] urdf string with robot's URDF
 * \return vector filled with information about robot's control resources
 * \throws std::runtime_error if a robot attribute or tag is not found
 */
HARDWARE_INTERFACE_PUBLIC
std::vector<HardwareInfo> parse_control_resources_from_urdf(const std::string & urdf);

/**
 * \param[in] component_info information about a component (gpio, joint, sensor)
 * \return vector filled with information about hardware's StateInterfaces for the component
 * which are exported
 */
HARDWARE_INTERFACE_PUBLIC
std::vector<InterfaceDescription> parse_state_interface_descriptions(
  const std::vector<ComponentInfo> & component_info);

/**
 * \param[in] component_info information about a component (gpio, joint, sensor)
 * \return vector filled with information about hardware's CommandInterfaces for the component
 * which are exported
 */
HARDWARE_INTERFACE_PUBLIC
std::vector<InterfaceDescription> parse_command_interface_descriptions(
  const std::vector<ComponentInfo> & component_info);

}  // namespace hardware_interface
#endif  // HARDWARE_INTERFACE__COMPONENT_PARSER_HPP_
