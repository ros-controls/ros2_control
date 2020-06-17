// Copyright 2020 ROS2-Control Development Team
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

#ifndef HARDWARE_INTERFACE__UTILS__COMPONENT_PARSER_HPP_
#define HARDWARE_INTERFACE__UTILS__COMPONENT_PARSER_HPP_

#include <string>
#include <tinyxml2.h>

#include "hardware_interface/component_info.hpp"
#include "hardware_interface/visibility_control.h"

namespace hardware_interface
{
namespace utils
{

/**
  * \brief Search XML snippet from URDF for informations about a control component.
  *
  * \param urdf string with robot's URDF
  * \return robot_control_components::ComponentInfo filled with informations about the robot
  * \throws std::runtime_error if a robot attribute or tag is not found
  */
HARDWARE_INTERFACE_PUBLIC
ComponentInfo parse_robot_from_urdf(const std::string & urdf);

/**
  * \brief Search XML snippet from URDF for informations about a control component.
  *
  * \param component_it pointer to the iterator where component info should be found
  * \return robot_control_components::ComponentInfo filled with informations about component
  * \throws std::runtime_error if a component attribute or tag is not found
  */
HARDWARE_INTERFACE_PUBLIC
ComponentInfo parse_component_from_xml(const tinyxml2::XMLElement * component_it);

/**
 * \brief Search XML snippet from URDF for parameters.
 *
 * \param params_it pointer to the iterator where parameters info should be found
 * \return std::map< std::__cxx11::string, std::__cxx11::string > key-value map with parameters
 * \throws std::runtime_error if a component attribute or tag is not found
 */
HARDWARE_INTERFACE_PUBLIC
std::map<std::string, std::string> parse_parameters_from_xml(
  const tinyxml2::XMLElement * params_it);

}  // namespace utils
}  // namespace hardware_interface
#endif  // HARDWARE_INTERFACE__UTILS__COMPONENT_PARSER_HPP_
