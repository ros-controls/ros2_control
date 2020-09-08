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

#include <tinyxml2.h>
#include <string>
#include <unordered_map>
#include <vector>

#include "hardware_interface/components/component_info.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/visibility_control.h"

namespace hardware_interface
{

/**
  * \brief Search XML snippet from URDF for information about a control component.
  *
  * \param urdf string with robot's URDF
  * \return vector filled with information about robot's control resources
  * \throws std::runtime_error if a robot attribute or tag is not found
  */
HARDWARE_INTERFACE_PUBLIC
std::vector<HardwareInfo> parse_control_resources_from_urdf(const std::string & urdf);

/**
 * \brief Parse a control resource from an "ros2_control" tag.
 *
 * \param ros2_control_it pointer to ros2_control element with informtions about resource.
 * \return robot_control_components::ComponentInfo filled with information about the robot
 * \throws std::runtime_error if a attributes or tag are not found
 */
HARDWARE_INTERFACE_PUBLIC
HardwareInfo parse_resource_from_xml(const tinyxml2::XMLElement * ros2_control_it);

/**
 * \brief Gets value of the attribute on an XMLelement.
 * If attribute is not found throws an error.
 *
 * \param element_it XMLElement iterator to search for the attribute
 * \param attribute_name atribute name to serach for and return value
 * \param tag_name parent tag name where attribute is searched for (used for error output)
 * \return attribute value
 * \throws std::runtime_error if attribute is not found
 */
HARDWARE_INTERFACE_PUBLIC
std::string get_attribute_value(
  const tinyxml2::XMLElement * element_it,
  const char * attribute_name, const char * tag_name);

/**
 * \brief Gets value of the text between tags.
 *
 * \param element_it XMLElement iterator to search for the text.
 * \param tag_name parent tag name where text is searched for (used for error output)
 * \return text of for the tag
 * \throws std::runtime_error if text is not found
 */
HARDWARE_INTERFACE_PUBLIC
std::string get_text_for_element(
  const tinyxml2::XMLElement * element_it,
  const std::string & tag_name);

/**
 * \brief Gets value of the attribute on an XMLelement.
 * If attribute is not found throws an error.
 *
 * \param element_it XMLElement iterator to search for the attribute
 * \param attribute_name atribute name to serach for and return value
 * \param tag_name parent tag name where attribute is searched for (used for error output)
 * \return attribute value
 * \throws std::runtime_error if attribute is not found
 */
HARDWARE_INTERFACE_PUBLIC
std::string get_attribute_value(
  const tinyxml2::XMLElement * element_it,
  const char * attribute_name, std::string tag_name);

/**
  * \brief Search XML snippet from URDF for information about a control component.
  *
  * \param component_it pointer to the iterator where component info should be found
  * \return robot_control_components::ComponentInfo filled with information about component
  * \throws std::runtime_error if a component attribute or tag is not found
  */
HARDWARE_INTERFACE_PUBLIC
components::ComponentInfo parse_component_from_xml(const tinyxml2::XMLElement * component_it);

/**
 * \brief Search XML snippet for definition of interfaceTypes.
 *
 * \param interfaces_it pointer to the interator over interfaces
 * \param interfaceTag interface type tag (command or state)
 * \return std::vector< std::__cxx11::string > list of interface types
 * \throws std::runtime_error if the interfaceType text not set in a tag
 */
HARDWARE_INTERFACE_PUBLIC
std::vector<std::string> parse_interfaces_from_xml(
  const tinyxml2::XMLElement * interfaces_it, const char * interfaceTag);

/**
 * \brief Search XML snippet from URDF for parameters.
 *
 * \param params_it pointer to the iterator where parameters info should be found
 * \return std::map< std::__cxx11::string, std::__cxx11::string > key-value map with parameters
 * \throws std::runtime_error if a component attribute or tag is not found
 */
HARDWARE_INTERFACE_PUBLIC
std::unordered_map<std::string, std::string> parse_parameters_from_xml(
  const tinyxml2::XMLElement * params_it);

}  // namespace hardware_interface
#endif  // HARDWARE_INTERFACE__COMPONENT_PARSER_HPP_
