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

#include <tinyxml2.h>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>

#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/component_parser.hpp"

namespace
{
constexpr const auto kRobotTag = "robot";
constexpr const auto kROS2ControlTag = "ros2_control";
constexpr const auto kHardwareTag = "hardware";
constexpr const auto kClassTypeTag = "plugin";
constexpr const auto kParamTag = "param";
constexpr const auto kJointTag = "joint";
constexpr const auto kSensorTag = "sensor";
constexpr const auto kTransmissionTag = "transmission";
constexpr const auto kCommandInterfaceTag = "command_interface";
constexpr const auto kStateInterfaceTag = "state_interface";
constexpr const auto kMinTag = "min";
constexpr const auto kMaxTag = "max";
constexpr const auto kNameAttribute = "name";
constexpr const auto kTypeAttribute = "type";
}  // namespace

namespace hardware_interface
{

namespace detail
{
/**
 * \brief Gets value of the text between tags.
 *
 * \param element_it XMLElement iterator to search for the text.
 * \param tag_name parent tag name where text is searched for (used for error output)
 * \return text of for the tag
 * \throws std::runtime_error if text is not found
 */
std::string get_text_for_element(
  const tinyxml2::XMLElement * element_it, const std::string & tag_name)
{
  const auto get_text_output = element_it->GetText();
  if (!get_text_output) {
    throw std::runtime_error("text not specified in the " + tag_name + " tag");
  }
  return get_text_output;
}

/**
 * \brief Gets value of the attribute on an XMLelement.
 * If attribute is not found throws an error.
 *
 * \param element_it XMLElement iterator to search for the attribute
 * \param attribute_name attribute name to search for and return value
 * \param tag_name parent tag name where attribute is searched for (used for error output)
 * \return attribute value
 * \throws std::runtime_error if attribute is not found
 */
std::string get_attribute_value(
  const tinyxml2::XMLElement * element_it,
  const char * attribute_name,
  std::string tag_name)
{
  const tinyxml2::XMLAttribute * attr;
  attr = element_it->FindAttribute(attribute_name);
  if (!attr) {
    throw std::runtime_error(
            "no attribute " + std::string(attribute_name) + " in " + tag_name + " tag");
  }
  return element_it->Attribute(attribute_name);
}

/**
 * \brief Gets value of the attribute on an XMLelement.
 * If attribute is not found throws an error.
 *
 * \param element_it XMLElement iterator to search for the attribute
 * \param attribute_name atribute name to search for and return value
 * \param tag_name parent tag name where attribute is searched for (used for error output)
 * \return attribute value
 * \throws std::runtime_error if attribute is not found
 */
std::string get_attribute_value(
  const tinyxml2::XMLElement * element_it,
  const char * attribute_name,
  const char * tag_name)
{
  return get_attribute_value(element_it, attribute_name, std::string(tag_name));
}

/**
 * \brief Search XML snippet from URDF for parameters.
 *
 * \param params_it pointer to the iterator where parameters info should be found
 * \return std::map< std::__cxx11::string, std::__cxx11::string > key-value map with parameters
 * \throws std::runtime_error if a component attribute or tag is not found
 */
std::unordered_map<std::string, std::string> parse_parameters_from_xml(
  const tinyxml2::XMLElement * params_it)
{
  std::unordered_map<std::string, std::string> parameters;
  const tinyxml2::XMLAttribute * attr;

  while (params_it) {
    // Fill the map with parameters
    attr = params_it->FindAttribute(kNameAttribute);
    if (!attr) {
      throw std::runtime_error("no parameter name attribute set in param tag");
    }
    const std::string parameter_name = params_it->Attribute(kNameAttribute);
    const std::string parameter_value = get_text_for_element(params_it, parameter_name);
    parameters[parameter_name] = parameter_value;

    params_it = params_it->NextSiblingElement(kParamTag);
  }
  return parameters;
}

/**
 * \brief Search XML snippet for definition of interfaceTypes.
 *
 * \param interfaces_it pointer to the iterator over interfaces
 * \param interfaceTag interface type tag (command or state)
 * \return std::vector< std::__cxx11::string > list of interface types
 * \throws std::runtime_error if the interfaceType text not set in a tag
 */
hardware_interface::InterfaceInfo parse_interfaces_from_xml(
  const tinyxml2::XMLElement * interfaces_it)
{
  hardware_interface::InterfaceInfo interface;

  const std::string interface_name = get_attribute_value(
    interfaces_it, kNameAttribute, interfaces_it->Name());
  interface.name = interface_name;

  // Optional min/max attributes
  std::unordered_map<std::string, std::string> interface_params =
    parse_parameters_from_xml(interfaces_it->FirstChildElement(kParamTag));
  auto interface_param =
    interface_params.find(kMinTag);
  if (interface_param != interface_params.end()) {
    interface.min = interface_param->second;
  }
  interface_param = interface_params.find(kMaxTag);
  if (interface_param != interface_params.end()) {
    interface.max = interface_param->second;
  }

  return interface;
}

/**
  * \brief Search XML snippet from URDF for information about a control component.
  *
  * \param component_it pointer to the iterator where component info should be found
  * \return ComponentInfo filled with information about component
  * \throws std::runtime_error if a component attribute or tag is not found
  */
ComponentInfo parse_component_from_xml(const tinyxml2::XMLElement * component_it)
{
  ComponentInfo component;

  // Find name, type and class of a component
  component.type = component_it->Name();
  component.name = get_attribute_value(component_it, kNameAttribute, component.type);

  // Parse all command interfaces
  const auto * command_interfaces_it = component_it->FirstChildElement(kCommandInterfaceTag);
  while (command_interfaces_it) {
    component.command_interfaces.push_back(parse_interfaces_from_xml(command_interfaces_it));
    command_interfaces_it = command_interfaces_it->NextSiblingElement(kCommandInterfaceTag);
  }

  // Parse state interfaces
  const auto * state_interfaces_it = component_it->FirstChildElement(kStateInterfaceTag);
  while (state_interfaces_it) {
    component.state_interfaces.push_back(parse_interfaces_from_xml(state_interfaces_it));
    state_interfaces_it = state_interfaces_it->NextSiblingElement(kStateInterfaceTag);
  }

  // Parse parameters
  const auto * params_it = component_it->FirstChildElement(kParamTag);
  if (params_it) {
    component.parameters = parse_parameters_from_xml(params_it);
  }

  return component;
}

/**
 * \brief Parse a control resource from an "ros2_control" tag.
 *
 * \param ros2_control_it pointer to ros2_control element with informtions about resource.
 * \return ComponentInfo filled with information about the robot
 * \throws std::runtime_error if a attributes or tag are not found
 */
HardwareInfo parse_resource_from_xml(const tinyxml2::XMLElement * ros2_control_it)
{
  HardwareInfo hardware;
  hardware.name = get_attribute_value(ros2_control_it, kNameAttribute, kROS2ControlTag);
  hardware.type = get_attribute_value(ros2_control_it, kTypeAttribute, kROS2ControlTag);

  // Parse everything under ros2_control tag
  hardware.hardware_class_type = "";
  const auto * ros2_control_child_it = ros2_control_it->FirstChildElement();
  while (ros2_control_child_it) {
    if (!std::string(kHardwareTag).compare(ros2_control_child_it->Name())) {
      const auto * type_it = ros2_control_child_it->FirstChildElement(kClassTypeTag);
      hardware.hardware_class_type = get_text_for_element(
        type_it, std::string("hardware ") + kClassTypeTag);
      const auto * params_it = ros2_control_child_it->FirstChildElement(kParamTag);
      if (params_it) {
        hardware.hardware_parameters = parse_parameters_from_xml(params_it);
      }
    } else if (!std::string(kJointTag).compare(ros2_control_child_it->Name())) {
      hardware.joints.push_back(parse_component_from_xml(ros2_control_child_it) );
    } else if (!std::string(kSensorTag).compare(ros2_control_child_it->Name())) {
      hardware.sensors.push_back(parse_component_from_xml(ros2_control_child_it) );
    } else if (!std::string(kTransmissionTag).compare(ros2_control_child_it->Name())) {
      hardware.transmissions.push_back(parse_component_from_xml(ros2_control_child_it) );
    } else {
      throw std::runtime_error("invalid tag name " + std::string(ros2_control_child_it->Name()));
    }
    ros2_control_child_it = ros2_control_child_it->NextSiblingElement();
  }

  return hardware;
}

}  // namespace detail

std::vector<HardwareInfo> parse_control_resources_from_urdf(const std::string & urdf)
{
  // Check if everything OK with URDF string
  if (urdf.empty()) {
    throw std::runtime_error("empty URDF passed to robot");
  }
  tinyxml2::XMLDocument doc;
  if (!doc.Parse(urdf.c_str()) && doc.Error()) {
    throw std::runtime_error("invalid URDF passed in to robot parser");
  }
  if (doc.Error()) {
    throw std::runtime_error("invalid URDF passed in to robot parser");
  }

  // Find robot tag
  const tinyxml2::XMLElement * robot_it = doc.RootElement();

  if (std::string(kRobotTag).compare(robot_it->Name())) {
    throw std::runtime_error("the robot tag is not root element in URDF");
  }

  const tinyxml2::XMLElement * ros2_control_it = robot_it->FirstChildElement(kROS2ControlTag);
  if (!ros2_control_it) {
    throw std::runtime_error("no " + std::string(kROS2ControlTag) + " tag");
  }

  std::vector<HardwareInfo> hardware_info;
  while (ros2_control_it) {
    hardware_info.push_back(detail::parse_resource_from_xml(ros2_control_it));
    ros2_control_it = ros2_control_it->NextSiblingElement(kROS2ControlTag);
  }

  return hardware_info;
}

}  // namespace hardware_interface
