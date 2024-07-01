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
#include <charconv>
#include <iostream>
#include <regex>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>

#include "hardware_interface/component_parser.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/lexical_casts.hpp"

namespace
{
constexpr const auto kRobotTag = "robot";
constexpr const auto kROS2ControlTag = "ros2_control";
constexpr const auto kHardwareTag = "hardware";
constexpr const auto kClassTypeTag = "plugin";
constexpr const auto kParamTag = "param";
constexpr const auto kActuatorTag = "actuator";
constexpr const auto kJointTag = "joint";
constexpr const auto kSensorTag = "sensor";
constexpr const auto kGPIOTag = "gpio";
constexpr const auto kTransmissionTag = "transmission";
constexpr const auto kCommandInterfaceTag = "command_interface";
constexpr const auto kStateInterfaceTag = "state_interface";
constexpr const auto kMinTag = "min";
constexpr const auto kMaxTag = "max";
constexpr const auto kInitialValueTag = "initial_value";
constexpr const auto kDataTypeAttribute = "data_type";
constexpr const auto kSizeAttribute = "size";
constexpr const auto kNameAttribute = "name";
constexpr const auto kTypeAttribute = "type";
constexpr const auto kRoleAttribute = "role";
constexpr const auto kReductionAttribute = "mechanical_reduction";
constexpr const auto kOffsetAttribute = "offset";
}  // namespace

namespace hardware_interface
{
namespace detail
{
/// Gets value of the text between tags.
/**
 * \param[in] element_it XMLElement iterator to search for the text.
 * \param[in] tag_name parent tag name where text is searched for (used for error output)
 * \return text of for the tag
 * \throws std::runtime_error if text is not found
 */
std::string get_text_for_element(
  const tinyxml2::XMLElement * element_it, const std::string & tag_name)
{
  const auto get_text_output = element_it->GetText();
  if (!get_text_output)
  {
    std::cerr << "text not specified in the " << tag_name << " tag" << std::endl;
    return "";
  }
  return get_text_output;
}

/// Gets value of the attribute on an XMLelement.
/**
 * If attribute is not found throws an error.
 *
 * \param[in] element_it XMLElement iterator to search for the attribute
 * \param[in] attribute_name attribute name to search for and return value
 * \param[in] tag_name parent tag name where attribute is searched for (used for error output)
 * \return attribute value
 * \throws std::runtime_error if attribute is not found
 */
std::string get_attribute_value(
  const tinyxml2::XMLElement * element_it, const char * attribute_name, std::string tag_name)
{
  const tinyxml2::XMLAttribute * attr;
  attr = element_it->FindAttribute(attribute_name);
  if (!attr)
  {
    throw std::runtime_error(
      "no attribute " + std::string(attribute_name) + " in " + tag_name + " tag");
  }
  return element_it->Attribute(attribute_name);
}

/// Gets value of the attribute on an XMLelement.
/**
 * If attribute is not found throws an error.
 *
 * \param[in] element_it XMLElement iterator to search for the attribute
 * \param[in] attribute_name attribute name to search for and return value
 * \param[in] tag_name parent tag name where attribute is searched for (used for error output)
 * \return attribute value
 * \throws std::runtime_error if attribute is not found
 */
std::string get_attribute_value(
  const tinyxml2::XMLElement * element_it, const char * attribute_name, const char * tag_name)
{
  return get_attribute_value(element_it, attribute_name, std::string(tag_name));
}

/// Gets value of the parameter on an XMLelement.
/**
 * If parameter is not found, returns specified default value
 *
 * \param[in] element_it XMLElement iterator to search for the attribute
 * \param[in] attribute_name attribute name to search for and return value
 * \param[in] default_value When the attribute is not found, this value is returned instead
 * \return attribute value or default
 */
double get_parameter_value_or(
  const tinyxml2::XMLElement * params_it, const char * parameter_name, const double default_value)
{
  while (params_it)
  {
    try
    {
      // Fill the map with parameters
      const auto tag_name = params_it->Name();
      if (strcmp(tag_name, parameter_name) == 0)
      {
        const auto tag_text = params_it->GetText();
        if (tag_text)
        {
          return hardware_interface::stod(tag_text);
        }
      }
    }
    catch (const std::exception & e)
    {
      return default_value;
    }

    params_it = params_it->NextSiblingElement();
  }
  return default_value;
}

/// Parse optional size attribute
/**
 * Parses an XMLElement and returns the value of the size attribute.
 * If not specified, defaults to 1. If not given a positive integer, throws an error.
 *
 * \param[in] elem XMLElement that has the size attribute.
 * \return The size.
 * \throws std::runtime_error if not given a positive non-zero integer as value.
 */
std::size_t parse_size_attribute(const tinyxml2::XMLElement * elem)
{
  const tinyxml2::XMLAttribute * attr = elem->FindAttribute(kSizeAttribute);

  if (!attr)
  {
    return 1;
  }

  std::size_t size;
  // Regex used to check for non-zero positive int
  std::string s = attr->Value();
  std::regex int_re("[1-9][0-9]*");
  if (std::regex_match(s, int_re))
  {
    size = std::stoi(s);
  }
  else
  {
    throw std::runtime_error(
      "Could not parse size tag in \"" + std::string(elem->Name()) + "\"." + "Got \"" + s +
      "\", but expected a non-zero positive integer.");
  }

  return size;
}

/// Parse data_type attribute
/**
 * Parses an XMLElement and returns the value of the data_type attribute.
 * Defaults to "double" if not specified.
 *
 * \param[in] elem XMLElement that has the data_type attribute.
 * \return string specifying the data type.
 */
std::string parse_data_type_attribute(const tinyxml2::XMLElement * elem)
{
  const tinyxml2::XMLAttribute * attr = elem->FindAttribute(kDataTypeAttribute);
  std::string data_type;
  if (!attr)
  {
    data_type = "double";
  }
  else
  {
    data_type = attr->Value();
  }

  return data_type;
}

/// Search XML snippet from URDF for parameters.
/**
 * \param[in] params_it pointer to the iterator where parameters info should be found
 * \return key-value map with parameters
 * \throws std::runtime_error if a component attribute or tag is not found
 */
std::unordered_map<std::string, std::string> parse_parameters_from_xml(
  const tinyxml2::XMLElement * params_it)
{
  std::unordered_map<std::string, std::string> parameters;
  const tinyxml2::XMLAttribute * attr;

  while (params_it)
  {
    // Fill the map with parameters
    attr = params_it->FindAttribute(kNameAttribute);
    if (!attr)
    {
      throw std::runtime_error("no parameter name attribute set in param tag");
    }
    const std::string parameter_name = params_it->Attribute(kNameAttribute);
    const std::string parameter_value = get_text_for_element(params_it, parameter_name);
    parameters[parameter_name] = parameter_value;

    params_it = params_it->NextSiblingElement(kParamTag);
  }
  return parameters;
}

/// Search XML snippet for definition of interfaceTypes.
/**
 * \param[in] interfaces_it pointer to the iterator over interfaces
 * \param[in] interfaceTag interface type tag (command or state)
 * \return list of interface types
 * \throws std::runtime_error if the interfaceType text not set in a tag
 */
hardware_interface::InterfaceInfo parse_interfaces_from_xml(
  const tinyxml2::XMLElement * interfaces_it)
{
  hardware_interface::InterfaceInfo interface;

  const std::string interface_name =
    get_attribute_value(interfaces_it, kNameAttribute, interfaces_it->Name());
  interface.name = interface_name;

  // Optional min/max attributes
  std::unordered_map<std::string, std::string> interface_params =
    parse_parameters_from_xml(interfaces_it->FirstChildElement(kParamTag));
  auto interface_param = interface_params.find(kMinTag);
  if (interface_param != interface_params.end())
  {
    interface.min = interface_param->second;
  }
  interface_param = interface_params.find(kMaxTag);
  if (interface_param != interface_params.end())
  {
    interface.max = interface_param->second;
  }

  // Optional initial_value attribute
  interface_param = interface_params.find(kInitialValueTag);
  if (interface_param != interface_params.end())
  {
    interface.initial_value = interface_param->second;
  }

  // Default to a single double
  interface.data_type = "double";
  interface.size = 1;

  return interface;
}

/// Search XML snippet from URDF for information about a control component.
/**
 * \param[in] component_it pointer to the iterator where component
 * info should be found
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
  while (command_interfaces_it)
  {
    component.command_interfaces.push_back(parse_interfaces_from_xml(command_interfaces_it));
    command_interfaces_it = command_interfaces_it->NextSiblingElement(kCommandInterfaceTag);
  }

  // Parse state interfaces
  const auto * state_interfaces_it = component_it->FirstChildElement(kStateInterfaceTag);
  while (state_interfaces_it)
  {
    component.state_interfaces.push_back(parse_interfaces_from_xml(state_interfaces_it));
    state_interfaces_it = state_interfaces_it->NextSiblingElement(kStateInterfaceTag);
  }

  // Parse parameters
  const auto * params_it = component_it->FirstChildElement(kParamTag);
  if (params_it)
  {
    component.parameters = parse_parameters_from_xml(params_it);
  }

  return component;
}

/// Search XML snippet from URDF for information about a complex component.
/**
 * A complex component can have a non-double data type specified on its interfaces,
 *  and the interface may be an array of a fixed size of the data type.
 *
 * \param[in] component_it pointer to the iterator where component
 * info should befound
 * \throws std::runtime_error if a required component attribute or tag is not found.
 */
ComponentInfo parse_complex_component_from_xml(const tinyxml2::XMLElement * component_it)
{
  ComponentInfo component;

  // Find name, type and class of a component
  component.type = component_it->Name();
  component.name = get_attribute_value(component_it, kNameAttribute, component.type);

  // Parse all command interfaces
  const auto * command_interfaces_it = component_it->FirstChildElement(kCommandInterfaceTag);
  while (command_interfaces_it)
  {
    component.command_interfaces.push_back(parse_interfaces_from_xml(command_interfaces_it));
    component.command_interfaces.back().data_type =
      parse_data_type_attribute(command_interfaces_it);
    component.command_interfaces.back().size = parse_size_attribute(command_interfaces_it);
    command_interfaces_it = command_interfaces_it->NextSiblingElement(kCommandInterfaceTag);
  }

  // Parse state interfaces
  const auto * state_interfaces_it = component_it->FirstChildElement(kStateInterfaceTag);
  while (state_interfaces_it)
  {
    component.state_interfaces.push_back(parse_interfaces_from_xml(state_interfaces_it));
    component.state_interfaces.back().data_type = parse_data_type_attribute(state_interfaces_it);
    component.state_interfaces.back().size = parse_size_attribute(state_interfaces_it);
    state_interfaces_it = state_interfaces_it->NextSiblingElement(kStateInterfaceTag);
  }

  // Parse parameters
  const auto * params_it = component_it->FirstChildElement(kParamTag);
  if (params_it)
  {
    component.parameters = parse_parameters_from_xml(params_it);
  }

  return component;
}

JointInfo parse_transmission_joint_from_xml(const tinyxml2::XMLElement * element_it)
{
  JointInfo joint_info;
  joint_info.name = get_attribute_value(element_it, kNameAttribute, element_it->Name());
  joint_info.role = get_attribute_value(element_it, kRoleAttribute, element_it->Name());
  joint_info.mechanical_reduction =
    get_parameter_value_or(element_it->FirstChildElement(), kReductionAttribute, 1.0);
  joint_info.offset =
    get_parameter_value_or(element_it->FirstChildElement(), kOffsetAttribute, 0.0);
  return joint_info;
}

ActuatorInfo parse_transmission_actuator_from_xml(const tinyxml2::XMLElement * element_it)
{
  ActuatorInfo actuator_info;
  actuator_info.name = get_attribute_value(element_it, kNameAttribute, element_it->Name());
  actuator_info.role = get_attribute_value(element_it, kRoleAttribute, element_it->Name());
  actuator_info.mechanical_reduction =
    get_parameter_value_or(element_it->FirstChildElement(), kReductionAttribute, 1.0);
  actuator_info.offset =
    get_parameter_value_or(element_it->FirstChildElement(), kOffsetAttribute, 0.0);
  return actuator_info;
}

/// Search XML snippet from URDF for information about a transmission.
/**
 * \param[in] transmission_it pointer to the iterator where transmission info should be found
 * \return TransmissionInfo filled with information about transmission
 * \throws std::runtime_error if an attribute or tag is not found
 */
TransmissionInfo parse_transmission_from_xml(const tinyxml2::XMLElement * transmission_it)
{
  TransmissionInfo transmission;

  // Find name, type and class of a transmission
  transmission.name = get_attribute_value(transmission_it, kNameAttribute, transmission_it->Name());
  const auto * type_it = transmission_it->FirstChildElement(kClassTypeTag);
  transmission.type = get_text_for_element(type_it, kClassTypeTag);

  // Parse joints
  const auto * joint_it = transmission_it->FirstChildElement(kJointTag);
  while (joint_it)
  {
    transmission.joints.push_back(parse_transmission_joint_from_xml(joint_it));
    joint_it = joint_it->NextSiblingElement(kJointTag);
  }

  // Parse actuators
  const auto * actuator_it = transmission_it->FirstChildElement(kActuatorTag);
  while (actuator_it)
  {
    transmission.actuators.push_back(parse_transmission_actuator_from_xml(actuator_it));
    actuator_it = actuator_it->NextSiblingElement(kActuatorTag);
  }

  // Parse parameters
  const auto * params_it = transmission_it->FirstChildElement(kParamTag);
  if (params_it)
  {
    transmission.parameters = parse_parameters_from_xml(params_it);
  }

  return transmission;
}

/// Auto-fill some contents of transmission info based on context
/**
 * \param[in,out] hardware HardwareInfo structure with elements already parsed.
 * \throws std::runtime_error
 */
void auto_fill_transmission_interfaces(HardwareInfo & hardware)
{
  for (auto & transmission : hardware.transmissions)
  {
    // fill joint interfaces for actuator from joints declared for this component
    for (auto & joint : transmission.joints)
    {
      auto found_it = std::find_if(
        hardware.joints.cbegin(), hardware.joints.cend(),
        [&joint](const auto & joint_info) { return joint.name == joint_info.name; });

      if (found_it == hardware.joints.cend())
      {
        throw std::runtime_error(
          "Error while parsing '" + hardware.name + "'. Transmission '" + transmission.name +
          "' declared joint '" + joint.name + "' is not available in component '" + hardware.name +
          "'.");
      }

      //  copy interface names from their definitions in the component
      std::transform(
        found_it->command_interfaces.cbegin(), found_it->command_interfaces.cend(),
        std::back_inserter(joint.command_interfaces),
        [](const auto & interface) { return interface.name; });

      std::transform(
        found_it->state_interfaces.cbegin(), found_it->state_interfaces.cend(),
        std::back_inserter(joint.state_interfaces),
        [](const auto & interface) { return interface.name; });
    }

    // we parsed an actuator component, here we fill in more details
    if (hardware.type == kActuatorTag)
    {
      if (transmission.joints.size() != 1)
      {
        throw std::runtime_error(
          "Error while parsing '" + hardware.name +
          "'. There should be exactly one joint defined in this component but found " +
          std::to_string(transmission.joints.size()));
      }

      transmission.actuators.push_back(ActuatorInfo{
        "actuator1", transmission.joints[0].state_interfaces,
        transmission.joints[0].command_interfaces, "actuator1", 1.0, 0.0});
    }
  }
}

/// Parse a control resource from an "ros2_control" tag.
/**
 * \param[in] ros2_control_it pointer to ros2_control element
 * with information about resource.
 * \return HardwareInfo filled with information about the robot
 * \throws std::runtime_error if a attributes or tag are not found
 */
HardwareInfo parse_resource_from_xml(
  const tinyxml2::XMLElement * ros2_control_it, const std::string & urdf)
{
  HardwareInfo hardware;
  hardware.name = get_attribute_value(ros2_control_it, kNameAttribute, kROS2ControlTag);
  hardware.type = get_attribute_value(ros2_control_it, kTypeAttribute, kROS2ControlTag);

  // Parse everything under ros2_control tag
  hardware.hardware_class_type = "";
  const auto * ros2_control_child_it = ros2_control_it->FirstChildElement();
  while (ros2_control_child_it)
  {
    if (!std::string(kHardwareTag).compare(ros2_control_child_it->Name()))
    {
      const auto * type_it = ros2_control_child_it->FirstChildElement(kClassTypeTag);
      hardware.hardware_class_type =
        get_text_for_element(type_it, std::string("hardware ") + kClassTypeTag);
      const auto * params_it = ros2_control_child_it->FirstChildElement(kParamTag);
      if (params_it)
      {
        hardware.hardware_parameters = parse_parameters_from_xml(params_it);
      }
    }
    else if (!std::string(kJointTag).compare(ros2_control_child_it->Name()))
    {
      hardware.joints.push_back(parse_component_from_xml(ros2_control_child_it));
    }
    else if (!std::string(kSensorTag).compare(ros2_control_child_it->Name()))
    {
      hardware.sensors.push_back(parse_component_from_xml(ros2_control_child_it));
    }
    else if (!std::string(kGPIOTag).compare(ros2_control_child_it->Name()))
    {
      hardware.gpios.push_back(parse_complex_component_from_xml(ros2_control_child_it));
    }
    else if (!std::string(kTransmissionTag).compare(ros2_control_child_it->Name()))
    {
      hardware.transmissions.push_back(parse_transmission_from_xml(ros2_control_child_it));
    }
    else
    {
      throw std::runtime_error("invalid tag name " + std::string(ros2_control_child_it->Name()));
    }
    ros2_control_child_it = ros2_control_child_it->NextSiblingElement();
  }

  auto_fill_transmission_interfaces(hardware);

  hardware.original_xml = urdf;

  return hardware;
}

}  // namespace detail

std::vector<HardwareInfo> parse_control_resources_from_urdf(const std::string & urdf)
{
  // Check if everything OK with URDF string
  if (urdf.empty())
  {
    throw std::runtime_error("empty URDF passed to robot");
  }
  tinyxml2::XMLDocument doc;
  if (!doc.Parse(urdf.c_str()) && doc.Error())
  {
    throw std::runtime_error(
      "invalid URDF passed in to robot parser: " + std::string(doc.ErrorStr()));
  }
  if (doc.Error())
  {
    throw std::runtime_error(
      "invalid URDF passed in to robot parser: " + std::string(doc.ErrorStr()));
  }

  // Find robot tag
  const tinyxml2::XMLElement * robot_it = doc.RootElement();

  if (std::string(kRobotTag).compare(robot_it->Name()))
  {
    throw std::runtime_error("the robot tag is not root element in URDF");
  }

  const tinyxml2::XMLElement * ros2_control_it = robot_it->FirstChildElement(kROS2ControlTag);
  if (!ros2_control_it)
  {
    throw std::runtime_error("no " + std::string(kROS2ControlTag) + " tag");
  }

  std::vector<HardwareInfo> hardware_info;
  while (ros2_control_it)
  {
    hardware_info.push_back(detail::parse_resource_from_xml(ros2_control_it, urdf));
    ros2_control_it = ros2_control_it->NextSiblingElement(kROS2ControlTag);
  }

  return hardware_info;
}

}  // namespace hardware_interface
