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
#include <iostream>
#include <regex>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>

#include "urdf/model.h"

#include "hardware_interface/component_parser.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/lexical_casts.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "joint_limits/joint_limits_urdf.hpp"

namespace
{
constexpr const auto kRobotTag = "robot";
constexpr const auto kROS2ControlTag = "ros2_control";
constexpr const auto kHardwareTag = "hardware";
constexpr const auto kPluginNameTag = "plugin";
constexpr const auto kParamTag = "param";
constexpr const auto kGroupTag = "group";
constexpr const auto kActuatorTag = "actuator";
constexpr const auto kJointTag = "joint";
constexpr const auto kSensorTag = "sensor";
constexpr const auto kGPIOTag = "gpio";
constexpr const auto kTransmissionTag = "transmission";
constexpr const auto kCommandInterfaceTag = "command_interface";
constexpr const auto kStateInterfaceTag = "state_interface";
constexpr const auto kMinTag = "min";
constexpr const auto kMaxTag = "max";
constexpr const auto kLimitsTag = "limits";
constexpr const auto kEnableAttribute = "enable";
constexpr const auto kInitialValueTag = "initial_value";
constexpr const auto kMimicAttribute = "mimic";
constexpr const auto kDataTypeAttribute = "data_type";
constexpr const auto kSizeAttribute = "size";
constexpr const auto kNameAttribute = "name";
constexpr const auto kTypeAttribute = "type";
constexpr const auto kRoleAttribute = "role";
constexpr const auto kReductionAttribute = "mechanical_reduction";
constexpr const auto kOffsetAttribute = "offset";
constexpr const auto kIsAsyncAttribute = "is_async";

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

/// Parse is_async attribute
/**
 * Parses an XMLElement and returns the value of the is_async attribute.
 * Defaults to "false" if not specified.
 *
 * \param[in] elem XMLElement that has the data_type attribute.
 * \return boolean specifying the if the value read was true or false.
 */
bool parse_is_async_attribute(const tinyxml2::XMLElement * elem)
{
  const tinyxml2::XMLAttribute * attr = elem->FindAttribute(kIsAsyncAttribute);
  return attr ? parse_bool(attr->Value()) : false;
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

  // Option enable or disable the interface limits, by default they are enabled
  interface.enable_limits = true;
  const auto * limits_it = interfaces_it->FirstChildElement(kLimitsTag);
  if (limits_it)
  {
    interface.enable_limits =
      parse_bool(get_attribute_value(limits_it, kEnableAttribute, limits_it->Name()));
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

  // Parse parameters
  const auto * params_it = interfaces_it->FirstChildElement(kParamTag);
  if (params_it)
  {
    interface.parameters = parse_parameters_from_xml(params_it);
  }

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

  if (std::string(kJointTag) == component.type)
  {
    try
    {
      component.is_mimic = parse_bool(get_attribute_value(component_it, kMimicAttribute, kJointTag))
                             ? MimicAttribute::TRUE
                             : MimicAttribute::FALSE;
    }
    catch (const std::runtime_error & e)
    {
      // mimic attribute not set
      component.is_mimic = MimicAttribute::NOT_SET;
    }
  }

  // Option enable or disable the interface limits, by default they are enabled
  bool enable_limits = true;
  const auto * limits_it = component_it->FirstChildElement(kLimitsTag);
  if (limits_it)
  {
    enable_limits = parse_bool(get_attribute_value(limits_it, kEnableAttribute, limits_it->Name()));
  }

  // Parse all command interfaces
  const auto * command_interfaces_it = component_it->FirstChildElement(kCommandInterfaceTag);
  while (command_interfaces_it)
  {
    InterfaceInfo cmd_info = parse_interfaces_from_xml(command_interfaces_it);
    cmd_info.enable_limits &= enable_limits;
    component.command_interfaces.push_back(cmd_info);
    command_interfaces_it = command_interfaces_it->NextSiblingElement(kCommandInterfaceTag);
  }

  // Parse state interfaces
  const auto * state_interfaces_it = component_it->FirstChildElement(kStateInterfaceTag);
  while (state_interfaces_it)
  {
    InterfaceInfo state_info = parse_interfaces_from_xml(state_interfaces_it);
    state_info.enable_limits &= enable_limits;
    component.state_interfaces.push_back(state_info);
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
 * info should be found
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
    component.command_interfaces.back().size =
      static_cast<int>(parse_size_attribute(command_interfaces_it));
    command_interfaces_it = command_interfaces_it->NextSiblingElement(kCommandInterfaceTag);
  }

  // Parse state interfaces
  const auto * state_interfaces_it = component_it->FirstChildElement(kStateInterfaceTag);
  while (state_interfaces_it)
  {
    component.state_interfaces.push_back(parse_interfaces_from_xml(state_interfaces_it));
    component.state_interfaces.back().data_type = parse_data_type_attribute(state_interfaces_it);
    component.state_interfaces.back().size =
      static_cast<int>(parse_size_attribute(state_interfaces_it));
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
  const auto * type_it = transmission_it->FirstChildElement(kPluginNameTag);
  transmission.type = get_text_for_element(type_it, kPluginNameTag);

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
  hardware.is_async = parse_is_async_attribute(ros2_control_it);

  // Parse everything under ros2_control tag
  hardware.hardware_plugin_name = "";
  const auto * ros2_control_child_it = ros2_control_it->FirstChildElement();
  while (ros2_control_child_it)
  {
    if (std::string(kHardwareTag) == ros2_control_child_it->Name())
    {
      const auto * type_it = ros2_control_child_it->FirstChildElement(kPluginNameTag);
      hardware.hardware_plugin_name =
        get_text_for_element(type_it, std::string("hardware ") + kPluginNameTag);
      const auto * group_it = ros2_control_child_it->FirstChildElement(kGroupTag);
      if (group_it)
      {
        hardware.group = get_text_for_element(group_it, std::string("hardware.") + kGroupTag);
      }
      const auto * params_it = ros2_control_child_it->FirstChildElement(kParamTag);
      if (params_it)
      {
        hardware.hardware_parameters = parse_parameters_from_xml(params_it);
      }
    }
    else if (std::string(kJointTag) == ros2_control_child_it->Name())
    {
      hardware.joints.push_back(parse_component_from_xml(ros2_control_child_it));
    }
    else if (std::string(kSensorTag) == ros2_control_child_it->Name())
    {
      hardware.sensors.push_back(parse_component_from_xml(ros2_control_child_it));
    }
    else if (std::string(kGPIOTag) == ros2_control_child_it->Name())
    {
      hardware.gpios.push_back(parse_complex_component_from_xml(ros2_control_child_it));
    }
    else if (std::string(kTransmissionTag) == ros2_control_child_it->Name())
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

/**
 * @brief Retrieve the min and max values from the interface tag.
 * @param itf The interface tag to retrieve the values from.
 * @param min The minimum value to be retrieved.
 * @param max The maximum value to be retrieved.
 * @return true if the values are retrieved, false otherwise.
 */
bool retrieve_min_max_interface_values(const InterfaceInfo & itf, double & min, double & max)
{
  try
  {
    if (itf.min.empty() && itf.max.empty())
    {
      // If the limits don't exist then return false as they are not retrieved
      return false;
    }
    if (!itf.min.empty())
    {
      min = hardware_interface::stod(itf.min);
    }
    if (!itf.max.empty())
    {
      max = hardware_interface::stod(itf.max);
    }
    return true;
  }
  catch (const std::invalid_argument & err)
  {
    std::cerr << "Error parsing the limits for the interface: " << itf.name << " from the tags ["
              << kMinTag << ": '" << itf.min << "' and " << kMaxTag << ": '" << itf.max
              << "'] within " << kROS2ControlTag << " tag inside the URDF. Skipping it"
              << std::endl;
    return false;
  }
}

/**
 * @brief Set custom values for acceleration and jerk limits (no URDF standard)
 * @param itr The interface tag to retrieve the values from.
 * @param limits The joint limits to be set.
 */
void set_custom_interface_values(const InterfaceInfo & itr, joint_limits::JointLimits & limits)
{
  if (itr.name == hardware_interface::HW_IF_ACCELERATION)
  {
    double max_decel(std::numeric_limits<double>::quiet_NaN()),
      max_accel(std::numeric_limits<double>::quiet_NaN());
    if (detail::retrieve_min_max_interface_values(itr, max_decel, max_accel))
    {
      if (std::isfinite(max_decel))
      {
        limits.max_deceleration = std::fabs(max_decel);
        if (!std::isfinite(max_accel))
        {
          limits.max_acceleration = std::fabs(limits.max_deceleration);
        }
        limits.has_deceleration_limits = itr.enable_limits;
      }
      if (std::isfinite(max_accel))
      {
        limits.max_acceleration = max_accel;

        if (!std::isfinite(limits.max_deceleration))
        {
          limits.max_deceleration = std::fabs(limits.max_acceleration);
        }
        limits.has_acceleration_limits = itr.enable_limits;
      }
    }
  }
  else if (itr.name == "jerk")
  {
    if (!itr.min.empty())
    {
      std::cerr << "Error parsing the limits for the interface: " << itr.name
                << " from the tag: " << kMinTag << " within " << kROS2ControlTag
                << " tag inside the URDF. Jerk only accepts max limits." << std::endl;
    }
    double min_jerk(std::numeric_limits<double>::quiet_NaN()),
      max_jerk(std::numeric_limits<double>::quiet_NaN());
    if (
      !itr.max.empty() && detail::retrieve_min_max_interface_values(itr, min_jerk, max_jerk) &&
      std::isfinite(max_jerk))
    {
      limits.max_jerk = std::abs(max_jerk);
      limits.has_jerk_limits = itr.enable_limits;
    }
  }
  else
  {
    if (!itr.min.empty() || !itr.max.empty())
    {
      std::cerr << "Unable to parse the limits for the interface: " << itr.name
                << " from the tags [" << kMinTag << " and " << kMaxTag << "] within "
                << kROS2ControlTag
                << " tag inside the URDF. Supported interfaces for joint limits are: "
                   "position, velocity, effort, acceleration and jerk."
                << std::endl;
    }
  }
}

/**
 * @brief Retrieve the limits from ros2_control command interface tags and override URDF limits if
 * restrictive
 * @param interfaces The interfaces to retrieve the limits from.
 * @param limits The joint limits to be set.
 */
void update_interface_limits(
  const std::vector<InterfaceInfo> & interfaces, joint_limits::JointLimits & limits)
{
  for (auto & itr : interfaces)
  {
    if (itr.name == hardware_interface::HW_IF_POSITION)
    {
      limits.min_position = limits.has_position_limits && itr.enable_limits
                              ? limits.min_position
                              : -std::numeric_limits<double>::max();
      limits.max_position = limits.has_position_limits && itr.enable_limits
                              ? limits.max_position
                              : std::numeric_limits<double>::max();
      double min_pos(limits.min_position), max_pos(limits.max_position);
      if (itr.enable_limits && detail::retrieve_min_max_interface_values(itr, min_pos, max_pos))
      {
        limits.min_position = std::max(min_pos, limits.min_position);
        limits.max_position = std::min(max_pos, limits.max_position);
        limits.has_position_limits = true;
      }
      limits.has_position_limits &= itr.enable_limits;
    }
    else if (itr.name == hardware_interface::HW_IF_VELOCITY)
    {
      limits.max_velocity =
        limits.has_velocity_limits ? limits.max_velocity : std::numeric_limits<double>::max();
      // Apply the most restrictive one in the case
      double min_vel(-limits.max_velocity), max_vel(limits.max_velocity);
      if (itr.enable_limits && detail::retrieve_min_max_interface_values(itr, min_vel, max_vel))
      {
        max_vel = std::min(std::abs(min_vel), max_vel);
        limits.max_velocity = std::min(max_vel, limits.max_velocity);
        limits.has_velocity_limits = true;
      }
      limits.has_velocity_limits &= itr.enable_limits;
    }
    else if (itr.name == hardware_interface::HW_IF_EFFORT)
    {
      limits.max_effort =
        limits.has_effort_limits ? limits.max_effort : std::numeric_limits<double>::max();
      // Apply the most restrictive one in the case
      double min_eff(-limits.max_effort), max_eff(limits.max_effort);
      if (itr.enable_limits && detail::retrieve_min_max_interface_values(itr, min_eff, max_eff))
      {
        max_eff = std::min(std::abs(min_eff), max_eff);
        limits.max_effort = std::min(max_eff, limits.max_effort);
        limits.has_effort_limits = true;
      }
      limits.has_effort_limits &= itr.enable_limits;
    }
    else
    {
      detail::set_custom_interface_values(itr, limits);
    }
  }
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

  if (std::string(kRobotTag) != robot_it->Name())
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

  // parse full URDF for mimic options
  urdf::Model model;
  if (!model.initString(urdf))
  {
    throw std::runtime_error("Failed to parse URDF file");
  }
  for (auto & hw_info : hardware_info)
  {
    for (auto i = 0u; i < hw_info.joints.size(); ++i)
    {
      const auto & joint = hw_info.joints.at(i);
      auto urdf_joint = model.getJoint(joint.name);
      if (!urdf_joint)
      {
        throw std::runtime_error("Joint '" + joint.name + "' not found in URDF");
      }
      if (!urdf_joint->mimic && joint.is_mimic == MimicAttribute::TRUE)
      {
        throw std::runtime_error(
          "Joint '" + joint.name + "' has no mimic information in the URDF.");
      }
      if (urdf_joint->mimic && joint.is_mimic != MimicAttribute::FALSE)
      {
        if (joint.command_interfaces.size() > 0)
        {
          throw std::runtime_error(
            "Joint '" + joint.name +
            "' has mimic attribute not set to false: Activated mimic joints cannot have command "
            "interfaces.");
        }
        auto find_joint = [&hw_info](const std::string & name)
        {
          auto it = std::find_if(
            hw_info.joints.begin(), hw_info.joints.end(),
            [&name](const auto & j) { return j.name == name; });
          if (it == hw_info.joints.end())
          {
            throw std::runtime_error("Mimic joint '" + name + "' not found in <ros2_control> tag");
          }
          return std::distance(hw_info.joints.begin(), it);
        };

        MimicJoint mimic_joint;
        mimic_joint.joint_index = i;
        mimic_joint.mimicked_joint_index = find_joint(urdf_joint->mimic->joint_name);
        mimic_joint.multiplier = urdf_joint->mimic->multiplier;
        mimic_joint.offset = urdf_joint->mimic->offset;
        hw_info.mimic_joints.push_back(mimic_joint);
      }

      if (urdf_joint->type == urdf::Joint::FIXED)
      {
        throw std::runtime_error(
          "Joint '" + joint.name +
          "' is of type 'fixed'. "
          "Fixed joints do not make sense in ros2_control.");
      }

      joint_limits::JointLimits limits;
      getJointLimits(urdf_joint, limits);
      // Take the most restricted one. Also valid for continuous-joint type only
      detail::update_interface_limits(joint.command_interfaces, limits);
      hw_info.limits[joint.name] = limits;
      joint_limits::SoftJointLimits soft_limits;
      if (getSoftJointLimits(urdf_joint, soft_limits))
      {
        if (limits.has_position_limits)
        {
          soft_limits.min_position = std::max(soft_limits.min_position, limits.min_position);
          soft_limits.max_position = std::min(soft_limits.max_position, limits.max_position);
        }
        hw_info.soft_limits[joint.name] = soft_limits;
      }
    }
  }

  return hardware_info;
}

std::vector<InterfaceDescription> parse_state_interface_descriptions(
  const std::vector<ComponentInfo> & component_info)
{
  std::vector<InterfaceDescription> component_state_interface_descriptions;
  component_state_interface_descriptions.reserve(component_info.size());

  for (const auto & component : component_info)
  {
    for (const auto & state_interface : component.state_interfaces)
    {
      component_state_interface_descriptions.emplace_back(
        InterfaceDescription(component.name, state_interface));
    }
  }
  return component_state_interface_descriptions;
}

std::vector<InterfaceDescription> parse_command_interface_descriptions(
  const std::vector<ComponentInfo> & component_info)
{
  std::vector<InterfaceDescription> component_command_interface_descriptions;
  component_command_interface_descriptions.reserve(component_info.size());

  for (const auto & component : component_info)
  {
    for (const auto & command_interface : component.command_interfaces)
    {
      component_command_interface_descriptions.emplace_back(
        InterfaceDescription(component.name, command_interface));
    }
  }
  return component_command_interface_descriptions;
}

}  // namespace hardware_interface
