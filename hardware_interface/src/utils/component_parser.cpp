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

#include "hardware_interface/utils/component_parser.hpp"

#include <stdexcept>
#include <string>
#include <tinyxml2.h>

#include "hardware_interface/component_info.hpp"

namespace
{
constexpr const auto kRobotTag = "robot";
constexpr const auto kROS2ControlTag = "ros2_control";
constexpr const auto kHardwareTag = "hardware";
constexpr const auto kClassTypeTag = "classType";
constexpr const auto kParamTag = "param";
constexpr const auto kJointTag = "joint";
constexpr const auto kInterfaceNameTag = "interfaceName";

// For compleate reference of syntax - not used in parser
//constexpr const auto kActuatorTag = "actuator";
//constexpr const auto kSensorTag = "sensor";
}  // namespace

namespace hardware_interface
{
namespace utils
{

ComponentInfo parse_robot_from_urdf(const std::string & urdf)
{
  // Check if everything OK with URDF string
  if (urdf.empty()) {
    throw std::runtime_error("empty URDF passed to robot");
  }
  tinyxml2::XMLDocument doc;
  if (!doc.Parse(urdf.c_str()) && doc.Error()) {
    throw std::runtime_error("invalid URDF passed in to robot parser");
  }

  // Find robot tag
  const tinyxml2::XMLElement * robot_it = doc.RootElement();

  if (std::string(kRobotTag).compare(robot_it->Name())) {
    throw std::runtime_error("the robot tag is not root element inURDF");
  }

  ComponentInfo robot;
  robot.name = robot_it->Attribute("name");
  if (robot.name.empty()) {
    throw std::runtime_error("no robot name attribute set");
  }
  robot.type = kRobotTag;

  const tinyxml2::XMLElement * ros2_control_it = robot_it->FirstChildElement(kROS2ControlTag);
  const std::string name = ros2_control_it->Attribute("name");
  if (name.empty()) {
    throw std::runtime_error("no attribute name in " + std::string(kROS2ControlTag) + " tag");
  }

  // Parse everything under ros2_control tag
  robot.hardware_class_type = "";
  const auto * ros2_control_child_it = ros2_control_it->FirstChildElement();
  while (ros2_control_child_it) {
    if (!std::string(kHardwareTag).compare(ros2_control_child_it->Name())) {
      const auto * type_it = ros2_control_child_it->FirstChildElement(kClassTypeTag);
      robot.hardware_class_type = type_it->GetText();
      const auto * params_it = ros2_control_child_it->FirstChildElement(kParamTag);
      if (params_it) {
        robot.hardware_parameters = parse_parameters_from_xml(params_it);
      }
    } else {
      robot.subcomponents.push_back(parse_component_from_xml(ros2_control_child_it) );
    }

    ros2_control_child_it = ros2_control_child_it->NextSiblingElement();
  }

  return robot;
}

ComponentInfo parse_component_from_xml(const tinyxml2::XMLElement * component_it)
{
  ComponentInfo component;
  // Find name, type and class for component
  component.type = component_it->Name();
  component.name = component_it->Attribute("name");
  if (component.name.empty()) {
    throw std::runtime_error("no name attribute set in " + component.type + " tag");
  }

  const auto * classType_it = component_it->FirstChildElement(kClassTypeTag);
  if (!classType_it) {
    throw std::runtime_error("no class type tag found in " + component.name);
  }
  component.class_type = classType_it->GetText();
  if (component.class_type.empty()) {
    throw std::runtime_error("no class type specified in " + component.name);
  }

  // Find joints and itnerface names in control component
  const auto * joint_it = component_it->FirstChildElement(kJointTag);
  if (!joint_it) {
    throw std::runtime_error("no joint element found in  " + component.name);
  }
  component.joint = joint_it->Attribute("name");
  if (component.joint.empty()) {
    throw std::runtime_error("no joint attribute name found in " + component.name);
  }
  const auto * interface_name_it = joint_it->FirstChildElement(kInterfaceNameTag);
  if (!interface_name_it) {
    throw std::runtime_error(
            "no interface names found for " + component.joint + " in " + component.name);
  }
  while (interface_name_it) {
    const std::string interface_name = interface_name_it->GetText();
    if (interface_name.empty()) {
      throw std::runtime_error(
              "no interface name value in " + component.joint + " of " + component.name);
    }
    component.interface_names.push_back(interface_name);

    interface_name_it = interface_name_it->NextSiblingElement();
  }

  // Parse paramter tags
  const auto * params_it = component_it->FirstChildElement(kParamTag);
  if (params_it) {
    component.parameters = parse_parameters_from_xml(params_it);
  }

  // Parse hardware tag
  const auto * hardware_it = component_it->FirstChildElement(kHardwareTag);
  if (hardware_it) {
    const auto * type_it = hardware_it->FirstChildElement(kClassTypeTag);
    component.hardware_class_type = type_it->GetText();
    const auto * params_it = hardware_it->FirstChildElement(kParamTag);
    if (params_it) {
      component.hardware_parameters = parse_parameters_from_xml(params_it);
    }
  } else {
    component.hardware_class_type = "";
  }

  return component;
}

std::unordered_map<std::string, std::string> parse_parameters_from_xml(
  const tinyxml2::XMLElement * params_it)
{
  std::unordered_map<std::string, std::string> parameters;
  while (params_it) {
    // Fill the map with parameters
    const std::string parameter_name = params_it->Attribute("name");
    if (parameter_name.empty()) {
      throw std::runtime_error("no parameter name attribute set in param tag");
    }
    const std::string parameter_value = params_it->GetText();
    if (parameter_name.empty()) {
      throw std::runtime_error("no parameter value set for " + parameter_name);
    }
    parameters[parameter_name] = parameter_value;

    params_it = params_it->NextSiblingElement();
  }
  return parameters;
}

}  // namespace utils
}  // namespace robot_control_components
