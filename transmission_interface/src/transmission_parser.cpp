// Copyright 2020 Open Source Robotics Foundation, Inc.
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

#include <transmission_interface/transmission_parser.hpp>

#include <sstream>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"


namespace transmission_interface
{
static const rclcpp::Logger g_logger(rclcpp::get_logger("transmission_parser"));

std::vector<TransmissionInfo> parse_transmissions_from_urdf(const std::string & urdf)
{
  TransmissionParser parser;
  std::vector<TransmissionInfo> transmissions;
  if (!parser.parse(urdf, transmissions)) {
    throw std::runtime_error("failed to parse transmissions in URDF");
  }
  return transmissions;
}


bool TransmissionParser::parse(
  const std::string & urdf,
  std::vector<TransmissionInfo> & transmissions)
{
  // initialize TiXmlDocument doc with a string
  TiXmlDocument doc;
  if (!doc.Parse(urdf.c_str()) && doc.Error()) {
    RCLCPP_ERROR(g_logger, "Can't parse transmissions. Invalid robot description.");
    return false;
  }

  // Find joints in transmission tags
  TiXmlElement * root = doc.RootElement();

  // Constructs the transmissions by parsing custom xml.
  TiXmlElement * trans_it = nullptr;
  for (trans_it = root->FirstChildElement("transmission"); trans_it;
    trans_it = trans_it->NextSiblingElement("transmission"))
  {
    transmission_interface::TransmissionInfo transmission;

    // Transmission name
    if (trans_it->Attribute("name")) {
      transmission.name = trans_it->Attribute("name");
      if (transmission.name.empty()) {
        RCLCPP_ERROR_STREAM(g_logger, "Empty name attribute specified for transmission.");
        throw std::runtime_error("Empty name attribute specified for transmission");
      }
    } else {
      RCLCPP_ERROR_STREAM(g_logger, "No name attribute specified for transmission.");
      throw std::runtime_error("No name attribute specified for transmission");
    }

    // Transmission type
    TiXmlElement * type_child = trans_it->FirstChildElement("type");
    if (!type_child) {
      RCLCPP_ERROR_STREAM(
        g_logger,
        "No type element found in transmission '" << transmission.name << "'.");
      throw std::runtime_error("No type element found in transmission");
    }
    if (!type_child->GetText()) {
      RCLCPP_ERROR_STREAM(
        g_logger, "Skipping empty type element in transmission '" <<
          transmission.name << "'.");
      throw std::runtime_error("empty type element in transmission");
    }
    transmission.control_type = type_child->GetText();

    // Load joints
    if (!parse_joints(trans_it, transmission.joints)) {
      RCLCPP_ERROR_STREAM(
        g_logger, "Failed to load joints for transmission '" <<
          transmission.name << "'.");
      throw std::runtime_error("Failed to load joints for transmission");
    }

    // Load actuators
    if (!parse_actuators(trans_it, transmission.actuators)) {
      RCLCPP_ERROR_STREAM(
        g_logger, "Failed to load actuators for transmission '" <<
          transmission.name << "'.");
      throw std::runtime_error("Failed to load actuators for transmission");
    }

    // Save loaded transmission
    transmissions.push_back(transmission);
  }  // end for <transmission>

  if (transmissions.empty()) {
    RCLCPP_DEBUG_STREAM(g_logger, "No valid transmissions found.");
  }

  return true;
}

bool TransmissionParser::parse_joints(TiXmlElement * trans_it, std::vector<JointInfo> & joints)
{
  // Loop through each available joint
  TiXmlElement * joint_it = nullptr;
  for (joint_it = trans_it->FirstChildElement("joint"); joint_it;
    joint_it = joint_it->NextSiblingElement("joint"))
  {
    // Create new joint
    transmission_interface::JointInfo joint;

    // Joint name
    if (joint_it->Attribute("name")) {
      joint.name = joint_it->Attribute("name");
      if (joint.name.empty()) {
        RCLCPP_ERROR_STREAM(g_logger, "Empty name attribute specified for joint.");
        continue;
      }
    } else {
      RCLCPP_ERROR_STREAM(g_logger, "No name attribute specified for joint.");
      return false;
    }

    TiXmlElement * role_it = joint_it->FirstChildElement("role");
    if (role_it) {
      joint.role_ = role_it->GetText() ? role_it->GetText() : std::string();
    }

    // Hardware interfaces (required)
    TiXmlElement * hw_iface_it = nullptr;
    for (hw_iface_it = joint_it->FirstChildElement("hardwareInterface"); hw_iface_it;
      hw_iface_it = hw_iface_it->NextSiblingElement("hardwareInterface"))
    {
      if (!hw_iface_it) {continue;}
      if (!hw_iface_it->GetText()) {
        RCLCPP_DEBUG_STREAM(
          g_logger, "Skipping empty hardware interface element in joint '" <<
            joint.name << "'.");
        throw std::runtime_error("empty hardware interface element in joint");
      }
      const std::string hw_iface_name = hw_iface_it->GetText();
      joint.hardware_interfaces.push_back(hw_iface_name);
    }

    if (joint.hardware_interfaces.empty()) {
      RCLCPP_ERROR_STREAM(
        g_logger, "No valid hardware interface element found in joint '" <<
          joint.name << "'.");
      throw std::runtime_error("No valid hardware interface element found in joint");
    }

    // Joint xml element
    std::stringstream ss;
    ss << *joint_it;
    joint.xml_element_ = ss.str();

    // Add joint to vector
    joints.push_back(joint);
  }

  if (joints.empty()) {
    RCLCPP_DEBUG(g_logger, "No valid joint element found.");
    return false;
  }

  return true;
}

bool TransmissionParser::parse_actuators(
  TiXmlElement * trans_it,
  std::vector<ActuatorInfo> & actuators)
{
  // Loop through each available actuator
  TiXmlElement * actuator_it = nullptr;
  for (actuator_it = trans_it->FirstChildElement("actuator"); actuator_it;
    actuator_it = actuator_it->NextSiblingElement("actuator"))
  {
    // Create new actuator
    transmission_interface::ActuatorInfo actuator;

    // Actuator name
    if (actuator_it->Attribute("name")) {
      actuator.name = actuator_it->Attribute("name");
      if (actuator.name.empty()) {
        RCLCPP_ERROR_STREAM(g_logger, "Empty name attribute specified for actuator.");
        throw std::runtime_error("Empty name attribute specified for actuator");
      }
    } else {
      RCLCPP_ERROR_STREAM(g_logger, "No name attribute specified for actuator.");
      throw std::runtime_error("No name attribute specified for actuator");
    }

    // Hardware interfaces (optional)
    TiXmlElement * hw_iface_it = nullptr;
    for (hw_iface_it = actuator_it->FirstChildElement("hardwareInterface"); hw_iface_it;
      hw_iface_it = hw_iface_it->NextSiblingElement("hardwareInterface"))
    {
      if (!hw_iface_it->GetText()) {
        RCLCPP_DEBUG_STREAM(
          g_logger, "Skipping empty hardware interface element in actuator '" <<
            actuator.name << "'.");
        continue;
      }
      const std::string hw_iface_name = hw_iface_it->GetText();
      actuator.hardware_interfaces.push_back(hw_iface_name);
    }
    if (actuator.hardware_interfaces.empty()) {
      RCLCPP_DEBUG_STREAM(
        g_logger, "No valid hardware interface element found in actuator '" <<
          actuator.name << "'.");
      // continue; // NOTE: Hardware interface is optional, so we keep on going
    }

    // mechanical reduction (optional)
    actuator.mechanical_reduction = 1;
    TiXmlElement * mechred_it = nullptr;
    for (mechred_it = actuator_it->FirstChildElement("mechanicalReduction"); mechred_it;
      mechred_it = mechred_it->NextSiblingElement("mechanicalReduction"))
    {
      if (!mechred_it->GetText()) {
        RCLCPP_DEBUG_STREAM(
          g_logger, "Skipping empty mechanicalReduction element in actuator '" <<
            actuator.name << "'.");
        continue;
      }
      const auto value = mechred_it->GetText();
      actuator.mechanical_reduction = atoi(value);
    }

    // Actuator xml element
    std::stringstream ss;
    ss << *actuator_it;
    actuator.xml_element_ = ss.str();

    // Add actuator to vector
    actuators.push_back(actuator);
  }

  if (actuators.empty()) {
    RCLCPP_DEBUG(g_logger, "No valid actuator element found.");
    return false;
  }

  return true;
}

}  // namespace transmission_interface
