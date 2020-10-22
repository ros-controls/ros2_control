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

namespace
{
constexpr const auto kTransmissionParserLoggerName = "transmission_parser";
}

namespace transmission_interface
{

std::vector<TransmissionInfo> parse_transmissions_from_urdf(const std::string & urdf)
{
  TransmissionParser parser;
  std::vector<TransmissionInfo> transmissions;
  try {
    parser.parse(urdf, transmissions);
  } catch (const std::runtime_error & err) {
    RCLCPP_ERROR(
      rclcpp::get_logger(kTransmissionParserLoggerName),
      err.what()
    );
    throw err;
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
    throw std::runtime_error("Can't parse transmissions. Invalid robot description.");
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
        throw std::runtime_error("empty name attribute specified for transmission");
      }
    } else {
      throw std::runtime_error("no name attribute specified for transmission");
    }

    // Transmission type
    TiXmlElement * type_child = trans_it->FirstChildElement("type");
    if (!type_child) {
      throw std::runtime_error(
              "no type element found in transmission '" + transmission.name + "'.");
    }
    if (!type_child->GetText()) {
      throw std::runtime_error(
              "expected non-empty type element in transmission '" + transmission.name + "'.");
    }
    transmission.control_type = type_child->GetText();

    try {
      // Load joints
      if (!parse_joints(trans_it, transmission.joints)) {
        throw std::runtime_error("requires one joint element.");
      }

      // Load actuators
      if (!parse_actuators(trans_it, transmission.actuators)) {
        throw std::runtime_error("requires one actuator element.");
      }
    } catch (const std::runtime_error & ex) {
      // add the transmission name and rethrow
      throw std::runtime_error("transmission '" + transmission.name + "' " + ex.what());
    }

    // Save loaded transmission
    transmissions.push_back(transmission);
  }  // end for <transmission>

  return !transmissions.empty();
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
        throw std::runtime_error("expected valid joint name attribute.");
        continue;
      }
    } else {
      throw std::runtime_error("expected name attribute for joint.");
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
      if (hw_iface_it->GetText()) {
        const std::string hw_iface_name = hw_iface_it->GetText();
        joint.hardware_interfaces.push_back(hw_iface_name);
      } else {
        throw std::runtime_error("expected hardware interface name for joint " + joint.name + '.');
      }
    }

    if (joint.hardware_interfaces.empty()) {
      throw std::runtime_error(
              "joint " + joint.name + " has no valid hardware interface.");
    }

    // Joint xml element
    std::stringstream ss;
    ss << *joint_it;
    joint.xml_element_ = ss.str();

    // Add joint to vector
    joints.push_back(joint);
  }

  return !joints.empty();
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
        throw std::runtime_error("expected valid actuator name attribute.");
      }
    } else {
      throw std::runtime_error("expected name attribute for actuator.");
    }

    // Hardware interfaces (optional)
    TiXmlElement * hw_iface_it = nullptr;
    for (hw_iface_it = actuator_it->FirstChildElement("hardwareInterface"); hw_iface_it;
      hw_iface_it = hw_iface_it->NextSiblingElement("hardwareInterface"))
    {
      // Skipping empty hardware interface element in actuator
      if (hw_iface_it->GetText()) {
        const std::string hw_iface_name = hw_iface_it->GetText();
        actuator.hardware_interfaces.push_back(hw_iface_name);
      } else {
        throw std::runtime_error(
                "expected hardware interface name for actuator " + actuator.name + ".");
      }
    }
    if (actuator.hardware_interfaces.empty()) {
      throw std::runtime_error(
              "actuator " + actuator.name + " has no valid hardware interface.");
    }

    // mechanical reduction (optional)
    actuator.mechanical_reduction = 1;
    TiXmlElement * mechred_it = nullptr;
    for (mechred_it = actuator_it->FirstChildElement("mechanicalReduction"); mechred_it;
      mechred_it = mechred_it->NextSiblingElement("mechanicalReduction"))
    {
      // optional tag, so no error if element is empty
      if (mechred_it->GetText()) {
        const auto value = mechred_it->GetText();
        actuator.mechanical_reduction = atoi(value);
      }
    }

    // Actuator xml element
    std::stringstream ss;
    ss << *actuator_it;
    actuator.xml_element_ = ss.str();

    // Add actuator to vector
    actuators.push_back(actuator);
  }

  return !actuators.empty();
}

}  // namespace transmission_interface
