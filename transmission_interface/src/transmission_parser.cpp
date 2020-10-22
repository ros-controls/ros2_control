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

constexpr const auto kTransmissionTag = "transmission";
constexpr const auto kJointTag = "joint";
constexpr const auto kActuatorTag = "actuator";
constexpr const auto kTypeTag = "type";
constexpr const auto kRoleTag = "role";
constexpr const auto kHardwareInterfaceTag = "hardwareInterface";
constexpr const auto kMechanicalReductionTag = "mechanicalReduction";
}  // namespace

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
  if (urdf.empty()) {
    throw std::runtime_error("empty URDF passed to robot");
  }

  // initialize TiXmlDocument doc with a string
  tinyxml2::XMLDocument doc;
  if (!doc.Parse(urdf.c_str()) && doc.Error()) {
    throw std::runtime_error("Can't parse transmissions. Invalid robot description.");
  }

  // Find joints in transmission tags
  tinyxml2::XMLElement * root = doc.RootElement();
  if (root == nullptr) {
    throw std::runtime_error("Can't parse transmissions. Invalid robot description.");
  }

  // Constructs the transmissions by parsing custom xml.
  tinyxml2::XMLElement * trans_it = nullptr;
  for (trans_it = root->FirstChildElement(kTransmissionTag); trans_it;
    trans_it = trans_it->NextSiblingElement(kTransmissionTag))
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
    tinyxml2::XMLElement * type_child = trans_it->FirstChildElement(kTypeTag);
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

bool TransmissionParser::parse_joints(
  tinyxml2::XMLElement * trans_it,
  std::vector<JointInfo> & joints)
{
  // Loop through each available joint
  tinyxml2::XMLElement * joint_it = nullptr;
  for (joint_it = trans_it->FirstChildElement(kJointTag); joint_it;
    joint_it = joint_it->NextSiblingElement(kJointTag))
  {
    // Create new joint
    transmission_interface::JointInfo joint;

    // Joint name
    if (joint_it->Attribute("name")) {
      joint.name = joint_it->Attribute("name");
      if (joint.name.empty()) {
        throw std::runtime_error("expected valid joint name attribute.");
      }
    } else {
      throw std::runtime_error("expected name attribute for joint.");
    }

    tinyxml2::XMLElement * role_it = joint_it->FirstChildElement(kRoleTag);
    if (role_it) {
      joint.role = role_it->GetText() ? role_it->GetText() : std::string();
    }

    // Hardware interfaces (required)
    tinyxml2::XMLElement * hw_iface_it = nullptr;
    for (hw_iface_it = joint_it->FirstChildElement(kHardwareInterfaceTag); hw_iface_it;
      hw_iface_it = hw_iface_it->NextSiblingElement(kHardwareInterfaceTag))
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

    // Add joint to vector
    joints.push_back(joint);
  }

  return !joints.empty();
}

bool TransmissionParser::parse_actuators(
  tinyxml2::XMLElement * trans_it,
  std::vector<ActuatorInfo> & actuators)
{
  // Loop through each available actuator
  tinyxml2::XMLElement * actuator_it = nullptr;
  for (actuator_it = trans_it->FirstChildElement(kActuatorTag); actuator_it;
    actuator_it = actuator_it->NextSiblingElement(kActuatorTag))
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
    tinyxml2::XMLElement * hw_iface_it = nullptr;
    for (hw_iface_it = actuator_it->FirstChildElement(kHardwareInterfaceTag); hw_iface_it;
      hw_iface_it = hw_iface_it->NextSiblingElement(kHardwareInterfaceTag))
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
    tinyxml2::XMLElement * mechred_it = nullptr;
    for (mechred_it = actuator_it->FirstChildElement(kMechanicalReductionTag); mechred_it;
      mechred_it = mechred_it->NextSiblingElement(kMechanicalReductionTag))
    {
      // optional tag, so no error if element is empty
      if (mechred_it->GetText()) {
        const auto value = mechred_it->GetText();
        actuator.mechanical_reduction = atoi(value);
      }
    }

    // Add actuator to vector
    actuators.push_back(actuator);
  }

  return !actuators.empty();
}

}  // namespace transmission_interface
