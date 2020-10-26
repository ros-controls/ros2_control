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

namespace
{
constexpr const auto kTransmissionParserLoggerName = "transmission_parser";

constexpr const auto kTransmissionTag = "transmission";
constexpr const auto kNameTag = "name";
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
  if (urdf.empty()) {
    throw std::runtime_error("empty URDF passed in to transmission parser");
  }

  tinyxml2::XMLDocument doc;
  if (!doc.Parse(urdf.c_str()) && doc.Error()) {
    throw std::runtime_error("invalid URDF passed in to transmission parser");
  }

  std::vector<TransmissionInfo> transmissions;
  tinyxml2::XMLElement * root_it = doc.RootElement();
  tinyxml2::XMLElement * trans_it = nullptr;
  for (trans_it = root_it->FirstChildElement(kTransmissionTag); trans_it;
    trans_it = trans_it->NextSiblingElement(kTransmissionTag))
  {
    transmission_interface::TransmissionInfo transmission;

    if (trans_it->Attribute(kNameTag)) {
      transmission.name = trans_it->Attribute(kNameTag);
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
    transmission.type = type_child->GetText();

    try {
      // Load joints
      transmission.joints = parse_joints(trans_it);
      // Load actuators
      transmission.actuators = parse_actuators(trans_it);
    } catch (const std::runtime_error & ex) {
      // add the transmission name and rethrow
      throw std::runtime_error("transmission '" + transmission.name + "' " + ex.what());
    }

    transmissions.push_back(transmission);
  }

  return transmissions;
}

std::vector<JointInfo> parse_joints(tinyxml2::XMLElement * trans_it)
{
  std::vector<JointInfo> joints;
  // Loop through each available joint
  auto joint_it = trans_it->FirstChildElement(kJointTag);
  if (!joint_it) {
    throw std::runtime_error("no joint child element found");
  }
  for (; joint_it; joint_it = joint_it->NextSiblingElement(kJointTag)) {
    transmission_interface::JointInfo joint;

    joint.name = joint_it->Attribute(kNameTag);
    if (joint.name.empty()) {
      throw std::runtime_error("no joint name attribute set");
    }

    const tinyxml2::XMLElement * role_it = joint_it->FirstChildElement(kRoleTag);
    if (role_it) {
      joint.role = role_it->GetText() ? role_it->GetText() : std::string();
    }

    // Interfaces (required)
    auto interface_it = joint_it->FirstChildElement(kHardwareInterfaceTag);
    if (!interface_it) {
      throw std::runtime_error(
              "no hardware interface tag found under transmission joint" + joint.name);
    }

    for (; interface_it; interface_it = interface_it->NextSiblingElement(kHardwareInterfaceTag)) {
      const std::string interface_name = interface_it->GetText();
      if (interface_name.empty()) {
        throw std::runtime_error("no hardware interface specified in joint " + joint.name);
      }
      joint.interfaces.push_back(interface_name);
    }

    if (joint.interfaces.empty()) {
      throw std::runtime_error(
              "joint " + joint.name + " has no valid hardware interface.");
    }

    joints.push_back(joint);
  }

  return joints;
}

std::vector<ActuatorInfo> parse_actuators(tinyxml2::XMLElement * trans_it)
{
  std::vector<ActuatorInfo> actuators;
  // Loop through each available actuator
  auto actuator_it = trans_it->FirstChildElement(kActuatorTag);
  if (!actuator_it) {
    throw std::runtime_error("no actuator child element found");
  }

  for (; actuator_it; actuator_it = actuator_it->NextSiblingElement(kActuatorTag)) {
    transmission_interface::ActuatorInfo actuator;

    actuator.name = actuator_it->Attribute(kNameTag);
    if (actuator.name.empty()) {
      throw std::runtime_error("no actuator name attribute set");
    }

    // Hardware interfaces (optional)
    auto interface_it = actuator_it->FirstChildElement(kHardwareInterfaceTag);
    if (!interface_it) {
      throw std::runtime_error(
              "no hardware interface tag found under transmission actuator" + actuator.name);
    }

    for (; interface_it; interface_it = interface_it->NextSiblingElement(kHardwareInterfaceTag)) {
      const std::string interface_name = interface_it->GetText();
      if (interface_name.empty()) {
        throw std::runtime_error("no hardware interface specified in actuator " + actuator.name);
      }
      actuator.interfaces.push_back(interface_name);
    }

    if (actuator.interfaces.empty()) {
      throw std::runtime_error(
              "actuator " + actuator.name + " has no valid hardware interface.");
    }

    // mechanical reduction (optional)
    actuator.mechanical_reduction = 1;
    auto mechred_it = actuator_it->FirstChildElement(kMechanicalReductionTag);
    for (; mechred_it; mechred_it = mechred_it->NextSiblingElement(kMechanicalReductionTag)) {
      // optional tag but if specified, it should not be empty!
      const std::string mech_red_str = mechred_it->GetText();
      if (mech_red_str.empty()) {
        throw std::runtime_error("mechanical reduction tag was specified without value");
      } else {
        actuator.mechanical_reduction = atoi(mech_red_str.c_str());
      }
    }

    actuators.push_back(actuator);
  }

  return actuators;
}

}  // namespace transmission_interface
