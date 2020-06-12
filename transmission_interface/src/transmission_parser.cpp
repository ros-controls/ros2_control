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

#include "transmission_interface/transmission_parser.hpp"
#include "transmission_interface/transmission_info.hpp"

#include <tinyxml2.h>
#include <stdexcept>
#include <string>
#include <vector>

namespace
{
constexpr const auto kTransmissionTag = "transmission";
constexpr const auto kJointTag = "joint";
constexpr const auto kHardwareInterfaceTag = "hardwareInterface";
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
  // Find joints in transmission tags
  const tinyxml2::XMLElement * root_it = doc.RootElement();
  const tinyxml2::XMLElement * trans_it = root_it->FirstChildElement(kTransmissionTag);
  while (trans_it) {
    // Joint name
    auto joint_it = trans_it->FirstChildElement(kJointTag);
    if (!joint_it) {
      throw std::runtime_error("no joint child element found");
    }

    const std::string joint_name = joint_it->Attribute("name");
    if (joint_name.empty()) {
      throw std::runtime_error("no joint name attribute set");
    }

    const auto hardware_interface_it = joint_it->FirstChildElement(kHardwareInterfaceTag);
    if (!hardware_interface_it) {
      throw std::runtime_error(
              "no hardware interface tag found under transmission joint" + joint_name);
    }

    const std::string interface_name = hardware_interface_it->GetText();
    if (interface_name.empty()) {
      throw std::runtime_error("no hardware interface specified in joint " + joint_name);
    }

    transmissions.push_back({joint_name, interface_name});

    trans_it = trans_it->NextSiblingElement(kTransmissionTag);
  }

  return transmissions;
}

}  // namespace transmission_interface
