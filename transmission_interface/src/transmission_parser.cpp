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

#include <string>
#include <vector>

namespace transmission_interface
{

static constexpr const char * kTransmissionTag = "transmission";
static constexpr const char * kJointTag = "joint";
static constexpr const char * kHardwareInterfaceTag = "hardwareInterface";

bool TransmissionParser::parse_transmission_info(
  const std::string & urdf, std::vector<TransmissionInfo> & transmissions)
{
  // initialize TiXmlDocument doc with a string
  tinyxml2::XMLDocument doc;
  if (!doc.Parse(urdf.c_str()) && doc.Error()) {
    return false;
  }

  // Find joints in transmission tags
  tinyxml2::XMLElement * root_it = doc.RootElement();
  tinyxml2::XMLElement * trans_it = root_it->FirstChildElement(kTransmissionTag);
  while (trans_it) {
    // Joint name
    TransmissionInfo info{"", ""};
    auto joint_it = trans_it->FirstChildElement(kJointTag);
    if (joint_it) {
      info.joint_name = joint_it->Attribute("name");

      auto hardware_interface_it = joint_it->FirstChildElement(kHardwareInterfaceTag);
      if (hardware_interface_it) {
        info.hardware_interface = hardware_interface_it->GetText();
      }
    }

    if (info.joint_name.empty() || info.hardware_interface.empty()) {
      return false;
    }
    transmissions.push_back(info);

    trans_it = trans_it->NextSiblingElement(kTransmissionTag);
  }

  return true;
}

}  // namespace transmission_interface
