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

#include <stdexcept>
#include <string>
#include <vector>

namespace transmission_interface
{

static constexpr const char * kTransmissionTag = "transmission";
static constexpr const char * kJointTag = "joint";
static constexpr const char * kHardwareInterfaceTag = "hardwareInterface";

TransmissionParser::TransmissionParser(const std::string & urdf)
: urdf_(urdf)
{
  if (urdf_.empty()) {
    throw std::runtime_error("empty URDF passed in to transmission parser");
  }
  if (!doc_.Parse(urdf_.c_str()) && doc_.Error()) {
    throw std::runtime_error("invalid URDF passed in to transmission parser");
  }
}

bool TransmissionParser::parse_transmission_info(std::vector<TransmissionInfo> & transmissions)
const
{
  // Find joints in transmission tags
  const tinyxml2::XMLElement * root_it = doc_.RootElement();
  const tinyxml2::XMLElement * trans_it = root_it->FirstChildElement(kTransmissionTag);
  while (trans_it) {
    // Joint name
    TransmissionInfo info;
    auto joint_it = trans_it->FirstChildElement(kJointTag);
    if (!joint_it) {
      set_error_msg("no joint child element found");
      return false;
    }

    info.joint_name = joint_it->Attribute("name");
    if (info.joint_name.empty()) {
      set_error_msg("no joint name attribute set");
      return false;
    }

    auto hardware_interface_it = joint_it->FirstChildElement(kHardwareInterfaceTag);
    if (!hardware_interface_it) {
      set_error_msg("no hardware interface tag found under transmission joint");
      return false;
    }

    std::string hardware_interface = hardware_interface_it->GetText();
    if (hardware_interface.empty()) {
      set_error_msg(std::string("no hardware interface specified in joint ") + info.joint_name);
      return false;
    }
    if (hardware_interface == "PositionJointInterface") {
      info.joint_control_type = JointControlType::POSITION;
    } else if (hardware_interface == "VelocityJointInterface") {
      info.joint_control_type = JointControlType::VELOCITY;
    } else if (hardware_interface == "EffortJointInterface") {
      info.joint_control_type = JointControlType::EFFORT;
    } else {
      set_error_msg(hardware_interface + " is no valid hardware interface");
      return false;
    }

    transmissions.push_back(info);

    trans_it = trans_it->NextSiblingElement(kTransmissionTag);
  }

  return true;
}

std::string TransmissionParser::get_error_msg() const
{
  auto error = error_msg_;
  reset_error_msg();
  return error;
}

void TransmissionParser::set_error_msg(std::string error_msg) const
{
  if (!error_msg_.empty()) {
    fprintf(stderr, "Warning: Overriding transmission parser error message\n");
  }
  error_msg_ = error_msg;
}

void TransmissionParser::reset_error_msg() const
{
  error_msg_ = "";
}

}  // namespace transmission_interface
