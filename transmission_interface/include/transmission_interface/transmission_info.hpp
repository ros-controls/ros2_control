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

#ifndef TRANSMISSION_INTERFACE__TRANSMISSION_INFO_HPP_
#define TRANSMISSION_INTERFACE__TRANSMISSION_INFO_HPP_

#include <string>

#include "transmission_interface/visibility_control.h"

namespace transmission_interface
{

enum class TRANSMISSION_INTERFACE_PUBLIC_TYPE JointControlType: std::uint8_t
{

  POSITION,
  VELOCITY,
  EFFORT,
  Count  // cardinality indicator
};

template<JointControlType joint_control_type>
struct JointControlTypeString;

template<>
struct JointControlTypeString<JointControlType::POSITION>
{
  static constexpr const char * const string = "PositionJointInterface";
};

template<>
struct JointControlTypeString<JointControlType::VELOCITY>
{
  static constexpr const char * const string = "VelocityJointInterface";
};

template<>
struct JointControlTypeString<JointControlType::EFFORT>
{
  static constexpr const char * const string = "EffortJointInterface";
};

struct TRANSMISSION_INTERFACE_PUBLIC_TYPE TransmissionInfo
{
  std::string joint_name{};
  JointControlType joint_control_type{JointControlType::Count};
};

std::string to_string(JointControlType jct)
{
  if (jct == JointControlType::POSITION) {
    return JointControlTypeString<JointControlType::POSITION>::string;
  }
  if (jct == JointControlType::VELOCITY) {
    return JointControlTypeString<JointControlType::VELOCITY>::string;
  }
  if (jct == JointControlType::EFFORT) {
    return JointControlTypeString<JointControlType::EFFORT>::string;
  }
  return "";
}

}  // namespace transmission_interface
#endif  // TRANSMISSION_INTERFACE__TRANSMISSION_INFO_HPP_
