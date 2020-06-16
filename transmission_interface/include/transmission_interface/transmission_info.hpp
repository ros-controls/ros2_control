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

#include "hardware_interface/control_type.hpp"

namespace transmission_interface
{

struct TransmissionInfo
{
  std::string joint_name;
  std::string joint_control_type;
};

}  // namespace transmission_interface
#endif  // TRANSMISSION_INTERFACE__TRANSMISSION_INFO_HPP_
