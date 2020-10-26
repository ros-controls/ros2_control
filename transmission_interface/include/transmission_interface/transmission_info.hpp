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

#include <vector>
#include <string>


namespace transmission_interface
{

/**
 * \brief Contains semantic info about a given joint loaded from XML (URDF)
 */
struct JointInfo
{
  std::string name;
  std::vector<std::string> interfaces;
  std::string role;
};

/**
 * \brief Contains semantic info about a given actuator loaded from XML (URDF)
 */
struct ActuatorInfo
{
  std::string name;
  std::vector<std::string> interfaces;
  double mechanical_reduction;
};

/**
 * \brief Contains semantic info about a given transmission loaded from XML (URDF)
 */
struct TransmissionInfo
{
  std::string name;
  std::string type;
  std::vector<JointInfo> joints;
  std::vector<ActuatorInfo> actuators;
};

}  // namespace transmission_interface

#endif  // TRANSMISSION_INTERFACE__TRANSMISSION_INFO_HPP_
